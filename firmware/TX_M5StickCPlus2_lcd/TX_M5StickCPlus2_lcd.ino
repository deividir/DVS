/*
========================================================
 M5StickC PLUS2 TRANSMITTER (IMU embutido via M5Unified)
 DVS / Phase DIY - ESP-NOW low latency motion packet
========================================================
*/

#include <M5Unified.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// Estado do deck: 0 = Sem Deck, 1 = Deck A, 2 = Deck B
uint8_t deckId = 0;

#define ESPNOW_CHANNEL 11
#define SEND_RATE_HZ 300
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define HANDSHAKE_INTERVAL_MS 250
#define HANDSHAKE_TIMEOUT_MS 2000

#define PROTOCOL_VERSION 2
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4

// LED simples embutido do M5StickC PLUS2 (nao e' RGB)
#define STATUS_LED_PIN 19
#define LED_COMMON_ANODE 0
#define LED_BLINK_MS 50

uint8_t receiverMAC[] = { 0x14, 0xC1, 0x9F, 0x2C, 0xDE, 0x7C };

// Filtro assimetrico: suavizado para estabilizar o timecode
// (jitter do gyro amplificado pelos filtros antigos derrubava o lock do Serato)
float ALPHA_SLOW = 0.35f;
float ALPHA_FAST = 0.70f;
float FAST_THRESHOLD_RPM = 1.5f;
float DEADZONE_RPM = 0.5f;
float RPM_MULTIPLIER = 1.001f;

// Auto-calibracao acelerada
#define AUTO_CALIBRATION_STABLE_SAMPLES 400
#define AUTO_CALIBRATION_SAMPLE_DELAY_MS 2
#define AUTO_CALIBRATION_STABLE_DPS_DELTA 2.0f
#define AUTO_CALIBRATION_MIN_COMPARE_SAMPLES 20
#define AUTO_CALIBRATION_MAX_DPS 6.0f
#define AUTO_CALIBRATION_TIMEOUT_MS 4000

// Auto-trim continuo: corrige o residual do giroscopio quando o prato esta parado
#define AUTO_TRIM_WINDOW_MS 2000
#define AUTO_TRIM_STABLE_DELTA_DPS 2.0f
#define AUTO_TRIM_MAX_ABS_DPS 50.0f

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;
  uint8_t batteryPct;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

dvs_packet packet;

volatile bool receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;

float gyroOffsetZ = 0.0f;
float filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;
uint32_t lastHelloMillis = 0; // ultima vez que MSG_HELLO foi enviado

// Intervalo de atualizacao da tela de bateria (nao precisa ser rapido)
#define BATTERY_DISPLAY_INTERVAL_MS 1000
uint32_t lastBatteryDisplayMillis = 0;
int8_t lastBatteryLevelShown = -1; // forca 1a atualizacao
volatile uint8_t batteryLevelPct = 100; // bateria enviada no pacote ESP-NOW

// RSSI (forca do sinal ESP-NOW) exibido abaixo da bateria
#define RSSI_DISPLAY_INTERVAL_MS 400
uint32_t lastRssiDisplayMillis = 0;
volatile int8_t lastRssi = -127; // -127 = Sem Sinal
volatile uint32_t linkDropCount = 0; // quantas vezes o link ESP-NOW caiu
volatile uint8_t rxBootId = 0; // ID de boot enviado pelo RX (muda se ele resetar)
uint8_t lastSeenRxBootId = 0;
volatile uint32_t rxResetCount = 0; // quantas vezes o RX reiniciou
bool rxBootIdInitialized = false;

// ---------------- ECONOMIA DE ENERGIA: TELA SO LIGA PARADO ----------------
#define MOVEMENT_DPS_THRESHOLD 3.0f      // acima disso, conta como "girando"
#define SCREEN_IDLE_TIMEOUT_MS 3000      // tempo parado ate a tela ligar
#define SCREEN_BRIGHTNESS 100            // brilho quando ligada (0-255)
#define SPLASH_DURATION_MS 4000
#define POWER_OFF_HOLD_MS 500

bool screenIsOn = true;
uint32_t lastMovementMillis = 0;

// ---------------- FORWARD DECLARATIONS ----------------
// IMPORTANTE: declaradas aqui manualmente (com valor padrao incluso)
// para evitar o bug classico do Arduino IDE, que ao gerar prototipos
// automaticos para funcoes usadas antes de serem definidas, NAO
// preserva valores padrao de parametros (ex: "bool force = false"),
// causando erro de compilacao tipo "too few arguments to function"
// ou "default argument given for parameter after previous specification".
void updateBatteryDisplay(bool force = false);
void updateRssiDisplay();
void showSplashScreen();
void turnScreenOn();
void turnScreenOff();
void updateScreenPowerState(float dps);
void setLED(bool on);
void setupLED();
void blinkLED(uint16_t times);
void setupIMU();
static inline bool readGyroZDps(float *dps);
static inline float dpsToRPM(float dps);
void autoCalibrateGyroZ();
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len);
void setupEspNow();
void sendControlMessage(uint8_t msgType);
void updateDeckDisplay();
void toggleDeck();

// ---------------- LED (status de erro apenas) ----------------

static inline uint8_t ledOnLevel() {
  return LED_COMMON_ANODE ? LOW : HIGH;
}

static inline uint8_t ledOffLevel() {
  return LED_COMMON_ANODE ? HIGH : LOW;
}

void setLED(bool on) {
  digitalWrite(STATUS_LED_PIN, on ? ledOnLevel() : ledOffLevel());
}

void setupLED() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  setLED(false);
}

void blinkLED(uint16_t times) {
  for (uint16_t i = 0; i < times; i++) {
    setLED(true);
    delay(LED_BLINK_MS);
    setLED(false);
    delay(LED_BLINK_MS);
  }
}

// ---------------- TELA: SOMENTE % DE BATERIA, FONTE GRANDE ----------------

// Liga a tela (backlight + painel) e forca redesenho da bateria.
void turnScreenOn() {
  if (screenIsOn) return;
  screenIsOn = true;
  M5.Lcd.wakeup();
  M5.Lcd.setBrightness(SCREEN_BRIGHTNESS);
  lastBatteryLevelShown = -1; // forca redesenho mesmo se % nao mudou
  updateBatteryDisplay(true);
}

// Desliga a tela (backlight + sleep do painel) para economizar energia
// enquanto o disco esta girando e ninguem precisa olhar pra tela.
void turnScreenOff() {
  if (!screenIsOn) return;
  screenIsOn = false;
  M5.Lcd.setBrightness(0);
  M5.Lcd.sleep();
}

// Chama a cada leitura do giroscopio: decide se liga/desliga a tela
// com base em ter havido movimento recente ou nao.
void updateScreenPowerState(float dps) {
  uint32_t now = millis();
  if (fabsf(dps) > MOVEMENT_DPS_THRESHOLD) {
    lastMovementMillis = now;
    turnScreenOff();
  } else if (!screenIsOn && (now - lastMovementMillis > SCREEN_IDLE_TIMEOUT_MS)) {
    turnScreenOn();
  }
}

// Mostra apenas a porcentagem da bateria, centralizada e em fonte grande.
// Chamada periodicamente (nao a cada loop) para nao piscar a tela.
// So desenha se a tela estiver ligada.
// NOTA: o valor padrao "= false" fica SOMENTE no forward declaration
// la em cima. Aqui na definicao ele NAO pode ser repetido, senao o
// compilador acusa erro de redefinicao do valor padrao.
void updateBatteryDisplay(bool force) {
  if (!screenIsOn) return;

  uint32_t now = millis();
  if (!force && (now - lastBatteryDisplayMillis < BATTERY_DISPLAY_INTERVAL_MS)) return;
  lastBatteryDisplayMillis = now;

  int8_t level = M5.Power.getBatteryLevel(); // 0-100, ou -1 se desconhecido

  if (level >= 0) {
    batteryLevelPct = (uint8_t)constrain(level, 0, 100);
  }

  if (!force && level == lastBatteryLevelShown) return; // evita redesenho desnecessario
  lastBatteryLevelShown = level;

  M5.Lcd.fillScreen(TFT_BLACK);
  updateDeckDisplay();
  M5.Lcd.setTextDatum(middle_center);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setFont(&fonts::Font7); // fonte grande estilo "digital", so numeros
  M5.Lcd.setTextSize(1);

  char buf[8];
  if (level >= 0) {
    snprintf(buf, sizeof(buf), "%d%%", level);
  } else {
    snprintf(buf, sizeof(buf), "--%%");
  }

  M5.Lcd.drawString(buf, M5.Lcd.width() / 2, M5.Lcd.height() / 2);

  // RSSI abaixo da bateria
  char rssiBuf[16];
  if (receiverReady) {
    snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", lastRssi);
  } else {
    snprintf(rssiBuf, sizeof(rssiBuf), "Sem Sinal");
  }
  M5.Lcd.setFont(&fonts::Font4);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.drawString(rssiBuf, M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 42);

  // Contador de quedas de link e resets do RX (diagnostico)
  M5.Lcd.setFont(&fonts::Font2);
  M5.Lcd.setTextSize(1);
  char dropBuf[24];
  snprintf(dropBuf, sizeof(dropBuf), "Q:%u RX:%u", linkDropCount, rxResetCount);
  M5.Lcd.setTextColor((linkDropCount > 0 || rxResetCount > 0) ? TFT_RED : TFT_GRAY, TFT_BLACK);
  M5.Lcd.drawString(dropBuf, M5.Lcd.width() / 2, M5.Lcd.height() - 8);
}

// Atualiza o RSSI periodicamente desenhando apenas a area do texto,
// sem redesenhar a tela toda (evita gastar tempo de SPI e perder pacotes).
void updateRssiDisplay() {
  if (!screenIsOn) return;

  uint32_t now = millis();
  if (now - lastRssiDisplayMillis < RSSI_DISPLAY_INTERVAL_MS) return;
  lastRssiDisplayMillis = now;

  char rssiBuf[16];
  if (receiverReady) {
    snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", lastRssi);
  } else {
    snprintf(rssiBuf, sizeof(rssiBuf), "Sem Sinal");
  }

  int16_t x = M5.Lcd.width() / 2;
  int16_t y = M5.Lcd.height() / 2 + 42;

  M5.Lcd.setTextDatum(middle_center);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.setFont(&fonts::Font4);
  M5.Lcd.setTextSize(1);

  M5.Lcd.fillRect(x - 80, y - 14, 160, 28, TFT_BLACK); // limpa so a area do texto
  M5.Lcd.drawString(rssiBuf, x, y);

  // Contador de quedas de link e resets do RX (diagnostico)
  M5.Lcd.setFont(&fonts::Font2);
  M5.Lcd.setTextSize(1);
  char dropBuf[24];
  snprintf(dropBuf, sizeof(dropBuf), "Q:%u RX:%u", linkDropCount, rxResetCount);
  M5.Lcd.setTextColor((linkDropCount > 0 || rxResetCount > 0) ? TFT_RED : TFT_GRAY, TFT_BLACK);
  int16_t dy = M5.Lcd.height() - 8;
  M5.Lcd.fillRect(x - 60, dy - 8, 120, 16, TFT_BLACK);
  M5.Lcd.drawString(dropBuf, x, dy);
}

// ---------------- SELETOR DE DECK (Sem Deck / A / B) ----------------

// Desenha o estado atual do deck no topo da tela (regiao incremental,
// nao redesenha a tela toda para nao piscar nem atrasar o envio).
void updateDeckDisplay() {
  if (!screenIsOn) return;

  const char *label = (deckId == 1) ? "Deck A" : (deckId == 2 ? "Deck B" : "Sem Deck");
  int16_t x = M5.Lcd.width() / 2;
  int16_t y = 10;

  M5.Lcd.setTextDatum(middle_center);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setFont(&fonts::Font4);
  M5.Lcd.setTextSize(1);
  M5.Lcd.fillRect(x - 60, y - 14, 120, 28, TFT_BLACK);
  M5.Lcd.drawString(label, x, y);
}

// Avanca o estado do deck: Sem Deck -> A -> B -> Sem Deck.
// Ao associar um deck, forca o re-handshake para o RX re-associar o MAC.
// Ao voltar para Sem Deck, para de enviar (o RX expira o slot sozinho).
void toggleDeck() {
  deckId = (deckId + 1) % 3;
  receiverReady = false;
  lastReceiverReplyMillis = 0;
  lastHelloMillis = 0;
  turnScreenOn();
  updateDeckDisplay();
  blinkLED(1);
  const char *label = (deckId == 1) ? "A" : (deckId == 2 ? "B" : "Sem Deck");
  Serial.printf("Deck: %s\n", label);
}

// ---------------- IMU (BMI270 embutido, via M5Unified) ----------------

void setupIMU() {
  auto ret = M5.Imu.begin();
  int attempts = 0;
  while (!ret) {
    Serial.println("IMU nao encontrado!");
    blinkLED(1);
    attempts++;
    if (attempts > 10) break;
    ret = M5.Imu.begin();
  }
  Serial.println("IMU Inicializado.");
}

// Le o eixo Z do giroscopio em graus/s (dps), igual ao original
// Retorna false se a leitura falhar, para o loop nao enviar RPM lixo
// (que o receptor transformaria em timecode invalido -> Serato perde track).
static inline bool readGyroZDps(float *dps) {
  float gx, gy, gz;
  if (!M5.Imu.getGyroData(&gx, &gy, &gz)) {
    return false;
  }
  *dps = gz;
  return true;
}

static inline float dpsToRPM(float dps) {
  float corrected = dps - gyroOffsetZ;
  float rpm = -(corrected / 6.0f) * RPM_MULTIPLIER;
  return rpm;
}

void autoCalibrateGyroZ() {
  int stableSamples = 0;
  float sum = 0.0f;
  uint32_t startCal = millis();

  Serial.println("Calibrando...");
  while (stableSamples < AUTO_CALIBRATION_STABLE_SAMPLES) {
    float dps = 0.0f;
    readGyroZDps(&dps);

    if (fabsf(dps) > AUTO_CALIBRATION_MAX_DPS) {
      stableSamples = 0;
      sum = 0.0f;
      if (millis() - startCal > AUTO_CALIBRATION_TIMEOUT_MS) break;
      continue;
    }

    if (stableSamples >= AUTO_CALIBRATION_MIN_COMPARE_SAMPLES) {
      float mean = sum / stableSamples;
      if (fabsf(dps - mean) > AUTO_CALIBRATION_STABLE_DPS_DELTA) {
        stableSamples = 0;
        sum = 0.0f;
        if (millis() - startCal > AUTO_CALIBRATION_TIMEOUT_MS) break;
        continue;
      }
    }

    sum += dps;
    stableSamples++;
    delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
  }

  if (stableSamples >= AUTO_CALIBRATION_STABLE_SAMPLES) {
    gyroOffsetZ = sum / (float)stableSamples;
    Serial.println("Calibrado.");
  } else {
    gyroOffsetZ = 0.0f;
    Serial.println("Calibracao ignorada (prato em movimento): offset = 0");
  }
}

// Ajusta gyroOffsetZ continuamente enquanto o prato estiver parado.
// Se o dps ficar praticamente constante por AUTO_TRIM_WINDOW_MS, o prato
// esta parado -> anula o residual (evita o timecode derivando para tras).
void autoTrimGyroOffset(float dps) {
  static uint32_t windowStart = 0;
  static float wMin = 1e9f, wMax = -1e9f, wSum = 0.0f;
  static uint32_t wCount = 0;

  wMin = fminf(wMin, dps);
  wMax = fmaxf(wMax, dps);
  wSum += dps;
  wCount++;

  if (windowStart == 0) windowStart = millis();

  if (millis() - windowStart >= AUTO_TRIM_WINDOW_MS) {
    if (wCount > 0 && (wMax - wMin) <= AUTO_TRIM_STABLE_DELTA_DPS) {
      float meanDps = wSum / (float)wCount;
      if (fabsf(meanDps) <= AUTO_TRIM_MAX_ABS_DPS) {
        gyroOffsetZ = meanDps;
      }
    }
    wMin = 1e9f; wMax = -1e9f; wSum = 0.0f; wCount = 0;
    windowStart = millis();
  }
}

// ---------------- ESP-NOW ----------------

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet incoming;
  memcpy(&incoming, dataPtr, sizeof(incoming));
  if (incoming.version != PROTOCOL_VERSION || incoming.deckId != deckId) return;
  if (incoming.msgType == MSG_WELCOME || incoming.msgType == MSG_PING) {
    receiverReady = true;
    lastReceiverReplyMillis = millis();
    rxBootId = (uint8_t)incoming.gyroRaw; // bootId que o RX carrega nas msgs de controle
    if (info->rx_ctrl != NULL) {
      lastRssi = info->rx_ctrl->rssi; // forca do sinal em dBm
    }
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(78); // potencia maxima de TX (melhora alcance ESP-NOW)
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Long Range: aumenta sensibilidade e resiste a interferencia (1Mbps maximo,
  // mas a 150Hz de dados o link usa so uma fracao da banda).
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

  if (esp_now_init() != ESP_OK) return;

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendControlMessage(uint8_t msgType) {
  dvs_packet control = {};
  control.msgType = msgType;
  control.version = PROTOCOL_VERSION;
  control.deckId = deckId;
  control.seq = sequenceNumber;
  control.timestampMicros = micros();
  esp_now_send(receiverMAC, (uint8_t *)&control, sizeof(control));
}

// ---------------- SPLASH SCREEN ----------------

void showSplashScreen() {
  M5.Lcd.setBrightness(SCREEN_BRIGHTNESS);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextDatum(middle_center);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setTextSize(3);
  M5.Lcd.drawString("DJ", M5.Lcd.width() / 2, M5.Lcd.height() / 2 - 25);
  M5.Lcd.setTextSize(2);
  M5.Lcd.drawString("Deividi", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 15);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.drawString("Sem Deck", M5.Lcd.width() / 2, M5.Lcd.height() / 2 + 40);
  delay(SPLASH_DURATION_MS);
  M5.Lcd.setTextSize(1);
}

// ---------------- SETUP / LOOP ----------------

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);

  M5.Lcd.setRotation(1);
  showSplashScreen();

  setupLED();
  setupIMU();
  setupEspNow();

  if (deckId != 0) sendControlMessage(MSG_HELLO);

  autoCalibrateGyroZ();
  lastMovementMillis = millis(); // considera "parado" a partir de agora
  updateBatteryDisplay(true); // desenha a bateria assim que tudo estiver pronto
  nextSendMicros = micros();
}

void loop() {
  M5.update();

  if (M5.BtnA.pressed()) {
    toggleDeck();
  }

  if (M5.BtnB.pressedFor(POWER_OFF_HOLD_MS)) {
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setTextDatum(middle_center);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.drawString("OFF", M5.Lcd.width() / 2, M5.Lcd.height() / 2);
    delay(500);
    M5.Power.powerOff();
  }

  updateBatteryDisplay(); // so redesenha quando o intervalo passar ou o valor mudar
  updateRssiDisplay(); // atualiza a forca do sinal ESP-NOW periodicamente

  uint32_t now = micros();

  if (deckId != 0 && !receiverReady && (millis() - lastHelloMillis > HANDSHAKE_INTERVAL_MS)) {
    sendControlMessage(MSG_HELLO);
    lastHelloMillis = millis();
  }

  if (deckId == 0) return;

  if ((int32_t)(now - nextSendMicros) < 0) return;

  nextSendMicros += SEND_INTERVAL_US;
  if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) {
    nextSendMicros = now + SEND_INTERVAL_US;
  }

  float dps = 0.0f;
  if (!readGyroZDps(&dps)) {
    // Leitura IMU falhou: nao envia pacote com dados invalidos.
    // O receptor mantem o ultimo RPM valido (timeout agora e de 2s).
    return;
  }
  autoTrimGyroOffset(dps);
  // Tela fica ligada o tempo todo (economia de energia desabilitada a pedido).
  // updateScreenPowerState(dps); <- desativado
  float rpm = dpsToRPM(dps);

  float delta = rpm - filteredRPM;
  float alpha = (fabsf(delta) > FAST_THRESHOLD_RPM) ? ALPHA_FAST : ALPHA_SLOW;
  filteredRPM += delta * alpha;

  if (fabsf(filteredRPM) < DEADZONE_RPM) filteredRPM = 0.0f;

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = deckId;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = (int16_t)constrain(lroundf(dps * 10.0f), -32768, 32767);
  packet.batteryPct = batteryLevelPct;
  packet.seq = sequenceNumber++;
  packet.timestampMicros = now;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (receiverReady && (millis() - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS)) {
    receiverReady = false;
    linkDropCount++;
    Serial.println("Conexao perdida...");
    blinkLED(2);
  }

  // Detecta reset do RX: o bootId enviado nas msgs de controle muda.
  if (receiverReady) {
    if (!rxBootIdInitialized) {
      lastSeenRxBootId = rxBootId;
      rxBootIdInitialized = true;
    } else if (rxBootId != lastSeenRxBootId) {
      rxResetCount++;
      lastSeenRxBootId = rxBootId;
      Serial.println("RX reiniciou!");
      blinkLED(3);
    }
  }
}
