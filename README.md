# RX\_com\_leds.ino (Receptor ESP32-S3)

# 1\. Alerta visual de sinal fraco/perda

# LEDs dos decks piscam rápido quando RSSI < -75 dBm ou pacotes perdidos > 20

# LEDs apagados quando deck sem sinal (timeout), sólidos quando OK

# Hold de 2s no aviso para não ficar piscando o tempo todo

# 2\. Tom de teste de DAC

# Comandos seriais TEST\_A, TEST\_B, TEST\_OFF enviam tom de 1kHz (10s, 25% amplitude) por deck

# Permite testar cada DAC individualmente sem mover fios

# Botões correspondentes no dashboard

# 3\. Modo isolamento SOLO\_DECK\_A

# SOLO\_DECK\_A / SOLO\_DECK\_A\_USA\_PINOS\_B para compilar apenas o deck A (uma porta I2S)

# Permite isolar se o problema é interferência entre as duas portas I2S ou hardware defeituoso

# 4\. Correção do clock I2S duplo (ESP32-S3)

# Reinstalação da porta I2S 0 após a porta 1 para corrompimento de clock no driver legado

# Reordenação: i2s\_set\_clk da porta 0 por último (a porta 1 por último corrompia o clock da 0)

# 5\. Telemetria expandida (24 campos)

# Adicionados: tempo desde último pacote, buffers de áudio por 0.5s, pico de amostra, erro I2S, status do tom de teste

# Cálculo de delta de audioWriteCount a cada 0.5s

# 6\. Parada robusta com debounce

# STOP\_DEBOUNCE\_MS 250 — debounce antes de considerar parado (evita falsas paradas por jitter)

# CALIB\_THRESHOLD\_RPM reduzido de 5.0 para 1.0 (detecta parada mais cedo)

# 7\. Suavização de RPM adaptativa

# RPM\_SMOOTHING de 0.12 para 0.25

# Alpha alto (0.8) para deltas > 10 RPM (resposta rápida em transientes), baixo (0.25) para deltas pequenos (estabilidade)

# 8\. Pino BCK do Deck A alterado

# DAC\_A\_BCK\_PIN de 10 para 2 (compatibilidade com hardware do original\_S3\_Receptor)

# 9\. Protocolo de recepção paralelo

# Arrays de hasPendingWelcome\[2], pendingWelcomeMac\[2]\[6], etc. — suporte a dois decks simultâneos no handshake

# 10\. Timeout estendido

# DECK\_TIMEOUT\_MS de 1000 para 1500ms (tolerância a perda momentânea de pacotes)

# TX\_M5StickCPlus2\_lcd.ino (Transmissor M5StickC+2)

# 11\. Seletor de Deck (A / B / Sem Deck)

# Botão A alterna entre Sem Deck → Deck A → Deck B → Sem Deck

# Display mostra o estado atual no topo

# Handshake automático ao trocar de deck

# Envio de pacotes para = 0 não acontece (para de transmitir)

# 12\. Auto-trim contínuo do giroscópio

# Quando o prato está parado por 2 segundos, recalcula o offset do giroscópio

# Elimina drift do timecode causado por residual do giroscópio parado

# 13\. Filtros recalibrados

# ALPHA\_SLOW: 0.15 → 0.35 | ALPHA\_FAST: 0.50 → 0.70

# FAST\_THRESHOLD\_RPM: 3.0 → 1.5 | DEADZONE\_RPM: 0.15 → 0.5

# RPM\_MULTIPLIER: 1.00 → 1.001 (micro-ajuste)

# 14\. Calibração automática mais robusta

# AUTO\_CALIBRATION\_STABLE\_SAMPLES: 150 → 400

# AUTO\_CALIBRATION\_SAMPLE\_DELAY\_MS: 1 → 2

# AUTO\_CALIBRATION\_STABLE\_DPS\_DELTA: 2.5 → 2.0

# AUTO\_CALIBRATION\_MIN\_COMPARE\_SAMPLES: 10 → 20

# Timeout configurável via AUTO\_CALIBRATION\_TIMEOUT\_MS (4000ms)

# 15\. Taxa de envio aumentada

# SEND\_RATE\_HZ: 150 → 300 Hz (latência reduzida pela metade)

# 16\. Power off mais rápido

# POWER\_OFF\_HOLD\_MS: 2000 → 500ms

# 17\. Splash screen atualizada

# Mostra "Sem Deck" no splash

# dvs\_dashboard.html (Dashboard)

# 18\. Botões de teste de DAC

# "Testar DAC A", "Testar DAC B", "Parar" — enviam comandos seriais via WebSerial API

# 19\. Telemetria expandida no display

# Novos campos por deck: Chegada de pacote (ms), Áudio (buf/0.5s), Pico amostra, Erro I2S, Status tom teste

# 20\. Correções de display

# Batteria: battery-fill agora usa display:block e cores inline (evita conflito de classe)

# Valores de bateria NaN tratados (mostra 0% em vez de quebrar)

# 21\. WebSerial bidirecional

# writer para enviar comandos ao RX (TEST\_A/TEST\_B/TEST\_OFF)

# Tratamento de erro no envio

