/*
 * ESP32 WROOM32 — Voice Assistant  v6  (WebSocket + OLED + PTT + Fan)
 * =====================================================================
 * Wake:     GPIO35 HIGH (KY-037 DO) → ext0 deep-sleep wakeup
 * PTT btn:  GPIO34  (input-only, RTC) — one side → 3.3 V, other → GPIO34
 *           Add a 10 kΩ pull-down resistor: GPIO34 → 10k → GND
 *           Hold = record, release = stop + send EOS
 * Mic:      INMP441  SCK=14, WS=15, SD=32, L/R=GND  (I2S_NUM_1)
 * Speaker:  PAM8403  DAC=GPIO25+GPIO26  (I2S_NUM_0 built-in DAC)
 * OLED:     128×64 I²C SSD1306  SDA=21, SCL=22
 * FAN:      Any relay/MOSFET module  IN=GPIO27  (HIGH = fan ON, LOW = fan OFF)
 *           Wire: GPIO27 → relay IN, relay COM → fan power, relay NO → fan
 *
 * Libraries (Arduino Library Manager):
 *   "WebSockets"        by Markus Sattler
 *   "Adafruit SSD1306"  by Adafruit
 *   "Adafruit GFX Library" by Adafruit
 *
 * Protocol (v6 — command field added to JSON):
 *   Audio frame : [0xAA][0x55][LEN_H][LEN_L][int16 PCM bytes...]
 *   EOS frame   : [0xAA][0x55][0x00][0x00]
 *   Server text : {"heard":"...","answer":"...","command":"fan_on"|"fan_off"|null}
 *   Server audio: same binary framing → DAC
 *
 * OLED states:
 *   COLD BOOT  → "Cold Boot"
 *   CONNECTING → progress
 *   IDLE       → "Ready" + countdown + fan status
 *   STREAMING  → "Listening..." + RMS bar
 *   WAIT_REPLY → "Thinking..." + spinner
 *   PLAYING    → "Speaking..." + answer text
 *   SLEEPING   → "Going to Sleep"
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>
#include <esp_sleep.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── Network ──────────────────────────────────────────────────────────────────
const char* SSID       = "Victus";
const char* PASSWORD   = "68986898";
const char* SERVER_IP  = "192.168.137.1";
const char* MDNS_HOST  = "local-model-server";
const int   WS_PORT    = 8765;
const char* WS_PATH    = "/";

// ─── Pins ─────────────────────────────────────────────────────────────────────
#define WAKE_PIN       GPIO_NUM_35
#define BTN_PIN        34
#define FAN_PIN        27      // ← Relay/MOSFET controlling the fan
#define I2S_MIC        I2S_NUM_1
#define I2S_DAC        I2S_NUM_0
#define MIC_SCK        14
#define MIC_WS_PIN     15
#define MIC_SD         32
#define OLED_SDA       21
#define OLED_SCL       22

// ─── OLED ─────────────────────────────────────────────────────────────────────
#define OLED_W    128
#define OLED_H    64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

// ─── Audio ────────────────────────────────────────────────────────────────────
#define SAMPLE_RATE    16000
#define CHUNK_SAMPLES  256
#define DAC_GAIN       4

// ─── VAD ──────────────────────────────────────────────────────────────────────
#define VAD_SPEECH_RMS    800
#define SILENCE_CHECK_MS  3000
#define MAX_STREAM_MS     10000

// ─── Timeouts ─────────────────────────────────────────────────────────────────
#define SERVER_REPLY_TIMEOUT_MS  120000
#define PLAY_DONE_TIMEOUT_MS      1000
#define INACTIVITY_SLEEP_MS      120000
#define WS_CONNECT_TIMEOUT_MS    20000

// ─── States ───────────────────────────────────────────────────────────────────
typedef enum {
  CONNECTING,
  IDLE,
  STREAMING,
  WAIT_REPLY,
  PLAYING
} AppState;

AppState state = CONNECTING;

WebSocketsClient webSocket;

// ─── Audio buffers ────────────────────────────────────────────────────────────
static int32_t  micRaw[CHUNK_SAMPLES];
static int16_t  micPcm[CHUNK_SAMPLES];
static uint8_t  txBuf[4 + CHUNK_SAMPLES * 2];
static uint16_t dacBuf[CHUNK_SAMPLES * 2];

// ─── Timing / state vars ──────────────────────────────────────────────────────
static uint32_t streamStartMs   = 0;
static uint32_t lastVoiceMs     = 0;
static uint32_t lastPktMs       = 0;
static uint32_t waitReplyStart  = 0;
static uint32_t totalPktsSent   = 0;
static uint32_t lastActivityMs  = 0;
static bool     speechDetected  = false;
static bool     wsConnected     = false;
static bool     btnWasPressed   = false;
static bool     pttActive       = false;

// ─── Fan state ────────────────────────────────────────────────────────────────
static bool     fanOn           = false;   // tracks current fan state

// ─── Answer / heard buffers ───────────────────────────────────────────────────
static char answerBuf[256] = {0};
static char heardBuf[128]  = {0};

// ─── Fan control ─────────────────────────────────────────────────────────────

void setFan(bool on) {
  fanOn = on;
  digitalWrite(FAN_PIN, on ? HIGH : LOW);
  Serial.printf("[FAN] %s (GPIO%d=%s)\n", on ? "ON" : "OFF", FAN_PIN, on ? "HIGH" : "LOW");
}

// ─── OLED helpers ─────────────────────────────────────────────────────────────

static void oledTitle(const char* title) {
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  int16_t x = (OLED_W - (int16_t)strlen(title) * 6) / 2;
  if (x < 0) x = 0;
  oled.setCursor(x, 0);
  oled.print(title);
}

static int oledWrap(const char* text, int y, int textSize = 1) {
  oled.setTextSize(textSize);
  int charW    = 6 * textSize;
  int lineH    = 9 * textSize;
  int maxChars = OLED_W / charW;
  int len      = strlen(text);
  int pos      = 0;
  while (pos < len && y < OLED_H) {
    int end = pos + maxChars;
    if (end >= len) end = len;
    else {
      int sp = end;
      while (sp > pos && text[sp] != ' ') sp--;
      if (sp > pos) end = sp;
    }
    oled.setCursor(0, y);
    for (int i = pos; i < end; i++) oled.print(text[i]);
    y += lineH;
    pos = end;
    if (pos < len && text[pos] == ' ') pos++;
  }
  return y;
}

static void oledBar(int y, int value) {
  oled.drawRect(0, y, OLED_W, 8, SSD1306_WHITE);
  int w = (value * (OLED_W - 4)) / 100;
  if (w > 0) oled.fillRect(2, y + 2, w, 4, SSD1306_WHITE);
}

static char spinChar(uint8_t n) {
  const char sp[] = {'-', '\\', '|', '/'};
  return sp[n & 3];
}

// ── State screens ─────────────────────────────────────────────────────────────

void oledShowConnecting(const char* detail) {
  oled.clearDisplay();
  oledTitle("-- CONNECTING --");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oledWrap(detail, 16);
  oled.display();
}

void oledShowIdle(uint32_t secondsLeft) {
  oled.clearDisplay();
  oledTitle("~~  READY  ~~");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 13);
  oled.println("Hold BTN to speak");
  // Fan status badge
  oled.setCursor(0, 24);
  oled.print("Fan: ");
  oled.print(fanOn ? "ON  [GPIO27=H]" : "OFF [GPIO27=L]");
  // Sleep countdown
  oled.setCursor(0, 35);
  oled.print("Sleep in: ");
  oled.print(secondsLeft);
  oled.print("s");
  // Last heard
  if (heardBuf[0]) {
    oled.setCursor(0, 47);
    char tmp[17]; strncpy(tmp, heardBuf, 16); tmp[16] = 0;
    oled.print("Q: "); oled.print(tmp);
  }
  oled.display();
}

void oledShowStreaming(float rms, bool ptt) {
  oled.clearDisplay();
  oledTitle("** LISTENING **");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 14);
  oled.println(ptt ? "PTT: Release to send" : "VAD: Speak now...");
  int barVal = (int)(rms / 100.0f);
  if (barVal > 100) barVal = 100;
  oledBar(28, barVal);
  oled.setCursor(0, 40);
  oled.print("RMS: ");
  oled.print((int)rms);
  oled.setCursor(70, 40);
  oled.print(rms > VAD_SPEECH_RMS ? "VOICE" : "quiet");
  oled.display();
}

void oledShowWaitReply(uint8_t spinIdx) {
  oled.clearDisplay();
  oledTitle("<< THINKING >>");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(56, 24);
  oled.print(spinChar(spinIdx));
  oled.setTextSize(1);
  oled.setCursor(10, 50);
  oled.print("STT + LLM + TTS...");
  oled.display();
}

void oledShowPlaying(const char* answer) {
  oled.clearDisplay();
  oledTitle(">> SPEAKING <<");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oledWrap(answer, 14);
  oled.display();
}

void oledShowFanAction(bool on) {
  oled.clearDisplay();
  oledTitle(on ? "  FAN  ON  " : "  FAN  OFF  ");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(20, 22);
  oled.print(on ? "GPIO27=H" : "GPIO27=L");
  oled.setTextSize(1);
  oled.setCursor(20, 50);
  oled.print(on ? "Fan turned ON" : "Fan turned OFF");
  oled.display();
}

void oledShowSleeping() {
  oled.clearDisplay();
  oledTitle("  Z z z . . .");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(14, 22);
  oled.println("Going to Sleep...");
  oled.setCursor(4, 36);
  oled.println("Clap/press to wake");
  oled.display();
}

// ─── Activity / sleep ─────────────────────────────────────────────────────────
void resetActivityTimer() { lastActivityMs = millis(); }
bool checkInactivity()    { return (state == IDLE && (millis() - lastActivityMs) > INACTIVITY_SLEEP_MS); }

// ─── Beep ─────────────────────────────────────────────────────────────────────
void playBeeps(int count, int freq_hz = 880, int dur_ms = 150) {
  int totalSamples = (SAMPLE_RATE * dur_ms) / 1000;
  static uint16_t tone[CHUNK_SAMPLES * 2];
  for (int b = 0; b < count; b++) {
    int samplesLeft = totalSamples, phase = 0;
    while (samplesLeft > 0) {
      int n = min(samplesLeft, CHUNK_SAMPLES);
      for (int i = 0; i < n; i++) {
        float s  = sinf(2.0f * M_PI * freq_hz * phase / SAMPLE_RATE);
        uint16_t v = (uint16_t)((int)(s * 20000) + 0x8000);
        tone[i*2] = v; tone[i*2+1] = v;
        phase++;
      }
      size_t w;
      i2s_write(I2S_DAC, tone, n * 4, &w, portMAX_DELAY);
      samplesLeft -= n;
    }
    if (b < count - 1) {
      int gap = SAMPLE_RATE / 10;
      while (gap > 0) {
        int n = min(gap, CHUNK_SAMPLES);
        memset(tone, 0x80, n * 4);
        size_t w;
        i2s_write(I2S_DAC, tone, n * 4, &w, portMAX_DELAY);
        gap -= n;
      }
    }
  }
}

// ─── I2S ──────────────────────────────────────────────────────────────────────
void setupMic() {
  const i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = CHUNK_SAMPLES,
    .use_apll             = true,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };
  const i2s_pin_config_t pins = {
    .bck_io_num   = MIC_SCK,
    .ws_io_num    = MIC_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = MIC_SD
  };
  ESP_ERROR_CHECK(i2s_driver_install(I2S_MIC, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_MIC, &pins));
  i2s_zero_dma_buffer(I2S_MIC);
  Serial.println("[MIC] Ready  SCK=14 WS=15 SD=32");
}

void setupDac() {
  const i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags     = 0,
    .dma_buf_count        = 8,
    .dma_buf_len          = CHUNK_SAMPLES,
    .use_apll             = true,
    .tx_desc_auto_clear   = true
  };
  ESP_ERROR_CHECK(i2s_driver_install(I2S_DAC, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN));
  i2s_zero_dma_buffer(I2S_DAC);
  Serial.println("[DAC] Ready  GPIO25(L)+GPIO26(R)");
}

// ─── Sleep ────────────────────────────────────────────────────────────────────
void goToSleep() {
  Serial.println("[SLEEP] Deep sleep → GPIO35 HIGH to wake");
  Serial.printf("[SLEEP] Fan state preserved: %s\n", fanOn ? "ON" : "OFF");
  // NOTE: GPIO27 output state is NOT preserved through deep sleep.
  // The fan will be OFF after wakeup (GPIO defaults to input/low).
  // If you need the fan to stay on across sleep, use RTC GPIO or a latch relay.
  Serial.flush();
  oledShowSleeping();
  delay(400);
  webSocket.disconnect();
  i2s_stop(I2S_MIC);  i2s_driver_uninstall(I2S_MIC);
  i2s_stop(I2S_DAC);  i2s_driver_uninstall(I2S_DAC);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  esp_sleep_enable_ext0_wakeup(WAKE_PIN, 1);
  esp_deep_sleep_start();
}

void sleepWithBeeps(int n, const char* reason) {
  Serial.printf("[SLEEP] %s\n", reason);
  playBeeps(n, 440, 200);
  delay(300);
  goToSleep();
}

// ─── Audio helpers ────────────────────────────────────────────────────────────
static float computeRms(const int16_t* buf, int n) {
  int64_t sum = 0;
  for (int i = 0; i < n; i++) { int32_t v = buf[i]; sum += (int64_t)v * v; }
  return (n > 0) ? sqrtf((float)sum / n) : 0.0f;
}

void sendAudioFrame(const int16_t* pcm, int samples) {
  uint16_t len = samples * 2;
  txBuf[0] = 0xAA;  txBuf[1] = 0x55;
  txBuf[2] = (len >> 8) & 0xFF;  txBuf[3] = len & 0xFF;
  memcpy(txBuf + 4, pcm, len);
  webSocket.sendBIN(txBuf, 4 + len);
  totalPktsSent++;
}

void sendEOS() {
  uint8_t eos[4] = {0xAA, 0x55, 0x00, 0x00};
  webSocket.sendBIN(eos, 4);
  Serial.printf("[MIC] EOS sent — %u packets\n", totalPktsSent);
}

void startStreaming() {
  size_t dummy;
  for (int i = 0; i < 8; i++)
    i2s_read(I2S_MIC, micRaw, sizeof(micRaw), &dummy, portMAX_DELAY);
  streamStartMs  = millis();
  lastVoiceMs    = millis();
  totalPktsSent  = 0;
  speechDetected = false;
  pttActive      = false;
  state          = STREAMING;
  Serial.printf("[STREAM] Started  VAD=%d  silence=%dms  max=%dms\n",
                VAD_SPEECH_RMS, SILENCE_CHECK_MS, MAX_STREAM_MS);
}

void playAudioFrame(const uint8_t* data, uint16_t payloadLen) {
  int16_t* samples    = (int16_t*)data;
  int      numSamples = payloadLen / 2;
  for (int i = 0; i < numSamples && i < CHUNK_SAMPLES; i++) {
    int32_t boosted  = (int32_t)samples[i] * DAC_GAIN;
    if (boosted >  32767) boosted =  32767;
    if (boosted < -32768) boosted = -32768;
    uint16_t v        = (uint16_t)(boosted + 0x8000);
    dacBuf[i * 2]     = v;
    dacBuf[i * 2 + 1] = v;
  }
  size_t written;
  i2s_write(I2S_DAC, dacBuf, numSamples * 4, &written, pdMS_TO_TICKS(50));
}

// ─── Simple JSON field extractor ─────────────────────────────────────────────
// Extracts the string value of a key from a flat JSON object.
// dest is filled with the value (without quotes). Returns true if found.
// ─── Robust JSON field extractor ─────────────────────────────────────────────
// Handles both  "key":"value"  and  "key": "value"  (space after colon)
// dest filled with the value without quotes. Returns true if found.
static bool jsonGetStr(const char* json, const char* key, char* dest, int destLen) {
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p += strlen(pat);

  // skip whitespace and colon
  while (*p == ' ' || *p == '\t') p++;
  if (*p != ':') return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;

  // handle null (unquoted)
  if (strncmp(p, "null", 4) == 0) {
    dest[0] = '\0';
    return false;   // treat null as "not present"
  }

  if (*p != '"') return false;
  p++;  // skip opening quote

  const char* e = strchr(p, '"');
  if (!e) return false;
  int n = (int)(e - p);
  if (n >= destLen) n = destLen - 1;
  strncpy(dest, p, n);
  dest[n] = '\0';
  return true;
}
// ─── WebSocket event handler ──────────────────────────────────────────────────
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      Serial.println("[WS] Connected");
      wsConnected = true;
      resetActivityTimer();
      playBeeps(1, 1760, 80);
      state = IDLE;
      oledShowIdle(INACTIVITY_SLEEP_MS / 1000);
      break;

    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      wsConnected = false;
      if (state != CONNECTING) state = CONNECTING;
      break;

    case WStype_TEXT: {
      char* txt = (char*)payload;
      Serial.println("[TEXT] Server:");
      Serial.println(txt);

      // ── Extract heard ──────────────────────────────────────────────────────
      heardBuf[0]  = '\0';
      answerBuf[0] = '\0';
      jsonGetStr(txt, "heard",  heardBuf,  sizeof(heardBuf));
      jsonGetStr(txt, "answer", answerBuf, sizeof(answerBuf));

      // ── Extract and execute command ────────────────────────────────────────
      char cmdBuf[32] = {0};
      if (jsonGetStr(txt, "command", cmdBuf, sizeof(cmdBuf))) {
        Serial.printf("[CMD] Received command: '%s'\n", cmdBuf);

        if (strcmp(cmdBuf, "fan_on") == 0) {
          setFan(true);
          oledShowFanAction(true);
          delay(600);   // brief visual feedback before next state update
        }
        else if (strcmp(cmdBuf, "fan_off") == 0) {
          setFan(false);
          oledShowFanAction(false);
          delay(600);
        }
        // Future commands (e.g. "led_on", "pump_on") can be added here
      }

      resetActivityTimer();
      break;
    }

    case WStype_BIN: {
      if (length < 4 || payload[0] != 0xAA || payload[1] != 0x55) break;
      uint16_t payloadLen = ((uint16_t)payload[2] << 8) | payload[3];

      if (payloadLen == 0) {
        // EOS from server
        if (state == WAIT_REPLY) {
          Serial.println("[PLAY] EOS with no audio");
          sleepWithBeeps(2, "no audio from server");
        }
        Serial.println("[PLAY] Done → IDLE");
        playBeeps(1, 1200, 80);
        resetActivityTimer();
        state = IDLE;
        oledShowIdle(INACTIVITY_SLEEP_MS / 1000);
        break;
      }

      // Audio data arriving
      if (state == WAIT_REPLY) {
        Serial.println("[PLAY] Audio arriving...");
        state     = PLAYING;
        lastPktMs = millis();
        resetActivityTimer();
        oledShowPlaying(answerBuf[0] ? answerBuf : "...");
      }
      lastPktMs = millis();
      playAudioFrame(payload + 4, payloadLen);
      break;
    }

    default: break;
  }
}

// ══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n========================================");
  Serial.println("  ESP32 Voice Assistant v6 (Fan+OLED)");
  Serial.println("========================================");

  // ── Fan GPIO — set LOW (off) immediately on boot ───────────────────────────
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  fanOn = false;
  Serial.printf("[FAN] GPIO%d initialised LOW (fan off)\n", FAN_PIN);

  // ── I²C + OLED ────────────────────────────────────────────────────────────
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] Init failed — check wiring/address");
  }

  // ── PTT button ────────────────────────────────────────────────────────────
  pinMode(BTN_PIN, INPUT);   // external 10k pull-down required

  // ── Wake cause ────────────────────────────────────────────────────────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause != ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("[BOOT] Cold boot → arming GPIO35");
    oled.clearDisplay();
    oledTitle("  COLD BOOT");
    oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 18);
    oled.println("Clap near KY-037");
    oled.println("or press PTT btn");
    oled.println("to wake assistant");
    oled.display();
    Serial.flush();
    esp_sleep_enable_ext0_wakeup(WAKE_PIN, 1);
    delay(100);
    esp_deep_sleep_start();
  }

  Serial.println("[WAKE] GPIO35 triggered");

  oled.clearDisplay();
  oledTitle("  WAKING UP...");
  oled.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 18);
  oled.println("Initialising...");
  oled.display();

  // ── I2S ───────────────────────────────────────────────────────────────────
  setupMic();
  setupDac();
  playBeeps(1, 1200, 200);

  // ── WiFi ──────────────────────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("[WiFi] Connecting");
  oledShowConnecting("Connecting WiFi...");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (++tries > 40) sleepWithBeeps(3, "WiFi failed");
    if (tries % 5 == 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "WiFi... (%ds)", tries / 3);
      oledShowConnecting(buf);
    }
  }
  Serial.printf("\n[WiFi] %s  RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  {
    char buf[40];
    snprintf(buf, sizeof(buf), "IP: %s", WiFi.localIP().toString().c_str());
    oledShowConnecting(buf);
  }

  // ── mDNS ──────────────────────────────────────────────────────────────────
  String serverHost = SERVER_IP;
  Serial.printf("[mDNS] Looking for %s.local...\n", MDNS_HOST);
  oledShowConnecting("mDNS discovery...");
  IPAddress ip = MDNS.queryHost(String(MDNS_HOST) + ".local", 2000);
  if (ip != INADDR_NONE) {
    serverHost = ip.toString();
    Serial.printf("[mDNS] Found: %s\n", serverHost.c_str());
  } else {
    Serial.println("[mDNS] Not found — using hardcoded IP");
  }

  // ── WebSocket ─────────────────────────────────────────────────────────────
  {
    char buf[48];
    snprintf(buf, sizeof(buf), "WS: %s:%d", serverHost.c_str(), WS_PORT);
    oledShowConnecting(buf);
  }
  Serial.printf("[WS] Connecting to ws://%s:%d%s\n",
                serverHost.c_str(), WS_PORT, WS_PATH);
  webSocket.begin(serverHost.c_str(), WS_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
  webSocket.enableHeartbeat(15000, 3000, 2);

  state = CONNECTING;
  resetActivityTimer();
  Serial.println("[WS] Waiting for connection...");
}

// ══════════════════════════════════════════════════════════════════════════════
void loop() {
  webSocket.loop();

  bool btnPressed = digitalRead(BTN_PIN) == HIGH;

  // ── CONNECTING ─────────────────────────────────────────────────────────────
  if (state == CONNECTING) {
    static uint32_t lastDot  = 0;
    static uint8_t  dotCount = 0;
    if (millis() - lastDot > 500) {
      lastDot = millis();
      char buf[24];
      const char* dots[] = {"", ".", "..", "..."};
      snprintf(buf, sizeof(buf), "Waiting WS%s", dots[dotCount % 4]);
      dotCount++;
      oledShowConnecting(buf);
    }
    if ((millis() - lastActivityMs) > WS_CONNECT_TIMEOUT_MS) {
      sleepWithBeeps(3, "WS connect timeout");
    }
    return;
  }

  // ── IDLE ───────────────────────────────────────────────────────────────────
  if (state == IDLE) {
    static uint32_t lastIdleUpdate = 0;
    if (millis() - lastIdleUpdate > 1000) {
      lastIdleUpdate = millis();
      uint32_t remaining = INACTIVITY_SLEEP_MS > (millis() - lastActivityMs)
                           ? (INACTIVITY_SLEEP_MS - (millis() - lastActivityMs)) / 1000
                           : 0;
      oledShowIdle(remaining);
    }
    if (btnPressed && !btnWasPressed) {
      Serial.println("[PTT] Button pressed → starting stream");
      pttActive = true;
      resetActivityTimer();
      startStreaming();
      btnWasPressed = true;
      return;
    }
    btnWasPressed = btnPressed;
    if (checkInactivity()) goToSleep();
    return;
  }

  // ── STREAMING ──────────────────────────────────────────────────────────────
  if (state == STREAMING) {
    if (pttActive && !btnPressed) {
      Serial.println("[PTT] Button released → EOS");
      sendEOS();
      state          = WAIT_REPLY;
      waitReplyStart = millis();
      oledShowWaitReply(0);
      btnWasPressed = false;
      return;
    }
    if (pttActive) btnWasPressed = true;

    size_t bytesRead = 0;
    i2s_read(I2S_MIC, micRaw, sizeof(micRaw), &bytesRead, pdMS_TO_TICKS(30));

    float rms = 0.0f;
    if (bytesRead > 0) {
      int n = bytesRead / 4;
      for (int i = 0; i < n; i++) {
        int32_t s = micRaw[i] >> 14;
        micPcm[i] = (int16_t)constrain(s, -32768, 32767);
      }
      rms = computeRms(micPcm, n);

      static int rmsLog = 0;
      if (++rmsLog >= 20) {
        Serial.printf("[VAD] RMS=%.0f  speech=%s  ptt=%s\n",
                      rms, speechDetected ? "YES" : "no",
                      pttActive ? "YES" : "no");
        rmsLog = 0;
      }

      if (rms > VAD_SPEECH_RMS) {
        lastVoiceMs    = millis();
        speechDetected = true;
        resetActivityTimer();
      }
      if (wsConnected) sendAudioFrame(micPcm, n);
    }

    static uint32_t lastOledStream = 0;
    if (millis() - lastOledStream > 80) {
      oledShowStreaming(rms, pttActive);
      lastOledStream = millis();
    }

    if (!pttActive) {
      bool silenceTimeout = speechDetected && (millis() - lastVoiceMs) > SILENCE_CHECK_MS;
      bool maxDuration    = (millis() - streamStartMs) >= MAX_STREAM_MS;

      if (silenceTimeout) {
        Serial.printf("[VAD] %.1fs silence → EOS\n", (millis()-lastVoiceMs)/1000.0f);
        sendEOS();
        state = WAIT_REPLY; waitReplyStart = millis();
        oledShowWaitReply(0);
      } else if (maxDuration) {
        if (!speechDetected) {
          Serial.println("[VAD] No speech in 10s");
          sendEOS();
          sleepWithBeeps(2, "no speech detected");
        }
        Serial.println("[VAD] Hard cap → EOS");
        sendEOS();
        state = WAIT_REPLY; waitReplyStart = millis();
        oledShowWaitReply(0);
      }
    }
    return;
  }

  // ── WAIT_REPLY ─────────────────────────────────────────────────────────────
  if (state == WAIT_REPLY) {
    static uint32_t lastSpin = 0;
    static uint8_t  spinIdx  = 0;
    if (millis() - lastSpin > 200) {
      oledShowWaitReply(spinIdx++);
      lastSpin = millis();
    }
    if ((millis() - waitReplyStart) > SERVER_REPLY_TIMEOUT_MS) {
      Serial.println("[WAIT] Server timeout");
      sleepWithBeeps(3, "server timeout");
    }
    return;
  }

  // ── PLAYING ────────────────────────────────────────────────────────────────
  if (state == PLAYING) {
    static uint32_t lastPlayUpdate = 0;
    if (millis() - lastPlayUpdate > 500) {
      oledShowPlaying(answerBuf[0] ? answerBuf : "Playing...");
      lastPlayUpdate = millis();
    }
    if ((millis() - lastPktMs) > PLAY_DONE_TIMEOUT_MS) {
      Serial.println("[PLAY] Timeout — assuming done");
      playBeeps(1, 1200, 80);
      resetActivityTimer();
      state = IDLE;
      oledShowIdle(INACTIVITY_SLEEP_MS / 1000);
    }
    return;
  }
}