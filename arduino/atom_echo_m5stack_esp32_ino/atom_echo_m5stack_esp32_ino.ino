#TODO: WiFi 이름, 비밀번호, 서버주소, 포트 등 파일로 분리하여 관리


#include <M5Unified.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <ESP32Servo.h>

#define SERVO_PIN 25 // M5Atom Echo Grove Port (G21) - Check your wiring!
Servo myServo;

const char* SSID = "KT_GiGA_3926"; # WIFI 이름
const char* PASS = "fbx7bef119"; # WIFI 비밀번호

const char* SERVER_IP = "172.30.1.20"; # 서버 IP 주소
const uint16_t SERVER_PORT = 5001; # 서버 포트
WiFiClient client; # WiFi 클라이언트

enum State { IDLE, TALKING }; # 상태
State state = IDLE;
static constexpr uint8_t PTYPE_PING = 0x10; # 패킷 타입
static uint32_t last_ping_ms = 0; # 마지막 패킷 시간

static void sendPingIfIdle() { # 패킷 전송
  if (!client.connected()) return; # 클라이언트가 연결되어 있지 않으면 리턴
  uint32_t now = millis();
  if (now - last_ping_ms >= 3000) {   // 3초마다
    sendPacket(PTYPE_PING, nullptr, 0);
    last_ping_ms = now;
  }
}

// ===== Audio config =====
static constexpr uint32_t SR = 16000;
static constexpr size_t FRAME = 320;              // 20ms @ 16k
static constexpr uint32_t FRAME_MS = 20;

// pre-roll: 200ms
static constexpr uint32_t PREROLL_MS = 200;
static constexpr size_t PREROLL_SAMPLES = (SR * PREROLL_MS) / 1000; // 3200
static int16_t preroll_buf[PREROLL_SAMPLES];
static size_t preroll_pos = 0;
static bool preroll_full = false;

// ===== VAD params (auto threshold) =====
static float noise_floor = 120.0f;
static constexpr float NOISE_ALPHA = 0.995f;
static constexpr float VAD_ON_MUL  = 3.0f;
static constexpr float VAD_OFF_MUL = 1.8f;

static constexpr uint32_t MIN_TALK_MS    = 500;
static constexpr uint32_t SILENCE_END_MS = 350;
static constexpr uint32_t MAX_TALK_MS    = 8000;

static uint32_t talk_samples = 0;
static uint32_t silence_samples = 0;
static uint8_t  start_hit = 0;

// ===== Packet TX =====
static void sendPacket(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (!client.connected()) return;
  client.write(&type, 1);

  uint8_t le[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
  client.write(le, 2);

  if (len && payload) client.write(payload, len);
}

static void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.begin(SSID, PASS);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(100);
  }

  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    client.stop();
    if (client.connect(SERVER_IP, SERVER_PORT)) {
      client.setNoDelay(true);
      Serial.println("✅ server re-connected");
    }
  }
}

static inline float frame_rms(const int16_t* x, size_t n) {
  double ss = 0.0;
  for (size_t i = 0; i < n; i++) {
    double v = (double)x[i];
    ss += v * v;
  }
  return (float)sqrt(ss / (double)n);
}

static void preroll_push(const int16_t* x, size_t n) {
  for (size_t i = 0; i < n; i++) {
    preroll_buf[preroll_pos++] = x[i];
    if (preroll_pos >= PREROLL_SAMPLES) {
      preroll_pos = 0;
      preroll_full = true;
    }
  }
}

static void send_preroll() {
  size_t count = preroll_full ? PREROLL_SAMPLES : preroll_pos;
  if (count == 0) return;

  if (!preroll_full) {
    sendPacket(0x02, (uint8_t*)preroll_buf, (uint16_t)(count * sizeof(int16_t)));
    return;
  }

  size_t tail = PREROLL_SAMPLES - preroll_pos;
  sendPacket(0x02, (uint8_t*)(preroll_buf + preroll_pos), (uint16_t)(tail * sizeof(int16_t)));
  if (preroll_pos > 0) {
    sendPacket(0x02, (uint8_t*)preroll_buf, (uint16_t)(preroll_pos * sizeof(int16_t)));
  }
}

/* ============================================================
   ✅ RX: PC -> ESP32 CMD packet parser (non-blocking)
   Protocol: 1B type + 2B len(LE) + payload
   We care: type==0x11 (JSON command)
   ============================================================ */

static constexpr uint8_t  PTYPE_CMD = 0x11;       // PC -> ESP32
static constexpr size_t   RX_MAX_PAYLOAD = 512;   // JSON은 이 정도면 충분(넘으면 잘라서 버림)

enum RxStage { RX_TYPE, RX_LEN0, RX_LEN1, RX_PAYLOAD };
static RxStage rx_stage = RX_TYPE;
static uint8_t  rx_type = 0;
static uint16_t rx_len = 0;
static uint16_t rx_pos = 0;
static uint8_t  rx_buf[RX_MAX_PAYLOAD]; // payload buffer (truncated if longer)

// --- tiny JSON getters (dependency-free) ---
static bool json_get_string(const char* json, const char* key, char* out, size_t out_sz) {
  // find "key"
  char pat[64];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p && isspace((unsigned char)*p)) p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < out_sz) {
    out[i++] = *p++;
  }
  out[i] = 0;
  return (*p == '"');
}

static bool json_get_int(const char* json, const char* key, int* out) {
  char pat[64];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p && isspace((unsigned char)*p)) p++;
  // number begins
  bool neg = false;
  if (*p == '-') { neg = true; p++; }
  if (!isdigit((unsigned char)*p)) return false;
  long v = 0;
  while (isdigit((unsigned char)*p)) {
    v = v * 10 + (*p - '0');
    p++;
  }
  if (neg) v = -v;
  *out = (int)v;
  return true;
}

static bool json_get_bool(const char* json, const char* key, bool* out) {
  char pat[64];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p && isspace((unsigned char)*p)) p++;
  if (!strncmp(p, "true", 4)) { *out = true; return true; }
  if (!strncmp(p, "false", 5)) { *out = false; return true; }
  return false;
}

static void handleCmdJson(const uint8_t* payload, uint16_t len) {
  // payload -> null-terminated string (truncate safe)
  static char json[RX_MAX_PAYLOAD + 1];
  uint16_t n = (len > RX_MAX_PAYLOAD) ? RX_MAX_PAYLOAD : len;
  memcpy(json, payload, n);
  json[n] = 0;

  // ✅ 1) 먼저 raw 출력 (return값 그대로)
  Serial.println("\n===== 📥 CMD from PC =====");
  Serial.print("raw json: ");
  Serial.println(json);

  // ✅ 2) 파싱해서 필드별 출력 (로봇 동작 전 확인용)
  char action[32] = {0};
  int sid = -1;
  int angle = -1;
  bool meaningful = false;
  bool recognized = false;

  bool has_action = json_get_string(json, "action", action, sizeof(action));
  json_get_int(json, "sid", &sid);
  bool has_angle = json_get_int(json, "angle", &angle);
  json_get_bool(json, "meaningful", &meaningful);
  json_get_bool(json, "recognized", &recognized);

  Serial.print("action     : "); Serial.println(has_action ? action : "(missing)");
  Serial.print("sid        : "); Serial.println(sid);
  Serial.print("meaningful : "); Serial.println(meaningful ? "true" : "false");
  Serial.print("recognized : "); Serial.println(recognized ? "true" : "false");
  if (has_angle) { 
    Serial.print("angle      : "); Serial.println(angle);
  } else {
    Serial.println("angle      : (none)");
  }
  Serial.println("===== (before robot action) =====\n");

  // 🚧 Robot/Servo Actions
  // Policy: Always start 0 -> Action(max 1 time or 3sec) -> Return 0
  if (!meaningful) {
    if (strcmp(action, "WIGGLE") == 0) {
      Serial.println("🤷 WIGGLE (Not meaningful)");
    }
  } 
  else if (strcmp(action, "ROTATE") == 0) {
    Serial.println("🌀 Action: ROTATE -> 3s Sweep");
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
        myServo.write(180); delay(250);
        myServo.write(0);   delay(250);
    }
    myServo.write(0); // Return to 0
  }
  else if (strcmp(action, "STOP") == 0) {
    Serial.println("🛑 Action: STOP -> Return to 0");
    myServo.write(0);
  }
  else if (strcmp(action, "SERVO_SET") == 0 && has_angle) {
    Serial.printf("🔧 Action: SERVO_SET (Angle: %d) -> Hold 3s -> Return 0\n", angle);
    myServo.write(angle);
    delay(3000);
    myServo.write(0);
  }
}

static void pollServerPackets() {
  if (!client.connected()) return;

  // non-blocking: available 만큼만 먹고 빠짐
  while (client.available() > 0) {
    int b = client.read();
    if (b < 0) break;

    uint8_t byte = (uint8_t)b;

    switch (rx_stage) {
      case RX_TYPE:
        rx_type = byte;
        rx_len = 0;
        rx_pos = 0;
        rx_stage = RX_LEN0;
        break;

      case RX_LEN0:
        rx_len = (uint16_t)byte;
        rx_stage = RX_LEN1;
        break;

      case RX_LEN1:
        rx_len |= ((uint16_t)byte << 8);
        // payload length 0이면 바로 처리
        if (rx_len == 0) {
          if (rx_type == PTYPE_CMD) {
            handleCmdJson((const uint8_t*)"", 0);
          }
          rx_stage = RX_TYPE;
        } else {
          rx_stage = RX_PAYLOAD;
        }
        break;

      case RX_PAYLOAD:
        // payload가 너무 길면 RX_MAX_PAYLOAD까지만 저장하고 나머지는 버림
        if (rx_pos < RX_MAX_PAYLOAD) {
          rx_buf[rx_pos] = byte;
        }
        rx_pos++;

        if (rx_pos >= rx_len) {
          // packet complete
          if (rx_type == PTYPE_CMD) {
            uint16_t kept = (rx_len > RX_MAX_PAYLOAD) ? RX_MAX_PAYLOAD : rx_len;
            handleCmdJson(rx_buf, kept);
          }
          rx_stage = RX_TYPE;
        }
        break;
    }
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = true;
  cfg.internal_spk = false;
  M5.begin(cfg);

  M5.Mic.setSampleRate(SR);

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print("."); }
  Serial.println("\nWiFi connected");

  if (client.connect(SERVER_IP, SERVER_PORT)) {
    client.setNoDelay(true);
    Serial.println("✅ server connected");
  } else {
    Serial.println("❌ server connect failed");
  }

  // Servo Init
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(0); // Initial position 0
}

void loop() {
  ensureConnections();

  // ✅ (추가) 서버에서 오는 CMD 먼저 읽어서 출력
  pollServerPackets();
  sendPingIfIdle();

  static int16_t samples[FRAME];
  if (!M5.Mic.record(samples, FRAME)) return;

  // 프리롤 채우기
  preroll_push(samples, FRAME);

  float rms = frame_rms(samples, FRAME);

  if (state == IDLE) {
    noise_floor = NOISE_ALPHA * noise_floor + (1.0f - NOISE_ALPHA) * rms;

    float vad_on = fmaxf(noise_floor * VAD_ON_MUL, noise_floor + 120.0f);

    if (rms > vad_on) {
      if (++start_hit >= 2) {
        state = TALKING;
        talk_samples = 0;
        silence_samples = 0;

        Serial.println("🎙️ START");
        sendPacket(0x01, nullptr, 0);

        send_preroll();
        sendPacket(0x02, (uint8_t*)samples, (uint16_t)(FRAME * sizeof(int16_t)));

        talk_samples += FRAME;
      }
    } else {
      start_hit = 0;
    }
    return;
  }

  // TALKING: 오디오 전송
  sendPacket(0x02, (uint8_t*)samples, (uint16_t)(FRAME * sizeof(int16_t)));
  talk_samples += FRAME;

  float vad_off = fmaxf(noise_floor * VAD_OFF_MUL, noise_floor + 80.0f);

  if (rms < vad_off) silence_samples += FRAME;
  else silence_samples = 0;

  uint32_t talk_ms = (uint32_t)((1000ULL * talk_samples) / SR);
  uint32_t silence_ms = (uint32_t)((1000ULL * silence_samples) / SR);

  if ((talk_ms >= MIN_TALK_MS && silence_ms >= SILENCE_END_MS) || (talk_ms >= MAX_TALK_MS)) {
    state = IDLE;
    start_hit = 0;
    Serial.println("🛑 END");
    sendPacket(0x03, nullptr, 0);
  }
}
