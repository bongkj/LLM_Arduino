// ============================================================
//  ESP32 (M5Unified) + Mic streaming to PC + CMD(JSON) receive
//  + 4x SG90 servo control (pins: 25, 21, 26, 32)
//  - Non-blocking servo animations (WAVE/WIGGLE)
// ============================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include <ESP32Servo.h>

// -------------------- WiFi / Server --------------------
const char* SSID = "KT_GiGA_3926";     // WIFI 이름
const char* PASS = "fbx7bef119";      // WIFI 비밀번호

const char* SERVER_IP = "172.30.1.20"; // 서버 IP 주소
const uint16_t SERVER_PORT = 5001;     // 서버 포트

WiFiClient client;

// -------------------- State --------------------
enum State { IDLE, TALKING };
State state = IDLE;

// -------------------- Packet Types --------------------
static constexpr uint8_t PTYPE_START = 0x01;
static constexpr uint8_t PTYPE_AUDIO = 0x02;
static constexpr uint8_t PTYPE_END   = 0x03;

static constexpr uint8_t PTYPE_PING  = 0x10;
static constexpr uint8_t PTYPE_CMD   = 0x11;

// -------------------- Ping --------------------
static uint32_t last_ping_ms = 0;

// Forward decl
static void sendPacket(uint8_t type, const uint8_t* payload, uint16_t len);
static void handleCmdJson(const uint8_t* payload, uint16_t len);

// -------------------- Audio config --------------------
static constexpr uint32_t SR = 16000;
static constexpr size_t FRAME = 320;              // 20ms @ 16k
static constexpr uint32_t FRAME_MS = 20;

// pre-roll: 200ms
static constexpr uint32_t PREROLL_MS = 200;
static constexpr size_t PREROLL_SAMPLES = (SR * PREROLL_MS) / 1000; // 3200
static int16_t preroll_buf[PREROLL_SAMPLES];
static size_t preroll_pos = 0;
static bool preroll_full = false;

// -------------------- VAD params --------------------
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

// ============================================================
//  Servo (4x SG90)
// ============================================================
static constexpr int SERVO_NUM = 4;
static const int SERVO_PINS[SERVO_NUM] = { 25, 21, 26, 32 };  // G25, G21, G26, G32

static constexpr int SERVO_MIN_DEG = 0;
static constexpr int SERVO_MAX_DEG = 180;
static constexpr int SERVO_CENTER  = 90;

// SG90 typical pulse range
static constexpr int SERVO_MIN_US = 500;
static constexpr int SERVO_MAX_US = 2400;

Servo servos[SERVO_NUM];
bool  servo_attached[SERVO_NUM] = {false,false,false,false};

int   servo_cur[SERVO_NUM] = { SERVO_CENTER, SERVO_CENTER, SERVO_CENTER, SERVO_CENTER };
int   servo_tgt[SERVO_NUM] = { SERVO_CENTER, SERVO_CENTER, SERVO_CENTER, SERVO_CENTER };

// “부드럽게” 움직이기 (20ms마다 몇 도 이동할지)
static constexpr uint32_t SERVO_UPDATE_MS = 20;
static constexpr int SERVO_STEP_DEG = 4;
static uint32_t last_servo_update_ms = 0;

// ---- Animation state machine (non-blocking) ----
enum AnimType { ANIM_NONE, ANIM_WAVE, ANIM_WIGGLE };
struct Anim {
  AnimType type = ANIM_NONE;
  int servo = 0;            // which servo index
  int phase = 0;            // step index
  uint32_t next_ms = 0;     // next action time
  bool active = false;
} anim;

// helpers
static inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static void servo_attach_all() {
  for (int i = 0; i < SERVO_NUM; i++) {
    servos[i].setPeriodHertz(50);
    bool ok = servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    servo_attached[i] = ok;
    Serial.printf("Servo[%d] pin=%d attach=%s\n", i, SERVO_PINS[i], ok ? "OK" : "FAIL");
    servos[i].write(servo_cur[i]);
    delay(50);
  }
}

static void servo_set_target(int idx, int angle) {
  idx = clampi(idx, 0, SERVO_NUM - 1);
  angle = clampi(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  servo_tgt[idx] = angle;
}

static void servo_stop_all_anim() {
  anim.active = false;
  anim.type = ANIM_NONE;
  anim.phase = 0;
}

static void anim_start_wave(int idx) {
  idx = clampi(idx, 0, SERVO_NUM - 1);
  anim.active = true;
  anim.type = ANIM_WAVE;
  anim.servo = idx;
  anim.phase = 0;
  anim.next_ms = millis();   // start now
}

static void anim_start_wiggle(int idx) {
  idx = clampi(idx, 0, SERVO_NUM - 1);
  anim.active = true;
  anim.type = ANIM_WIGGLE;
  anim.servo = idx;
  anim.phase = 0;
  anim.next_ms = millis();
}

static void anim_tick() {
  if (!anim.active) return;
  uint32_t now = millis();
  if (now < anim.next_ms) return;

  // 각 phase에서 target 각도를 바꿔주는 방식
  if (anim.type == ANIM_WAVE) {
    // 손 흔들기: 60 <-> 120 을 여러 번, 마지막에 센터로
    // phase: 0..7 정도로 구성
    const int A = 60;
    const int B = 120;
    switch (anim.phase) {
      case 0: servo_set_target(anim.servo, A); break;
      case 1: servo_set_target(anim.servo, B); break;
      case 2: servo_set_target(anim.servo, A); break;
      case 3: servo_set_target(anim.servo, B); break;
      case 4: servo_set_target(anim.servo, A); break;
      case 5: servo_set_target(anim.servo, B); break;
      case 6: servo_set_target(anim.servo, SERVO_CENTER); break;
      default:
        anim.active = false;
        anim.type = ANIM_NONE;
        return;
    }
    anim.phase++;
    anim.next_ms = now + 180; // 손 흔드는 템포
    return;
  }

  if (anim.type == ANIM_WIGGLE) {
    // 짧게 흔들기: 80 <-> 100 반복 후 센터
    const int A = 80;
    const int B = 100;
    switch (anim.phase) {
      case 0: servo_set_target(anim.servo, A); break;
      case 1: servo_set_target(anim.servo, B); break;
      case 2: servo_set_target(anim.servo, A); break;
      case 3: servo_set_target(anim.servo, B); break;
      case 4: servo_set_target(anim.servo, SERVO_CENTER); break;
      default:
        anim.active = false;
        anim.type = ANIM_NONE;
        return;
    }
    anim.phase++;
    anim.next_ms = now + 120;
    return;
  }
}

// “서보를 조금씩” 움직이는 업데이트 (네트워크/오디오 안 멈춤)
static void servo_update_tick() {
  uint32_t now = millis();
  if (now - last_servo_update_ms < SERVO_UPDATE_MS) return;
  last_servo_update_ms = now;

  for (int i = 0; i < SERVO_NUM; i++) {
    if (!servo_attached[i]) continue;

    int cur = servo_cur[i];
    int tgt = servo_tgt[i];

    if (cur == tgt) continue;

    int diff = tgt - cur;
    int step = (diff > 0) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;
    if (abs(diff) < abs(step)) step = diff;

    cur += step;
    servo_cur[i] = cur;
    servos[i].write(cur);
  }
}

// ============================================================
//  Packet TX
// ============================================================
static void sendPacket(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (!client.connected()) return;
  client.write(&type, 1);

  uint8_t le[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
  client.write(le, 2);

  if (len && payload) client.write(payload, len);
}

static void sendPingIfIdle() {
  if (!client.connected()) return;
  uint32_t now = millis();
  if (now - last_ping_ms >= 3000) {   // 3초마다
    sendPacket(PTYPE_PING, nullptr, 0);
    last_ping_ms = now;
  }
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

// ============================================================
//  Audio helpers
// ============================================================
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
    sendPacket(PTYPE_AUDIO, (uint8_t*)preroll_buf, (uint16_t)(count * sizeof(int16_t)));
    return;
  }

  size_t tail = PREROLL_SAMPLES - preroll_pos;
  sendPacket(PTYPE_AUDIO, (uint8_t*)(preroll_buf + preroll_pos), (uint16_t)(tail * sizeof(int16_t)));
  if (preroll_pos > 0) {
    sendPacket(PTYPE_AUDIO, (uint8_t*)preroll_buf, (uint16_t)(preroll_pos * sizeof(int16_t)));
  }
}

// ============================================================
//  RX: CMD packet parser (non-blocking)
// ============================================================
static constexpr size_t RX_MAX_PAYLOAD = 512;

enum RxStage { RX_TYPE, RX_LEN0, RX_LEN1, RX_PAYLOAD };
static RxStage rx_stage = RX_TYPE;
static uint8_t  rx_type = 0;
static uint16_t rx_len = 0;
static uint16_t rx_pos = 0;
static uint8_t  rx_buf[RX_MAX_PAYLOAD];

// --- tiny JSON getters (dependency-free) ---
static bool json_get_string(const char* json, const char* key, char* out, size_t out_sz) {
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
  while (*p && *p != '"' && i + 1 < out_sz) out[i++] = *p++;
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
  bool neg = false;
  if (*p == '-') { neg = true; p++; }
  if (!isdigit((unsigned char)*p)) return false;
  long v = 0;
  while (isdigit((unsigned char)*p)) { v = v * 10 + (*p - '0'); p++; }
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
  if (!strncmp(p, "true", 4))  { *out = true;  return true; }
  if (!strncmp(p, "false", 5)) { *out = false; return true; }
  return false;
}

// ✅ 실제 로봇 액션 수행
static void applyAction(const char* action, int servoIdx, bool hasAngle, int angle) {
  if (!action || !action[0]) return;

  // STOP: 애니메이션 중단
  if (!strcmp(action, "STOP")) {
    servo_stop_all_anim();
    Serial.println("🧊 Action: STOP (animations cleared)");
    return;
  }

  // NOOP: 아무것도 안 함
  if (!strcmp(action, "NOOP")) {
    return;
  }

  // SERVO_SET: 특정 각도로
  if (!strcmp(action, "SERVO_SET")) {
    if (hasAngle) {
      servo_stop_all_anim();                 // 수동 각도 명령 들어오면 애니메이션 멈추고
      servo_set_target(servoIdx, angle);     // 목표 각도 지정
      Serial.printf("🎯 Action: SERVO_SET servo=%d angle=%d\n", servoIdx, angle);
    } else {
      Serial.println("⚠️ SERVO_SET but angle missing");
    }
    return;
  }

  // WAVE / HELLO: 손 흔들기
  if (!strcmp(action, "WAVE") || !strcmp(action, "HELLO")) {
    servo_stop_all_anim();
    anim_start_wave(servoIdx);
    Serial.printf("👋 Action: WAVE servo=%d\n", servoIdx);
    return;
  }

  // WIGGLE: 짧게 흔들기
  if (!strcmp(action, "WIGGLE")) {
    servo_stop_all_anim();
    anim_start_wiggle(servoIdx);
    Serial.printf("🪇 Action: WIGGLE servo=%d\n", servoIdx);
    return;
  }

  Serial.print("⚠️ Unknown action: ");
  Serial.println(action);
}

static void handleCmdJson(const uint8_t* payload, uint16_t len) {
  static char json[RX_MAX_PAYLOAD + 1];
  uint16_t n = (len > RX_MAX_PAYLOAD) ? RX_MAX_PAYLOAD : len;
  memcpy(json, payload, n);
  json[n] = 0;

  Serial.println("\n===== 📥 CMD from PC =====");
  Serial.print("raw json: ");
  Serial.println(json);

  char action[32] = {0};
  int sid = -1;
  int angle = -1;
  int servoIdx = 0; // default 0
  bool meaningful = false;
  bool recognized = false;

  bool has_action = json_get_string(json, "action", action, sizeof(action));
  json_get_int(json, "sid", &sid);
  bool has_angle = json_get_int(json, "angle", &angle);
  bool has_servo = json_get_int(json, "servo", &servoIdx);
  if (!has_servo) {
    // 혹시 PC가 "idx"라고 보낼 수도 있으니 한번 더
    json_get_int(json, "idx", &servoIdx);
  }
  json_get_bool(json, "meaningful", &meaningful);
  json_get_bool(json, "recognized", &recognized);

  Serial.print("action     : "); Serial.println(has_action ? action : "(missing)");
  Serial.print("sid        : "); Serial.println(sid);
  Serial.print("servo      : "); Serial.println(servoIdx);
  Serial.print("meaningful : "); Serial.println(meaningful ? "true" : "false");
  Serial.print("recognized : "); Serial.println(recognized ? "true" : "false");
  if (has_angle) {
    Serial.print("angle      : "); Serial.println(angle);
  } else {
    Serial.println("angle      : (none)");
  }
  Serial.println("===== (apply robot action) =====");

  // ✅ 여기서 실제 서보 동작 수행
  if (has_action) {
    servoIdx = clampi(servoIdx, 0, SERVO_NUM - 1);
    applyAction(action, servoIdx, has_angle, angle);
  }
}

static void pollServerPackets() {
  if (!client.connected()) return;

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
        if (rx_len == 0) {
          if (rx_type == PTYPE_CMD) handleCmdJson((const uint8_t*)"", 0);
          rx_stage = RX_TYPE;
        } else {
          rx_stage = RX_PAYLOAD;
        }
        break;

      case RX_PAYLOAD:
        if (rx_pos < RX_MAX_PAYLOAD) rx_buf[rx_pos] = byte;
        rx_pos++;

        if (rx_pos >= rx_len) {
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

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = true;
  cfg.internal_spk = false;
  M5.begin(cfg);

  M5.Mic.setSampleRate(SR);
  Serial.begin(115200);

  // ✅ Servo init
  Serial.println("\n=== Servo Init ===");
  servo_attach_all();
  Serial.println("=== Servo Ready ===\n");

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
}

void loop() {
  ensureConnections();

  // ✅ always tick: RX packets + keepalive + servo update + animations
  pollServerPackets();
  sendPingIfIdle();

  anim_tick();
  servo_update_tick();

  // ===== Mic streaming (your original logic) =====
  static int16_t samples[FRAME];
  if (!M5.Mic.record(samples, FRAME)) return;

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
        sendPacket(PTYPE_START, nullptr, 0);

        send_preroll();
        sendPacket(PTYPE_AUDIO, (uint8_t*)samples, (uint16_t)(FRAME * sizeof(int16_t)));

        talk_samples += FRAME;
      }
    } else {
      start_hit = 0;
    }
    return;
  }

  // TALKING: 오디오 전송
  sendPacket(PTYPE_AUDIO, (uint8_t*)samples, (uint16_t)(FRAME * sizeof(int16_t)));
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
    sendPacket(PTYPE_END, nullptr, 0);
  }
}
