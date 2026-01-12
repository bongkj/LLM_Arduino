#include <M5Unified.h>
#include <WiFi.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <ESP32Servo.h>

#define SERVO_PIN 25 // M5Atom Echo Grove Port (G21) - Check your wiring!
Servo myServo;

const char* SSID = "KT_GiGA_3926"; // WIFI 이름
const char* PASS = "fbx7bef119"; // WIFI 비밀번호

const char* SERVER_IP = "172.30.1.20"; // 서버 IP 주소
const uint16_t SERVER_PORT = 5001; // 서버 포트
WiFiClient client; // WiFi 클라이언트

enum State { IDLE, TALKING }; // 상태
State state = IDLE;
static constexpr uint8_t PTYPE_PING = 0x10; // 패킷 타입
static uint32_t last_ping_ms = 0; // 마지막 패킷 시간

static void sendPacket(uint8_t type, const uint8_t* payload, uint16_t len); // Forward declaration

static void sendPingIfIdle() { // 패킷 전송
  if (!client.connected()) return; // 클라이언트가 연결되어 있지 않으면 리턴
  uint32_t now = millis();
  if (now - last_ping_ms >= 3000) {   // 3초마다
    if (!sendPacket(PTYPE_PING, nullptr, 0)) {
      Serial.println("❌ Ping failed - connection lost");
      return;
    }
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
static bool sendPacket(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (!client.connected()) return false;
  
  // 실제 전송 성공 여부 확인
  size_t written = client.write(&type, 1);
  if (written != 1) {
    Serial.println("⚠️ Write failed (type)");
    client.stop();
    return false;
  }

  uint8_t le[2] = { (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF) };
  written = client.write(le, 2);
  if (written != 2) {
    Serial.println("⚠️ Write failed (len)");
    client.stop();
    return false;
  }

  if (len && payload) {
    written = client.write(payload, len);
    if (written != len) {
      Serial.println("⚠️ Write failed (payload)");
      client.stop();
      return false;
    }
  }
  
  return true;
}

static unsigned long last_connect_attempt = 0;
static constexpr unsigned long CONNECT_INTERVAL_MS = 5000;

static void manageConnections() {
  // 1. Check WiFi
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - last_connect_attempt > CONNECT_INTERVAL_MS) {
      Serial.println("📡 Connecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      last_connect_attempt = millis();
    }
    return;
  }

  // 2. Check Server
  if (!client.connected()) {
    if (millis() - last_connect_attempt > CONNECT_INTERVAL_MS) {
      Serial.printf("🔌 Connecting to Server %s:%d ...\n", SERVER_IP, SERVER_PORT);
      if (client.connect(SERVER_IP, SERVER_PORT)) {
        client.setNoDelay(true);
        Serial.println("✅ Server Connected!");
        // Send a ping immediately to register
        sendPacket(PTYPE_PING, nullptr, 0);
      } else {
        Serial.println("❌ Server Connect Failed");
      }
      last_connect_attempt = millis();
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
  if (!sendPacket(0x02, (uint8_t*)(preroll_buf + preroll_pos), (uint16_t)(tail * sizeof(int16_t)))) {
    return;
  }
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
static constexpr uint8_t  PTYPE_AUDIO_OUT = 0x12; // PC -> ESP32 Audio
static constexpr size_t   RX_MAX_PAYLOAD = 512;   // JSON은 이 정도면 충분(넘으면 잘라서 버림)
// Audio handling variables - 오디오는 최대 64KB까지 가능하므로 동적 할당 필요
static constexpr size_t AUDIO_BUFFER_MAX = 32768; // 16KB (약 1초 @ 16kHz)
static uint8_t* rx_audio_buf = nullptr;
static size_t rx_audio_buf_size = 0; 


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
  // 연결 상태 확인: available()이 0이어도 연결이 끊어졌는지 확인
  if (client.available() == 0) {
    // 연결이 끊어졌는지 확인하기 위해 peek() 시도
    // peek()는 데이터가 없으면 -1을 반환하지만, 연결이 끊어지면 0을 반환할 수 있음
    // 더 정확한 방법: 마지막으로 데이터를 받은 시간을 추적
    return;
  }
  
  while (client.available() > 0) {
    int b = client.read();
    if (b < 0) {
      // 읽기 실패 시 연결 종료
      Serial.println("⚠️ Read failed - connection lost");
      client.stop();
      break;
    }

    uint8_t byte = (uint8_t)b;

    switch (rx_stage) {
      case RX_TYPE:
        rx_type = byte;
        rx_len = 0;
        rx_pos = 0;
        // 오디오 버퍼 초기화 (새 패킷 시작)
        if (rx_type == PTYPE_AUDIO_OUT && rx_audio_buf != nullptr) {
          // 이전 버퍼는 유지 (재사용)
        }
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
        if (rx_type == PTYPE_AUDIO_OUT) {
             // 오디오 패킷 수신: 전체 패킷을 받은 후 재생
             // 버퍼 할당 (필요한 경우)
             if (rx_audio_buf == nullptr || rx_audio_buf_size < rx_len) {
                 if (rx_audio_buf != nullptr) {
                     free(rx_audio_buf);
                 }
                 rx_audio_buf_size = rx_len;
                 rx_audio_buf = (uint8_t*)malloc(rx_audio_buf_size);
                 if (rx_audio_buf == nullptr) {
                     Serial.println("❌ Audio buffer alloc failed");
                     rx_stage = RX_TYPE;
                     break;
                 }
             }
             
             // 바이트 단위로 버퍼에 저장
             if (rx_pos < rx_len) {
                 rx_audio_buf[rx_pos] = byte;
                 rx_pos++;
             }
             
             // 패킷 완료 시 재생
             if (rx_pos >= rx_len) {
                 // 샘플 수 계산 (int16 = 2바이트)
                 size_t sample_count = rx_len / sizeof(int16_t);
                 if (sample_count > 0 && rx_len % sizeof(int16_t) == 0) {
                     // 큐가 가득 찬 경우 대기 (최대 100ms)
                     uint32_t wait_start = millis();
                     while (M5.Speaker.isPlaying() && (millis() - wait_start < 100)) {
                         delay(1);
                     }
                     
                     // M5.Speaker.playRaw는 큐를 지원하므로 연속 재생 가능
                     // true = wait for previous to finish (큐잉)
                     // 볼륨을 약간 낮춰서 클리핑 방지 (0.95)
                     bool queued = M5.Speaker.playRaw((const int16_t*)rx_audio_buf, sample_count, SR, true, 0.95, false);
                     if (!queued) {
                         Serial.println("⚠️ Audio queue full, dropped packet");
                     }
                 } else {
                     Serial.printf("⚠️ Invalid audio packet size: %d (not multiple of 2)\n", rx_len);
                 }
                 rx_stage = RX_TYPE;
                 rx_pos = 0;
             }
        } else {
            // Normal JSON Command buffering
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
        }
        break;
    }
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = true;
  cfg.internal_spk = true; // Speaker Enabled
  M5.begin(cfg);

  // Set speaker volume (200/255 = 약 78% - 클리핑 방지)
  M5.Speaker.setVolume(255);

  M5.Mic.setSampleRate(SR);
s
  Serial.begin(115200);

  delay(1000); // Stabilize power
  Serial.println("\n=== SYSTEM BOOT ===");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(SSID, PASS);

  // Initial wait for WiFi (Blocking is OK here in setup)
  Serial.print("Connecting access point");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
     delay(500); 
     Serial.print("."); 
  }
  
  if (WiFi.status() == WL_CONNECTED) {
     Serial.println("\n✅ WiFi Connected!");
  } else {
     Serial.println("\n⚠️ WiFi Not Connected (will retry in loop)");
  }
  
  // Server connect will be handled in loop

  // Servo Init
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(0); // Initial position 0
}

void loop() {
  manageConnections();


  // ✅ (추가) 서버에서 오는 CMD 먼저 읽어서 출력
  pollServerPackets();
  sendPingIfIdle();

  // If playing audio, we might want to mute Mic or pause recording?
  // M5Atom Echo shares I2S. M5Unified handles half-duplex switching usually or we need to manage it.
  // For Atom Echo, Mic and Speaker share I2S0? No, checking docs...
  // Atom Echo: Mic is PDM, Speaker is I2S (NS4168). They might use same I2S port?
  // If we try to record while playing, it leads to feedback.
  // Ideally, if Speaker is busy, we shouldn't record or at least we should expect feedback.
  // But for now, let's just implement receive and play.
  if (M5.Speaker.isPlaying()) {
      // Don't record if speaker is playing to avoid echo loop
      return;
  }

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
        if (!sendPacket(0x01, nullptr, 0)) {
          state = IDLE;
          start_hit = 0;
          return;
        }

        send_preroll();
        if (!sendPacket(0x02, (uint8_t*)samples, (uint16_t)(FRAME * sizeof(int16_t)))) {
          state = IDLE;
          start_hit = 0;
          return;
        }

        talk_samples += FRAME;
      }
    } else {
      start_hit = 0;
    }
    return;
  }

  // TALKING: 오디오 전송
  if (!sendPacket(0x02, (uint8_t*)samples, (uint16_t)(FRAME * sizeof(int16_t)))) {
    state = IDLE;
    start_hit = 0;
    return;
  }
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
    sendPacket(0x03, nullptr, 0); // 실패해도 상태는 IDLE로 변경
  }
}
