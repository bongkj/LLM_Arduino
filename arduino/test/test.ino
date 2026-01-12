#include <Arduino.h>
#include <ESP32Servo.h>

static const int SERVO_PIN = 25;   // G25
Servo servo;

// ===== 연속회전 서보 튜닝 값 =====
// 1바퀴 도는 데 걸리는 시간(ms) (서보/전압/부하에 따라 달라짐)
static const uint32_t MS_PER_REV = 900;   // <- 여기 튜닝!
static const int REV_COUNT = 5;

void setup() {
  Serial.begin(115200);
  delay(300);

  servo.setPeriodHertz(50);
  bool ok = servo.attach(SERVO_PIN, 500, 2400);

  Serial.println("=== Servo Test Start ===");
  Serial.print("Attach result: ");
  Serial.println(ok ? "OK" : "FAIL");

  // “처음에 0도로 세팅” 요구사항:
  // 연속회전 서보에서 write(0)는 보통 '한 방향 최대속도'라서
  // 여기서는 "기준 동작"을 위해 잠깐 0 신호를 주고, 바로 정지로 둠.
  Serial.println("Set to 0 (command), then stop");
  servo.write(0);     // 한쪽 방향 회전(혹은 최대속도)
  delay(200);
  servo.write(90);    // 정지(중립)
  delay(500);
}

void loop() {
  // 1) 0으로 한번 맞추고(신호), 정지
  Serial.println("\n--- Reset: command 0 then stop ---");
  servo.write(0);
  delay(200);
  servo.write(90);
  delay(500);

  // 2) 5바퀴 회전 (시간 기반)
  Serial.println("--- Spin 5 revolutions (time-based) ---");
  servo.write(180);   // 회전 방향/속도 (반대로 돌리려면 0으로 바꿔봐)
  uint32_t spin_ms = MS_PER_REV * REV_COUNT;
  uint32_t t0 = millis();

  while (millis() - t0 < spin_ms) {
    // 여기서도 로그를 보고 싶으면 주기적으로 출력
    // (너무 많이 찍으면 시리얼 때문에 타이밍 흔들림)
    delay(10);
  }

  // 3) 정지
  servo.write(90);
  Serial.println("--- Stop ---");
  delay(800);
}
