// ESP32-S3 조이스틱 에뮬레이터
//
// Raspberry Pi(autonomous_wheelchair 패키지의 base_driver 노드)로부터 USB 시리얼로
// "<throttle,steering>\n" (각각 -1.0~1.0 로 정규화된 float) 을 받아서,
// 휠체어 원래 조이스틱 신호를 대신하는 아날로그 전압을 MCP4725 DAC 2개로 출력한다.
// 릴레이로 "원래 조이스틱 <-> ESP32 출력" 경로를 전환한다 (LOW = 자율주행 모드).
//
// 배선:
//   I2C: SDA=GPIO21, SCL=GPIO20
//   DAC: 0x60 = 좌우(steering), 0x61 = 전후진(throttle)  (둘 다 MCP4725A0, 주소핀만 다름)
//   릴레이: IN1=GPIO4, IN2=GPIO5, LOW 출력 시 자율주행 모드로 전환
//
// ★ 비상 정지/수동 조작 전환 버튼은 아직 미구현 (추후 작업) ★

#include <Wire.h>

static const int PIN_SDA = 21;
static const int PIN_SCL = 20;
static const int PIN_RELAY_IN1 = 4;
static const int PIN_RELAY_IN2 = 5;

static const uint8_t DAC_ADDR_STEERING = 0x60;  // 좌우
static const uint8_t DAC_ADDR_THROTTLE = 0x61;  // 전후진

// ★ 실측 캘리브레이션 값 (멀티미터로 확인한 값, 임의로 바꾸지 말 것) ★
static const int DAC_NEUTRAL = 2397;   // 중립 (2.56V)
static const int DAC_COUNT_MIN = 0;    // 0V
static const int DAC_COUNT_MAX = 4095; // 4.4V (12bit 풀스케일)

// 전진/후진, 좌/우 최대 편차를 대칭으로 제한한다 (중립 기준 위/아래 여유가 다르므로
// 더 작은 쪽에 맞춰서 한쪽 방향만 과도하게 꺾이는 것을 방지 = 안전상 대칭 클램프).
static const int DAC_MAX_DEFLECTION =
    min(DAC_NEUTRAL - DAC_COUNT_MIN, DAC_COUNT_MAX - DAC_NEUTRAL);

// ★ 실측 데드밴드 보정값 (2026-08-10, 바퀴 들어놓고 실측) ★
// 순정 컨트롤러가 중립 근처의 작은 편차는 그냥 씹어버리고 무시한다.
// 전진은 0.1(정규화값)만 줘도 즉시 반응했지만, 후진은 0.3까지도 거의 무반응이고
// 0.6은 정상 반응 / 조향(좌회전)도 0.3 무반응, 0.6 정상 반응으로 확인됨.
// 그래서 0이 아닌 명령이 들어오면 곧바로 이 데드밴드를 넘는 지점까지 점프시키고
// 거기서부터 1.0까지 선형으로 늘려서, ROS 쪽에서 저속 명령을 보내도 실제로
// 움직이게 만든다 (0은 그대로 0 = 중립 유지, 워치독 안전에는 영향 없음).
// ★ 우회전(steering<0) 데드밴드는 아직 실측 안 됨 -> 일단 좌회전과 동일하다고
//   가정. 실측 후 다르면 STEERING_DEADBAND_NEG 로 분리해서 반영할 것.
static const float THROTTLE_DEADBAND_FWD = 0.0f;   // 전진: 데드밴드 거의 없음(실측 0.1도 반응)
static const float THROTTLE_DEADBAND_REV = 0.45f;  // 후진: 0.3 무반응 / 0.6 반응 -> 중간값
static const float STEERING_DEADBAND = 0.45f;      // 좌회전 기준(0.3 무반응/0.6 반응), 우회전 미검증

static const unsigned long CMD_TIMEOUT_MS = 500;

String rxBuffer;
unsigned long lastCmdMillis = 0;

void writeDac(uint8_t addr, int count) {
  count = constrain(count, DAC_COUNT_MIN, DAC_COUNT_MAX);
  Wire.beginTransmission(addr);
  Wire.write((count >> 8) & 0x0F);  // MCP4725 fast-mode: 0 0 PD1 PD0 D11..D8
  Wire.write(count & 0xFF);         // D7..D0
  Wire.endTransmission();
}

// value(0~1, 부호 없음)를 [deadband, 1] 구간으로 다시 매핑한다.
// value==0 은 그대로 0 (중립 유지). 0이 아니면 곧바로 deadband 지점까지 점프한 뒤
// 거기서부터 1.0까지 선형으로 늘어난다.
float remapPastDeadband(float value, float deadband) {
  if (value <= 0.0f) {
    return 0.0f;
  }
  return deadband + value * (1.0f - deadband);
}

void setThrottleSteering(float throttle, float steering) {
  throttle = constrain(throttle, -1.0f, 1.0f);
  steering = constrain(steering, -1.0f, 1.0f);

  float throttle_deadband = (throttle >= 0.0f) ? THROTTLE_DEADBAND_FWD : THROTTLE_DEADBAND_REV;
  float throttle_mapped = (throttle >= 0.0f)
      ? remapPastDeadband(throttle, throttle_deadband)
      : -remapPastDeadband(-throttle, throttle_deadband);
  float steering_mapped = (steering >= 0.0f)
      ? remapPastDeadband(steering, STEERING_DEADBAND)
      : -remapPastDeadband(-steering, STEERING_DEADBAND);

  int throttle_count = DAC_NEUTRAL + (int)lroundf(throttle_mapped * DAC_MAX_DEFLECTION);
  int steering_count = DAC_NEUTRAL + (int)lroundf(steering_mapped * DAC_MAX_DEFLECTION);

  writeDac(DAC_ADDR_THROTTLE, throttle_count);
  writeDac(DAC_ADDR_STEERING, steering_count);
}

void handleLine(const String &line) {
  // 기대 포맷: "<throttle,steering>"
  int start = line.indexOf('<');
  int end = line.indexOf('>');
  if (start < 0 || end < 0 || end <= start) {
    return;
  }
  String inner = line.substring(start + 1, end);
  int comma = inner.indexOf(',');
  if (comma < 0) {
    return;
  }

  float throttle = inner.substring(0, comma).toFloat();
  float steering = inner.substring(comma + 1).toFloat();

  setThrottleSteering(throttle, steering);
  lastCmdMillis = millis();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);

  pinMode(PIN_RELAY_IN1, OUTPUT);
  pinMode(PIN_RELAY_IN2, OUTPUT);
  // 부팅하자마자 자율주행 모드로 고정 (LOW = 자율주행)
  digitalWrite(PIN_RELAY_IN1, LOW);
  digitalWrite(PIN_RELAY_IN2, LOW);

  setThrottleSteering(0.0f, 0.0f);  // 중립으로 초기화
  lastCmdMillis = millis();

  rxBuffer.reserve(64);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      if (rxBuffer.length() < 63) {
        rxBuffer += c;
      } else {
        rxBuffer = "";  // 비정상적으로 긴 줄은 버린다
      }
    }
  }

  // cmd_vel 워치독과 동일한 타임아웃: 명령이 끊기면 중립으로 복귀
  if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    setThrottleSteering(0.0f, 0.0f);
  }
}
