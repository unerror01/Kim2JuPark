// 엔코더 배선 점검용 임시 스케치 (진단 전용)
//
// DAC / 릴레이 / 조이스틱 에뮬레이션은 전혀 건드리지 않는다. 엔코더 4선
// (좌 A/B = GPIO15/16, 우 A/B = GPIO17/18) 이 제대로 물렸는지만 확인한다.
// 배선 확정되면 이 스케치는 지우고 esp32_joystick_emulator.ino 로 되돌린다.
//
// 사용법: 이 스케치 업로드 후 시리얼 모니터 115200 으로 열고 바퀴를 손으로
// 천천히 돌리면서 아래를 확인한다.
//
//  1) 정지 상태에서 L[A B], R[A B] 가 전부 1 (HIGH) 이어야 한다.
//     -> 0 으로 읽히는 핀이 있으면: 그 선이 GND 에 붙었거나, 3.3V 외부
//        풀업(4.7kΩ)이 빠졌거나, 0V(Blue) 선을 GPIO 에 잘못 꽂은 것.
//  2) 좌 바퀴를 돌리면 L ticks 만, 우 바퀴를 돌리면 R ticks 만 변해야 한다.
//     -> 안 변하면: 그 신호선이 GPIO 에 안 닿았거나 엔코더 전원(Brown=5V,
//        Blue=GND) 이 안 들어온 것. 반대쪽 카운트가 같이 변하면 두 엔코더가
//        같은 핀에 물린 것.
//  3) 롤러 1 회전 ≈ 1440 ticks (360 P/R × 4). edges 도 같이 늘어야 한다.
//  4) 바퀴를 세워둔 채로 ticks 가 혼자 흐르면: 외부 풀업 부족 / 노이즈.
//     A·B 페어 트위스트, 실드 접지, 신호–GND 100nF 캡 추가.
//  5) 휠체어를 전진 방향으로 밀 때 L ticks 와 R ticks 가 같은 부호로
//     움직여야 한다. 한쪽만 반대로 가면 그 엔코더의 A/B 두 선을 맞바꾼다.

static const int PIN_ENC_L_A = 15;
static const int PIN_ENC_L_B = 16;
static const int PIN_ENC_R_A = 17;
static const int PIN_ENC_R_B = 18;

// (이전 AB << 2) | (현재 AB) -> -1/0/+1 (x4 quadrature)
static const int8_t QUAD_DECODE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile uint8_t leftEncState = 0;
volatile uint8_t rightEncState = 0;
volatile unsigned long leftEdges = 0;
volatile unsigned long rightEdges = 0;

void IRAM_ATTR onLeftEncoderChange() {
  uint8_t s = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  leftTicks += QUAD_DECODE_TABLE[(leftEncState << 2) | s];
  leftEncState = s;
  leftEdges++;
}

void IRAM_ATTR onRightEncoderChange() {
  uint8_t s = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);
  rightTicks += QUAD_DECODE_TABLE[(rightEncState << 2) | s];
  rightEncState = s;
  rightEdges++;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  leftEncState = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  rightEncState = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), onLeftEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), onLeftEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), onRightEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), onRightEncoderChange, CHANGE);

  Serial.println("encoder wiring check ready (turn each wheel by hand)");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) {
    return;
  }
  lastPrint = millis();

  int la = digitalRead(PIN_ENC_L_A);
  int lb = digitalRead(PIN_ENC_L_B);
  int ra = digitalRead(PIN_ENC_R_A);
  int rb = digitalRead(PIN_ENC_R_B);

  noInterrupts();
  long lt = leftTicks;
  long rt = rightTicks;
  unsigned long le = leftEdges;
  unsigned long re = rightEdges;
  interrupts();

  Serial.printf("L[A%d B%d] ticks=%ld edges=%lu | R[A%d B%d] ticks=%ld edges=%lu\n",
                la, lb, lt, le, ra, rb, rt, re);
}
