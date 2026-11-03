#include <Wire.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "QGPMaker_MotorShield.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();

QGPMaker_DCMotor *motorFL = AFMS.getMotor(1);
QGPMaker_DCMotor *motorRL = AFMS.getMotor(2);
QGPMaker_DCMotor *motorRR = AFMS.getMotor(3);
QGPMaker_DCMotor *motorFR = AFMS.getMotor(4);

// 배열 순서: FL, FR, RL, RR
volatile long encoderCount[4] = {0, 0, 0, 0};
volatile byte lastState[4];

const int8_t quadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// 기존 실측 기준: 로봇 전진 시 양수 속도가 되도록 보정
const int8_t encoderSign[4] = {-1, -1, -1, 1};

const float COUNTS_PER_REV = 4300.0;
const float TWO_PI_F = 6.28318530718;

// 초기 안전 설정
const float MAX_TARGET_RAD_S = 12.0;
const int MAX_PWM_NORMAL = 200;   // 1임무. 228 로 올리니 명령을 넘어 0.36 m/s 까지 나와 (6,2) 앞벽에 부딪힘, 얻는 시간은 1~2 초 (11-03)
// 출력 올리기 (2임무): P,값 (200~255) 을 받는 동안만 그 상한. 1 초 안 오면 저절로 200 (노드가 죽어도 안전). 안 보내면 예전 판과 똑같이 동작.
const int MAX_PWM_FAST = 255;
const unsigned long FAST_TIMEOUT_MS = 1000;
int maxPwm = MAX_PWM_NORMAL;
unsigned long lastFastMs = 0;

// 초기 속도 제어 계수: 추후 실측으로 조정
const float KFF = 25.0;
const float KP = 6.0;
// 적분: P 만으로는 정지 마찰을 못 넘는 작은 목표(몸 회전 0.3 rad/s ≈ 바퀴 1.5 rad/s → PWM 46)가 영영 안 돌고
// (10-31 실측: 회전 0.3 → 실제 0.00, 0.6 → 0.2, 0.8 → 0.45), 큰 목표도 끝까지 못 따라감 → 오차를 쌓아 더 밂
const float KI = 60.0;        // PWM / (rad/s · s)
const float I_LIMIT = 60.0;   // 적분 몫 최대 PWM

const unsigned long CONTROL_PERIOD_US = 20000;  // 50Hz
const unsigned long FEEDBACK_PERIOD_MS = 50;    // 20Hz
const unsigned long COMMAND_TIMEOUT_MS = 300;

float targetVelocity[4] = {0, 0, 0, 0};
float measuredVelocity[4] = {0, 0, 0, 0};
float integralPwm[4] = {0, 0, 0, 0};
float previousTarget[4] = {0, 0, 0, 0};

long previousCount[4] = {0, 0, 0, 0};

unsigned long lastControlUs = 0;
unsigned long lastFeedbackMs = 0;
unsigned long lastCommandMs = 0;

char receiveBuffer[96];
byte receiveIndex = 0;

inline byte readFL() {
  byte b = PINB;
  return ((b & _BV(PB0)) ? 2 : 0) |
         ((b & _BV(PB1)) ? 1 : 0);  // D8, D9
}

inline byte readFR() {
  byte d = PIND;
  return ((d & _BV(PD4)) ? 2 : 0) |
         ((d & _BV(PD5)) ? 1 : 0);  // D4, D5
}

inline byte readRL() {
  byte d = PIND;
  return ((d & _BV(PD6)) ? 2 : 0) |
         ((d & _BV(PD7)) ? 1 : 0);  // D6, D7
}

inline byte readRR() {
  byte d = PIND;
  return ((d & _BV(PD3)) ? 2 : 0) |
         ((d & _BV(PD2)) ? 1 : 0);  // D3, D2
}

void updateEncoder(byte index, byte state) {
  byte tableIndex = (lastState[index] << 2) | state;
  encoderCount[index] += quadTable[tableIndex];
  lastState[index] = state;
}

ISR(PCINT0_vect) {
  updateEncoder(0, readFL());
}

ISR(PCINT2_vect) {
  updateEncoder(1, readFR());
  updateEncoder(2, readRL());
  updateEncoder(3, readRR());
}

void setupEncoders() {
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);

  noInterrupts();

  lastState[0] = readFL();
  lastState[1] = readFR();
  lastState[2] = readRL();
  lastState[3] = readRR();

  PCICR |= _BV(PCIE0) | _BV(PCIE2);

  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1);
  PCMSK2 |= _BV(PCINT18) | _BV(PCINT19) |
            _BV(PCINT20) | _BV(PCINT21) |
            _BV(PCINT22) | _BV(PCINT23);

  interrupts();
}

void copyEncoderCounts(long output[4]) {
  noInterrupts();

  for (byte i = 0; i < 4; i++) {
    output[i] = encoderCount[i];
  }

  interrupts();
}

void stopAll() {
  motorFL->run(RELEASE);
  motorFR->run(RELEASE);
  motorRL->run(RELEASE);
  motorRR->run(RELEASE);
}

void setMotorSigned(QGPMaker_DCMotor *motor, float pwm) {
  int magnitude = constrain((int)abs(pwm), 0, maxPwm);

  if (magnitude == 0) {
    motor->run(RELEASE);
    return;
  }

  // 기존 차량 기준 BACKWARD가 물리적 전진
  if (pwm > 0) {
    motor->run(BACKWARD);
  } else {
    motor->run(FORWARD);
  }

  motor->setSpeed(magnitude);
}

void applyWheelOutputs(float pwm[4]) {
  // 제어 배열 순서: FL, FR, RL, RR
  setMotorSigned(motorFL, pwm[0]);
  setMotorSigned(motorFR, pwm[1]);
  setMotorSigned(motorRL, pwm[2]);
  setMotorSigned(motorRR, pwm[3]);
}

// 그리퍼 서보: D10 = OC1B, Timer1 하드웨어 PWM 50 Hz (인터럽트 없음 → 엔코더 PCINT 와 무관).
// 켜질 때는 펄스 없음(서보 힘 빠진 채, 안 움직임). 명령 G,<마이크로초> 로 펄스 길이 지정, G,0 = 펄스 끔.
const int GRIP_US_MIN = 500;
const int GRIP_US_MAX = 2500;
int gripUs = 0;

void setupGripper() {
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  ICR1 = 39999;                                  // 16 MHz / 8 / 40000 = 50 Hz
  OCR1B = 0;
  TCCR1A = _BV(WGM11);                           // Fast PWM, TOP = ICR1 (mode 14), 출력은 아직 끊어 둠
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // 분주 8 → 0.5 us 단위
}

void setGripperUs(int us) {
  if (us <= 0) {
    TCCR1A &= ~_BV(COM1B1);                      // 펄스 끔 (D10 LOW)
    digitalWrite(10, LOW);
    gripUs = 0;
    return;
  }
  us = constrain(us, GRIP_US_MIN, GRIP_US_MAX);
  OCR1B = (unsigned int)us * 2;
  TCCR1A |= _BV(COM1B1);                         // D10 에 펄스 연결
  gripUs = us;
}

void setTargetsToZero() {
  for (byte i = 0; i < 4; i++) {
    targetVelocity[i] = 0.0;
  }
}

void processCommand(char *line) {
  if (strcmp(line, "S") == 0) {
    setTargetsToZero();
    stopAll();
    lastCommandMs = millis();
    Serial.println("OK,S");
    return;
  }

  if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  if (strncmp(line, "G,", 2) == 0) {
    setGripperUs(atoi(line + 2));
    Serial.print("OK,G,");
    Serial.println(gripUs);
    return;
  }

  if (strncmp(line, "P,", 2) == 0) {   // 응답 없음: 주행 명령 응답 흐름을 건드리지 않음
    maxPwm = constrain(atoi(line + 2), MAX_PWM_NORMAL, MAX_PWM_FAST);
    lastFastMs = millis();
    return;
  }

  if (strncmp(line, "V,", 2) != 0) {
    Serial.println("ERR,UNKNOWN");
    return;
  }

  char *token = strtok(line, ",");

  for (byte i = 0; i < 4; i++) {
    token = strtok(NULL, ",");

    if (token == NULL) {
      Serial.println("ERR,FORMAT");
      return;
    }

    targetVelocity[i] = constrain(
      atof(token),
      -MAX_TARGET_RAD_S,
      MAX_TARGET_RAD_S
    );
  }

  lastCommandMs = millis();
  Serial.println("OK,V");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char value = Serial.read();

    if (value == '\n' || value == '\r') {
      if (receiveIndex > 0) {
        receiveBuffer[receiveIndex] = '\0';
        processCommand(receiveBuffer);
        receiveIndex = 0;
      }
    } else if (receiveIndex < sizeof(receiveBuffer) - 1) {
      receiveBuffer[receiveIndex++] = value;
    } else {
      receiveIndex = 0;
      Serial.println("ERR,OVERFLOW");
    }
  }
}

void updateVelocityControl() {
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastControlUs;

  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  lastControlUs = nowUs;

  float dt = elapsedUs / 1000000.0;
  long currentCount[4];

  copyEncoderCounts(currentCount);

  for (byte i = 0; i < 4; i++) {
    long delta = currentCount[i] - previousCount[i];
    previousCount[i] = currentCount[i];

    measuredVelocity[i] =
      encoderSign[i] *
      delta *
      TWO_PI_F /
      COUNTS_PER_REV /
      dt;
  }

  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    setTargetsToZero();
  }

  if (maxPwm != MAX_PWM_NORMAL && millis() - lastFastMs > FAST_TIMEOUT_MS) {
    maxPwm = MAX_PWM_NORMAL;
  }

  float pwm[4];

  for (byte i = 0; i < 4; i++) {
    // 바퀴 도는 방향이 바뀌면 그동안 쌓은 적분은 맞지 않으므로 비움
    if (targetVelocity[i] * previousTarget[i] < 0.0) {
      integralPwm[i] = 0.0;
    }
    previousTarget[i] = targetVelocity[i];

    if (abs(targetVelocity[i]) < 0.01) {
      pwm[i] = 0.0;
      integralPwm[i] = 0.0;   // 서 있으면 비움 → 다시 출발할 때 지난 적분으로 튀지 않음
    } else {
      float error = targetVelocity[i] - measuredVelocity[i];
      float unclamped = KFF * targetVelocity[i] + KP * error + integralPwm[i];

      // 상한에 걸린 채 같은 쪽으로 더 쌓지 않음 (풀릴 때 튀지 않게)
      bool pushingHigh = unclamped >= maxPwm && error > 0.0;
      bool pushingLow = unclamped <= -maxPwm && error < 0.0;
      if (!pushingHigh && !pushingLow) {
        integralPwm[i] = constrain(integralPwm[i] + KI * error * dt, -I_LIMIT, I_LIMIT);
      }

      pwm[i] =
        KFF * targetVelocity[i] +
        KP * error +
        integralPwm[i];

      pwm[i] = constrain(pwm[i], -maxPwm, maxPwm);
    }
  }

  applyWheelOutputs(pwm);
}

void sendEncoderFeedback() {
  unsigned long nowMs = millis();

  if (nowMs - lastFeedbackMs < FEEDBACK_PERIOD_MS) {
    return;
  }

  lastFeedbackMs = nowMs;

  Serial.print("E,");
  Serial.print(measuredVelocity[0], 4);
  Serial.print(",");
  Serial.print(measuredVelocity[1], 4);
  Serial.print(",");
  Serial.print(measuredVelocity[2], 4);
  Serial.print(",");
  Serial.println(measuredVelocity[3], 4);
}

void setup() {
  Serial.begin(115200);
  AFMS.begin();

  setupEncoders();
  setupGripper();
  stopAll();

  copyEncoderCounts(previousCount);

  lastControlUs = micros();
  lastFeedbackMs = millis();
  lastCommandMs = millis();

  Serial.println("READY,MECANUM_ROS2_CONTROL_PI");
  Serial.println("FORMAT,V,FL,FR,RL,RR");
}

void loop() {
  readSerialCommands();
  updateVelocityControl();
  sendEncoderFeedback();
}
