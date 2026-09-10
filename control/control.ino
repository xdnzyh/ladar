#include "calibration.h"
#include <Wire.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "utils.h"
#include "runtime_config.h"
#include "runtime_accessors.h"
#include "motion_result.h"

// ============================================================
// MECANUM UNIVERSAL V6.3 COMM
// 2026-09-09: 停车时序、非阻塞简报、超长行恢复；详见本次修改记录.md。
//
// 人工单键控制：
//
//        Q   W   E       R = 顺时针
//        A   S   D       F = 逆时针
//        Z   X   C
//
// W : 前
// S : 后
// A : 左
// D : 右
// Q : 左前
// E : 右前
// Z : 左后
// C : 右后
// R : 顺时针
// F : 逆时针
//
// X : 紧急停止
// P : 状态
// G : 重置
//
// ------------------------------------------------------------
// 外部通信协议：
//
// @MOVE,<MODE>,<VALUE>,CNT
//
// 示例：
//
// @MOVE,D,1400,CNT
//
// 回复：
//
// @ACK,D,1400,CNT
//
// 执行完成：
//
// @DONE,D,TARGET,REQ=1400,UNIT=CNT,
// BRAKE=...,ENC=...,DX=...,DY=...,DR=...,DS=...,
// Q1=...,Q2=...,Q3=...,Q4=...
//
// 错误：
//
// @ERR,BUSY
// @ERR,BAD_CMD
// @ERR,BAD_MODE
// @ERR,BAD_UNIT
// @ERR,BAD_VALUE
// @ERR,LINE_TOO_LONG
//
// ------------------------------------------------------------
// 支持 CNT 与八方向 MM；CONFIG=1 提供空闲时原子参数更新。
// 安装后建议电脑统一毫米换算并发送CNT；DEG尚未标定。
// ============================================================


// ============================================================
// 尽早关闭 watchdog
// ============================================================

void disableWatchdogEarly(void)
  __attribute__((naked, used, section(".init3")));

void disableWatchdogEarly(void) {
  MCUSR = 0;
  wdt_disable();
}


// ============================================================
// SRAM
// ============================================================

extern char __heap_start;
extern char *__brkval;

int freeRam() {
  char top;

  if (__brkval == 0) {
    return &top - &__heap_start;
  }

  return &top - __brkval;
}


// ============================================================
// 电机
//
// M1：右前
// M2：左前
// M3：左后
// M4：右后
// ============================================================

const uint8_t MOTOR_A[4] = {
  8, 10, 14, 12
};

const uint8_t MOTOR_B[4] = {
  9, 11, 15, 13
};

const int8_t MOTOR_POLARITY[4] = {
   1,
  -1,
  -1,
   1
};


// ============================================================
// 编码器 GPIO
//
// E1=M1 → D8/D9
// E2=M2 → D6/D7
// E3=M3 → D2/D3
// E4=M4 → D4/D5
// ============================================================

const uint8_t ENC1_A = 8;
const uint8_t ENC1_B = 9;

const uint8_t ENC2_A = 6;
const uint8_t ENC2_B = 7;

const uint8_t ENC3_A = 2;
const uint8_t ENC3_B = 3;

const uint8_t ENC4_A = 4;
const uint8_t ENC4_B = 5;


// ============================================================
// 编码器
// ============================================================

volatile long rawCount[4] = {
  0, 0, 0, 0
};

volatile uint8_t encState[4] = {
  0, 0, 0, 0
};

const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};


// ============================================================
// 控制周期
// ============================================================

const unsigned long CONTROL_PERIOD_MS = 20;
const float CONTROL_DT = 0.020f;

const float VELOCITY_ALPHA = 0.35f;


// ============================================================
// 人工单键测试时默认目标
// ============================================================

const long CARDINAL_TEST_COUNTS = 1400;
const long DIAGONAL_TEST_COUNTS = 1200;
const long ROTATE_TEST_COUNTS = 500;


// ============================================================
// 运行保护
// ============================================================

const float WRONG_DIRECTION_LIMIT = -250.0f;


// ============================================================
// 运动类别
//
// 不使用 enum，避免 Arduino .ino 自动原型问题。
// ============================================================

const uint8_t MOTION_NONE = 0;
const uint8_t MOTION_LONGITUDINAL = 1;
const uint8_t MOTION_LATERAL = 2;
const uint8_t MOTION_DIAGONAL = 3;
const uint8_t MOTION_ROTATION = 4;

uint8_t currentMotionClass = MOTION_NONE;


// ============================================================
// 前后参数
//
// 当前已验证，保持不变。
// ============================================================







// ============================================================
// 侧移参数
//
// 已验证的高驱动力侧移参数。
// ============================================================







// ============================================================
// 斜移参数
//
// 当前已经通过 Q 实车验证有效。
// ============================================================







// ============================================================
// 旋转参数
//
// 尚未专项调整，保持旧值。
// ============================================================







// ============================================================
// 当前 PWM 范围
// ============================================================

int currentMinPWM = LONG_MIN_PWM;
int currentMaxPWM = LONG_MAX_PWM;


// ============================================================
// Wheel differential PI
// ============================================================




// ============================================================
// 平移航向稳定
// ============================================================






// ============================================================
// 前后弱 Y 抑制
// ============================================================




// ============================================================
// S damping
// ============================================================




// ============================================================
// 激活轮目标速度范围
// ============================================================

const float MIN_ACTIVE_TARGET = 12.0f;
const float MAX_ACTIVE_TARGET = 50.0f;


// ============================================================
// 编码器逻辑状态
// ============================================================

long q[4] = {
  0, 0, 0, 0
};

long lastQ[4] = {
  0, 0, 0, 0
};

long segmentStartQ[4] = {
  0, 0, 0, 0
};


// ============================================================
// 四轮速度
// ============================================================

float v[4] = {
  0, 0, 0, 0
};


// ============================================================
// 运动学
// ============================================================

float Xglobal = 0;
float Yglobal = 0;
float Rglobal = 0;
float Sglobal = 0;

float Vx = 0;
float Vy = 0;
float Vr = 0;
float Vs = 0;


// ============================================================
// 当前动作起点
// ============================================================

float segmentStartX = 0;
float segmentStartY = 0;
float segmentStartR = 0;
float segmentStartS = 0;


// ============================================================
// 当前动作变化
// ============================================================

float Xsegment = 0;
float Ysegment = 0;
float Rsegment = 0;
float Ssegment = 0;


// ============================================================
// 当前主运动方向
//
// +1 = 逻辑正转
// -1 = 逻辑反转
//  0 = 不主动驱动
// ============================================================

int8_t baseDir[4] = {
  0, 0, 0, 0
};

char currentMode = '-';

bool currentIsRotation = false;
bool currentUseHeadingHold = false;
bool currentUseYHold = false;

// 请求目标继续用于 REQ、速度规划和进度报告。
long currentTargetCounts = 0;
long brakeTriggerCounts = 0;
long currentBrakeLead = 0;

// ============================================================
// 当前运动进度
// ============================================================

float progress = 0;
float progressSpeed = 0;


// ============================================================
// Wheel PI
// ============================================================

float wheelI[4] = {
  0, 0, 0, 0
};

float wheelIOut[4] = {
  0, 0, 0, 0
};


// ============================================================
// 四轮目标 / PWM
// ============================================================

float targetV[4] = {
  0, 0, 0, 0
};

float rawPWM[4] = {
  0, 0, 0, 0
};

int pwm[4] = {
  0, 0, 0, 0
};


// ============================================================
// 航向反馈
// ============================================================

float rMem = 0;

float currentMemCorrection = 0;

float currentCr = 0;
float currentCy = 0;
float currentCs = 0;


// ============================================================
// Desaturation
// ============================================================

float currentCommonShift = 0;

unsigned long shiftCycles = 0;
unsigned long compressionCycles = 0;

float maxAbsShift = 0;


// ============================================================
// PWM 统计
// ============================================================

unsigned long controlCycles = 0;

unsigned long lowHit[4] = {
  0, 0, 0, 0
};

unsigned long highHit[4] = {
  0, 0, 0, 0
};

unsigned long stallHigh[4] = {
  0, 0, 0, 0
};


// ============================================================
// 运动统计
// ============================================================

float maxAbsRSegment = 0;
float maxAbsYSegment = 0;
float maxAbsSSegment = 0;

float maxAbsVr = 0;
float maxAbsCr = 0;
float maxAbsRmem = 0;


// ============================================================
// 25/50/75% R
// ============================================================

bool r25Recorded = false;
bool r50Recorded = false;
bool r75Recorded = false;

float r25 = 0;
float r50 = 0;
float r75 = 0;


// ============================================================
// 停车数据
// ============================================================

float brakeProgress = 0;
float finalProgress = 0;

float brakeR = 0;
float brakeVr = 0;
float brakeCr = 0;
float brakeRmem = 0;


// ============================================================
// 运行状态
// ============================================================

bool running = false;

unsigned long runStartTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;


// ============================================================
// 当前动作是否来自外部协议
// ============================================================

bool protocolMotionActive = false;
long protocolRequestedValue = 0;
bool protocolUnitMM = false;
bool verboseProtocolLog = false; // @LOG,1恢复详细报告；默认减少无线回传量。


// ============================================================
// 外部协议接收缓冲区
// ============================================================

const uint8_t RX_BUFFER_SIZE = 48;

char rxBuffer[RX_BUFFER_SIZE];

uint8_t rxLength = 0;

// 所有输入先收完整行；无换行单键在静默 100 ms 后确认。
const unsigned long MANUAL_GAP_MS = 100;
unsigned long rxLastByteTime = 0;
bool rxInvalid = false;

// 超长行保持丢弃状态，直到 CR/LF，避免尾部触发单键移动。
bool protocolDiscarding = false;


// ============================================================
// 停止原因
// ============================================================

const uint8_t STOP_TARGET = 0;
const uint8_t STOP_EMERGENCY = 1;
const uint8_t STOP_TIMEOUT = 2;
const uint8_t STOP_WRONG_DIRECTION = 3;


// ============================================================
// 工具函数
// ============================================================

float absFloat(float x) {
  return (x < 0) ? -x : x;
}


float clampFloat(
  float x,
  float minimum,
  float maximum
) {
  if (x < minimum) {
    return minimum;
  }

  if (x > maximum) {
    return maximum;
  }

  return x;
}


int clampInt(
  int x,
  int minimum,
  int maximum
) {
  if (x < minimum) {
    return minimum;
  }

  if (x > maximum) {
    return maximum;
  }

  return x;
}


// 已测区间内按方向插值；区间外不外推。参数单位均为 CNT。
// 实测长档系数：CNT/mm × 10000。整数定点换算避免AVR浮点精度与溢出。
long countsPerMm10000(char mode) {
  switch (mode) {
    case 'W': return CAL_FIXED(CAL_W_COUNTS_PER_MM);
    case 'S': return CAL_FIXED(CAL_S_COUNTS_PER_MM);
    case 'A': return CAL_FIXED(CAL_A_COUNTS_PER_MM);
    case 'D': return CAL_FIXED(CAL_D_COUNTS_PER_MM);
    case 'Q': return CAL_FIXED(CAL_Q_COUNTS_PER_MM);
    case 'E': return CAL_FIXED(CAL_E_COUNTS_PER_MM);
    case 'Z': return CAL_FIXED(CAL_Z_COUNTS_PER_MM);
    case 'C': return CAL_FIXED(CAL_C_COUNTS_PER_MM);
    default: return 0;
  }
}

long millimetersToCounts(char mode, long mm) {
  long scale = countsPerMm10000(mode);
  if (scale == 0 || mm <= 0) return 0;
  int64_t counts = ((int64_t)mm * scale + 5000LL) / 10000LL;
  if (counts > 2147483647LL) return 0;
  return (long)counts;
}

// UNIT仅描述REQ；BRAKE/ENC和运动学量仍为CNT。MM附带内部目标便于联调。
void printProtocolRequest() {
  Serial.print(F(",REQ="));
  Serial.print(protocolRequestedValue);
  if (protocolUnitMM) {
    Serial.print(F(",UNIT=MM,TARGET_CNT="));
    Serial.print(currentTargetCounts);
  } else {
    Serial.print(F(",UNIT=CNT"));
  }
}

long calibratedBrakeLead(char mode, long target) {
  return CarConfig::brakeLead(mode, target);
}


char upperModeChar(char c) {
  if (
    c >= 'a'
    &&
    c <= 'z'
  ) {
    return c - 'a' + 'A';
  }

  return c;
}


bool isMovementCommand(char cmd) {
  return
       cmd == 'w'
    || cmd == 's'
    || cmd == 'a'
    || cmd == 'd'
    || cmd == 'q'
    || cmd == 'e'
    || cmd == 'z'
    || cmd == 'c'
    || cmd == 'r'
    || cmd == 'f';
}


// ============================================================
// Encoder GPIO
// ============================================================

inline uint8_t readE1() {
  uint8_t p = PINB;

  return
      ((p & _BV(PB0)) ? 2 : 0)
    | ((p & _BV(PB1)) ? 1 : 0);
}


inline uint8_t readE2() {
  uint8_t p = PIND;

  return
      ((p & _BV(PD6)) ? 2 : 0)
    | ((p & _BV(PD7)) ? 1 : 0);
}


inline uint8_t readE3() {
  uint8_t p = PIND;

  return
      ((p & _BV(PD2)) ? 2 : 0)
    | ((p & _BV(PD3)) ? 1 : 0);
}


inline uint8_t readE4() {
  uint8_t p = PIND;

  return
      ((p & _BV(PD4)) ? 2 : 0)
    | ((p & _BV(PD5)) ? 1 : 0);
}


// ============================================================
// Encoder ISR
// ============================================================

ISR(PCINT0_vect) {
  uint8_t newState = readE1();

  rawCount[0] +=
    QUAD_TABLE[
      (encState[0] << 2)
      | newState
    ];

  encState[0] = newState;
}


ISR(PCINT2_vect) {
  uint8_t new2 = readE2();
  uint8_t new3 = readE3();
  uint8_t new4 = readE4();

  rawCount[1] +=
    QUAD_TABLE[
      (encState[1] << 2)
      | new2
    ];

  rawCount[2] +=
    QUAD_TABLE[
      (encState[2] << 2)
      | new3
    ];

  rawCount[3] +=
    QUAD_TABLE[
      (encState[3] << 2)
      | new4
    ];

  encState[1] = new2;
  encState[2] = new3;
  encState[3] = new4;
}


// ============================================================
// Encoder setup
// ============================================================

void setupEncoders() {
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);

  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  encState[0] = readE1();
  encState[1] = readE2();
  encState[2] = readE3();
  encState[3] = readE4();

  PCICR |= _BV(PCIE0);
  PCICR |= _BV(PCIE2);

  PCMSK0 |= _BV(PCINT0);
  PCMSK0 |= _BV(PCINT1);

  PCMSK2 |= _BV(PCINT18);
  PCMSK2 |= _BV(PCINT19);
  PCMSK2 |= _BV(PCINT20);
  PCMSK2 |= _BV(PCINT21);
  PCMSK2 |= _BV(PCINT22);
  PCMSK2 |= _BV(PCINT23);
}


// ============================================================
// Atomic encoder read
// ============================================================

void getRawCounts(long out[4]) {
  noInterrupts();

  out[0] = rawCount[0];
  out[1] = rawCount[1];
  out[2] = rawCount[2];
  out[3] = rawCount[3];

  interrupts();
}


// ============================================================
// 更新逻辑编码器位置
// ============================================================

void updateLogicalPosition() {
  long raw[4];

  getRawCounts(raw);

  q[0] = -raw[0];
  q[1] =  raw[1];
  q[2] = -raw[2];
  q[3] =  raw[3];

  Xglobal =
    (q[0] + q[1] + q[2] + q[3])
    / 4.0f;

  Yglobal =
    (q[0] - q[1] + q[2] - q[3])
    / 4.0f;

  Rglobal =
    (-q[0] + q[1] + q[2] - q[3])
    / 4.0f;

  Sglobal =
    (q[0] + q[1] - q[2] - q[3])
    / 4.0f;
}


// ============================================================
// 更新当前动作状态
// ============================================================

void updateSegmentState() {
  Xsegment =
    Xglobal - segmentStartX;

  Ysegment =
    Yglobal - segmentStartY;

  Rsegment =
    Rglobal - segmentStartR;

  Ssegment =
    Sglobal - segmentStartS;


  float sumProgress = 0;
  int activeCount = 0;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] != 0) {
      long dq =
        q[i] - segmentStartQ[i];

      sumProgress +=
        baseDir[i] * dq;

      activeCount++;
    }
  }


  if (activeCount > 0) {
    progress =
      sumProgress
      / activeCount;
  }
  else {
    progress = 0;
  }
}


// ============================================================
// Kinematics
// ============================================================

void updateKinematics() {
  updateLogicalPosition();
  updateSegmentState();


  long delta[4];


  for (int i = 0; i < 4; i++) {
    delta[i] =
      q[i] - lastQ[i];

    lastQ[i] =
      q[i];

    v[i] =
      VELOCITY_ALPHA * delta[i]
      +
      (1.0f - VELOCITY_ALPHA)
      * v[i];
  }


  Vx =
    (v[0] + v[1] + v[2] + v[3])
    / 4.0f;

  Vy =
    (v[0] - v[1] + v[2] - v[3])
    / 4.0f;

  Vr =
    (-v[0] + v[1] + v[2] - v[3])
    / 4.0f;

  Vs =
    (v[0] + v[1] - v[2] - v[3])
    / 4.0f;


  float speedSum = 0;
  int activeCount = 0;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] != 0) {
      speedSum +=
        baseDir[i] * v[i];

      activeCount++;
    }
  }


  if (activeCount > 0) {
    progressSpeed =
      speedSum
      / activeCount;
  }
  else {
    progressSpeed = 0;
  }


  if (
    absFloat(Rsegment)
    > maxAbsRSegment
  ) {
    maxAbsRSegment =
      absFloat(Rsegment);
  }


  if (
    absFloat(Ysegment)
    > maxAbsYSegment
  ) {
    maxAbsYSegment =
      absFloat(Ysegment);
  }


  if (
    absFloat(Ssegment)
    > maxAbsSSegment
  ) {
    maxAbsSSegment =
      absFloat(Ssegment);
  }


  if (
    absFloat(Vr)
    > maxAbsVr
  ) {
    maxAbsVr =
      absFloat(Vr);
  }


  if (
    !r25Recorded
    &&
    progress
      >= currentTargetCounts * 0.25f
  ) {
    r25 = Rsegment;
    r25Recorded = true;
  }


  if (
    !r50Recorded
    &&
    progress
      >= currentTargetCounts * 0.50f
  ) {
    r50 = Rsegment;
    r50Recorded = true;
  }


  if (
    !r75Recorded
    &&
    progress
      >= currentTargetCounts * 0.75f
  ) {
    r75 = Rsegment;
    r75Recorded = true;
  }
}


// ============================================================
// Base direction
// ============================================================

void setBaseDirection(
  int8_t d1,
  int8_t d2,
  int8_t d3,
  int8_t d4
) {
  baseDir[0] = d1;
  baseDir[1] = d2;
  baseDir[2] = d3;
  baseDir[3] = d4;
}


// ============================================================
// Configure mode
// ============================================================

bool configureMode(char cmd) {
  currentIsRotation = false;
  currentUseHeadingHold = true;
  currentUseYHold = false;
  currentMotionClass = MOTION_NONE;


  switch (cmd) {

    case 'w':
      setBaseDirection(
        1, 1, 1, 1
      );

      currentMode = 'W';

      currentMotionClass =
        MOTION_LONGITUDINAL;

      currentTargetCounts =
        CARDINAL_TEST_COUNTS;

      currentUseYHold = true;

      currentMinPWM = LONG_MIN_PWM;
      currentMaxPWM = LONG_MAX_PWM;

      return true;


    case 's':
      setBaseDirection(
        -1, -1, -1, -1
      );

      currentMode = 'S';

      currentMotionClass =
        MOTION_LONGITUDINAL;

      currentTargetCounts =
        CARDINAL_TEST_COUNTS;

      currentUseYHold = true;

      currentMinPWM = LONG_MIN_PWM;
      currentMaxPWM = LONG_MAX_PWM;

      return true;


    case 'a':
      setBaseDirection(
         1, -1, 1, -1
      );

      currentMode = 'A';

      currentMotionClass =
        MOTION_LATERAL;

      currentTargetCounts =
        CARDINAL_TEST_COUNTS;

      currentMinPWM = LAT_MIN_PWM;
      currentMaxPWM = LAT_MAX_PWM;

      return true;


    case 'd':
      setBaseDirection(
        -1, 1, -1, 1
      );

      currentMode = 'D';

      currentMotionClass =
        MOTION_LATERAL;

      currentTargetCounts =
        CARDINAL_TEST_COUNTS;

      currentMinPWM = LAT_MIN_PWM;
      currentMaxPWM = LAT_MAX_PWM;

      return true;


    case 'q':
      setBaseDirection(
        1, 0, 1, 0
      );

      currentMode = 'Q';

      currentMotionClass =
        MOTION_DIAGONAL;

      currentTargetCounts =
        DIAGONAL_TEST_COUNTS;

      currentMinPWM = DIAG_MIN_PWM;
      currentMaxPWM = DIAG_MAX_PWM;

      return true;


    case 'e':
      setBaseDirection(
        0, 1, 0, 1
      );

      currentMode = 'E';

      currentMotionClass =
        MOTION_DIAGONAL;

      currentTargetCounts =
        DIAGONAL_TEST_COUNTS;

      currentMinPWM = DIAG_MIN_PWM;
      currentMaxPWM = DIAG_MAX_PWM;

      return true;


    case 'z':
      setBaseDirection(
        0, -1, 0, -1
      );

      currentMode = 'Z';

      currentMotionClass =
        MOTION_DIAGONAL;

      currentTargetCounts =
        DIAGONAL_TEST_COUNTS;

      currentMinPWM = DIAG_MIN_PWM;
      currentMaxPWM = DIAG_MAX_PWM;

      return true;


    case 'c':
      setBaseDirection(
        -1, 0, -1, 0
      );

      currentMode = 'C';

      currentMotionClass =
        MOTION_DIAGONAL;

      currentTargetCounts =
        DIAGONAL_TEST_COUNTS;

      currentMinPWM = DIAG_MIN_PWM;
      currentMaxPWM = DIAG_MAX_PWM;

      return true;


    case 'r':
      setBaseDirection(
        -1, 1, 1, -1
      );

      currentMode = 'R';

      currentMotionClass =
        MOTION_ROTATION;

      currentTargetCounts =
        ROTATE_TEST_COUNTS;

      currentIsRotation = true;
      currentUseHeadingHold = false;
      currentUseYHold = false;

      currentMinPWM = ROT_MIN_PWM;
      currentMaxPWM = ROT_MAX_PWM;

      return true;


    case 'f':
      setBaseDirection(
        1, -1, -1, 1
      );

      currentMode = 'F';

      currentMotionClass =
        MOTION_ROTATION;

      currentTargetCounts =
        ROTATE_TEST_COUNTS;

      currentIsRotation = true;
      currentUseHeadingHold = false;
      currentUseYHold = false;

      currentMinPWM = ROT_MIN_PWM;
      currentMaxPWM = ROT_MAX_PWM;

      return true;
  }


  return false;
}


// ============================================================
// Reset PI
// ============================================================

void resetWheelPI() {
  for (int i = 0; i < 4; i++) {
    wheelI[i] = 0;
    wheelIOut[i] = 0;

    targetV[i] = 0;
    rawPWM[i] = 0;
    pwm[i] = 0;
  }
}


// ============================================================
// Reset statistics
// ============================================================

void resetStatistics() {
  maxAbsRSegment = 0;
  maxAbsYSegment = 0;
  maxAbsSSegment = 0;

  maxAbsVr = 0;
  maxAbsCr = 0;
  maxAbsRmem = 0;


  r25Recorded = false;
  r50Recorded = false;
  r75Recorded = false;

  r25 = 0;
  r50 = 0;
  r75 = 0;


  brakeProgress = 0;
  finalProgress = 0;

  brakeR = 0;
  brakeVr = 0;
  brakeCr = 0;
  brakeRmem = 0;


  shiftCycles = 0;
  compressionCycles = 0;

  currentCommonShift = 0;
  maxAbsShift = 0;


  controlCycles = 0;


  for (int i = 0; i < 4; i++) {
    lowHit[i] = 0;
    highHit[i] = 0;
    stallHigh[i] = 0;
  }
}


// ============================================================
// Full reset
// ============================================================

void fullReset() {
  CarResult::reset();
  noInterrupts();

  for (int i = 0; i < 4; i++) {
    rawCount[i] = 0;
  }

  encState[0] = readE1();
  encState[1] = readE2();
  encState[2] = readE3();
  encState[3] = readE4();

  interrupts();


  for (int i = 0; i < 4; i++) {
    q[i] = 0;
    lastQ[i] = 0;
    segmentStartQ[i] = 0;
    v[i] = 0;
  }


  Xglobal = 0;
  Yglobal = 0;
  Rglobal = 0;
  Sglobal = 0;

  Vx = 0;
  Vy = 0;
  Vr = 0;
  Vs = 0;


  segmentStartX = 0;
  segmentStartY = 0;
  segmentStartR = 0;
  segmentStartS = 0;


  Xsegment = 0;
  Ysegment = 0;
  Rsegment = 0;
  Ssegment = 0;


  progress = 0;
  progressSpeed = 0;


  rMem = 0;

  currentMemCorrection = 0;

  currentCr = 0;
  currentCy = 0;
  currentCs = 0;


  currentMode = '-';

  currentMotionClass = MOTION_NONE;

  currentTargetCounts = 0;
  brakeTriggerCounts = 0;
  currentBrakeLead = 0;

  currentIsRotation = false;
  currentUseHeadingHold = false;
  currentUseYHold = false;


  currentMinPWM = LONG_MIN_PWM;
  currentMaxPWM = LONG_MAX_PWM;


  protocolMotionActive = false;


  setBaseDirection(
    0, 0, 0, 0
  );


  resetWheelPI();
  resetStatistics();
}


// ============================================================
// PWM conversion
// ============================================================

uint16_t speedToPWM(int value) {
  value =
    clampInt(
      value,
      0,
      255
    );

  return
    (uint32_t)value
    * 4095
    / 255;
}


// ============================================================
// Motor output
// ============================================================

void releaseMotor(int i) {
  setPin(
    MOTOR_A[i],
    0
  );

  setPin(
    MOTOR_B[i],
    0
  );
}


void driveMotorSigned(
  int i,
  int command
) {
  if (command == 0) {
    releaseMotor(i);
    return;
  }


  int logicalSign =
    (command > 0)
    ? 1
    : -1;


  int magnitude =
    abs(command);


  magnitude =
    clampInt(
      magnitude,
      currentMinPWM,
      currentMaxPWM
    );


  int physicalSign =
    logicalSign
    * MOTOR_POLARITY[i];


  uint16_t p =
    speedToPWM(
      magnitude
    );


  if (physicalSign > 0) {
    setPin(
      MOTOR_B[i],
      0
    );

    setPWM(
      MOTOR_A[i],
      p
    );
  }
  else {
    setPin(
      MOTOR_A[i],
      0
    );

    setPWM(
      MOTOR_B[i],
      p
    );
  }
}


void releaseAll() {
  for (int i = 0; i < 4; i++) {
    releaseMotor(i);
    pwm[i] = 0;
  }
}


void brakeAll() {
  for (int i = 0; i < 4; i++) {
    setPin(
      MOTOR_A[i],
      1
    );

    setPin(
      MOTOR_B[i],
      1
    );

    pwm[i] = 0;
  }
}


// ============================================================
// Motion profile
// ============================================================

void getMotionProfile(
  float &speedTarget,
  int &feedForward
) {
  float fastSpeed;
  float midSpeed;
  float slowSpeed;

  int ffFast;
  int ffMid;
  int ffSlow;

  float startSpeed;
  int startFF;

  unsigned long rampTime;


  if (
    currentMotionClass
    == MOTION_LATERAL
  ) {
    fastSpeed = LAT_FAST;
    midSpeed  = LAT_MID;
    slowSpeed = LAT_SLOW;

    ffFast = LAT_FF_FAST;
    ffMid  = LAT_FF_MID;
    ffSlow = LAT_FF_SLOW;

    startSpeed = LAT_START_SPEED;
    startFF = LAT_START_FF;

    rampTime = LAT_RAMP_MS;
  }
  else if (
    currentMotionClass
    == MOTION_DIAGONAL
  ) {
    fastSpeed = DIAG_FAST;
    midSpeed  = DIAG_MID;
    slowSpeed = DIAG_SLOW;

    ffFast = DIAG_FF_FAST;
    ffMid  = DIAG_FF_MID;
    ffSlow = DIAG_FF_SLOW;

    startSpeed = DIAG_START_SPEED;
    startFF = DIAG_START_FF;

    rampTime = DIAG_RAMP_MS;
  }
  else if (
    currentMotionClass
    == MOTION_ROTATION
  ) {
    fastSpeed = ROT_FAST;
    midSpeed  = ROT_MID;
    slowSpeed = ROT_SLOW;

    ffFast = ROT_FF_FAST;
    ffMid  = ROT_FF_MID;
    ffSlow = ROT_FF_SLOW;

    startSpeed = ROT_START_SPEED;
    startFF = ROT_START_FF;

    rampTime = ROT_RAMP_MS;
  }
  else {
    fastSpeed = LONG_FAST;
    midSpeed  = LONG_MID;
    slowSpeed = LONG_SLOW;

    ffFast = LONG_FF_FAST;
    ffMid  = LONG_FF_MID;
    ffSlow = LONG_FF_SLOW;

    startSpeed = LONG_START_SPEED;
    startFF = LONG_START_FF;

    rampTime = LONG_RAMP_MS;
  }


  float remaining =
    currentTargetCounts
    - progress;


  long midZone =
    currentTargetCounts
    * (long)MID_ZONE_PERCENT
    / 100L;


  long slowZone =
    currentTargetCounts
    * (long)SLOW_ZONE_PERCENT
    / 100L;


  if (
    remaining <= slowZone
  ) {
    speedTarget = slowSpeed;
    feedForward = ffSlow;
  }
  else if (
    remaining <= midZone
  ) {
    speedTarget = midSpeed;
    feedForward = ffMid;
  }
  else {
    speedTarget = fastSpeed;
    feedForward = ffFast;
  }


  unsigned long elapsed =
    millis()
    - runStartTime;


  if (
    elapsed < rampTime
  ) {
    float ratio =
      (float)elapsed
      /
      (float)rampTime;


    speedTarget =
      startSpeed
      +
      (speedTarget - startSpeed)
      * ratio;


    float ffInterpolated =
      startFF
      +
      (feedForward - startFF)
      * ratio;


    feedForward =
      (int)(
        ffInterpolated
        + 0.5f
      );
  }
}


// ============================================================
// Heading memory
// ============================================================

void updateHeadingMemory() {
  if (!currentUseHeadingHold) {
    rMem = 0;
    currentMemCorrection = 0;
    currentCr = 0;
    return;
  }


  rMem =
    R_MEM_LEAK * rMem
    +
    Vr;


  rMem =
    clampFloat(
      rMem,
      -R_MEM_LIMIT,
       R_MEM_LIMIT
    );


  currentMemCorrection =
    K_RMEM
    * rMem;


  currentMemCorrection =
    clampFloat(
      currentMemCorrection,
      -MAX_RMEM_CORRECTION,
       MAX_RMEM_CORRECTION
    );


  currentCr =
    -(
      KR_D * Vr
      +
      currentMemCorrection
    );


  currentCr =
    clampFloat(
      currentCr,
      -MAX_R_CORRECTION,
       MAX_R_CORRECTION
    );


  if (
    absFloat(rMem)
    > maxAbsRmem
  ) {
    maxAbsRmem =
      absFloat(rMem);
  }


  if (
    absFloat(currentCr)
    > maxAbsCr
  ) {
    maxAbsCr =
      absFloat(currentCr);
  }
}


// ============================================================
// Wheel targets
// ============================================================

void buildWheelTargets(
  float moveSpeed
) {
  if (currentUseYHold) {
    currentCy =
      -(
        KY_P * Ysegment
        +
        KY_D * Vy
      );


    currentCy =
      clampFloat(
        currentCy,
        -MAX_Y_CORRECTION,
         MAX_Y_CORRECTION
      );
  }
  else {
    currentCy = 0;
  }


  currentCs =
    -KS_D * Vs;


  currentCs =
    clampFloat(
      currentCs,
      -MAX_S_CORRECTION,
       MAX_S_CORRECTION
    );


  float correction[4];


  correction[0] =
      currentCy
    - currentCr
    + currentCs;


  correction[1] =
     -currentCy
    + currentCr
    + currentCs;


  correction[2] =
      currentCy
    + currentCr
    - currentCs;


  correction[3] =
     -currentCy
    - currentCr
    - currentCs;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      targetV[i] = 0;
      continue;
    }


    targetV[i] =
      baseDir[i] * moveSpeed
      +
      correction[i];


    float alignedTarget =
      baseDir[i]
      * targetV[i];


    alignedTarget =
      clampFloat(
        alignedTarget,
        MIN_ACTIVE_TARGET,
        MAX_ACTIVE_TARGET
      );


    targetV[i] =
      baseDir[i]
      * alignedTarget;
  }
}


// ============================================================
// Differential wheel PI
// ============================================================

void updateWheelControllers(
  int feedForward
) {
  float error[4] = {
    0, 0, 0, 0
  };

  float alignedError[4] = {
    0, 0, 0, 0
  };


  float alignedSum = 0;
  int activeCount = 0;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      wheelI[i] = 0;
      wheelIOut[i] = 0;
      rawPWM[i] = 0;
      continue;
    }


    error[i] =
      targetV[i]
      - v[i];


    alignedError[i] =
      baseDir[i]
      * error[i];


    alignedSum +=
      alignedError[i];


    activeCount++;
  }


  if (activeCount <= 0) {
    return;
  }


  float meanAlignedError =
    alignedSum
    / activeCount;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      continue;
    }


    float differentialError =
      alignedError[i]
      -
      meanAlignedError;


    wheelI[i] +=
      differentialError
      * CONTROL_DT;
  }


  float tempOut[4] = {
    0, 0, 0, 0
  };


  float outputSum = 0;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      continue;
    }


    tempOut[i] =
      WHEEL_KI
      * wheelI[i];


    outputSum +=
      tempOut[i];
  }


  float meanOutput =
    outputSum
    / activeCount;


  float maxAbsOutput = 0;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      continue;
    }


    tempOut[i] -=
      meanOutput;


    if (
      absFloat(tempOut[i])
      > maxAbsOutput
    ) {
      maxAbsOutput =
        absFloat(tempOut[i]);
    }
  }


  if (
    maxAbsOutput
    > WHEEL_I_OUT_LIMIT
  ) {
    float scale =
      WHEEL_I_OUT_LIMIT
      /
      maxAbsOutput;


    for (int i = 0; i < 4; i++) {
      if (baseDir[i] != 0) {
        tempOut[i] *= scale;
      }
    }
  }


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      rawPWM[i] = 0;
      continue;
    }


    wheelIOut[i] =
      tempOut[i];


    wheelI[i] =
      wheelIOut[i]
      /
      WHEEL_KI;


    rawPWM[i] =
        baseDir[i]
        * feedForward
      +
        WHEEL_KP
        * error[i]
      +
        baseDir[i]
        * wheelIOut[i];
  }
}


// ============================================================
// Signed desaturation
// ============================================================

void applyDesaturation() {
  float alignedRaw[4] = {
    0, 0, 0, 0
  };


  bool first = true;

  float minimum = 0;
  float maximum = 0;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      pwm[i] = 0;
      continue;
    }


    alignedRaw[i] =
      baseDir[i]
      * rawPWM[i];


    if (first) {
      minimum =
        alignedRaw[i];

      maximum =
        alignedRaw[i];

      first = false;
    }
    else {
      if (
        alignedRaw[i]
        < minimum
      ) {
        minimum =
          alignedRaw[i];
      }

      if (
        alignedRaw[i]
        > maximum
      ) {
        maximum =
          alignedRaw[i];
      }
    }
  }


  if (first) {
    return;
  }


  float span =
    maximum
    - minimum;


  float availableRange =
    currentMaxPWM
    - currentMinPWM;


  currentCommonShift = 0;


  if (
    span <= availableRange
  ) {
    float lowestAllowedShift =
      currentMinPWM
      - minimum;


    float highestAllowedShift =
      currentMaxPWM
      - maximum;


    if (
      lowestAllowedShift > 0
    ) {
      currentCommonShift =
        lowestAllowedShift;
    }
    else if (
      highestAllowedShift < 0
    ) {
      currentCommonShift =
        highestAllowedShift;
    }
    else {
      currentCommonShift = 0;
    }


    if (
      absFloat(currentCommonShift)
      > 0.01f
    ) {
      shiftCycles++;
    }


    if (
      absFloat(currentCommonShift)
      > maxAbsShift
    ) {
      maxAbsShift =
        absFloat(currentCommonShift);
    }


    for (int i = 0; i < 4; i++) {
      if (baseDir[i] == 0) {
        pwm[i] = 0;
        continue;
      }


      float magnitude =
        alignedRaw[i]
        +
        currentCommonShift;


      int finalMagnitude =
        clampInt(
          (int)(
            magnitude
            + 0.5f
          ),
          currentMinPWM,
          currentMaxPWM
        );


      pwm[i] =
        baseDir[i]
        * finalMagnitude;
    }
  }
  else {
    compressionCycles++;


    float scale =
      availableRange
      /
      span;


    for (int i = 0; i < 4; i++) {
      if (baseDir[i] == 0) {
        pwm[i] = 0;
        continue;
      }


      float magnitude =
        currentMinPWM
        +
        (
          alignedRaw[i]
          - minimum
        )
        * scale;


      int finalMagnitude =
        clampInt(
          (int)(
            magnitude
            + 0.5f
          ),
          currentMinPWM,
          currentMaxPWM
        );


      pwm[i] =
        baseDir[i]
        * finalMagnitude;
    }
  }
}


// ============================================================
// PWM / stall stats
// ============================================================

void updateSaturationStats() {
  controlCycles++;


  unsigned long elapsed =
    millis()
    - runStartTime;


  for (int i = 0; i < 4; i++) {
    if (baseDir[i] == 0) {
      continue;
    }


    int magnitude =
      abs(pwm[i]);


    if (
      magnitude <= currentMinPWM
    ) {
      lowHit[i]++;
    }


    if (
      magnitude >= currentMaxPWM
    ) {
      highHit[i]++;
    }


    if (
      elapsed > 600
      &&
      magnitude
        >= currentMaxPWM - 2
      &&
      absFloat(v[i]) < 2.0f
    ) {
      stallHigh[i]++;
    }
  }
}


// ============================================================
// Stop reason
// ============================================================

void printStopReason(
  uint8_t reason
) {
  switch (reason) {

    case STOP_TARGET:
      Serial.print(
        F("TARGET")
      );
      break;

    case STOP_EMERGENCY:
      Serial.print(
        F("EMERGENCY")
      );
      break;

    case STOP_TIMEOUT:
      Serial.print(
        F("TIMEOUT")
      );
      break;

    case STOP_WRONG_DIRECTION:
      Serial.print(
        F("WRONG_DIRECTION")
      );
      break;

    default:
      Serial.print(
        F("UNKNOWN")
      );
      break;
  }
}


// ============================================================
// Motion class
// ============================================================

void printMotionClass() {
  if (
    currentMotionClass
    == MOTION_LONGITUDINAL
  ) {
    Serial.print(
      F("LONG")
    );
  }
  else if (
    currentMotionClass
    == MOTION_LATERAL
  ) {
    Serial.print(
      F("LAT")
    );
  }
  else if (
    currentMotionClass
    == MOTION_DIAGONAL
  ) {
    Serial.print(
      F("DIAG")
    );
  }
  else if (
    currentMotionClass
    == MOTION_ROTATION
  ) {
    Serial.print(
      F("ROT")
    );
  }
  else {
    Serial.print(
      F("NONE")
    );
  }
}


// ============================================================
// 对外 DONE
//
// ENC = 沿本次要求运动方向的最终编码器运动量
// ============================================================

void printProtocolDone(
  uint8_t reason
) {
  Serial.print(
    F("@DONE,")
  );

  Serial.print(
    currentMode
  );

  Serial.print(',');

  printStopReason(
    reason
  );


  printProtocolRequest();

  Serial.print(
    F(",BRAKE=")
  );

  Serial.print(
    brakeProgress,
    2
  );


  Serial.print(
    F(",ENC=")
  );

  Serial.print(
    finalProgress,
    2
  );


  Serial.print(
    F(",DX=")
  );

  Serial.print(
    Xsegment,
    2
  );


  Serial.print(
    F(",DY=")
  );

  Serial.print(
    Ysegment,
    2
  );


  Serial.print(
    F(",DR=")
  );

  Serial.print(
    Rsegment,
    2
  );


  Serial.print(
    F(",DS=")
  );

  Serial.print(
    Ssegment,
    2
  );


  Serial.print(
    F(",Q1=")
  );

  Serial.print(
    q[0]
    -
    segmentStartQ[0]
  );


  Serial.print(
    F(",Q2=")
  );

  Serial.print(
    q[1]
    -
    segmentStartQ[1]
  );


  Serial.print(
    F(",Q3=")
  );

  Serial.print(
    q[2]
    -
    segmentStartQ[2]
  );


  Serial.print(
    F(",Q4=")
  );

  Serial.println(
    q[3]
    -
    segmentStartQ[3]
  );

}


// ============================================================
// Detailed final report
// ============================================================

void printFinalReport(
  uint8_t reason,
  unsigned long runTime
) {
  // Uno program space is reserved for checked commands and recoverable results.
  // Keep the endpoint/calibration diagnostics; machine replies are independent.
  Serial.print(F("RESULT MODE=")); Serial.print(currentMode);
  Serial.print(F(" REASON=")); printStopReason(reason); Serial.println();
  Serial.print(F("TARGET=")); Serial.print(currentTargetCounts);
  Serial.print(F(" BRAKE_P=")); Serial.print(brakeProgress,2);
  Serial.print(F(" FINAL_P=")); Serial.println(finalProgress,2);
  Serial.print(F("TRIGGER=")); Serial.print(brakeTriggerCounts);
  Serial.print(F(" LEAD=")); Serial.print(currentBrakeLead);
  Serial.print(F(" STOP_ERROR=")); Serial.println(finalProgress-currentTargetCounts,2);
  Serial.print(F("DX=")); Serial.print(Xsegment,2);
  Serial.print(F(" DY=")); Serial.print(Ysegment,2);
  Serial.print(F(" DR=")); Serial.print(Rsegment,2);
  Serial.print(F(" DS=")); Serial.println(Ssegment,2);
  Serial.print(F("TIME=")); Serial.print(runTime);
  Serial.print(F(" FREE_RAM=")); Serial.println(freeRam());
}


// ============================================================
// Stop motion
// ============================================================

void stopMotion(
  uint8_t reason
) {
  if (!running) {
    if (
      reason == STOP_EMERGENCY
    ) {
      brakeAll();
      delay(IDLE_BRAKE_HOLD_MS);
      releaseAll();
    }

    return;
  }


  bool wasProtocolMotion =
    protocolMotionActive;


  running = false;

  // 急停/超时也读取当前位置；BRAKE 紧邻制动前采样。
  updateLogicalPosition();
  updateSegmentState();

  brakeProgress =
    progress;

  brakeR =
    Rsegment;

  brakeVr =
    Vr;

  brakeCr =
    currentCr;

  brakeRmem =
    rMem;


  // 先制动，停车数据采集完成后再发送日志。
  brakeAll();

  delay(BRAKE_HOLD_MS);

  releaseAll();


  updateLogicalPosition();
  updateSegmentState();


  finalProgress =
    progress;


  unsigned long runTime =
    millis()
    -
    runStartTime;


  // 这两行按事件顺序补发，串口接收时刻不代表制动时刻。
  if (!wasProtocolMotion || verboseProtocolLog) Serial.println(F("STOP_BEGIN"));
  if (!wasProtocolMotion || verboseProtocolLog) Serial.println(F("STOP_RELEASED"));


  // ----------------------------------------------------------
  // 先输出人工调试报告。
  //
  // @DONE 一定放在最后。
  // 因此另一端收到 @DONE 时，
  // 当前动作及其串口报告都已经完全结束。
  // ----------------------------------------------------------

  if (!wasProtocolMotion || verboseProtocolLog) {
    printFinalReport(reason, runTime);
  }


  if (wasProtocolMotion) {
    if(CarResult::inFlight) {
      CarResult::last.tag=CarResult::activeTag;
      CarResult::last.mode=currentMode;
      CarResult::last.request=protocolRequestedValue;
      CarResult::last.mm=protocolUnitMM;
      CarResult::last.reason=reason;
      CarResult::last.brake=brakeProgress;
      CarResult::last.enc=finalProgress;
      for(uint8_t i=0;i<4;i++) CarResult::last.wheel[i]=q[i]-segmentStartQ[i];
      CarResult::last.valid=true;
      CarResult::inFlight=false;
      CarResult::reply(CarResult::last.tag);
    } else printProtocolDone(reason);

    Serial.flush();
  }


  protocolMotionActive = false;
}


// ============================================================
// Start motion
//
// requestedCounts > 0：使用外部指定目标
// requestedCounts <=0：使用默认测试目标
// ============================================================

void startMotion(
  char cmd,
  long requestedCounts,
  bool fromProtocol
) {
  if (CarConfig::editing()) { Serial.println(F("@ERR,BUSY")); return; }
  if (
    !configureMode(cmd)
  ) {
    return;
  }


  if (
    requestedCounts > 0
  ) {
    currentTargetCounts =
      requestedCounts;
  }


  protocolRequestedValue = currentTargetCounts;
  CarResult::reset();
  protocolUnitMM = false;
  currentBrakeLead = calibratedBrakeLead(currentMode, currentTargetCounts);
  brakeTriggerCounts = currentTargetCounts - currentBrakeLead;


  protocolMotionActive =
    fromProtocol;


  updateLogicalPosition();


  for (int i = 0; i < 4; i++) {
    segmentStartQ[i] =
      q[i];

    lastQ[i] =
      q[i];

    v[i] = 0;
  }


  segmentStartX =
    Xglobal;

  segmentStartY =
    Yglobal;

  segmentStartR =
    Rglobal;

  segmentStartS =
    Sglobal;


  Xsegment = 0;
  Ysegment = 0;
  Rsegment = 0;
  Ssegment = 0;


  progress = 0;
  progressSpeed = 0;


  Vx = 0;
  Vy = 0;
  Vr = 0;
  Vs = 0;


  resetWheelPI();


  rMem = 0;

  currentMemCorrection = 0;

  currentCr = 0;
  currentCy = 0;
  currentCs = 0;


  resetStatistics();


  runStartTime =
    millis();

  lastControlTime =
    millis();

  lastTelemetryTime =
    millis();


  running = true;


  // Quiet protocol mode emits ACK/DONE only; diagnostics are opt-in via LOG=1.
  if (!fromProtocol || verboseProtocolLog) {
  Serial.println();


  Serial.print(
    F("START MODE=")
  );

  Serial.print(
    currentMode
  );


  Serial.print(
    F(" CLASS=")
  );

  printMotionClass();


  Serial.print(
    F(" TARGET=")
  );

  Serial.println(
    currentTargetCounts
  );


  Serial.print(
    F("PWM_RANGE=")
  );

  Serial.print(
    currentMinPWM
  );

  Serial.print('-');

  Serial.println(
    currentMaxPWM
  );


  Serial.print(
    F("DIR=")
  );

  for (int i = 0; i < 4; i++) {
    Serial.print(
      baseDir[i]
    );

    if (i < 3) {
      Serial.print(',');
    }
  }

  Serial.println();


  Serial.print(
    F("SOURCE=")
  );

  if (fromProtocol) {
    Serial.println(
      F("PROTOCOL")
    );
  }
  else {
    Serial.println(
      F("MANUAL")
    );
  }


  Serial.print(
    F("FREE_RAM_START=")
  );

  Serial.println(
    freeRam()
  );
  }
}


// ============================================================
// Main controller
// ============================================================

void updateControl() {
  updateKinematics();


  if (
    progress
    >= brakeTriggerCounts
  ) {
    stopMotion(
      STOP_TARGET
    );

    return;
  }


  if (
    progress
    <= WRONG_DIRECTION_LIMIT
  ) {
    stopMotion(
      STOP_WRONG_DIRECTION
    );

    return;
  }


  float moveSpeed;
  int feedForward;


  getMotionProfile(
    moveSpeed,
    feedForward
  );


  updateHeadingMemory();


  buildWheelTargets(
    moveSpeed
  );


  updateWheelControllers(
    feedForward
  );


  applyDesaturation();


  updateSaturationStats();


  for (int i = 0; i < 4; i++) {
    driveMotorSigned(
      i,
      pwm[i]
    );
  }
}


// ============================================================
// Telemetry
// ============================================================

void printTelemetry() {
  // UNO TX 缓冲区可用上限为 63 字节。整行能放下才发送，绝不等待。
  // 详细运动学数据继续保留在停车报告中。
  char line[64];
  int length = snprintf(
    line,
    sizeof(line),
    "M=%c P=%ld/%ld PWM=%d,%d,%d,%d\r\n",
    currentMode,
    (long)progress,
    currentTargetCounts,
    pwm[0], pwm[1], pwm[2], pwm[3]
  );

  if (length <= 0 || length >= (int)sizeof(line)) {
    return;
  }

  if (Serial.availableForWrite() < length) {
    return;
  }

  Serial.write((const uint8_t *)line, (size_t)length);
}


// ============================================================
// 外部协议解析
//
// @MOVE,D,1400,CNT
// ============================================================

void processProtocolCommand(
  char *line
) {
  if(CarResult::query(line)) return;
  if (CarConfig::process(line, running)) return;
  if (strncmp(line, "@MOVE,", 6) == 0 && CarConfig::editing()) {
    Serial.println(F("@ERR,BUSY")); return;
  }
  // 只读握手：带8位十六进制标记，便于发送端排除旧应答。
  if (strncmp(line, "@PING,", 6) == 0) {
    if (strlen(line) != 14) { Serial.println(F("@ERR,BAD_CMD")); return; }
    for (uint8_t i = 6; i < 14; ++i) {
      char c = line[i];
      if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) {
        Serial.println(F("@ERR,BAD_CMD")); return;
      }
    }
    if (running) { Serial.println(F("@ERR,BUSY")); return; }
    CarResult::arm(strtoul(line+6,NULL,16));
    Serial.print(F("@PONG,")); Serial.println(line + 6);
    return;
  }
  if (strcmp(line, "@LOG,0") == 0 || strcmp(line, "@LOG,1") == 0) {
    if (running) { Serial.println(F("@ERR,BUSY")); return; }
    verboseProtocolLog = line[5] == '1';
    Serial.print(F("@LOG,")); Serial.println(verboseProtocolLog ? 1 : 0);
    return;
  }
  if (
    strncmp(
      line,
      "@MOVE,",
      6
    ) != 0
  ) {
    Serial.println(
      F("@ERR,BAD_CMD")
    );

    return;
  }


  uint8_t commaCount = 0;
  for (char *v = line; *v; ++v) if (*v == ',') ++commaCount;
  uint32_t resultTag=0;
  bool protectedMove=commaCount==5;
  if(protectedMove) {
    if(!CarResult::checked(line,resultTag,true)) {Serial.println(F("@ERR,BAD_CMD"));return;}
    if(running) {Serial.println(F("@ERR,BUSY"));return;}
    if(CarResult::last.valid && resultTag==CarResult::last.tag) {
      CarResult::reply(resultTag); return;
    }
    if(!CarResult::armed || resultTag!=CarResult::armedTag) {Serial.println(F("@ERR,BAD_CMD"));return;}
    commaCount=3;
  }
  if (commaCount != 3) { Serial.println(F("@ERR,BAD_CMD")); return; }

  char *modeToken =
    strtok(
      line + 6,
      ","
    );


  char *valueToken =
    strtok(
      NULL,
      ","
    );


  char *unitToken =
    strtok(
      NULL,
      ","
    );


  char *extraToken =
    strtok(
      NULL,
      ","
    );


  if (
       modeToken == NULL
    || valueToken == NULL
    || unitToken == NULL
    || extraToken != NULL
  ) {
    Serial.println(
      F("@ERR,BAD_CMD")
    );

    return;
  }


  if (
       modeToken[0] == '\0'
    || modeToken[1] != '\0'
  ) {
    Serial.println(
      F("@ERR,BAD_MODE")
    );

    return;
  }


  char mode =
    modeToken[0];


  if (
    mode >= 'A'
    &&
    mode <= 'Z'
  ) {
    mode =
      mode
      - 'A'
      + 'a';
  }


  if (
    !isMovementCommand(mode)
  ) {
    Serial.println(
      F("@ERR,BAD_MODE")
    );

    return;
  }


  bool requestMM = strcmp(unitToken, "MM") == 0;
  if (!requestMM && strcmp(unitToken, "CNT") != 0) {
    Serial.println(F("@ERR,BAD_UNIT")); return;
  }
  if (requestMM && countsPerMm10000(upperModeChar(mode)) == 0) {
    Serial.println(F("@ERR,BAD_UNIT")); return;
  }
  // 正整数输入，显式拒绝溢出，避免strtol饱和后接受错误距离。
  long requestedValue = 0;
  for (char *v = valueToken; *v; ++v) {
    if (*v < '0' || *v > '9'
        || requestedValue > (2147483647L - (*v - '0')) / 10L) {
      Serial.println(F("@ERR,BAD_VALUE")); return;
    }
    requestedValue = requestedValue * 10L + (*v - '0');
  }
  long requestedCounts = requestMM
      ? millimetersToCounts(upperModeChar(mode), requestedValue) : requestedValue;
  if (requestedCounts <= 0) { Serial.println(F("@ERR,BAD_VALUE")); return; }

  if (running) {
    Serial.println(
      F("@ERR,BUSY")
    );

    return;
  }


  // ==========================================================
  // ACK 必须先完整发送，
  // 然后才开始运动。
  // ==========================================================

  Serial.print(
    F("@ACK,")
  );

  Serial.print(
    upperModeChar(mode)
  );

  Serial.print(',');

  Serial.print(requestedValue);

  Serial.println(requestMM ? F(",MM") : F(",CNT"));


  Serial.flush();


  startMotion(
    mode,
    requestedCounts,
    true
  );
  protocolRequestedValue = requestedValue;
  protocolUnitMM = requestMM;
  if(protectedMove) {
    CarResult::activeTag=resultTag;
    CarResult::inFlight=true;
  }
}


// ============================================================
// 原电脑单键控制
// ============================================================

void handleLegacyCommand(
  char cmd
) {
  if (
       cmd == '\r'
    || cmd == '\n'
    || cmd == ' '
    || cmd == '\t'
  ) {
    return;
  }


  if (
    cmd >= 'A'
    &&
    cmd <= 'Z'
  ) {
    cmd =
      cmd
      - 'A'
      + 'a';
  }


  // X 随时急停
  if (
    cmd == 'x'
  ) {
    stopMotion(
      STOP_EMERGENCY
    );

    return;
  }


  // P 状态
  if (
    cmd == 'p'
  ) {
    if (running) {
      printTelemetry();
    }
    else {
      updateLogicalPosition();


      Serial.print(
        F("IDLE X=")
      );

      Serial.print(
        Xglobal,
        1
      );


      Serial.print(
        F(" Y=")
      );

      Serial.print(
        Yglobal,
        1
      );


      Serial.print(
        F(" R=")
      );

      Serial.print(
        Rglobal,
        1
      );


      Serial.print(
        F(" S=")
      );

      Serial.println(
        Sglobal,
        1
      );
    }

    return;
  }


  // G reset
  if (
    cmd == 'g'
    &&
    !running
  ) {
    fullReset();
    releaseAll();


    Serial.println(
      F("FULL RESET")
    );


    Serial.print(
      F("FREE_RAM=")
    );

    Serial.println(
      freeRam()
    );

    return;
  }


  // 人工移动，继续使用默认测试距离
  if (
    !running
    &&
    isMovementCommand(cmd)
  ) {
    startMotion(
      cmd,
      0,
      false
    );
  }
}


// ============================================================
// 串口字节处理
//
// '@' 开始协议命令。
// 其他字符保持旧单键控制。
// ============================================================

// 仅空闲时打印接收字节，定位前缀丢失；不改变 @ 机器协议。
void printRxDiagnostic(uint8_t length) {
  if (running || !verboseProtocolLog) return;
  Serial.print(F("RX LEN="));
  Serial.print(length);
  Serial.print(F(" HEX="));
  for (uint8_t i = 0; i < length; i++) {
    uint8_t value = (uint8_t)rxBuffer[i];
    if (value < 16) Serial.print('0');
    Serial.print(value, HEX);
    if (i + 1 < length) Serial.print(' ');
  }
  Serial.println();
}

bool isLegacyInput(char c) {
  c = upperModeChar(c);
  return isMovementCommand(c - 'A' + 'a')
      || c == 'X' || c == 'P' || c == 'G';
}

void finishSerialInput() {
  uint8_t length = rxLength;
  bool invalid = rxInvalid;
  rxLength = 0;
  rxInvalid = false;
  if (length == 0) return;
  rxBuffer[length] = '\0';
  // 半双工收发换向：仅空闲时在整行接收后留出15ms再回复。
  // 不在运动中的控制循环插入等待。实际模块未接AUX，此值为保守试验值。
  if (!running) delay(15);
  printRxDiagnostic(length);
  if (invalid) {
    Serial.println(F("@ERR,BAD_CMD"));
    return;
  }
  if (length == 1 && isLegacyInput(rxBuffer[0])) {
    handleLegacyCommand(rxBuffer[0]);
    return;
  }
  // 多字符只允许完整协议，绝不逐字符回退成单键动作。
  processProtocolCommand(rxBuffer);
}

void serviceSerialInput() {
  if (!protocolDiscarding && rxLength == 1 && !rxInvalid
      && isLegacyInput(rxBuffer[0])
      && millis() - rxLastByteTime >= MANUAL_GAP_MS) {
    finishSerialInput();
  }
}

void handleSerialByte(char c) {
  if (protocolDiscarding) {
    if (c == '\r' || c == '\n') protocolDiscarding = false;
    return;
  }
  if (c == '\r' || c == '\n') {
    finishSerialInput();
    return;
  }
  // 保留独立 X 的即时急停；指令内部字符仍按整行校验。
  if (rxLength == 0 && (c == 'X' || c == 'x')) {
    handleLegacyCommand(c);
    return;
  }
  if (rxLength == 0 && (c == ' ' || c == '\t')) return;
  if (rxLength >= RX_BUFFER_SIZE - 1) {
    printRxDiagnostic(rxLength);
    rxLength = 0;
    rxInvalid = false;
    protocolDiscarding = true;
    Serial.println(F("@ERR,LINE_TOO_LONG"));
    return;
  }
  // 嵌入 NUL 或非 ASCII 字节必须整行拒绝，避免字符串截断执行。
  if ((uint8_t)c < 32 || (uint8_t)c > 126) rxInvalid = true;
  rxBuffer[rxLength++] = c;
  rxLastByteTime = millis();
}


// ============================================================
// Setup
// ============================================================

void setup() {
  CarConfig::init();
  Serial.begin(9600, SERIAL_8N1);

  Wire.begin();

  PCA9685_Init();

  setupEncoders();

  releaseAll();

  fullReset();


  Serial.println();

  Serial.println(
    F("==============================")
  );


  Serial.println(
    F("MECANUM UNIVERSAL V6.3 COMM READY")
  );


  Serial.println(F("RX_FRAME_GUARD=1 SERIAL=9600,8N1"));
  Serial.println(F("BRAKE_CAL=4 ALL8 SHORT-LONG"));
  Serial.println(F("DIST_CAL=1 MM=WSADQEZC"));
  Serial.println(F("COMM_GUARD=3 LOG=0 PING=1"));
  Serial.println(F("CONFIG=1 RAM=1 CRC=CCITT"));
  Serial.println(F("RESULT=1 CRC=CCITT QUERY=1"));

  Serial.println(
    F("LONG PWM 90-120")
  );


  Serial.println(
    F("LAT PWM 90-180")
  );


  Serial.println(
    F("DIAG PWM 90-180")
  );


  Serial.println(
    F("ROT PWM 90-120")
  );


  Serial.println();


  Serial.println(
    F("MANUAL:")
  );


  Serial.println(
    F("Q W E   R=CW")
  );


  Serial.println(
    F("A S D   F=CCW")
  );


  Serial.println(
    F("Z X C")
  );


  Serial.println();


  Serial.println(
    F("PROTOCOL:")
  );


  Serial.println(
    F("@MOVE,D,1400,CNT")
  );


  Serial.println(
    F("@ACK / @DONE / @ERR")
  );


  Serial.println();


  Serial.print(
    F("FREE_RAM=")
  );

  Serial.println(
    freeRam()
  );


  Serial.println(
    F("==============================")
  );
}


// ============================================================
// Loop
// ============================================================

void loop() {
  // ==========================================================
  // Serial input
  // ==========================================================

  while (
    Serial.available() > 0
  ) {
    char c =
      Serial.read();


    handleSerialByte(
      c
    );
  }


  // ==========================================================
  // Running
  // ==========================================================

  // 确认不带换行的独立单键，不延迟控制循环。
  serviceSerialInput();

  if (running) {
    unsigned long now =
      millis();


    // --------------------------------------------------------
    // 20 ms controller
    // --------------------------------------------------------

    if (
      now - lastControlTime
      >= CONTROL_PERIOD_MS
    ) {
      lastControlTime =
        now;


      updateControl();
    }


    // --------------------------------------------------------
    // 1 s telemetry
    // --------------------------------------------------------

    if (
      running
      &&
      now - lastTelemetryTime
      >= 1000
    ) {
      lastTelemetryTime =
        now;


      if (!protocolMotionActive || verboseProtocolLog) printTelemetry();
    }


    // --------------------------------------------------------
    // Timeout
    // --------------------------------------------------------

    if (
      running
      &&
      now - runStartTime
      >= RUN_TIMEOUT_MS
    ) {
      stopMotion(
        STOP_TIMEOUT
      );
    }
  }
}
