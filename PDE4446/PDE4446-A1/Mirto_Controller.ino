#include <SoftwareSerial.h>

// ESP32 TX -> Mirto pin 11
SoftwareSerial sensorSerial(11, 13);  // RX, TX

int averageDistance = 0;

// Motor pins
const int AIN2 = 5;
const int BIN1 = 6;

// Bridge signal from ESP32
const int BRIDGE_IN = 12;

// Encoder pins
const int LEFT_ENCODER = 3;
const int RIGHT_ENCODER = 2;

// LEDs
const int RED_LED = A1;
const int YELLOW_LED = A2;
const int GREEN_LED = A3;

// Encoder counts
volatile long leftCount = 0;
volatile long rightCount = 0;

// Distance calculation
const float COUNT_PER_MM = 2.133;

// Final safety buffer before the barrier
const float FINAL_BUFFER_MM = 0.0;

// Speed settings
const int NORMAL_SPEED = 150;
const int BRIDGE_SPEED = 70;

const int MIN_SPEED = 70;
const int MAX_SPEED = 255;

// Smooth acceleration/deceleration
const int SPEED_STEP = 10;
const unsigned long SPEED_UPDATE_MS = 500;

unsigned long lastSpeedUpdate = 0;

// Steering trim
const int LEFT_TRIM = 5;

// Stop calculations
float stopDistance = 0;
float targetCount = 0;
int times = 0;

int targetSpeed = NORMAL_SPEED;
int currentSpeed = BRIDGE_SPEED;

// Straight-line correction
const float KP = 2.5;

long error = 0;

long segmentLeftStart = 0;
long segmentRightStart = 0;

// ============================================
// Encoder interrupts
// ============================================

void leftEncoder() {
  leftCount++;
}

void rightEncoder() {
  rightCount++;
}

// ============================================
// Setup
// ============================================

void setup() {

  delay(2000);

  Serial.begin(9600);
  sensorSerial.begin(9600);

  Serial.println("Waiting for distance from ESP32...");

  while (averageDistance < 100) {

    if (sensorSerial.available() > 0) {

      String s =
        sensorSerial.readStringUntil('\n');

      Serial.print("Received: [");
      Serial.print(s);
      Serial.println("]");

      averageDistance = s.toInt();

      Serial.print("Parsed value: ");
      Serial.println(averageDistance);
    }
  }

  Serial.print("Received distance = ");
  Serial.print(averageDistance);
  Serial.println(" mm");

  // Divide measured distance into 6 equal sections
  stopDistance =
    (averageDistance / 6.0) *
    COUNT_PER_MM;

  targetCount = stopDistance;

  Serial.print("Segment distance (mm) = ");
  Serial.println(averageDistance / 6.0);

  Serial.print("Stop distance counts = ");
  Serial.println(stopDistance);

  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);

  pinMode(BRIDGE_IN, INPUT);

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);

  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER),
    leftEncoder,
    RISING);

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER),
    rightEncoder,
    RISING);

  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);

  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();

  segmentLeftStart = 0;
  segmentRightStart = 0;

  Serial.print("Final buffer counts = ");
  Serial.println(FINAL_BUFFER_MM * COUNT_PER_MM);
}

// ============================================
// Main loop
// ============================================

void loop() {

  driveForward();

  while (true) {
  }
}

// ============================================
// Driving logic
// ============================================

void driveForward() {

  while (true) {

    if (digitalRead(BRIDGE_IN) == HIGH) {

      targetSpeed = BRIDGE_SPEED;

      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, HIGH);

    } else {

      targetSpeed = NORMAL_SPEED;

      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
    }

    // Smooth speed ramp
    unsigned long now = millis();

    if (now - lastSpeedUpdate >= SPEED_UPDATE_MS) {

      lastSpeedUpdate = now;

      if (currentSpeed < targetSpeed) {

        currentSpeed += SPEED_STEP;

        if (currentSpeed > targetSpeed) {
          currentSpeed = targetSpeed;
        }

      } else if (currentSpeed > targetSpeed) {

        currentSpeed -= SPEED_STEP;

        if (currentSpeed < targetSpeed) {
          currentSpeed = targetSpeed;
        }
      }
    }

    long left;
    long right;

    noInterrupts();
    left = leftCount;
    right = rightCount;
    interrupts();

    float averageCount =
      (left + right) / 2.0;

    if (averageCount >= targetCount) {

      stop();

      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, HIGH);

      Serial.print("Stop ");
      Serial.println(times + 1);

      Serial.print("Average count = ");
      Serial.println(averageCount);

      Serial.print("Stop error = ");
      Serial.println(error);

      if (times < 5) {

        delay(5000);

        segmentLeftStart = left;
        segmentRightStart = right;

        times++;

        if (times < 5) {

          targetCount += stopDistance;

        } else {

          float finalBufferCounts =
            FINAL_BUFFER_MM *
            COUNT_PER_MM;

          targetCount +=
            stopDistance -
            finalBufferCounts;
        }

        digitalWrite(RED_LED, LOW);

        if (digitalRead(BRIDGE_IN) == HIGH) {
          digitalWrite(YELLOW_LED, HIGH);
        } else {
          digitalWrite(GREEN_LED, HIGH);
        }

        currentSpeed = BRIDGE_SPEED;

        continue;

      } else {

        digitalWrite(GREEN_LED, LOW);
        digitalWrite(YELLOW_LED, LOW);
        digitalWrite(RED_LED, HIGH);

        Serial.println("Finished");
        return;
      }
    }

    error =
      (left - segmentLeftStart) -
      (right - segmentRightStart);

    float correction =
      constrain(
        KP * error,
        -80,
        80);

    int leftSpeed =
      constrain(
        currentSpeed +
        LEFT_TRIM -
        correction,
        MIN_SPEED,
        MAX_SPEED);

    int rightSpeed =
      constrain(
        currentSpeed +
        correction,
        MIN_SPEED,
        MAX_SPEED);

    analogWrite(
      AIN2,
      leftSpeed);

    analogWrite(
      BIN1,
      rightSpeed);
  }
}

// ============================================
// Stop motors
// ============================================

void stop() {

  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
}
