#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define MIRTO_TX 17

#define BRIDGE_OUT 16

#define TRIG_PIN 4
#define ECHO_PIN 5

VL53L4CX sensor_vl53l4cx_sat(&Wire, -1);
HardwareSerial MirtoSerial(1);

const int NUM_SAMPLES = 20;

bool vlDone = false;

int getAverageDistance() {
  long sum = 0;
  int count = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    uint8_t ready = 0;

    while (!ready) {
      sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&ready);
      delay(1);
    }

    VL53L4CX_MultiRangingData_t data;

    sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(&data);

    if (data.NumberOfObjectsFound > 0) {
      int d = data.RangeData[0].RangeMilliMeter;
      int status = data.RangeData[0].RangeStatus;

      float signal =
        (float)data.RangeData[0].SignalRateRtnMegaCps / 65536.0f;

      float ambient =
        (float)data.RangeData[0].AmbientRateRtnMegaCps / 65536.0f;

      Serial.print("VL Sample ");
      Serial.print(i + 1);

      Serial.print(" | Status=");
      Serial.print(status);

      Serial.print(" | Distance=");
      Serial.print(d);

      Serial.print(" | Signal=");
      Serial.print(signal);

      Serial.print(" | Ambient=");
      Serial.println(ambient);

      // Only reject obviously garbage values
      if (status == 0 && d > 0 && d < 6000) {
        sum += d;
        count++;
      }
    } else {
      Serial.print("VL Sample ");
      Serial.print(i + 1);
      Serial.println(" | No Object Found");
    }

    sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
  }

  if (count == 0)
    return -1;

  return (int)(sum / count);
}

float getUltrasonicDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0)
    return -1;

  return duration * 0.0343f / 2.0f;
}

void setup() {
  Serial.begin(115200);

  MirtoSerial.begin(
    9600,
    SERIAL_8N1,
    -1,
    MIRTO_TX);

  pinMode(BRIDGE_OUT, OUTPUT);
  digitalWrite(BRIDGE_OUT, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(1000);

  Serial.println("Starting VL53L4CX...");

  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  VL53L4CX_Error initStatus =
    sensor_vl53l4cx_sat.InitSensor(0x29);

  Serial.print("Init status = ");
  Serial.println((int)initStatus);

  VL53L4CX_DistanceModes mode;

  sensor_vl53l4cx_sat.VL53L4CX_GetDistanceMode(&mode);

  Serial.print("Distance mode before start = ");
  Serial.println((int)mode);

  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

  sensor_vl53l4cx_sat.VL53L4CX_GetDistanceMode(&mode);

  Serial.print("Distance mode after start = ");
  Serial.println((int)mode);

  Serial.println("VL53L4CX Ready");
}

void loop() {
  if (!vlDone) {
    delay(2000);

    int avgDistance = getAverageDistance();

    Serial.print("Average VL distance = ");
    Serial.print(avgDistance);
    Serial.println(" mm");

    if (avgDistance > 0) {
      MirtoSerial.println(avgDistance);

      Serial.print("Sent to Mirto: ");
      Serial.print(avgDistance);
      Serial.println(" mm");
    } else {
      Serial.println("No valid VL distance");
    }

    sensor_vl53l4cx_sat.VL53L4CX_StopMeasurement();

    Serial.println("VL53L4CX stopped");

    vlDone = true;
  }

  float hcDistance = getUltrasonicDistanceCM();

  if (hcDistance >= 25.0f && hcDistance <= 80.0f) {
    digitalWrite(BRIDGE_OUT, HIGH);
  } else {
    digitalWrite(BRIDGE_OUT, LOW);
  }

  Serial.print("HC=");
  Serial.print(hcDistance);
  Serial.print(" cm");

  Serial.print(" | Bridge=");
  Serial.println(digitalRead(BRIDGE_OUT));

  delay(100);
}