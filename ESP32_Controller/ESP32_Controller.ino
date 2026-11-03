#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define XSHUT_PIN 18

// Mirto communication
#define MIRTO_TX 17

// Bridge detection output
#define BRIDGE_OUT 16

// HC-SR04
#define TRIG_PIN 4
#define ECHO_PIN 5

VL53L4CX sensor_vl53l4cx_sat(&Wire, -1);
HardwareSerial MirtoSerial(1);

const int NUM_SAMPLES = 20;

bool vlDone = false;

bool initVL53()
{
    Serial.println("Resetting VL53L4CX...");

    pinMode(XSHUT_PIN, OUTPUT);

    digitalWrite(XSHUT_PIN, LOW);
    delay(500);

    digitalWrite(XSHUT_PIN, HIGH);
    delay(1000);

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(200);

    sensor_vl53l4cx_sat.begin();

    VL53L4CX_Error status =
        sensor_vl53l4cx_sat.InitSensor(0x29);

    Serial.print("Init status = ");
    Serial.println((int)status);

    if (status != 0)
    {
        return false;
    }

    sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

    delay(500);

    Serial.println("VL53L4CX Ready");

    return true;
}

int getMedianDistance()
{
    int distances[NUM_SAMPLES];
    int count = 0;

    while (count < NUM_SAMPLES)
    {
        uint8_t ready = 0;

        unsigned long startTime = millis();

        while (!ready)
        {
            sensor_vl53l4cx_sat
                .VL53L4CX_GetMeasurementDataReady(&ready);

            if (millis() - startTime > 3000)
            {
                Serial.println("Measurement timeout");
                return -1;
            }

            delay(1);
        }

        VL53L4CX_MultiRangingData_t data;

        sensor_vl53l4cx_sat
            .VL53L4CX_GetMultiRangingData(&data);

        int bestDistance = -1;

        for (int obj = 0;
             obj < data.NumberOfObjectsFound;
             obj++)
        {
            int distance =
                data.RangeData[obj].RangeMilliMeter;

            int status =
                data.RangeData[obj].RangeStatus;

            Serial.print("Obj ");
            Serial.print(obj);

            Serial.print(" Status=");
            Serial.print(status);

            Serial.print(" Distance=");
            Serial.println(distance);

            // Ignore impossible values
            if (distance >= 100 &&
                distance <= 6000)
            {
                if (distance > bestDistance)
                {
                    bestDistance = distance;
                }
            }
        }

        if (bestDistance > 0)
        {
            distances[count] = bestDistance;
            count++;

            Serial.print("Accepted = ");
            Serial.print(bestDistance);
            Serial.println(" mm");
        }

        sensor_vl53l4cx_sat
            .VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    // Sort samples
    for (int i = 0; i < count - 1; i++)
    {
        for (int j = i + 1; j < count; j++)
        {
            if (distances[j] < distances[i])
            {
                int temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }

    Serial.println("Sorted samples:");

    for (int i = 0; i < count; i++)
    {
        Serial.println(distances[i]);
    }

    // Median
    int medianDistance;

    if (count % 2 == 0)
    {
        medianDistance =
            (distances[count / 2 - 1] +
             distances[count / 2]) / 2;
    }
    else
    {
        medianDistance =
            distances[count / 2];
    }

    return medianDistance;
}

float getUltrasonicDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    long duration =
        pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0)
    {
        return -1;
    }

    return duration * 0.0343f / 2.0f;
}

void setup()
{
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

    delay(1000);

    if (!initVL53())
    {
        Serial.println("VL53L4CX FAILED");

        while (true)
        {
            delay(1000);
        }
    }
}

void loop()
{
    if (!vlDone)
    {
        delay(2000);

        int medianDistance =
            getMedianDistance();

        Serial.print("Median VL distance = ");
        Serial.print(medianDistance);
        Serial.println(" mm");

        if (medianDistance > 0)
        {
            MirtoSerial.println(medianDistance);

            Serial.print("Sent to Mirto: ");
            Serial.print(medianDistance);
            Serial.println(" mm");
        }
        else
        {
            Serial.println("No valid VL distance");
        }

        sensor_vl53l4cx_sat
            .VL53L4CX_StopMeasurement();

        Serial.println("VL53L4CX stopped");

        vlDone = true;
    }

    float hcDistance =
        getUltrasonicDistanceCM();

    if (hcDistance >= 25.0f &&
        hcDistance <= 80.0f)
    {
        digitalWrite(BRIDGE_OUT, HIGH);
    }
    else
    {
        digitalWrite(BRIDGE_OUT, LOW);
    }

    delay(100);
}