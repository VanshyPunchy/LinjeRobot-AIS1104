#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t sensor_count = 6;
uint16_t sensor_values[sensor_count];

void setup() {
    Serial.begin(9600);

    qtr.setTypeRC();                              // RC-varianten
    qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5}, sensor_count);
    qtr.setEmitterPin(2);
    qtr.setTimeout(2500);                         // matcher forventet maks ≈ 2500
    qtr.setSamplesPerSensor(4);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("Starter QTR-sensor-kalibrering.");
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();    // flytt sensoren over lys/mørk
        Serial.println(i);
        delay(5);
    }
    Serial.println("Ferdig med kalibrering.");
    digitalWrite(LED_BUILTIN, LOW);

    // Skriv ut min/max (emitters på)
    for (uint8_t i = 0; i < sensor_count; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(' ');
    }
    Serial.println();
    for (uint8_t i = 0; i < sensor_count; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]); Serial.print(' ');
    }
    Serial.println(); Serial.println();
    delay(1000);
}

void loop() {
    // For svart linje på lys bakgrunn:
    uint16_t position = qtr.readLineBlack(sensor_values); // fyller sensor_values kalibrert 0–1000

    for (uint8_t i = 0; i < sensor_count; i++) {
        Serial.print(sensor_values[i]); Serial.print('\t');
    }
    Serial.println(position);

    delay(250);
}
