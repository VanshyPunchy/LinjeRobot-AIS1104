#include <Arduino.h>

void setup() {
// write your initialization code here
    Serial.begin(9600);
}

void loop() {
// write your code here
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensor_values);

    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    for (uint8_t i = 0; i < sensor_count; i++)
    {
        Serial.print(sensor_values[i]);
        Serial.print('\t');
    }
    Serial.println(position);

    delay(250);
}

/*
 *      Forventet output:
 *      Starter QTRs Sensor-kalibrering.
 *      1
 *      2
 *      3
 *      ...
 *      399
 *      Ferdig med kalibrering.
 *
 *      392 456 429 489 389 289         // Dette tallet vil vi skal være lavt (det er laveste målingene
 *                                      // under kalibreringen
 *      2450 2500 2500 2500 2500 2499   // dette ønsker vi skal være på så nærme 2500 som mulig (høyeste
 *                                      // målingene under kalibreringen)
 *      0 0 0 0 0 0 0
 *      800 90 0 0 0 0 300 <----- de 6 første verdiene er hver sensor sin data (over svar linje betyr høy verdi)
 *      0 0 102 708 302 80 2000 <--- den 7 verdien er posisjonen verdi (hvor den svarte linjen er på sensoren)
 */