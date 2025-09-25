#include <Arduino.h>
#include <QTRSensors.h> // kan være denne ikke funker ordentlig

class Motor {
    //Hver motor trenger 3 pins, in1 og in2 er fremover/bakover mens pwm kontroller fart
    int pwmPin, in1, in2;

public:
    Motor(int pwmPin, int pin1, int pin2) : pwmPin(pwmPin), in1(pin1), in2(pin2) {

        pinMode(pwmPin, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
    }
    //Spinner motoren i en fast hastighet
    void drive(int speed) {
        //Hvis motor fart er større enn eller lik 0 spin framover
        if (speed >= 0) {
            //Setter motorfart høy
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
        }
        else{
            //Setter motorfart lav
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            // Slik at fart holder seg innenfor rekkevidden 0-255
            speed = -speed;
        }
        analogWrite(pwmPin, speed);
    }
};

class MotorDriver {
    //standby Pin, MÅ være høy for at motorer skal kjøre
    int stbyPin;
    //To motor objekter fra motor klassen over
    Motor leftMotor, rightMotor;

public:
    //Member initializer list, kopierer to motor objekter in til dette MotorDriver objektet
    MotorDriver(int stbyPin, Motor l,Motor r) : stbyPin(stbyPin), leftMotor(l), rightMotor(r) {
        pinMode(stbyPin, OUTPUT);
        digitalWrite(stbyPin, HIGH); //Skru på driver
    }
    void move(int leftSpeed, int rightSpeed) {
        leftMotor.drive(leftSpeed);
        rightMotor.drive(rightSpeed);
    }
};
// koden er tatt fra QTRSensors sin eksempeloppgave

QTRSensors qtr;

const uint8_t sensor_count = 6;
uint16_t sensor_values[sensor_count];

Motor leftMotor(10,8,9);
Motor rightMotor(11,12,13);
MotorDriver driver(7, leftMotor, rightMotor);


//PID
float Kp = 0.15;
float Ki = 0.0;
float Kd = 0.2;

int lastError = 0;
int integral = 0;
const int baseSpeed = 50;

void setup() {
// write your initialization code here
    Serial.begin(9600);

    qtr.setTypeRC(); // her kunne man også ha brukt setTypeAnalog() men RC (Resistor-Capacitor) skal visst være raskere
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensor_count); // bruk disse pinsene på arduinoen
    qtr.setEmitterPin(2);


    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    Serial.println("Starter QTRs Sensor-kalibrering.");
    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 400; i++)
    {
        Serial.println(i);
        qtr.calibrate();
        delay(5);
    }

    Serial.println("Ferdig med kalibrering.");
    digitalWrite(LED_BUILTIN, LOW); // signalisere at vi er ferdig å kalibrere

    // print the calibration minimum values measured when emitters were on
    Serial.begin(9600);
    for (uint8_t i = 0; i < sensor_count; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < sensor_count; i++)
    {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(5000);
}

void loop() {
// write your code here
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineBlack(sensor_values);

    //Beregn error fra midten (2500)
    int error = position - 2500;

    //Pid kalkulasjon
    integral += error;
    int derivative = error - lastError;
    int corrcection = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;


    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    for (uint8_t i = 0; i < sensor_count; i++)
    {
        Serial.print(sensor_values[i]);
        Serial.print('\t');
    }
    Serial.println(position);

    delay(100);

    int leftSpeed = baseSpeed + corrcection;
    int rightSpeed = baseSpeed - corrcection;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    driver.move(leftSpeed, rightSpeed);

    delay(10);
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