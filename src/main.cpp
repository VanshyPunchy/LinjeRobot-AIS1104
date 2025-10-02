#include <Arduino.h>
#include <QTRSensors.h>
#include <vector>

QTRSensors qtr;

class Motor {
    uint8_t pwmPin, in1, in2;
public:
    Motor(uint8_t pwmPin, uint8_t pin1, uint8_t pin2)
        : pwmPin(pwmPin), in1(pin1), in2(pin2) {}

    void begin() {
        pinMode(pwmPin, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        // start i "coast"
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, 0);
    }

    // speed: -255..255  (>=0 = fremover, <0 = bakover)
    void drive(int speed) {
        bool forward = (speed >= 0);
        int duty = forward ? speed : -speed;
        if (duty > 255) duty = 255;

        if (duty == 0) {
            // Coast (TB6612FNG): IN1=LOW, IN2=LOW, PWM=0
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            analogWrite(pwmPin, 0);
            return;
        }

        if (forward) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
        else         { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }

        analogWrite(pwmPin, duty);
    }
};

class MotorDriver {
    uint8_t stbyPin;
    Motor &left, &right;
public:
    MotorDriver(uint8_t stbyPin, Motor &l, Motor &r)
        : stbyPin(stbyPin), left(l), right(r) {}

    void begin() {
        pinMode(stbyPin, OUTPUT);
        digitalWrite(stbyPin, HIGH); // enable driver
        left.begin();
        right.begin();
    }

    void move(int leftSpeed, int rightSpeed) {
        left.drive(leftSpeed);
        right.drive(rightSpeed);
    }

    void standby(bool enable) { // enable=true -> LOW (standby)
        digitalWrite(stbyPin, enable ? LOW : HIGH);
    }
};

// Konstanter
const int16_t mid_sensor  = 2500; // 0..5000
const int16_t base_speed  = 200;
const uint8_t sensor_count = 6;

uint16_t sensor_values[sensor_count];
int8_t   binVals[sensor_count];   // 0/1 etter terskel

// TB6612FNG til Arduino-pins
Motor leftMotor(10, 9, 8);     // PWMA=10, AIN1=9, AIN2=8
Motor rightMotor(11, 12, 13);  // PWMB=11, BIN1=12, BIN2=13
MotorDriver driver(7, leftMotor, rightMotor); // STBY=7

void setup() {
    Serial.begin(9600);
    driver.begin();

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensor_count);
    qtr.setEmitterPin(2);
    qtr.setTimeout(2500);
    qtr.setSamplesPerSensor(4);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("Starter QTR-sensor-kalibrering.");
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate(); // flytt sensoren over lys/mørk
        digitalWrite(LED_BUILTIN, (i & 1) ? HIGH : LOW);
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
    delay(500);
}

void loop() {
    // Leser posisjon for svart linje på lys bakgrunn (0..5000)
    uint16_t position = qtr.readLineBlack(sensor_values);

    // Lag 0/1 fra kalibrerte verdier (0..1000)
    for (uint8_t i = 0; i < sensor_count; i++) {
        binVals[i] = (sensor_values[i] > 675) ? 1 : 0;
    }

    // Enkel følge-logikk basert på midtsensorene
    if (binVals[2] == 1 && binVals[3] == 1 && binVals[1] == 0 && binVals[4] == 0) {
        driver.move(base_speed, base_speed); // rett fram
    }
    else if (binVals[2] == 1 && binVals[3] == 0) {
        driver.move(base_speed, base_speed - 30); // litt høyre
    }
    else if (binVals[2] == 0 && binVals[3] == 1) {
        driver.move(base_speed - 30, base_speed); // litt venstre
    }
    else if ( (binVals[1] == 1 || binVals[0] == 1) && (binVals[4] == 0 && binVals[5] == 0) ) {
        driver.move(base_speed - 50, base_speed + 50); // sterk venstre
    }
    else if ( (binVals[4] == 1 || binVals[5] == 1) && (binVals[0] == 0 && binVals[1] == 0) ) {
        driver.move(base_speed + 50, base_speed - 50); // sterk høyre
    }
    else {
        // Linje mistet: stopp kort, rygg litt, og prøv igjen
        driver.move(0, 0);
        delay(80);
        driver.move(-80, -80);
        delay(120);
    }

    // Debug-utskrift
    for (uint8_t i = 0; i < sensor_count; i++) {
        Serial.print(sensor_values[i]); Serial.print('\t');
    }
    Serial.print("pos="); Serial.println(position);

    delay(30);
}
