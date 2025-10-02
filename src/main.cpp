#include <Arduino.h>
#include <QTRSensors.h>

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
        // start i “coast”
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
            // Coast: PWM=0 uansett retning på TB6612FNG
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
    Motor &left, &right;   // legg merke til referanser (ingen kopiering)
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

    void standby(bool enable) {
        digitalWrite(stbyPin, enable ? LOW : HIGH);
    }
};


const int16_t mid_sensor = 2500;   // 0..5000 => midten 2500
const int16_t dead_zone = 600;   // død sone rundt midten
const int16_t base_speed = 200;


const uint8_t sensor_count = 6;
uint16_t sensor_values[sensor_count];

Motor leftMotor(10, 8, 9);  // pwmPin, in1, in2
Motor rightMotor(11, 12, 13); // pwmPin, in1, in2
MotorDriver driver(7, leftMotor, rightMotor);

void setup() {
    Serial.begin(9600);
    driver.begin();

    qtr.setTypeRC();                              // RC-varianten
    qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5}, sensor_count);
    qtr.setEmitterPin(2);
    qtr.setTimeout(2500);
    qtr.setSamplesPerSensor(4);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("Starter QTR-sensor-kalibrering.");
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();    // flytt sensoren over lys/mørk
        Serial.println(i);
        digitalWrite(LED_BUILTIN, LOW);
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
    int16_t error = (int16_t)position - mid_sensor;

    if (abs(error) <= dead_zone) {
        driver.move(base_speed, base_speed);
    } else if (error < 0) {
        driver.move(base_speed - (base_speed * abs(error) / mid_sensor),
                    base_speed + (base_speed * abs(error) / mid_sensor));
    } else {
        driver.move(base_speed + (base_speed * abs(error) / mid_sensor),
                    base_speed - (base_speed * abs(error) / mid_sensor));
    }

    for (uint8_t i = 0; i < sensor_count; i++) {
        Serial.print(sensor_values[i]); Serial.print('\t');
    }
    Serial.println(position);

    delay(50);
}
