#include <Arduino.h>
#include <QTRSensors.h> // for IR sensoren

//Motor class
class Motor {
    //Hver motor trenger 3 pins, in1 og in2 er fremover/bakover mens pwm kontroller fart
    int pwmPin, in1, in2;

public:
    Motor(int pwmPin, int in1, int in2) {
        in1 = pin1;
        in2 = pin2;
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
    }
};

class MotorDriver {
    //standby Pin, MÅ være høy for at motorer skal kjøre
    int stbyPin;
    //To motor objekter fra motor klassen over
    Motor leftMotor, rightMotor;

public:
    //Member initializer list, kopierer to motor objekter in til dette MotorDriver objektet
    MotorDriver(int stbyPin, Motor l,Motor r) : leftMotor(l), rightMotor(r) {
        stbyPin = stby;
        pinMode(stbyPin, OUTPUT);
        digitalWrite(stbyPin, HIGH); //Skru på driver
    }
    void move(int leftSpeed, int rightSpeed) {
        leftMotor.drive(leftSpeed);
        rightMotor.drive(rightSpeed);
    }
};
void setup() {
// write your initialization code here
}

void loop() {
// write your code here
}