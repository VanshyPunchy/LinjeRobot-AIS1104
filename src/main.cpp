#include <Arduino.h>
#include <QTRSensors.h>
// #include <vector>  // ikke brukt

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

// --- Konstanter og oppsett ---
const uint8_t SENSOR_COUNT = 6;
const int16_t CENTER       = 2500;   // QTR readLineBlack: 0..5000
const int16_t BASE_SPEED   = 180;    // 0..255 (start lavt, øk når det funker)
const int16_t DEAD_BAND    = 400;    // rett frem når |error| <= dette
const int16_t MAX_PWM      = 255;

// PD-tuning (start her; finjustér på banen)
const float KP = 0.12f;   // proporsjonal
const float KD = 0.75f;   // derivasjon (stabiliserer sving)

uint16_t sensor_values[SENSOR_COUNT];

// TB6612FNG til Arduino-pins (som du hadde)
Motor leftMotor(10, 8, 9);     // PWMA=10, AIN1=8, AIN2=9
Motor rightMotor(11, 12, 13);  // PWMB=11, BIN1=12, BIN2=13
MotorDriver driver(7, leftMotor, rightMotor); // STBY=7

// QTR-parametre
const uint8_t QTR_PINS[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5};
const uint8_t EMITTER_PIN = 2;
const uint16_t QTR_TIMEOUT_US = 2500;
const uint8_t QTR_SAMPLES = 4;

// PD-tilstand
int16_t lastError = 0;
uint32_t lastMs = 0;

// Hjelp: begrens og returner int
static inline int16_t clampi(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

void setup() {
    Serial.begin(9600);
    driver.begin();

    qtr.setTypeRC();
    qtr.setSensorPins(QTR_PINS, SENSOR_COUNT);
    qtr.setEmitterPin(EMITTER_PIN);
    qtr.setTimeout(QTR_TIMEOUT_US);
    qtr.setSamplesPerSensor(QTR_SAMPLES);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Starter QTR-sensor-kalibrering.");
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate(); // flytt sensoren over lys/mørk
        digitalWrite(LED_BUILTIN, (i & 1) ? HIGH : LOW);
        delay(5);
    }
    Serial.println("Ferdig med kalibrering.");
    digitalWrite(LED_BUILTIN, LOW);

    // (valgfritt) skriv min/max for å verifisere
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(' ');
    }
    Serial.println();
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]); Serial.print(' ');
    }
    Serial.println("\nKlar.");

    lastMs = millis();
}

void loop() {
    // 1) Les posisjon (svart linje på lys bakgrunn)
    uint16_t pos = qtr.readLineBlack(sensor_values); // 0..5000
    int16_t error = CENTER - (int16_t)pos;           // -2500..+2500

    // 2) Enkel heuristikk for "linje mistet" (svak refleks over hele linja)
    //    Sjekk høyeste kalibrerte sensorverdi; veldig lav => sannsynlig tapt
    uint16_t maxVal = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) if (sensor_values[i] > maxVal) maxVal = sensor_values[i];

    // 3) Beregn tidssteg for D-ledd
    uint32_t now = millis();
    float dt = (now - lastMs) / 1000.0f;
    if (dt <= 0) dt = 0.001f; // unngå div 0

    // 4) PD-korreksjon
    float derr = (error - lastError) / dt;      // endring per sekund
    float correctionF = KP * error + KD * derr;

    // Begrens korreksjonen slik at differensialen ikke saturerer fullstendig
    int16_t correction = clampi((int32_t)correctionF, -255, 255);

    // 5) Deadband: rett frem når nesten midt
    int16_t leftCmd, rightCmd;
    if (abs(error) <= DEAD_BAND) {
        leftCmd  = BASE_SPEED;
        rightCmd = BASE_SPEED;
    } else {
        // differensiell styring: positiv error => linje til høyre => sving høyre
        leftCmd  = clampi(BASE_SPEED - correction, 0, MAX_PWM);
        rightCmd = clampi(BASE_SPEED + correction, 0, MAX_PWM);
    }

    // 6) Håndter tapt linje: brems litt og sving mot "sist kjente side"
    if (maxVal < 80) { // terskel 0..1000 (justér ved behov)
        int8_t dir = (lastError >= 0) ? 1 : -1; // husk hvilken side den sist var på
        leftCmd  = clampi(BASE_SPEED - dir * 120, 0, MAX_PWM);
        rightCmd = clampi(BASE_SPEED + dir * 120, 0, MAX_PWM);
    }

    driver.move(leftCmd, rightCmd);

    // 7) Oppdater PD-tilstand
    lastError = error;
    lastMs = now;

    // 8) (valgfritt) debug til Serial Plotter
    // Serial.print(error); Serial.print('\t');
    // Serial.print((int)pos); Serial.print('\t');
    // Serial.print((int)leftCmd); Serial.print('\t');
    // Serial.println((int)rightCmd);

    delay(10);
}
