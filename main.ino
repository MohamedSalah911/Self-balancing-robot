#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIN_ABS_SPEED 80
#define MAX_TURN_SPEED 150 // Cap the turning speed
#define OBSTACLE_DISTANCE 20 // Minimum distance to detect obstacle (in cm)
#define TURN_EXTENSION_DURATION 70 // Extra time (ms) to keep turning after avoiding obstacle
#define SPEED_RAMP_STEP 20 // Step size to increase/decrease speed
#define SPEED_RAMP_INTERVAL 50 // Interval (ms) for speed ramping
#define OBSTACLE_COOLDOWN 500 // Cooldown period after turning (in ms)

// Ultrasonic Sensor Pins
#define trigPin 3
#define echoPin 2

MPU6050 mpu;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];        

// PID
double setpoint = 0; // Initial setpoint
double input, output;
double Kp = 4;
double Kd = 0.4; 
double Ki = 105;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.5;

int ENA = 10;
int IN1 = 5;
int IN2 = 4;
int IN3 = 7;
int IN4 = 6;
int ENB = 11;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool measuringDistance = false;

bool turningLeft = false;         // Track if turning left
bool rampingSpeed = false;        // Track if ramping speed after turning
unsigned long turnStartTime = 0; // Track when turning started
unsigned long lastObstacleTime = 0; // Time of the last obstacle encounter

double currentSpeed = MIN_ABS_SPEED; // Current speed for ramping
unsigned long lastSpeedUpdateTime = 0; // Last time speed was updated for ramping

void triggerUltrasonic() {
    if (!measuringDistance) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        measuringDistance = true;
        echoStartTime = micros();
    }
}

float getDistance() {
    if (measuringDistance && echoEndTime > echoStartTime) {
        unsigned long duration = echoEndTime - echoStartTime;
        measuringDistance = false; // Reset for next measurement
        return (duration * 0.034) / 2.0;
    }
    return -1; // Distance not ready
}

void echoISR() {
    if (digitalRead(echoPin) == HIGH) {
        echoStartTime = micros();
    } else {
        echoEndTime = micros();
    }
}

void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // MPU-6050 Calibration offsets
    mpu.setXGyroOffset(46);
    mpu.setYGyroOffset(15);
    mpu.setZGyroOffset(32);
    mpu.setXAccelOffset(-1462);
    mpu.setYAccelOffset(226);
    mpu.setZAccelOffset(1781);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("DMP ready!"));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void rampSpeed(double targetSpeed) {
    unsigned long currentTime = millis();
    if (currentTime - lastSpeedUpdateTime >= SPEED_RAMP_INTERVAL) {
        if (currentSpeed < targetSpeed) {
            currentSpeed = min(currentSpeed + SPEED_RAMP_STEP, targetSpeed);
        } else if (currentSpeed > targetSpeed) {
            currentSpeed = max(currentSpeed - SPEED_RAMP_STEP, targetSpeed);
        }
        lastSpeedUpdateTime = currentTime;

        if (currentSpeed == targetSpeed) {
            rampingSpeed = false; // Stop ramping once the target speed is reached
        }
    }
}

void loop() {
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        double input = ypr[2] * 180 * 1.6 / M_PI;
        pid.Compute();

        // Non-blocking ultrasonic distance measurement
        triggerUltrasonic();
        float distance = getDistance();

        // Obstacle avoidance logic
        if (distance > 0) { // Valid distance measurement
            Serial.print("Distance: ");
            Serial.println(distance);

            if (distance < OBSTACLE_DISTANCE && millis() - lastObstacleTime > OBSTACLE_COOLDOWN) {
                turningLeft = true;                // Start turning
                rampingSpeed = false;              // Disable ramping during turning
                currentSpeed = MIN_ABS_SPEED;      // Reset speed to minimum
                lastObstacleTime = millis();       // Record time of obstacle detection
                turnStartTime = millis();          // Record start time of turn
                Serial.println("Obstacle detected! Starting turn left.");
            }
        }

        // Continue turning with balancing
        if (turningLeft) {
            if (millis() - turnStartTime <= TURN_EXTENSION_DURATION) {
                motorController.turnLeft(MAX_TURN_SPEED, false); 
                Serial.println("Turning left.");
                return; // Skip normal movement while turning
            } else {
                turningLeft = false; // Stop turning after extension
                rampingSpeed = true; // Enable ramping after turning
                Serial.println("Turning complete. Resuming normal operation.");
            }
        }

        // Ramping speed after turning
        if (rampingSpeed) {
            rampSpeed(MIN_ABS_SPEED + abs(output));
        }

        // Normal balancing or movement
        motorController.move(output, rampingSpeed ? currentSpeed : MIN_ABS_SPEED + abs(output));

        // Debugging
        Serial.print("Setpoint: "); Serial.print(setpoint); Serial.print(" | ");
        Serial.print("PID Output: "); Serial.print(output); Serial.print(" | ");
        Serial.print("Current Speed: "); Serial.println(currentSpeed);
    }
}
