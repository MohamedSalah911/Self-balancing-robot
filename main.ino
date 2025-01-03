#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIN_ABS_SPEED 80

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID
double setpoint = 0.0;  // Target angle is 0 (upright position)
double input, output;
double Kp =4; 
double Kd = 1;        // Proportional to angular velocity feedback
double Ki = 100;
double KdVelocity = 0.1; // Scaling factor for angular velocity
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.5;

// MOTOR CONTROLLER
int ENA = 10;
int IN1 = 5;
int IN2 = 4;
int IN3 = 7;
int IN4 = 6;
int ENB = 11;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
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

void loop() {
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Extract angular velocity (roll rate) from gyroscope
        double rollAngle = ypr[2] * 180 *1.6 / M_PI;  // Roll angle in degrees
        double angularVelocity = mpu.getRotationZ() / 131.0;  // Gyro roll rate in degrees/second

        // Combine roll angle and angular velocity for PID input
        input = rollAngle + angularVelocity * KdVelocity;

        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);

        // Debugging information
        Serial.print("Roll Angle: "); Serial.print(rollAngle); Serial.print(" | ");
        Serial.print("Angular Velocity: "); Serial.print(angularVelocity); Serial.print(" | ");
        Serial.print("PID Output: "); Serial.println(output);
    }
}