#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "PID_v1.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
bool isCalibrated = false; // Calibration status flag

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Define pins for A4988
#define DIR_PIN_A 5
#define STEP_PIN_A 6
#define DIR_PIN_B 7
#define STEP_PIN_B 8

// Runtime variables
bool initialised = false; // Is the balancing system on?

// PID Variables
double setPoint = 0;  // Desired angle, adjust based on your setup
double input, output;
double Kp = 45.0, Ki = 1.0, Kd = 1.0; // PID coefficients, tune these
#define MAX_ACCEL 3200

// PID Controller
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool isInStablePosition() {
  // Function to determine if the current position is stable for calibration
  // This might check for minimal movement over a certain period
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Example threshold check for stability
  return (abs(ax) < 1000 && abs(ay) < 1000 && abs(az) > 16384 - 1000 && abs(az) < 16384 + 1000);
}

void calibrateMPU() {
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(79);
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(-13);
    mpu.setZAccelOffset(971);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setWireTimeout(3000, true); // timeout value in uSec
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
  
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Setup A4988 control pins
    pinMode(DIR_PIN_A, OUTPUT);
    pinMode(STEP_PIN_A, OUTPUT);
    pinMode(DIR_PIN_B, OUTPUT);
    pinMode(STEP_PIN_B, OUTPUT);
  
    pid.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL); // Set output limits to allow full reverse
    pid.SetSampleTime(100); // 10ms
    pid.SetMode(AUTOMATIC); // Ensure the PID controller is turned on
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // Check if the MPU is in a stable orientation suitable for calibration
    if (!isCalibrated) {
        if (isInStablePosition()) {
            calibrateMPU();
            isCalibrated = true;
            Serial.println("Sensor calibrated successfully!");
        }
    }

    // if programming failed or the robot fallen down, don't try to do anything
    if (!dmpReady || !isCalibrated) return;
    
    if (Serial.available() > 0) {
        // Send strings like “P2.5\n”, “I0.1\n”, “D0.01\n” over the serial port to adjust Kp, Ki, and Kd respectively
        String data = Serial.readStringUntil('\n');  // Read the data until newline
        char command = data.charAt(0);  // Get the first character
        double value = data.substring(1).toDouble();  // Convert the rest of the string to double

        switch (command) {
        case 'P':  // If the command is 'P', set Kp
            Kp = value;
            break;
        case 'I':  // If the command is 'I', set Ki
            Ki = value;
            break;
        case 'D':  // If the command is 'D', set Kd
            Kd = value;
            break;
        }
        Serial.print("New Kp: ");
        Serial.print(Kp);
        Serial.print("\t Ki: ");
        Serial.print(Ki);
        Serial.print("\t Kd: ");
        Serial.println(Kd);
        pid.SetTunings(Kp, Ki, Kd);
    }

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180/M_PI; // Get Pitch
        float inputAbs = abs(input);

        if (initialised && (inputAbs > 50)) {
            Serial.println("--- Balancing System Stopped ---");
            initialised = false;
            return;
        }

        if (!initialised && (inputAbs < setPoint + 0.5)) {
            Serial.println("--- Balancing System Active ---");
            initialised = true;
        }

        if (initialised) {
            // if (inputAbs < 0.2) return;

            // Compute PID output
            pid.Compute();

            // Calculate speed and steps based on the PID output
            int steps = abs(int(output)); // Convert PID output to step count
            int speed = constrain(steps, 0, 400); // Limit speed to a maximum for safety

            Serial.print("Input: ");
            Serial.print(input);
            Serial.print("\t Output: ");
            Serial.print(output);
            Serial.print("\t Speed: ");
            Serial.print(speed);
            Serial.print("\t Steps: ");
            Serial.println(steps);

            if (steps < 10) return;

            // Determine direction and magnitude
            bool dirA = output > 0;
            bool dirB = !dirA;

            digitalWrite(DIR_PIN_A, dirA ? HIGH : LOW);
            digitalWrite(DIR_PIN_B, dirB ? HIGH : LOW);

            // Apply steps based on the PID output
            for (int i = 0; i < steps; i++) {
                digitalWrite(STEP_PIN_A, HIGH);
                // delayMicroseconds(10);  // Slight delay might help with synchronization
                digitalWrite(STEP_PIN_B, HIGH);
                delayMicroseconds(500 - speed); // Adjust pulse width based on speed
                digitalWrite(STEP_PIN_A, LOW);
                // delayMicroseconds(10);  // Maintain overall timing
                digitalWrite(STEP_PIN_B, LOW);
                delayMicroseconds(500 - speed); // Adjust pulse width based on speed
            }
        }
    }

    delay(100);
}