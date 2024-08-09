#include <stdio.h>

// Arduino libraries
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// PID library
#include <PID_v1.h>


// MPU
MPU6050 mpu;

#define DEBUG 0

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

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

// Motor vars
int16_t motorSpeedA = 0;
int16_t motorSpeedB = 0;
int16_t motorSpeedPeriodA = 0;
int16_t motorSpeedPeriodB = 0;
int16_t motorCounterA = 0;
int16_t motorCounterB = 0;

// Runtime vars
bool initialised = false; // Is the balancing system on?

// PID vars
double setPoint = 0; // Desired angle, adjust based on your setup
double pidInput, pidOutput;
double KP = 13, KI = 1, KD = 0.55;
#define MAX_ACCEL 50

// PID Controller
PID pid_controller_speed(&pidInput, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

// ================================================================
// ===                      PID SETUP                           ===
// ================================================================

void setupPID() {
  pid_controller_speed.SetMode(AUTOMATIC);
  pid_controller_speed.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL);
  pid_controller_speed.SetSampleTime(10); // 10ms
  pid_controller_speed.SetTunings(KP, KI, KD);
}

// ================================================================
// ===                      MPU SETUP                           ===
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
// ===                     Motors Functions                     ===
// ================================================================

// pulse the motor one step
void step(int stepPin){
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(stepPin, LOW);
}

// interrupt function that is called every 25hkz (once every 40 microseconds)
ISR(TIMER1_COMPA_vect) {
  if (!motorSpeedPeriodA && !motorSpeedPeriodB) return;

  motorCounterA++;
  motorCounterB++;
  
  if (motorCounterA >= motorSpeedPeriodA) {
    motorCounterA = 0;
    digitalWrite(DIR_PIN_A, motorSpeedA > 0 ? HIGH : LOW);
    step(STEP_PIN_A);
  }

  if (motorCounterB >= motorSpeedPeriodB) {
    motorCounterB = 0;
    digitalWrite(DIR_PIN_B, motorSpeedB >= 0 ? HIGH : LOW);
    step(STEP_PIN_B);
  }
}

// ================================================================
// ===                       Timer Setup                        ===
// ================================================================

// timer setup, frequency in milliseconds as argument - we are using a divider of 8
// resulting in a 2MHz on a 16MHz cpu
// 40 - 25Khz

void setupTimer(uint16_t freq) {
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
  OCR1A = freq;   // 25Khz
  TCNT1 = 0;

  delay(3000);
  TIMSK1 |= (1<<OCIE1A);  
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); // timeout value in uSec

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

  // Setup A4988 control pins
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(STEP_PIN_A, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);
  pinMode(STEP_PIN_B, OUTPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // setup timer
  setupTimer(60);

  // pid setup
  setupPID();
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
      KP = value;
      break;
    case 'I':  // If the command is 'I', set Ki
      KI = value;
      break;
    case 'D':  // If the command is 'D', set Kd
      KD = value;
      break;
    }
    Serial.print("New KP: ");
    Serial.print(KP);
    Serial.print("\t KI: ");
    Serial.print(KI);
    Serial.print("\t KD: ");
    Serial.println(KD);
    pid_controller_speed.SetTunings(KP, KI, KD);
  }

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float angle = ypr[1] * 180/M_PI; // Get Pitch
    float angleAbs = abs(angle);

    if (initialised && (angleAbs > 35)) {
      Serial.println("--- Balancing System Stopped ---");
      initialised = false;
      motorSpeedA = motorSpeedB = 0;
      motorSpeedPeriodA = motorSpeedPeriodB = 0;
      return;
    }

    if (!initialised && (angleAbs < 1)) {
      Serial.println("--- Balancing System Active ---");
      initialised = true;
    }

    // if we are in an recoverable position
    if (initialised) {
      pidInput = angle;
      // Compute PID output
      pid_controller_speed.Compute();

      // we integrate the acceleration
      motorSpeedA += (int16_t)pidOutput;
      motorSpeedB -= (int16_t)pidOutput;

      //apply steering
      /*
      bool should = millis() - stimer > 5000 && millis() - stimer < 5200;

      if (should) {
        motorSpeedA += 15;
        motorSpeedB -= 15;
      } else {
        motorSpeedB = -motorSpeedA;
      }
      */

      //constrain motor speed
      motorSpeedA = constrain(motorSpeedA, -500, 500);
      motorSpeedB = constrain(motorSpeedB, -500, 500);

      // calculate motor period by the function of f(x)=-0.017*x+10.5
      motorSpeedPeriodA = -0.054*abs(motorSpeedA) + 30;
      motorSpeedPeriodB = -0.054*abs(motorSpeedB) + 30;
    }
  }

  #if DEBUG
    Serial.print("Angle: ");
    Serial.print(pidInput);
    Serial.print("\t M1: ");
    Serial.print(motorSpeedA);
    Serial.print("\t M2: ");
    Serial.println(motorSpeedB);
  #endif
}
