#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include "PID.h"

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

// Pins
#define batteryLowIndicator 13
#define leftPulse           6
#define leftDirection       5
#define rightPulse          8
#define rightDirection      7

// PID Paramaters
float pidP = 10, pidI = 1.5, pidD = 2;
#define pidDB     5
#define pidOPMin  -400
#define pidOPMax  400

// Dynamic Paramaters
float turnSpeed = 30;
float maxSpeed = 150;

// State Engine
int robotState;
#define Starting  0
#define Balancing 1

// Battery Monitoring
int   batteryVoltage;
bool  lowBattery;

// Bluetooth Comms
char state;
int   receivedCounter;

// Loop Timing
unsigned long loopTimer;
unsigned long reportCounter;

// Control Variables
// float angle;
float outputLeft;
float outputRight;

int leftMotor;
int throttleLeftMotor;
int throttleCounterLeftMotor;
int throttleLeftMotorMemory;

int rightMotor;
int throttleRightMotor;
int throttleCounterRightMotor;
int throttleRightMotorMemory;

// MPU6050 sensor setup
MPU6050 mpu;

PID pid;

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

//******************
// SETUP
//******************

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

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while(1); // Endless loop
  }

  pinMode(batteryLowIndicator, OUTPUT);
  // Setup A4988 control pins
  pinMode(leftPulse, OUTPUT);
  pinMode(leftDirection, OUTPUT);
  pinMode(rightPulse, OUTPUT);
  pinMode(rightDirection, OUTPUT);
  
  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  // Setup PID
  pid.reset();
  pid.setTuningParameters(pidP, pidI, pidD);
  pid.setDeadband(pidDB);
  pid.setOutputLimited(pidOPMin, pidOPMax);
 
  robotState = Starting;
  loopTimer = micros() + 4000;
}


//******************
// LOOP
//******************

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
    String data = Serial.readStringUntil('\n');  // Read the data until newline
    char command = data.charAt(0);  // Get the first character

    if (command == 'P' | command == 'I' || command == 'D') {
      // Send strings like “P2.5\n”, “I0.1\n”, “D0.01\n” over the serial port to adjust Kp, Ki, and Kd respectively
      double value = data.substring(1).toDouble();  // Convert the rest of the string to double
      if (command == 'P') {
        pidP = value;
      } else if (command == 'I') {
        pidI = value;
      } else if (command == 'D') {
        pidD = value;
      }
      Serial.print("KP: ");
      Serial.print(pidP);
      Serial.print("\t KI: ");
      Serial.print(pidI);
      Serial.print("\t KD: ");
      Serial.println(pidD);
      pid.setTuningParameters(pidP, pidI, pidD);
    }

    if (command == 'S') {
      state = data.charAt(1);
      receivedCounter = 0;
      Serial.print("State: ");
      Serial.println(state);
    }
  }

  if (receivedCounter <= 25) {
    receivedCounter++;
  }
  else {
    state = 'a';
  }
  
  // batteryVoltage = map(analogRead(0),0,1023,0,1250);


  // if (batteryVoltage < 1070 && batteryVoltage > 1050) {
  //   Serial.println("Battery Warning");
  // }
  
  // if (batteryVoltage < 1050 && batteryVoltage > 800) {
  //   digitalWrite(batteryLowIndicator, HIGH);
  //   lowBattery = true;  
  //   Serial.println("Battery Low");
  // }

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // Read sensor data
    // int16_t ax, ay, az;
    // mpu.getAcceleration(&ax, &ay, &az);
    
    // Calculate tilt angle from accelerometer
    // float angle = atan2(ax, az) * 180.0 / PI;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
    float angle = ypr[1] * 180/M_PI; // Get Pitch

    // State Machine
    switch (robotState) {
      case Starting:
      {
        // angle = mpu.getAccelAngle();
        if (angle > -0.5 && angle < 0.5) {
          // mpu.setTiltAngle(angle);
          robotState = Balancing;
        }
        break;
      }
      case Balancing:
      {
        // angle = mpu.getTiltAngle();
        if (angle > 30 || angle < -30 || lowBattery) {
          robotState = Starting;
          pid.reset();
        }
        else {
          // Process PID
          pid.processVariable = angle;
          pid.calculate();
        }
        break;
      }
    }

    // Calculate control signals
    outputLeft = pid.output;
    outputRight = pid.output;

    // Turn Left
    if (state == 'l'){
      outputLeft += turnSpeed;
      outputRight -= turnSpeed;
    }

      // Turn Right
    if (state == 'r'){
      outputLeft -= turnSpeed;
      outputRight += turnSpeed;
    }

      // Forward
    if (state == 'f'){
      if (pid.setpoint > -2.5) {
        pid.setpoint -= 0.05;
      }
      if (pid.output > -maxSpeed) {
        pid.setpoint -= 0.005;
      }
    }
    
      // Reverse
    if (state == 'b'){
      if (pid.setpoint < 2.5) {
        pid.setpoint += 0.05;
      }
      if (pid.output < maxSpeed) {
        pid.setpoint += 0.005;
      }
    }

      // Not forward or reverse
    if (state == 's') {
      if (pid.setpoint > 0.5) {
        pid.setpoint -= 0.05; 
      }
      else if (pid.setpoint < -0.5) {
        pid.setpoint += 0.05;
      }
      else {
        pid.setpoint = 0;
      }
    }

    if (pid.setpoint == 0) {
      if (pid.output < 0) {
        pid.selfBalanceSetpoint += 0.0015;
      }
      if (pid.output > 0) {
        pid.selfBalanceSetpoint -= 0.0015;
      }
    }

    // Motor Calculations

    // Compensate for the non-linear behaviour of the stepper motors
    if (outputLeft > 0) {
      outputLeft = 405 - (1/(outputLeft + 9)) * 5500;
    }
    else if (outputLeft < 0) {
      outputLeft = -405 - (1/(outputLeft - 9)) * 5500;
    }
    
    if (outputRight > 0) {
      outputRight = 405 - (1/(outputRight + 9)) * 5500;
    }
    else if (outputRight < 0) {
      outputRight = -405 - (1/(outputRight - 9)) * 5500;
    }

    // Calculate the needed pulse time for the left and right stepper motor controllers
    if (outputLeft > 0) {
      leftMotor = 400 - outputLeft;
    }
    else if (outputLeft < 0) {
      leftMotor = -400 - outputLeft;
    }
    else {
      leftMotor = 0;
    }

    if (outputRight > 0) {
      rightMotor = 400 - outputRight;
    }
    else if (outputRight < 0) {
      rightMotor = -400 - outputRight;
    }
    else {
      rightMotor = 0;
    }

    // Copy for interrupt to use
    throttleLeftMotor = leftMotor;
    throttleRightMotor = rightMotor;

    if (reportCounter > 50) {
      reportCounter = 0;
    
      Serial.print("angle: ");
      Serial.print(angle);
      Serial.print(", \t selfBalanceSetpoint: ");
      Serial.print(pid.selfBalanceSetpoint);
      Serial.print(", \t output: ");
      Serial.print(pid.output);
      Serial.print(", \t throttleLeftMotor: ");
      Serial.print(throttleLeftMotor); 
      Serial.print(", \t throttleRightMotor: ");
      Serial.print(throttleRightMotor);
      Serial.println("");
    }
  }
 
  // Delay 4 milliseconds
  while(loopTimer > micros());
  loopTimer += 4000;
  reportCounter ++;
}

//******************
// INTERUPT HANDLERS
//******************

ISR(TIMER2_COMPA_vect) {
  // Left motor pulse calculations
  throttleCounterLeftMotor ++;                                //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttleCounterLeftMotor > throttleLeftMotorMemory) {   //If the number of loops is larger then the throttle_left_motor_memory variable
    throttleCounterLeftMotor = 0;                             //Reset the throttle_counter_left_motor variable
    throttleLeftMotorMemory = throttleLeftMotor;              //Load the next throttle_left_motor variable
    if (throttleLeftMotorMemory < 0) {                        //If the throttle_left_motor_memory is negative
      PORTD |= 0b00100000;                                     //Set output 5 low to reverse the direction of the stepper controller
      throttleLeftMotorMemory *= -1;                          //Invert the throttle_left_motor_memory variable
    }
    else {
      PORTD &= 0b11011111;                                    // Set output 5 high for a forward direction of the stepper motor
    }
  }
  else if (throttleCounterLeftMotor == 1) {
    PORTD |= 0b01000000;                                      //Set output 6 high to create a pulse for the stepper controller
  }
  else if (throttleCounterLeftMotor == 2) {
    PORTD &= 0b10111111;                                      //Set output 6 low because the pulse only has to last for 20us 
  }

  // Right motor pulse calculations
  throttleCounterRightMotor ++;                               //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttleCounterRightMotor > throttleRightMotorMemory) { //If the number of loops is larger then the throttle_right_motor_memory variable
    throttleCounterRightMotor = 0;                            //Reset the throttle_counter_right_motor variable
    throttleRightMotorMemory = throttleRightMotor;            //Load the next throttle_right_motor variable
    if (throttleRightMotorMemory < 0) {                       //If the throttle_right_motor_memory is negative
      PORTD &= 0b01111111;                                    //Set output 7 low to reverse the direction of the stepper controller
      throttleRightMotorMemory *= -1;                         //Invert the throttle_right_motor_memory variable
    }
    else {
      PORTD |= 0b10000000;                                    //Set output 7 high for a forward direction of the stepper motor
    }
  }
  else if (throttleCounterRightMotor == 1) {
    PORTB |= 0b00000001;                                      //Set output 8 high to create a pulse for the stepper controller
  }
  else if (throttleCounterRightMotor == 2) {
    PORTB &= 0b11111110;                                      //Set output 8 low because the pulse only has to last for 20us
  }
}

