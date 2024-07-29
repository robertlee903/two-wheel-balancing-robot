/* * * * * * * * * * * * * * * * * * * * * *
 * SELF-BALANCING ROBOT
 * =========================================
 * Here are some hints when you try to use this code:
 *
 *  > Ensure pin-mapping is correct for your robot (line 54)
 *  > Ensure calibration values are correct for your sensor (line 181)
 *  > Uncomment (line 700) in order to see if your sensor is working
 *  > Play with your PID values on (line 93)
 *  > Ensure that your left & right motors aren't inverted (line 355)
 *  > Confirm whether you want the Pitch ypr[1] or Roll ypr[2] sensor readings!
 * * * * * * * * * * * * * * * * * * * * * */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

// Local includes
#include "pid_motor_speed.h"

// Specific I2C addresses may be passed as a parameter here
MPU6050 mpu;        			      // Default: AD0 low = 0x68


// Define the pin-mapping
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define DIR_A 7                 // Direction Pin, Motor A
#define STEP_A 8                // PWM, Motor A (Left Motor)
#define DIR_B 5                 // Direction Pin, Motor B
#define STEP_B 6                // PWM, Motor B (Right Motor)

// Max PWM parameters
#define MAX_TURN 30


// MPU Control/Status
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
bool dmpReady = false;         	// Set true if DMP init was successful
uint8_t devStatus;              // Return status after device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer


// Orientation/Motion
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Quaternion q;                   // [w, x, y, z]       Quaternion Container
VectorFloat gravity;           	// [x, y, z]            Gravity Vector
int16_t gyro[3];               	// [x, y, z]            Gyro Vector
float ypr[3];                   // [yaw, pitch, roll]   Yaw/Pitch/Roll & gravity vector


// For PID Controller
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
float Kp = 1.0;                 // (P)roportional Tuning Parameter
float Ki = 0.01;					      // (I)ntegral Tuning Parameter        
float Kd = 0.05;					      // (D)erivative Tuning Parameter       
float lastroll;                 // Keeps track of error over time
float iTerm = 0, dTerm = 0;     // Used to accumulate error (integral)
float targetAngle = -0.046;    	// Can be adjusted according to centre of gravity
float prevAngle = 0;
int maxPID = 200;               // Max number of steps per cycle or max delay

// You can Turn off YAW control, by setting
// the Tp and Td constants below to 0.
float Tp = 0.6;        			    // Yaw Proportional Tuning Parameter
float Td = 0.1;				        	// Yaw Derivative Tuning Parameter
float targetYaw = 0.204;        // Used to maintain the robot's yaw
float lastYawError = 0;


// Runtime variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// int modeSelect = 0;             // System Mode (0 = off, 1 = on)
bool initialised = false;        // Is the balancing system on?

char inchar = 0;                // Hold any incoming characters

bool newCalibration = false;	  // If set TRUE, the target angles are recalibrated


// Variables used for timing control
// Aim is 10ms per cycle (100Hz)
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define STD_LOOP_TIME 9

unsigned long loopStartTime = 0;
unsigned long lastTime;         // Time since PID was called last (should be ~10ms)

// 0 = Off, 1 = On
int modes = 0;

// Motor variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int16_t motor_1_speed;
int16_t motor_2_speed;
int16_t speed_period_m1;
int16_t speed_period_m2;
uint8_t running_mode;


// ------------------------------------------------------------------
// 					      MPU SETUP
// ------------------------------------------------------------------

void setup_mpu() {
  mpu.initialize();
  Serial.println("Testing MPU connection:");

  Serial.println(mpu.testConnection() ? "> MPU6050 connection successful" : "> MPU6050 connection failed");
  Serial.println("Initialising DMP");
  devStatus = mpu.dmpInitialize();

  /* * * * * * * * * * * * * * * * * * * *
  * IMPORTANT!
  * Supply your own MPU6050 offsets here
  * Otherwise robot will not balance properly.
  * * * * * * * * * * * * * * * * * * * */
  mpu.setXGyroOffset(78);
  mpu.setYGyroOffset(-28);
  mpu.setZGyroOffset(-12);
  mpu.setXAccelOffset(-2867);
  mpu.setYAccelOffset(1030);
  mpu.setZAccelOffset(956);

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Serial.println("Enabling DMP");
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP Ready! Let's Proceed.");
    Serial.println("Robot is now ready to balance. Hold the robot steady");
    Serial.println("in a vertical position, and the motors should start.");
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // In case of an error with the DMP
    if(devStatus == 1) Serial.println("> Initial Memory Load Failed");
    else if (devStatus == 2) Serial.println("> DMP Configuration Updates Failed");
  }
}


// ------------------------------------------------------------------
// 					      INITIAL SETUP
// ------------------------------------------------------------------

void setup() {
  Wire.begin();

  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure Motor I/O
  pinMode(DIR_A, OUTPUT);      // Left Motor Direction
  pinMode(DIR_B, OUTPUT);      // Right Motor Direction
  pinMode(STEP_A, OUTPUT);     // Left Motor Step
  pinMode(STEP_B, OUTPUT);     // Right Motor Step

  // Initialize MPU6050
  setup_mpu();

  // setup timer
  // setup_timer(60);

  // PID setup
  setup_pid();
}



// -------------------------------------------------------------------
// 			 PID CONTROLLER
// -------------------------------------------------------------------

// float PID(float currentAngle) {
//   // Calculate the time since function was last called
//   float thisTime = millis();
//   float dT = (thisTime - lastTime) / 1000.0; // time in seconds
//   lastTime = thisTime;

//   // Calculate error
//   float error = targetAngle - currentAngle;

//   // Integral term
//   iTerm += error * dT;

//   // Derivative term
//   dTerm = (prevAngle - currentAngle) / dT;
//   prevAngle = currentAngle;

//   // Calculate result
//   float result = (error * Kp) + (iTerm * Ki) + (dTerm * Kd);

//   // Limit result to maximum step change
//   result = constrain(result, -maxPID, maxPID);

//   return result;
// }



// -------------------------------------------------------------------
// 			 YAW CONTROLLER
// -------------------------------------------------------------------

int yawPD(int yawError) {
  // Calculate our PD terms
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  float pTerm = Tp * yawError;
  float dTerm = Td * (yawError - lastYawError) / 10; 

  lastYawError = yawError;

  // Obtain PD output value
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
  int yawPDvalue = int(-pTerm + dTerm);

  // Limit PD value to maximum
  if (yawPDvalue > MAX_TURN) yawPDvalue = MAX_TURN;
  else if (yawPDvalue < -MAX_TURN) yawPDvalue = -MAX_TURN; 

  //Serial.print("Error: ");
  //Serial.print(yawError);
  //Serial.print(" - PD: ");
  //Serial.println(yawPDvalue);
  return yawPDvalue;
}



// -------------------------------------------------------------------
// 			 	MOVEMENT CONTROLLER
// -------------------------------------------------------------------
// This function calculate the PWM output required to keep the robot 
// balanced, to move it back and forth, and to control the yaw.

// void MoveControl(int pidValue, float yaw){
//   // Set both motors to this speed
//   int left_PWM = pidValue;
//   int right_PWM = pidValue;


//   /* YAW CONTROLLER */

//   // Check if turning left or right is faster
//   // -- -- -- -- -- -- -- -- -- -- -- -- -- --
//   int leftTurn, rightTurn;

//   float newYaw = targetYaw;

//   if((yaw > 0) && (newYaw < 0)) {
//     rightTurn = yaw + abs(newYaw);
//     leftTurn = (180 - yaw) + (180 - abs(newYaw));
//   } else if ((yaw < 0) && (newYaw > 0)) {
//     rightTurn = (180 - abs(yaw)) + (180 - newYaw);
//     leftTurn = abs(yaw) + newYaw;
//   } else if (((yaw > 0) && (newYaw > 0)) || ((yaw < 0) && (newYaw < 0))) {
//     rightTurn = newYaw - yaw;

//     if (rightTurn > 0){
//       leftTurn = rightTurn;
//       rightTurn = 360 - leftTurn;
//     } else if (rightTurn < 0){
//       rightTurn = abs(rightTurn);
//       leftTurn = 360 - abs(rightTurn);
//     } else if (rightTurn == 0){
//       rightTurn = leftTurn = 0;
//     }
//   }

//   // Apply yaw PD controller to motor output
//   // -- -- -- -- -- -- -- -- -- -- -- -- -- --
//   if ((leftTurn == 0) && (rightTurn == 0)) {
//     // Do nothing
//   } else if (leftTurn <= rightTurn) {
//     leftTurn = yawPD(leftTurn);
//     left_PWM = left_PWM - leftTurn;
//     right_PWM = right_PWM + leftTurn;
//   } else if (rightTurn < leftTurn){
//     rightTurn = yawPD(rightTurn);
//     left_PWM = left_PWM + rightTurn;
//     right_PWM = right_PWM - rightTurn;
//   }


//   // Limits PID to max motor speed
//   // -- -- -- -- -- -- -- -- -- -- -- -- -- --
//   if (left_PWM > 255) left_PWM = 255;
//   else if (left_PWM < -255) left_PWM = -255; 
//   if (right_PWM > 255) right_PWM = 255;
//   else if (right_PWM < -255) right_PWM = -255; 

//   // Send command to left motor
//   if (left_PWM >= 0) controlMotor(0, 0, int(left_PWM), 500);   	// '0' = Left-motor, '1' = Right-motor
//   else controlMotor(0, 1, (int(left_PWM) * -1), 500);
//   // Send command to right motor
//   if (right_PWM >= 0) controlMotor(1, 1, int(right_PWM), 500); 	// '0' = Forward, '1' = Backward
//   else controlMotor(1, 0, (int(right_PWM) * -1), 500);    
// }



// -------------------------------------------------------------------
// 			 MOTOR CONTROLLER
// -------------------------------------------------------------------

// void stepMotor(int delayBetweenSteps) {
//   // Example stepping function, activate STEP pin of A4988
//   digitalWrite(STEP_A, HIGH);
//   digitalWrite(STEP_B, HIGH);
//   delayMicroseconds(delayBetweenSteps); // pulse width high
//   digitalWrite(STEP_A, LOW);
//   digitalWrite(STEP_B, LOW);
//   delayMicroseconds(delayBetweenSteps); // delay between steps to control speed
// }


// -------------------------------------------------------------------
// 			 READ INPUT FROM SERIAL
// -------------------------------------------------------------------

void readSerial() {
  // Examples:
  // P1.000
  // I1.000
  // D1.000

  // Initiate all of the variables
  // -- -- -- -- -- -- -- -- -- -- -- -- -- --
	int changestate = 0;		  // Which action needs to be taken?
	int no_before = 0;			  // Numbers before decimal point
	int no_after = 0;			    // Numbers after decimal point
	bool minus = false;			  // See if number is negative
	inchar = Serial.read();		// Read incoming data

  if (inchar == 'P') changestate = 1;
  else if (inchar == 'I') changestate = 2;
  else if (inchar == 'D') changestate = 3;

  // Tell robot to calibrate the Centre of Gravity
  else if (inchar == 'G') calibrateTargets();

  // Records all of the incoming data (format: 00.000)
  // And converts the chars into a float number
  if (changestate > 0){
    if (Serial.available() > 0){
      // Is the number negative?
      inchar = Serial.read();
      if(inchar == '-'){
        minus = true;
        inchar = Serial.read();
      }
      no_before = inchar - '0';
      if (Serial.available() > 0) {
        inchar = Serial.read();

        if (inchar != '.'){
          no_before = (no_before * 10) + (inchar - '0');

          if (Serial.available() > 0){
            inchar = Serial.read();
          }
        }

        if (inchar == '.'){
          inchar = Serial.read();
          if (inchar != '0'){
            no_after = (inchar - '0') * 100;
          }

          if (Serial.available() > 0){
            inchar = Serial.read();
            if (inchar != '0'){
              no_after = no_after + ((inchar - '0') * 10);
            }

            if (Serial.available() > 0){
              inchar = Serial.read();
              if (inchar != '0'){
                no_after = no_after + (inchar - '0');
              }
            }
          }
        }
      }

      // Combine the chars into a single float
      float answer = float(no_after) / 1000;
      answer = answer + no_before;
      if (minus) answer = answer * -1;

      // Update the PID constants
      if (changestate == 1){
        Kp = answer;
        Serial.print("P - ");
      } else if (changestate == 2){
        Ki = answer;
        Serial.print("I - ");
      } else if (changestate == 3){ 
        Kd = answer;
        Serial.print("D - ");
      }
      Serial.print("Constant Set: ");
      Serial.println(answer, 3);
    } else {
      changestate = 0;
    }
  }
}



// -------------------------------------------------------------------
// 			 RECALIBRATE TARGET VALUES
// -------------------------------------------------------------------
// Takes a number of readings and gets new values for the target angles.
// Robot must be held upright while this process is being completed.

void calibrateTargets() {
	targetAngle = 0;
	targetYaw = 0;
	
  for(int calibrator = 0; calibrator < 50; calibrator++){
		accelgyroData();
		targetAngle += roll();
		targetYaw += yaw();
		delay(10);
	}
	
	// Set our new value for Roll and Yaw
	targetAngle = targetAngle / 50;
	targetYaw = targetYaw / 50;
	Serial.println("\n");
	Serial.print("Target Roll: ");
	Serial.print(targetAngle, 3);
	Serial.print(", Target Yaw: ");
	Serial.println(targetYaw, 3);
}



// -------------------------------------------------------------------
// 			 GET YAW, PITCH, AND ROLL VALUES
// -------------------------------------------------------------------
// This simply converts the values from the accel-gyro arrays into degrees.

float yaw(){
	return (ypr[0] * 180/M_PI);
}

float pitch(){
	return (ypr[1] * 180/M_PI);
}

float roll(){
	return (ypr[2] * 180/M_PI);
}

float angRate(){
	return -((float)gyro[1]/131.0);
}



// -------------------------------------------------------------------
// 			 GET ACCEL_GYRO DATA
// -------------------------------------------------------------------

void accelgyroData(){
  // Reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // Reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println("Warning - FIFO Overflowing!");

    // otherwise, check for DMP data ready interrupt (this should happen exactly once per loop: 100Hz)
  } else if (mpuIntStatus & 0x02) {
    // Wait for correct available data length, should be less than 1-2ms, if any!
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.resetFIFO();
  }
}



// -------------------------------------------------------------------
// 			 MAIN PROGRAM LOOP
// -------------------------------------------------------------------

#define stepsPerRevolution 200

void loop() {
	// If the "SET" button is pressed
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	// if (newCalibration) {
  //   delay(10000);
	// 	// digitalWrite(LED_1, HIGH);
  //   calibrateTargets();

  //   lastroll = 0;
  //   iTerm = 0;

  //   Serial.println("> Setting new centre of gravity <");

	// 	 delay(250);
  //   mpu.resetFIFO();
  //   newCalibration = false;
  //   // digitalWrite(LED_1, LOW);
	// }

  // Gather data from MPU6050
  accelgyroData();
  float currentAngle = roll(); // actual position from sensor
  Serial.println(currentAngle);
    
  if (!initialised) {
    // Wait until robot is vertical and angular rate is almost zero:
    if ((currentAngle < targetAngle+0.1) && (currentAngle > targetAngle-0.1) && (abs(angRate()) < 0.3)){
      initialised = true;
      lastroll = currentAngle;
      iTerm = 0;
      Serial.println(">>>> Balancing System Active <<<<");
    }
  } else {
    // Stop the system if it has fallen over:
    if ((currentAngle < -45) || (currentAngle > 45)) {
      // Reset runtime variables
      initialised = false;
      lastroll = 0;
      iTerm = 0;
      Serial.println(">>>> Balancing System Stopped <<<<");
      motor_1_speed = motor_2_speed = 0;
      speed_period_m1 = speed_period_m2 = 0;
    } else {
      // A bit of function-ception happening here:
      int16_t motor_accel[2];
      get_pid_motor_speed(motor_accel, currentAngle);

      // we integrate the acceleration
      motor_1_speed += motor_accel[0];
      motor_2_speed -= motor_accel[1];

      //constrain motor speed
      motor_1_speed = constrain(motor_1_speed, -500, 500);
      motor_2_speed = constrain(motor_2_speed, -500, 500);

      // calculate motor period by the function of f(x)=-0.017*x+10.5
      speed_period_m1 = -0.054*abs(motor_1_speed) + 30;
      speed_period_m2 = -0.054*abs(motor_2_speed) + 30;

      // Serial.print("Motor A Accel: ");
      // Serial.print(motor_accel[0]);
      // Serial.print("\t Motor B Accel: ");
      // Serial.println(motor_accel[1]);

      static int16_t counter_m1 = 0;
      static int16_t counter_m2 = 0;

      counter_m1++;
      counter_m2++;

      if (!speed_period_m1 && !speed_period_m2) return;

      digitalWrite(DIR_A, motor_1_speed >= 0 ? LOW : HIGH);
      digitalWrite(DIR_B, motor_2_speed >= 0 ? LOW : HIGH);

      if (counter_m1 >= speed_period_m1) {
        counter_m1 = 0;
        step(MOTOR_1_STEP);
      }

      if (counter_m2 >= speed_period_m2) {
        counter_m2 = 0;
        step(MOTOR_2_STEP);
      }
    }
  }


  if (Serial.available() > 0){    // If new PID values are being sent by the interface
    readSerial();                 // Run the read serial method
  }

  // Call the timing function
  // Very important to keep the response time consistent!
  timekeeper();
}



// -------------------------------------------------------------------
// 			 	TIME KEEPER
// -------------------------------------------------------------------

void timekeeper() {
  // Calculate time since loop began
  float timeChange = millis() - loopStartTime;

  // If the required loop time has not been reached, please wait!

  if (timeChange < STD_LOOP_TIME) {
    delay(STD_LOOP_TIME - timeChange);
  } 


  // Update loop timer variables
  loopStartTime = millis();   
}

//-----------------------------------------
// Timer Setup
// ----------------------------------------
// timer setup, frequency in milliseconds as argument - we are using a divider of 8
// resulting in a 2MHz on a 16MHz cpu
// 40 - 25Khz
// void setup_timer(uint16_t freq) {
//   TCCR1B &= ~(1<<WGM13);
//   TCCR1B |=  (1<<WGM12);
//   TCCR1A &= ~(1<<WGM11); 
//   TCCR1A &= ~(1<<WGM10);

//   // output mode = 00 (disconnected)
//   TCCR1A &= ~(3<<COM1A0); 
//   TCCR1A &= ~(3<<COM1B0); 

//   // Set the timer pre-scaler
//   // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
//   TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
//   OCR1A = freq;   // 25Khz
//   TCNT1 = 0;

//   delay(3000);
//   TIMSK1 |= (1<<OCIE1A);  
// }

// // interrupt function that is called every 25hkz (once every 40 microseconds)
// ISR(TIMER1_COMPA_vect) {
//   static int16_t counter_m1 = 0;
//   static int16_t counter_m2 = 0;

//   counter_m1++;
//   counter_m2++;

//   if (!speed_period_m1 && !speed_period_m2) return;

//   motorDirection(motor_1_speed, motor_2_speed);
  
//   // Serial.print("counter_m1: ");
//   // Serial.print(counter_m1);
//   // Serial.print("\t speed_period_m1: ");
//   // Serial.println(speed_period_m1);

//   if (counter_m1 >= speed_period_m1) {
//     counter_m1 = 0;
//     step(STEP_A);
//   }

//   if (counter_m2 >= speed_period_m2) {
//     counter_m2 = 0;
//     step(STEP_B);
//   }
// }