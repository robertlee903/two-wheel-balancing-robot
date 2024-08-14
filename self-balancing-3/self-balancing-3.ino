#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>

// Bluetooth setup
SoftwareSerial btSerial(10, 11); // RX, TX pins

// MPU6050 sensor setup
MPU6050 sensor;

// Stepper motor setup (Pin assignments for A4988)
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3); // Step pin, direction pin
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5); // Step pin, direction pin

// Constants for PID control
const float targetAngle = 0; // Target angle for balance
float Kp = 100, Ki = 0.5, Kd = 20;
float integral = 0, previous_error = 0;
unsigned long lastTime;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600); // Initialize Bluetooth communication
  Wire.begin();
  sensor.initialize();

  if (!sensor.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while(1); // Endless loop
  }

  stepper1.setMaxSpeed(10000);
  stepper1.setAcceleration(5000);
  stepper2.setMaxSpeed(10000);
  stepper2.setAcceleration(5000);

  lastTime = millis();
}

void loop() {
  // Read sensor data
  int16_t ax, ay, az;
  sensor.getAcceleration(&ax, &ay, &az);
  
  // Calculate tilt angle from accelerometer
  float angle = atan2(ax, az) * 180.0 / PI;
  Serial.println(angle);

  // PID controller calculations
  float error = angle - targetAngle;
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - lastTime) / 1000.0; // Time in seconds

  integral += error * elapsedTime;
  float derivative = (error - previous_error) / elapsedTime;
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Motor control based on PID output
  stepper1.move(output);
  stepper2.move(-output);

  // Prepare for next iteration
  previous_error = error;
  lastTime = currentTime;

  // Execute motor movements
  stepper1.run();
  stepper2.run();

  // Optional: Bluetooth command processing
  if (btSerial.available()) {
    char cmd = btSerial.read();
    // Implement your command processing logic here
  }
}