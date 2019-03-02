/**
   Yurdaer Dalkic & Hadi Deknache
   This sketch is created for an intelligent transportation system
   using car (Chasis kit 2X2) and an ESP32 evelopment board.
   An ultrasonic sensor (HC-SR04) and two IR sensor.
   This car using ultrasonic sensor to detect objects and
   IR sensors for to fallow a specific route/path.
*/

#include <analogWrite.h>
// Defination of states that will be used for state machine
enum State_enum {STOP, FORWARD, ROTATE_RIGHT, ROTATE_LEFT};
State_enum State = STOP; // Default state

// Defination of digital and analog pins
const int m_left = 25;
const int m_right = 26;
const int ir_left = 14;
const int ir_right = 16;
const int trig_pin = 32;
const int echo_pin = 33;

const int m_speed = 140; // Normal speed of the car. Maximum is 255.
const int m_rotate = 80; // Rotation speed for one wheel. Maximum is 255.

// The booleans will present the status of the sensor results
bool obstacle = false;
bool left_sensor = false;
bool right_sensor = false;

void setup() {
  Serial.begin(9600); //sets serial port for communication
  analogReadResolution(8);
  analogWriteResolution(8);
  //Configurs the pins as INPUT or OUTPUT
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(ir_left, INPUT);
  pinMode(ir_right, INPUT);
  pinMode(m_left, OUTPUT);
  pinMode(m_right, OUTPUT);
  Stop(); // Stop the motors
}

void loop() {
  delay(10); // Add some delays before reading the sensors
  // State machine starts here
  switch (State)
  {
    case STOP:
      Stop(); // Stop the motor
      break;

    case FORWARD:
      MoveForward();
      break;

    case ROTATE_RIGHT:
      MoveRight();
      break;

    case ROTATE_LEFT:
      MoveLeft();
      break;
  }
  ReadSensors(); // Read the sensor values


  if (obstacle) {
    State = STOP; // Next state will be STOP
  }
  // The line is on the left side
  else if (left_sensor && !right_sensor) {
    State = ROTATE_LEFT; // next state will be ROTATE_LEFT
  }
  // The line is on the right side
  else if (!left_sensor && right_sensor) {
    State = ROTATE_RIGHT;
  }
  // The car is fallowing the line
  else if (left_sensor && right_sensor) {
    State = FORWARD; // Next state will be FORWARD
  }



}

/**
   This method calculate the distance between
   the sensor (HC-SR04) and nearest object based on the sensor values
*/
int ReadDistance() {
  int distance = 0;

  // Clears the trigPin
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Sets the trig_pin on HIGH state for 10 micro seconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  // Reads the echo_pin, returns the sound wave travel time in microseconds
  int duration = pulseIn(echo_pin, HIGH);
  // Calculating the distance
  distance = (duration * 0.034 / 2); // calculate the distance by using the speed of sound
  return distance;
}

/**
   This method sets same voltage for both wheels so the car moves straight forward
*/
void MoveForward() {
  analogWrite(m_right, m_speed);
  analogWrite(m_left, m_speed);
}
/**
   This method sets lower voltage to left wheel than right wheel so the care rotate to left
*/
void MoveLeft() {
  analogWrite(m_right, m_speed);
  analogWrite(m_left, m_rotate);
}
/**
   This method sets lower voltage to right wheel than left wheel so the care rotate to right
*/
void MoveRight() {
  analogWrite(m_right, m_rotate);
  analogWrite(m_left, m_speed);
}
/**
   This method sets 0V to both wheels so the car is not moving
*/
void Stop() {
  analogWrite(m_right, 0);
  analogWrite(m_left, 0);
}
/**
   This method reads the ulrasonic sensor and IR sensors.
*/
void ReadSensors() {
  // Read the distance three times and calculate the mean value in order to minimize sensor reading errors
  int distance = (ReadDistance() + ReadDistance() + ReadDistance()) / 3;
  // If the the distance is closer than 20 cm set the status of the sensor to the true.
  // That means the car need to stop.
  if (distance <= 20) {
    obstacle = true;
  }
  // Otherwise road is clear.
  else {
    obstacle = false;
  }
  // If the sensor value is HIGH, left IR sensor on the line
  if (digitalRead(ir_left)) {
    left_sensor = true;
  }
  // Otherwise left IR sensor out of the line
  else {
    left_sensor = false;
  }
  // If the sensor value is HIGH, right IR sensor on the line
  if (digitalRead(ir_right)) {
    right_sensor = true;
  }
  // Otherwise right IR sensor out of the line
  else {
    right_sensor = false;
  }

}
