#include <analogWrite.h>

enum State_enum {STOP, FORWARD, ROTATE_RIGHT, ROTATE_LEFT, ROTATE};
State_enum State = STOP;
const int m_left = 25;
const int m_right = 26;
const int ir_left = 14;
const int ir_right = 16;
const int trig_pin = 32;
const int echo_pin = 33;
const int m_speed = 140;
const int m_rotate = 80;

bool obstacle = false;
bool left_sensor = false;
bool right_sensor = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //sets serial port for communication
  analogReadResolution(8);
  analogWriteResolution(8);
  pinMode(trig_pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo_pin, INPUT); // Sets the echoPin as an Input
  pinMode(ir_left, INPUT);
  pinMode(ir_right, INPUT);
  pinMode(m_left, OUTPUT);
  pinMode(m_right, OUTPUT);
  Stop();
}

void loop() {
 delay(10);
  ReadSensors();
  switch (State)
  {
    case STOP:
      if (obstacle) {
        Stop();
        State = STOP;
      }
      else if (left_sensor && !right_sensor) {
        MoveLeft();
        State = ROTATE_LEFT;
      }
      else if (!left_sensor && right_sensor) {
        MoveRight();
        State = ROTATE_RIGHT;
      }
      else if (left_sensor && right_sensor) {
        MoveForward();
        State = FORWARD;
      }
      Serial.println("STOP");
      break;

    case FORWARD:
      if (obstacle ) {
        Stop();
        State = STOP;
      }
      else if (left_sensor && !right_sensor) {
        MoveLeft();
        State = ROTATE_LEFT;
      }
      else if (!left_sensor && right_sensor) {
        MoveRight();
        State = ROTATE_RIGHT;
      }
      else if (left_sensor && right_sensor) {
        MoveForward();
        State = FORWARD;
      }
      Serial.println("FORWARD");
      break;

    case ROTATE_RIGHT:
      if (obstacle) {
        Stop();
        State = STOP;
      }
      else if (left_sensor && !right_sensor) {
        MoveLeft();
        State = ROTATE_LEFT;
      }
      else if (!left_sensor && right_sensor) {
        MoveRight();
        State = ROTATE_RIGHT;
      }
      else if (left_sensor && right_sensor) {
        MoveForward();
        State = FORWARD;
      }
      Serial.println("ROTATE RIGHT");
      break;

    case ROTATE_LEFT:
      if (obstacle) {
        Stop();
        State = STOP;
      }
      else if (left_sensor && !right_sensor) {
        MoveLeft();
        State = ROTATE_LEFT;
      }
      else if (!left_sensor && right_sensor) {
        MoveRight();
        State = ROTATE_RIGHT;
      }
      else if (left_sensor && right_sensor) {
        MoveForward();
        State = FORWARD;
      }
      Serial.println("ROTATE LEFT");
      break;

    case ROTATE:
      if (left_sensor && right_sensor) {
        MoveForward();
        State = FORWARD;
      }
      Serial.println("ROTATE");
      break;
  }
  

}

int ReadDistance() {
  int distance = 0;
  
    // Clears the trigPin
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    int duration = pulseIn(echo_pin, HIGH);
    // Calculating the distance
    distance = (duration * 0.034 / 2);

  
  return distance;
}

void MoveForward() {
  analogWrite(m_right, m_speed);
  analogWrite(m_left, m_speed);
}

void MoveLeft() {
  analogWrite(m_right, m_speed);
  analogWrite(m_left, m_rotate);
}

void MoveRight() {
  analogWrite(m_right, m_rotate);
  analogWrite(m_left, m_speed);
}

void MoveStopRight() {
  analogWrite(m_right, 0);
  analogWrite(m_left, m_speed);
}


void MoveStopLeft() {
  analogWrite(m_right, m_speed);
  analogWrite(m_left, 0);
}

void Stop() {
  analogWrite(m_right, 0);
  analogWrite(m_left, 0);
}

void ReadSensors() {
  int distance = (ReadDistance() + ReadDistance() + ReadDistance()) / 3;
  if (distance <= 20) {
    obstacle = true;
  }
  else {
    obstacle = false;
  }
  if (digitalRead(ir_left)) {
    left_sensor = true;
  }
  else {
    left_sensor = false;
  }
  if (digitalRead(ir_right)) {
    right_sensor = true;
  }
  else {
    right_sensor = false;
  }
  /*
    Serial.print("LEFT : ");
    Serial.println(digitalRead(ir_left));
    Serial.print("RIGHT : ");
    Serial.println(digitalRead(ir_right));
  */
}
