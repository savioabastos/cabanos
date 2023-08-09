// código desenvolvido para o arduno uno
//

#define DEBUG true  // ativa ou desativa o print
#define PRINT_INTERVAL 200
#define FOWARD 0
#define REVERSE 1
#include <Servo.h>

Servo servo;

unsigned long last_milis = 0;

int motor_pulse_width, servo_pulse_width = 1500;
int speed, angle = 0;
bool direction = FOWARD;

const int ch1 = 2;  // motor principal
const int ch2 = 3;  // servo motor
const int servo_pin = 6;
const int foward_pwm = 10;
const int reverse_pwm = 9;

const int forward_enable = 4;
const int reverse_enable = 5;

volatile unsigned long motor_start_time = 0, motor_current_time = 0, motor_pulses = 0;
volatile unsigned long servo_start_time = 0, servo_current_time = 0, servo_pulses = 0;


void debbuger();
void motorPulseTime();
void servoPulseTime();
bool setDirection();
int setSpeed();
int setAngle();
void motorControl();

void setup() {
  Serial.begin(9600);
  pinMode(ch2, INPUT_PULLUP);
  pinMode(ch1, INPUT_PULLUP);

  pinMode(forward_enable, OUTPUT);
  pinMode(reverse_enable, OUTPUT);
  pinMode(foward_pwm, OUTPUT);
  pinMode(reverse_pwm, OUTPUT);
  //pinMode(servo_pin, OUTPUT);
  servo.attach(servo_pin);

  attachInterrupt(digitalPinToInterrupt(ch2), servoPulseTime, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch1), motorPulseTime, CHANGE);
  digitalWrite(forward_enable,LOW);
  digitalWrite(reverse_enable,LOW);
  digitalWrite(foward_pwm,LOW);
  digitalWrite(reverse_pwm,LOW);

  servo.write(0);
}

void loop() {
  delay(100);

  setAngle();
  motorControl();
  if (DEBUG) {
    debbuger();
  }
}

void motorPulseTime() {
  motor_current_time = micros();
  if (motor_current_time > motor_start_time) {
    motor_pulses = motor_current_time - motor_start_time;
    if (motor_pulses < 2000) {
      motor_pulse_width = motor_pulses;
    }

    motor_start_time = motor_current_time;
  }
}

void servoPulseTime() {
  servo_current_time = micros();
  if (servo_current_time > servo_start_time) {
    servo_pulses = servo_current_time - servo_start_time;
    if (servo_pulses < 2000) {
      servo_pulse_width = servo_pulses;
    }

    servo_start_time = servo_current_time;
  }
}

bool setDirection() {
  if (motor_pulse_width - 1500 >= 0) {
    return FOWARD;
  } else {
    return REVERSE;
  }
}

int setSpeed() {
  int intensity = abs(motor_pulse_width - 1500);
  speed = map(intensity, 0, 500, 0, 255);
  return speed;
}

int setAngle() {
  angle = map(servo_pulse_width - 1000, 0, 1000, 0, 180);
  angle = constrain(angle, 0, 180);
  servo.write(angle);
  if(DEBUG){
    return angle;
  }
 
}

void motorControl() {
  bool dir = setDirection();
  int spe = setSpeed();

  if (dir == 0) {
    digitalWrite(forward_enable, HIGH);
    digitalWrite(reverse_enable, HIGH);
    analogWrite(foward_pwm, spe);
    analogWrite(reverse_pwm, 0);
  } else {
    digitalWrite(forward_enable, HIGH);
    digitalWrite(reverse_enable, HIGH);
    analogWrite(reverse_pwm, spe);
    analogWrite(foward_pwm, 0);
  }
}

void debbuger() {

  if (millis() >= last_milis + PRINT_INTERVAL) {
    last_milis = millis();
    Serial.print("CH2 :");
    Serial.print(servo_pulse_width);
    Serial.print("\t");

    Serial.print("CH1 :");
    Serial.print(motor_pulse_width);
    Serial.print("\t");

    Serial.print("Direction: ");
    Serial.print(setDirection());
    Serial.print("\t");

    Serial.print("Speed: ");
    Serial.print(setSpeed());
    Serial.print("\t");

    Serial.print("SERVO ANGLE: ");
    Serial.print(setAngle());
    Serial.print("\n");
  }
}
