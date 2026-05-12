#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;

// ============================
// Pines motores (Arduino Nano)
// ============================

// Motor A
const int pinPWMA = 5;   // PWM
const int pinAIN1 = 7;
const int pinAIN2 = 8;

// Motor B
const int pinPWMB = 6;   // PWM
const int pinBIN1 = 9;
const int pinBIN2 = 10;

// STBY
const int pinSTBY = 4;

// ============================
// Variables MPU6050
// ============================

int16_t ax, ay, az;
int16_t gx, gy, gz;

long tiempo_prev;
float dt;

float ang_x = 0;
float ang_y = 0;

float ang_x_prev = 0;
float ang_y_prev = 0;

// ============================
// PID
// ============================

float Kp = 25.0;
float Ki = 0.0;
float Kd = 0.8;

float integral = 0.0;
float derivative = 0.0;
float previous_error = 0.0;

void setup() {

  Serial.begin(115200);

  // I2C Arduino Nano
  Wire.begin();

  sensor.initialize();

  if (sensor.testConnection())
    Serial.println("MPU6050 conectado");
  else
    Serial.println("Error MPU6050");

  // Pines motores
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinPWMA, OUTPUT);

  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  pinMode(pinSTBY, OUTPUT);

  digitalWrite(pinSTBY, HIGH);

  tiempo_prev = millis();
}

void loop() {

  // Leer sensor
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Tiempo
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // Ángulos acelerómetro
  float accel_ang_x =
      atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / PI);

  float accel_ang_y =
      atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / PI);

  // Filtro complementario
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt)
        + 0.02 * accel_ang_x;

  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt)
        + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  // PID
  float desired_angle = 0.0;

  float error = desired_angle - ang_x;

  integral += error * dt;

  derivative = (error - previous_error) / dt;

  float motor_speed =
      Kp * error +
      Ki * integral +
      Kd * derivative;

  // Limitar PWM
  motor_speed = constrain(motor_speed, -255, 255);

  // Mover motores
  moveMotor(pinPWMA, pinAIN1, pinAIN2, motor_speed);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, motor_speed);

  // Debug
  Serial.print("Angulo X: ");
  Serial.print(ang_x);

  Serial.print(" Error: ");
  Serial.print(error);

  Serial.print(" Motor: ");
  Serial.println(motor_speed);

  previous_error = error;

  delay(5);
}

// ============================
// FUNCION MOTOR
// ============================

void moveMotor(int pwmPin, int pinIN1, int pinIN2, float speed) {

  if (speed > 0) {

    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);

    analogWrite(pwmPin, (int)speed);

  } else {

    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);

    analogWrite(pwmPin, (int)(-speed));
  }
}