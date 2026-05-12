#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

// RAW
int16_t ax, ay, az;
int16_t gx, gy, gz;

// tiempo
long tiempo_prev = 0;
float dt;

// ángulos
float ang_x = 0;
float ang_y = 0;
float ang_x_prev = 0;
float ang_y_prev = 0;

void setup() {
  Serial.begin(57600);
  Wire.begin();

  sensor.initialize();

  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");

  tiempo_prev = millis();
}

void loop() {

  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // tiempo en segundos
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // ángulo con acelerómetro
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / PI);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / PI);

  // filtro complementario
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  // salida
  Serial.print("Rotacion en X: ");
  Serial.print(ang_x);
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);

  delay(10);
}