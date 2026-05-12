// Librerías I2C para controlar el MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Crear objeto del sensor
MPU6050 sensor;

// Valores RAW del acelerómetro y giroscopio
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(57600);   // Iniciar puerto serie
  Wire.begin();          // Iniciar I2C

  sensor.initialize();   // Iniciar sensor

  // Comprobar conexión
  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
  }
}

void loop() {
  // Leer aceleración y giroscopio
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Mostrar datos
  Serial.print("a[x y z] g[x y z]:\t");

  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");

  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(100); // Evitar saturar el puerto serie
}
