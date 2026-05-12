#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

// RAW
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Filtro pasa bajos
long f_ax = 0, f_ay = 0, f_az = 0;
int p_ax = 0, p_ay = 0, p_az = 0;

long f_gx = 0, f_gy = 0, f_gz = 0;
int p_gx = 0, p_gy = 0, p_gz = 0;

int counter = 0;

// Offsets
int ax_o, ay_o, az_o;
int gx_o, gy_o, gz_o;

void setup() {
  Serial.begin(57600);
  Wire.begin();

  sensor.initialize();

  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");

  // Leer offsets actuales
  ax_o = sensor.getXAccelOffset();
  ay_o = sensor.getYAccelOffset();
  az_o = sensor.getZAccelOffset();

  gx_o = sensor.getXGyroOffset();
  gy_o = sensor.getYGyroOffset();
  gz_o = sensor.getZGyroOffset();

  Serial.println("Offsets:");
  Serial.print(ax_o); Serial.print("\t");
  Serial.print(ay_o); Serial.print("\t");
  Serial.print(az_o); Serial.print("\t");
  Serial.print(gx_o); Serial.print("\t");
  Serial.print(gy_o); Serial.print("\t");
  Serial.println(gz_o);

  Serial.println("Envía cualquier carácter para empezar calibración");

  while (!Serial.available()) {}
  Serial.println("Calibrando, no mover IMU");
}

void loop() {

  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Filtro pasa bajos acelerómetro
  f_ax = f_ax - (f_ax >> 5) + ax;
  p_ax = f_ax >> 5;

  f_ay = f_ay - (f_ay >> 5) + ay;
  p_ay = f_ay >> 5;

  f_az = f_az - (f_az >> 5) + az;
  p_az = f_az >> 5;

  // Filtro giroscopio
  f_gx = f_gx - (f_gx >> 3) + gx;
  p_gx = f_gx >> 3;

  f_gy = f_gy - (f_gy >> 3) + gy;
  p_gy = f_gy >> 3;

  f_gz = f_gz - (f_gz >> 3) + gz;
  p_gz = f_gz >> 3;

  // Cada 100 muestras
  if (counter == 100) {

    Serial.print("Promedio:\t");
    Serial.print(p_ax); Serial.print("\t");
    Serial.print(p_ay); Serial.print("\t");
    Serial.print(p_az); Serial.print("\t");
    Serial.print(p_gx); Serial.print("\t");
    Serial.print(p_gy); Serial.print("\t");
    Serial.println(p_gz);

    // Calibración acelerómetro
    if (p_ax > 0) ax_o--;
    else ax_o++;

    if (p_ay > 0) ay_o--;
    else ay_o++;

    if (p_az - 16384 > 0) az_o--;
    else az_o++;

    sensor.setXAccelOffset(ax_o);
    sensor.setYAccelOffset(ay_o);
    sensor.setZAccelOffset(az_o);

    // Calibración giroscopio
    if (p_gx > 0) gx_o--;
    else gx_o++;

    if (p_gy > 0) gy_o--;
    else gy_o++;

    if (p_gz > 0) gz_o--;
    else gz_o++;

    sensor.setXGyroOffset(gx_o);
    sensor.setYGyroOffset(gy_o);
    sensor.setZGyroOffset(gz_o);

    counter = 0;
  }

  counter++;
}