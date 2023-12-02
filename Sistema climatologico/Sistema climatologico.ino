// Fotorecistencia
#define LED 3
int FOTOPIN = A0;
int valorSensor = 0;

// Humedad
#include <DHT.h>
#include <DHT_U.h>

#define Type DHT11
int dhtPin = 4;
DHT HT(dhtPin, Type);
int humidity;
float tempC;
float tempF;
int dth(500);

// Rotación - MPU6050 (Acelerómetro y Giroscopio)
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

int ax, ay, az;  // Valores RAW (sin procesar) del acelerómetro en los ejes x, y, z
int gx, gy, gz;  // Valores RAW (sin procesar) del giroscopio en los ejes x, y, z

long tiempo_prev;
float dt;
float ang_x, ang_y;  // Ángulos calculados con el acelerómetro
float ang_x_prev, ang_y_prev;

// Sensor de velocidad del viento
#define VIENTO_PIN 2              // Pin digital al que está conectado el sensor de velocidad del viento
#define INTERUPCIONES_POR_CM 4   // Número de interrupciones por cada 1 cm recorridos (ajusta según tu sensor)
volatile int interrupciones = 0;  // Contador de interrupciones
float distanciaRecorrida = 0;     // Distancia recorrida en cm
float velocidadViento = 0;        // Velocidad del viento en km/h

void setup() {
  // Fotorecistencia
  pinMode(FOTOPIN, INPUT);
  pinMode(LED, OUTPUT);

  // Humedad
  HT.begin();

  // Rotación - MPU6050
  Serial.begin(9600);   // Iniciando puerto serial
  Wire.begin();         // Iniciando I2C
  sensor.initialize();  // Iniciando el sensor

  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
  }

  // Configurar el sensor de velocidad del viento
  pinMode(VIENTO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VIENTO_PIN), contarInterrupciones, RISING);
}

void loop() {
  // Fotorecistencia
  valorSensor = analogRead(FOTOPIN);
  Serial.print("Valor del Sensor: ");
  Serial.println(valorSensor);

  if (valorSensor <= 30) {    // un valor bajo representa oscuridad
    digitalWrite(LED, HIGH);  // enciende LED
    delay(1000);              // demora de 1 seg. para evitar parpadeo de LED
    Serial.println("NOCHE");
    Serial.print("Valor: ");
    Serial.println(valorSensor);
  } else {
    Serial.println("DIA");
  }
  digitalWrite(LED, LOW);

  // Humedad
  humidity = HT.readHumidity();
  tempC = HT.readTemperature();
  tempF = HT.readTemperature(true);
  Serial.print("Humedad Relativa: ");
  Serial.print(humidity);
  Serial.print("% / Temperatura: ");
  Serial.print(tempC);
  Serial.print("ºC / ");
  Serial.print(tempF);
  Serial.println("ºF");
  delay(dth);

  // Rotación - MPU6050
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  // Calcular los ángulos con acelerómetro
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  // Calcular el ángulo de rotación con giroscopio y filtro complemento
  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  // Mostrar los ángulos separados por un [tab]
  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x);
  Serial.print("\tRotacion en Y: ");
  Serial.println(ang_y);

  // Mostrar el número de interrupciones realizadas en este ciclo
  Serial.print("Interrupciones en este ciclo: ");
  Serial.println(interrupciones);

  // Calcular distancia recorrida y velocidad del viento
  distanciaRecorrida = interrupciones / INTERUPCIONES_POR_CM;
  velocidadViento = distanciaRecorrida * (3600.0 / 100000);  // Convertir a km/h

  // Mostrar la velocidad del viento
  Serial.print("Velocidad del Viento: ");
  Serial.print(velocidadViento);
  Serial.println(" km/h");

  // Reiniciar el contador de interrupciones y la distancia
  interrupciones = 0;
  distanciaRecorrida = 0;

  delay(1000);
}

void contarInterrupciones() {
  interrupciones++;
}

// Conexiones Físicas:
// - Fotorecistencia (LDR) conectada al pin analógico A0.
// - LED conectado al pin digital 3.
// - Sensor de Humedad (DHT11) conectado al pin digital 4.
// - MPU6050 (Acelerómetro y Giroscopio) conectado a través de I2C (SCL y SDA).
// - Sensor de Velocidad del Viento conectado al pin digital 2 para las interrupciones.
//   Asegúrate de conectar correctamente la alimentación y la tierra según las especificaciones de cada sensor.
