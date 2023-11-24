// Importar las bibliotecas necesarias
#include <serial.h>
#include <time.h>

// Definir el nombre del archivo y abrirlo en modo escritura
archivo = open('C:\\Users\\Admin\\OneDrive\\Documentos\\Arduino\\arduino.txt','w')

// Inicializar el objeto de comunicación serial con el Arduino en el puerto COM10 y una velocidad de 9600 baudios
serialArduino = serial.Serial("COM10", 9600)
time.sleep(1)

// Bucle principal
while True:
    // Leer una línea de datos desde el Arduino y decodificarla como ASCII
    cad = serialArduino.readline().decode('ascii')
    
    // Imprimir la cadena recibida desde el Arduino en la consola
    print(cad)

    // Escribir la cadena en el archivo previamente abierto
    archivo.write(cad)

// Fotorecistencia
// Definir el pin del LED y los pines de la fotocélula y el valor del sensor
#define LED 3
int FOTOPIN = A0;
int valorSensor = 0;

// Humedad
// Definir el tipo de sensor DHT (DHT11), el pin al que está conectado y las variables para almacenar los datos
#include <DHT.h>
#include <DHT_U.h>
#define Type DHT11
int dhtPin = 4;
DHT HT(dhtPin, Type);
int humidity;
float tempC;
float tempF;
int dth(500);

// Rotación
// Incluir las bibliotecas necesarias para controlar el MPU6050 a través de I2C
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Declarar el objeto MPU6050
MPU6050 sensor;

// Declarar variables para almacenar los valores crudos de aceleración y giroscopio
int ax, ay, az;
int gx, gy, gz;

// Declarar variables para calcular los ángulos
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

// Configuración inicial
void setup() {
    // Configuración de la fotocélula
    pinMode(FOTOPIN, INPUT);
    pinMode(LED, OUTPUT);

    // Configuración del sensor DHT
    HT.begin();

    // Configuración del sensor MPU6050
    Serial.begin(9600);
    Wire.begin();
    sensor.initialize();

    // Verificar si el sensor MPU6050 se ha iniciado correctamente
    if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
    else Serial.println("Error al iniciar el sensor");
}

// Bucle principal
void loop() {
    // Lectura de la fotocélula
    valorSensor = analogRead(FOTOPIN);
    Serial.print("Valor del Sensor: ");
    Serial.println(valorSensor);

    // Control del LED en función del valor de la fotocélula
    if (valorSensor <= 30) {
        digitalWrite(LED, HIGH);
        delay(1000);
        Serial.print("Valor: ");
        Serial.println(valorSensor);
        Serial.print("NOCHE");
    }
    Serial.print("DIA");
    digitalWrite(LED, LOW);

    // Lectura de la humedad y temperatura
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

    // Lectura de la rotación utilizando el sensor MPU6050
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);
    
    // Cálculos para obtener los ángulos de rotación
    dt = (millis()-tiempo_prev)/1000.0;
    tiempo_prev = millis();
    
    float accel_ang_x = atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
    float accel_ang_y = atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
    ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
    ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
    ang_x_prev = ang_x;
    ang_y_prev = ang_y;

    // Mostrar los ángulos en la consola
    Serial.print("Rotacion en X:  ");
    Serial.print(ang_x); 
    Serial.print(" Rotacion en Y: ");
    Serial.println(ang_y);

    delay(1000);
}
