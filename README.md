# weather_system
# Proyecto de Monitoreo Meteorológico y de Rotación

Este proyecto utiliza un Arduino para realizar un monitoreo meteorológico que incluye la lectura de datos de una fotorecistencia (LDR), un sensor de humedad (DHT11), y un sensor de rotación (MPU6050). Además, se ha integrado un sensor de velocidad del viento para medir la velocidad del viento en tiempo real.

## Contenido

1. [Requisitos del Proyecto](#requisitos-del-proyecto)
2. [Conexiones Físicas](#conexiones-físicas)
3. [Librerías Necesarias](#librerías-necesarias)
4. [Configuración](#configuración)
5. [Uso](#uso)
6. [Contribuciones](#contribuciones)
7. [Licencia](#licencia)

## Requisitos del Proyecto

- Arduino compatible (probado en Arduino Uno).
- Sensores: Fotorecistencia (LDR), Sensor de Humedad (DHT11), MPU6050 (Acelerómetro y Giroscopio), Sensor de Velocidad del Viento.

## Conexiones Físicas

- Fotorecistencia (LDR) conectada al pin analógico A0.
- LED conectado al pin digital 3.
- Sensor de Humedad (DHT11) conectado al pin digital 4.
- MPU6050 (Acelerómetro y Giroscopio) conectado a través de I2C (SCL y SDA).
- Sensor de Velocidad del Viento conectado al pin digital 2 para las interrupciones.
  Asegúrate de conectar correctamente la alimentación y la tierra según las especificaciones de cada sensor.

## Librerías Necesarias

- [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library) para el sensor de humedad (DHT11).
- [I2Cdev](https://github.com/jrowberg/i2cdevlib) y [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) para el sensor MPU6050.
- Librería estándar de Arduino para las funciones principales.

## Configuración

1. Instala las librerías mencionadas en la sección "Librerías Necesarias".
2. Conecta los sensores según las indicaciones en la sección "Conexiones Físicas".
3. Carga el código del proyecto en tu Arduino.

## Uso

1. Enciende tu Arduino y observa la salida en el puerto serie.
2. Los datos de la fotorecistencia, humedad, rotación y velocidad del viento se mostrarán en tiempo real.

## Contribuciones

Las contribuciones son bienvenidas. Si tienes mejoras o correcciones, por favor, abre un problema o envía una solicitud de extracción.

## Licencia

Este proyecto está licenciado bajo la [Licencia MIT](LICENSE).
