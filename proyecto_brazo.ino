#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

int ax, ay, az;
int gx, gy, gz;

float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

float accel_ang_x_inicial, accel_ang_y_inicial, accel_ang_z_inicial;
float accel_ang_x_final, accel_ang_y_final, accel_ang_z_final;

const int ledPin = 13; // Pin del LED en la placa de Arduino

long tiempo_prev;
float dt;

const int botonCalibracionPin = 3;
const int botonFinalPin = 4;
bool calibracionActiva = false;

long f_ax, f_ay, f_az;
int p_ax, p_ay, p_az;
long f_gx, f_gy, f_gz;
int p_gx, p_gy, p_gz;
int counter = 0;

int ax_o, ay_o, az_o;
int gx_o, gy_o, gz_o;


const int numSamples = 32;
int16_t ax_samples[numSamples], ay_samples[numSamples], az_samples[numSamples];
int16_t gx_samples[numSamples], gy_samples[numSamples], gz_samples[numSamples];


int guardarInicial = 0;
int guardarFinal = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.initialize();

  pinMode(botonCalibracionPin, INPUT_PULLUP);
  pinMode(botonFinalPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT); // Configurar el pin del LED como salida

  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");

  ax_o = sensor.getXAccelOffset();
  ay_o = sensor.getYAccelOffset();
  az_o = sensor.getZAccelOffset();
  gx_o = sensor.getXGyroOffset();
  gy_o = sensor.getYGyroOffset();
  gz_o = sensor.getZGyroOffset();

  Serial.println("Offsets:");
  Serial.print(ax_o);
  Serial.print("\t");
  Serial.print(ay_o);
  Serial.print("\t");
  Serial.print(az_o);
  Serial.print("\t");
  Serial.print(gx_o);
  Serial.print("\t");
  Serial.print(gy_o);
  Serial.print("\t");
  Serial.print(gz_o);
  Serial.print("\t");
  Serial.println("Presione el botón de calibración para empezar");
}

void loop() {
  if (digitalRead(botonCalibracionPin) == LOW && !calibracionActiva) {
    calibracionActiva = true;
    Serial.println("Iniciando calibración, no mover IMU");
  }

  if (calibracionActiva) {
    calibrar();
  } else {
    // Obtener datos del sensor
    for (int i = 0; i < numSamples; ++i) {
      sensor.getAcceleration(&ax_samples[i], &ay_samples[i], &az_samples[i]);
      sensor.getRotation(&gx_samples[i], &gy_samples[i], &gz_samples[i]);
    }

    // Ordenar arrays
    // sortArray(ax_samples, numSamples);
    // sortArray(ay_samples, numSamples);
    // sortArray(az_samples, numSamples);
    // sortArray(gx_samples, numSamples);
    // sortArray(gy_samples, numSamples);
    // sortArray(gz_samples, numSamples);

    // Calcular media entre el dato 20 y el dato 80
    auto ax_avg = calculateAverage(ax_samples);
    auto ay_avg = calculateAverage(ay_samples);
    auto az_avg = calculateAverage(az_samples);
    auto gx_avg = calculateAverage(gx_samples);
    auto gy_avg = calculateAverage(gy_samples);
    auto gz_avg = calculateAverage(gz_samples);

    // Escalado de lecturas
    //float ax_m_s2 = ax_avg * (9.81 / 16384.0);
    //float ay_m_s2 = ay_avg * (9.81 / 16384.0);
    //float az_m_s2 = az_avg * (9.81 / 16384.0);
    //float gx_deg_s = gx_avg * (250.0 / 32768.0);
    //float gy_deg_s = gy_avg * (250.0 / 32768.0);
    //float gz_deg_s = gz_avg * (250.0 / 32768.0);

    // Mostrar las lecturas escaladas
    //Serial.print("Aceleracion (m/s^2): [");
    //Serial.print(ax_m_s2, 2);
    //Serial.print(", ");
    //Serial.print(ay_m_s2, 2);
    //Serial.print(", ");
    //Serial.print(az_m_s2, 2);
    //Serial.print("]\tGiroscopio (deg/s): [");
    //Serial.print(gx_deg_s, 2);
    //Serial.print(", ");
    //Serial.print(gy_deg_s, 2);
    //Serial.print(", ");
    //Serial.print(gz_deg_s, 2);
    //Serial.println("]");

    // Calcular ángulo de inclinación con el acelerómetro
    float accel_ang_x = atan(ay_avg / sqrt(pow(ax_avg, 2) + pow(az_avg, 2))) * (180.0 / PI);
    float accel_ang_y = atan(-ax_avg / sqrt(pow(ay_avg, 2) + pow(az_avg, 2))) * (180.0 / PI);
    float accel_ang_z = atan(sqrt(pow(ax_avg, 2) + pow(ay_avg, 2)) / az_avg) * (180.0 / PI);

    // Mostrar los ángulos de inclinación
    Serial.print("Inclinacion (grados): X=");
    Serial.print(accel_ang_x, 2);
    Serial.print("\tY=");
    Serial.print(accel_ang_y, 2);
    Serial.print("\tZ=");
    Serial.println(accel_ang_z, 2);

    
    dt = (millis()-tiempo_prev)/1000.0;
    //if (dt > 0.1) {
      //dt -= 0.1;
    //}
    Serial.print("dt=");
    Serial.println(dt, 4);
    tiempo_prev = millis();
    // Calcular ángulo de rotación con el giroscopio y filtro complemento

    ang_x = 0.98 * (ang_x_prev + (gx_avg / 131) * dt) + 0.02 * accel_ang_x;
    ang_y = 0.98 * (ang_y_prev + (gy_avg / 131) * dt) + 0.02 * accel_ang_y;

    ang_x_prev = ang_x;
    ang_y_prev = ang_y;

    // Mostrar los ángulos de rotación
    Serial.print("Rotacion (grados): X=");
    Serial.print(ang_x, 2);
    Serial.print("\tY=");
    Serial.println(ang_y, 2);


    // Verificar y guardar ángulo final
    verificarBoton(botonFinalPin, guardarFinal);
    guardarAngulo(guardarFinal, "Guardando ángulo final...");
    if (guardarFinal) {
      accel_ang_x_final = accel_ang_x;
      accel_ang_y_final = accel_ang_y;
      accel_ang_z_final = accel_ang_z;
    }
    controlarLED(accel_ang_x, 10); // 10 grados de margen
  }
}

// Función para ordenar un array de números en orden ascendente
void sortArray(int16_t arr[], int size) {
  // Bucle externo para recorrer el array
  for (int i = 0; i < size - 1; ++i) {
    // Bucle interno para comparar y ordenar los elementos
    for (int j = 0; j < size - i - 1; ++j) {
      // Verificar si el elemento actual es mayor que el siguiente
      if (arr[j] > arr[j + 1]) {
        // Swap: intercambiar los elementos si están en orden incorrecto
        int16_t temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

void calibrar() {
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  f_ax = f_ax - (f_ax >> 5) + ax;
  p_ax = f_ax >> 5;

  f_ay = f_ay - (f_ay >> 5) + ay;
  p_ay = f_ay >> 5;

  f_az = f_az - (f_az >> 5) + az;
  p_az = f_az >> 5;

  f_gx = f_gx - (f_gx >> 3) + gx;
  p_gx = f_gx >> 3;

  f_gy = f_gy - (f_gy >> 3) + gy;
  p_gy = f_gy >> 3;

  f_gz = f_gz - (f_gz >> 3) + gz;
  p_gz = f_gz >> 3;

  if (counter == 100) {
    Serial.print("promedio:");
    Serial.print("t");
    Serial.print(p_ax);
    Serial.print("\t");
    Serial.print(p_ay);
    Serial.print("\t");
    Serial.print(p_az);
    Serial.print("\t");
    Serial.print(p_gx);
    Serial.print("\t");
    Serial.print(p_gy);
    Serial.print("\t");
    Serial.println(p_gz);

    if (p_ax > 0)
      ax_o--;
    else {
      ax_o++;
    }
    if (p_ay > 0)
      ay_o--;
    else {
      ay_o++;
    }
    if (p_az - 16384 > 0)
      az_o--;
    else {
      az_o++;
    }

    sensor.setXAccelOffset(ax_o);
    sensor.setYAccelOffset(ay_o);
    sensor.setZAccelOffset(az_o);

    if (p_gx > 0)
      gx_o--;
    else {
      gx_o++;
    }
    if (p_gy > 0)
      gy_o--;
    else {
      gy_o++;
    }
    if (p_gz > 0)
      gz_o--;
    else {
      gz_o++;
    }

    sensor.setXGyroOffset(gx_o);
    sensor.setYGyroOffset(gy_o);
    sensor.setZGyroOffset(gz_o);

    counter = 0;
  }
  counter++;

  // Detener la calibración al presionar el botón
  if (digitalRead(botonFinalPin) == LOW) {
    calibracionActiva = false;
    Serial.println("Calibración completada");
  }
}

int16_t calculateAverage(int16_t *buf) {
  int32_t sum = 0;
  for (auto size = numSamples; size; size--) {
    sum += *buf++;
  }
  return (int16_t)(sum >> 5);
}

void verificarBoton(int pin, int &flag) {
  if (digitalRead(pin) == LOW && flag == 0) {
    flag = 1;
  }
}

void guardarAngulo(int &flag, const char *mensaje) {
  if (flag) {
    Serial.println(mensaje);
    flag = 0;
  }
}

void controlarLED(float angulo, int margen) {
  // Verificar si el ángulo se desvía más de +-margen grados de 0
  if (angulo > margen || angulo < -margen) {
    // Encender el LED más rápido cuanto más se desvíe, pero limitar la velocidad máxima
    int velocidadParpadeo = map(abs(angulo), margen, 70, 1000, 50); // Ajustar el rango de velocidad
    velocidadParpadeo = constrain(velocidadParpadeo, 50, 1000); // Limitar la velocidad máxima
    parpadearLED(velocidadParpadeo);
  } else {
    // Apagar el LED si está dentro del margen
    digitalWrite(ledPin, LOW);
  }
}

void parpadearLED(int velocidad) {
  static unsigned long tiempoPrevioParpadeo = 0;
  static bool estadoLED = LOW;

  // Verificar si ha pasado suficiente tiempo para cambiar el estado del LED
  if (millis() - tiempoPrevioParpadeo >= velocidad) {
    // Cambiar el estado del LED
    estadoLED = !estadoLED;
    digitalWrite(ledPin, estadoLED);

    // Actualizar el tiempo previo para el siguiente cambio de estado
    tiempoPrevioParpadeo = millis();
  }
}