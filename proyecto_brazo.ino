#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float ang_x, ang_y, ang_z;
float ang_x_prev, ang_y_prev, ang_z_prev;

long tiempo_prev;
float dt;

const int botonInicialPin = 3;  // Pin del botón para almacenar posición inicial
const int botonFinalPin = 4;    // Pin del botón para almacenar posición final
const int numSamples = 100;
int16_t ax_samples[numSamples], ay_samples[numSamples], az_samples[numSamples];
int16_t gx_samples[numSamples], gy_samples[numSamples], gz_samples[numSamples];


bool guardarInicial = false;
bool guardarFinal = false;


void setup() {
  Serial.begin(57600);
  Wire.begin();
  sensor.initialize();

  pinMode(botonInicialPin, INPUT_PULLUP);
  pinMode(botonFinalPin, INPUT_PULLUP);

  if (sensor.testConnection())
    Serial.println("Sensor iniciado correctamente");
  else
    Serial.println("Error al iniciar el sensor");


}

void loop() {
  // Obtener datos del sensor
  for (int i = 0; i < numSamples; ++i) {
    sensor.getAcceleration(&ax_samples[i], &ay_samples[i], &az_samples[i]);
    sensor.getRotation(&gx_samples[i], &gy_samples[i], &gz_samples[i]);
    delay(10);
  }

  // Ordenar arrays
  sortArray(ax_samples, numSamples);
  sortArray(ay_samples, numSamples);
  sortArray(az_samples, numSamples);
  sortArray(gx_samples, numSamples);
  sortArray(gy_samples, numSamples);
  sortArray(gz_samples, numSamples);

  // Calcular media entre el dato 20 y el dato 80
  auto ax_avg = calculateAverage(&ax_samples[18], 64);
  auto ay_avg = calculateAverage(&ay_samples[18], 64);
  auto az_avg = calculateAverage(&az_samples[18], 64);
  auto gx_avg = calculateAverage(&gx_samples[18], 64);
  auto gy_avg = calculateAverage(&gy_samples[18], 64);
  auto gz_avg = calculateAverage(&gz_samples[18], 64);

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
  Serial.print("dt=");
  Serial.println(dt, 6);
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

    // Verificar si el botón de guardar inicial está presionado
  if (digitalRead(botonInicialPin) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(botonInicialPin) == LOW) {
      guardarInicial = true;
    }
  }

  // Verificar si el botón de guardar final está presionado
  if (digitalRead(botonFinalPin) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(botonFinalPin) == LOW) {
      guardarFinal = true;
    }
  }

  // Guardar ángulo inicial si se presiona el botón correspondiente
  if (guardarInicial) {
    Serial.println("Guardando ángulo inicial...");
    guardarInicial = false;
    // Lógica para ángulo inicial si es necesario
  }

  // Guardar ángulo final si se presiona el botón correspondiente
  if (guardarFinal) {
    Serial.println("Guardando ángulo final...");
    guardarFinal = false;
    // Lógica para ángulo final si es necesario
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

int16_t calculateAverage(int16_t *buf, size_t size) {
  int32_t sum = 0;
  for (; size; size--) {
    sum += *buf++;
  }
  return (int16_t)(sum >> 6);
}
