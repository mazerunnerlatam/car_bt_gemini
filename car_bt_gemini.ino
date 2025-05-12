/*
 * Robot Seguidor de Línea Avanzado con Arduino
 * Control PID + Manejo de Giros 90° y Bifurcaciones T
 * 
 * Componentes: Arduino Uno, L298N, 6 Sensores IR Analógicos, 2 Motores DC
 * 
 * NOTA: Este código es un ejemplo y requiere AJUSTE Y CALIBRACIÓN
 *       para funcionar correctamente con tu hardware específico.
 */

/*

Ancho de la Línea vs. Separación de Sensores IR

La relación ideal entre el ancho de la línea negra y la distancia entre los sensores IR adyacentes depende un poco del algoritmo que uses, pero aquí hay una guía general:

Idealmente: Ancho de línea ≈ Distancia entre los dos sensores centrales.

Si tienes 6 sensores (numerados 0 a 5), la distancia clave es entre el sensor 2 y el sensor 3.
¿Por qué? Cuando el robot está perfectamente centrado, los sensores 2 y 3 estarán justo en los bordes de la línea negra (o uno justo dentro y otro justo fuera). Esto proporciona la máxima sensibilidad para detectar pequeñas desviaciones. El algoritmo de promedio ponderado o PID puede calcular un error muy preciso cercano a cero. Si el robot se desvía ligeramente, uno de estos sensores centrales cruzará claramente al blanco o al negro, generando una señal de error inmediata y clara.
Aceptable: Ancho de línea < Distancia entre los dos sensores centrales.

En este caso, cuando el robot está centrado, ambos sensores centrales (2 y 3) estarán sobre el fondo blanco, con la línea negra pasando entre ellos.
Esto también funciona bien. Si el robot se desvía, uno de los sensores centrales detectará la línea negra, generando una señal de error. El algoritmo de promedio ponderado seguirá funcionando correctamente.
Menos Ideal: Ancho de línea > Distancia entre los dos sensores centrales.

Si la línea es significativamente más ancha que la separación entre los sensores 2 y 3, ambos podrían detectar negro simultáneamente cuando el robot está (aproximadamente) centrado.
Esto puede dificultar que el algoritmo de promedio ponderado o PID calcule un error preciso cercano a cero, ya que la lectura central podría "saturarse" (ambos sensores centrales dan la misma lectura de "negro"). El robot podría tender a deambular un poco dentro de la línea ancha en lugar de seguir un camino preciso. La detección del centro exacto es más difícil.
En resumen: Para un control preciso con 6 sensores y algoritmos como PID o promedio ponderado, apunta a que el ancho de la línea negra sea aproximadamente igual o ligeramente menor que la distancia física entre los dos sensores IR más centrales (sensores 2 y 3). Esto suele dar la mejor sensibilidad para detectar desviaciones y mantener el robot centrado. Sin embargo, la calibración y el ajuste del algoritmo son igualmente importantes. 


Sí, en el código unificado que te proporcioné anteriormente, la forma en que se definen y utilizan los pines de los sensores junto con los pesos asignados implica que:

A0 está configurado para ser el sensor más a la izquierda.
A5 está configurado para ser el sensor más a la derecha.
Esto se deduce de la combinación de estas dos líneas en el código:

int sensorPins = {A0, A1, A2, A3, A4, A5}; - Define el orden en que se leen los pines.
int weights = {-5, -3, -1, 1, 3, 5}; - Asigna los pesos correspondientes a cada sensor en ese orden.
*/

// --- Definiciones de Pines (¡AJUSTAR SEGÚN TU CABLEADO!) ---
#define ENA 9   // Pin PWM para Velocidad Motor Izquierdo (Motor A)
#define IN1 8   // Pin Dirección Motor Izquierdo 1
#define IN2 7   // Pin Dirección Motor Izquierdo 2
#define ENB 6   // Pin PWM para Velocidad Motor Derecho (Motor B)
#define IN3 5   // Pin Dirección Motor Derecho 1
#define IN4 4   // Pin Dirección Motor Derecho 2

// Pines para los 6 Sensores IR Analógicos (A0 a A5)
int sensorPins[6]= {A0, A1, A2, A3, A4, A5}; 
const int NUM_SENSORS = 6;

// --- Constantes Globales (¡AJUSTAR!) ---
int baseSpeed = 130;      // Velocidad base de los motores (0-255)
int turnSpeed = 100;      // Velocidad para giros bruscos (0-255)
int turnDelay = 150;      // Pequeño delay para avanzar en T (ms) - Opcional

// Constantes PID (¡REQUIEREN AJUSTE PRECISO!)
float Kp = 0.5;           // Ganancia Proporcional
float Ki = 0.001;         // Ganancia Integral (puede empezar en 0)
float Kd = 0.6;           // Ganancia Derivativa

// Pesos para el cálculo de posición (Promedio Ponderado)
// Asume centro entre sensor 2 y 3. Ajustar si es necesario.
int weights[6] = {-5, -3, -1, 1, 3, 5}; 
// Alternativa escalada: int weights = {-2500, -1500, -500, 500, 1500, 2500};

// Umbrales de Detección (¡AJUSTAR SEGÚN CALIBRACIÓN Y SENSORES!)
// Estos valores dependen de si tus sensores dan valor ALTO o BAJO para negro.
// Este ejemplo asume ALTO = NEGRO, BAJO = BLANCO. ¡Verifica tus sensores!
const int SENSOR_THRESHOLD_BLACK = 700; // Valor analógico por encima del cual se considera negro
const int SENSOR_THRESHOLD_WHITE = 350; // Valor analógico por debajo del cual se considera blanco

// --- Variables Globales ---
int sensorValues[6]; // Array para almacenar lecturas de sensores
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// --- Función setup() ---
void setup() {
  Serial.begin(9600); // Iniciar comunicación serial para depuración

  // Configurar pines de motor como SALIDA
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // (Opcional pero recomendado) Añadir rutina de calibración de sensores aquí
  // calibrateSensors(); 

  stopMotors(); // Asegurar que los motores estén parados al inicio
  Serial.println("Robot listo. Esperando 2 segundos...");
  //delay(2000); // Pequeña pausa antes de empezar
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i<3; i++)
  { 
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(250);
  }
  for (int i = 0; i<3; i++)
  { 
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(500);
  }
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1500);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW


  Serial.println("READY");

}

// --- Función loop() ---
void loop() {
  readSensors(); // Leer los 6 sensores

  // --- Lógica de Detección Prioritaria (Giros/T/Perdida) ---
  if (detectTJunction(sensorValues)) {
    handleTJunction_TurnLeft(); // Estrategia: Siempre girar a la izquierda en T
    resetPID(); // Resetear variables PID después de maniobra
  } else if (detectSharpLeftTurn(sensorValues)) {
    turnLeftSharp_SensorBased();
    resetPID();
  } else if (detectSharpRightTurn(sensorValues)) {
    turnRightSharp_SensorBased();
    resetPID();
  } else if (detectLineLost(sensorValues)) {
     Serial.println("Linea perdida!");
     stopMotors(); // Estrategia: Parar si se pierde la línea
     resetPID();
     // Podrías implementar una rutina de búsqueda aquí
  } else {
    // --- Control PID Normal ---
    int position = calculatePosition(); // Calcular posición/error
    runPID(position);                   // Aplicar control PID
  }

  // Pequeño delay para estabilizar el ciclo (opcional, ajustar)
  delay(5); 
}

// --- Funciones Auxiliares ---
// Lee los valores analógicos de los 6 sensores
void readSensors() {
  //Serial.print("Sensores: "); // Descomentar para depuración
  for (int i = 0; i < NUM_SENSORS-1; i++) {
    //sensorValues[i] = analogRead(sensorPins[i]);
    sensorValues[i] = 1023 - analogRead(sensorPins[i]);
    // Aquí se podría aplicar normalización si se implementó calibración
    Serial.print(sensorValues[i]); Serial.print("\t"); // Descomentar para depuración
  }

  //sensorValues[NUM_SENSORS-1] = 1023-analogRead(sensorPins[NUM_SENSORS-1]);
  sensorValues[NUM_SENSORS-1] = analogRead(sensorPins[NUM_SENSORS-1]);
  Serial.print(sensorValues[NUM_SENSORS-1]); Serial.print("\t"); // Descomentar para depuración

  Serial.println(); // Descomentar para depuración
}

// Calcula la posición de la línea usando promedio ponderado
// Devuelve un valor de error (0 = centrado, negativo = izq, positivo = der)
int calculatePosition() {
  long weightedSum = 0;
  long sumOfReadings = 0; 
  bool lineDetected = false;

  // Usar lecturas umbralizadas o normalizadas es más robusto
  // Este ejemplo simplifica usando un valor fijo (1000) si detecta negro
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > SENSOR_THRESHOLD_BLACK) { // Asume ALTO = NEGRO
        weightedSum += (long)1000 * weights[i]; 
        sumOfReadings += 1000;
        lineDetected = true;
    }
  }

  if (!lineDetected) {
    // Si no se detecta línea (todos blancos), mantener último error conocido
    return lastError > 0? 5 : -5; // Devuelve error máximo a izq/der
                                   // O podrías devolver 0 o un valor especial
  }
  
  // Evitar división por cero
  if (sumOfReadings == 0) {
      return lastError > 0? 5 : -5; // O devolver 0
  }

  return weightedSum / sumOfReadings; // Devuelve la posición/error calculado
}

// Calcula y aplica la corrección PID a los motores
void runPID(int position) {
  error = 0 - position; // Calcular error (Setpoint = 0, queremos estar centrados)

  integral += error; // Acumular error para el término I
  // Opcional: Limitar integral (Anti-windup) - ajustar límites
  // integral = constrain(integral, -5000, 5000); 

  derivative = error - lastError; // Calcular cambio en el error para el término D

  // Calcular salida PID total
  float PID_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error; // Guardar error actual para el próximo ciclo

  // Calcular velocidades finales para cada motor
  int leftSpeed = baseSpeed + PID_output;
  int rightSpeed = baseSpeed - PID_output;

  // Limitar velocidades al rango PWM válido (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Aplicar velocidades a los motores (dirección adelante)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Motor Izq Adelante
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Motor Der Adelante
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Imprimir valores para depuración/ajuste (opcional)
  // Serial.print("Err:"); Serial.print(error);
  // Serial.print(" PID:"); Serial.print(PID_output);
  // Serial.print(" LSpd:"); Serial.print(leftSpeed);
  // Serial.print(" RSpd:"); Serial.println(rightSpeed);
}

// Resetea las variables del PID
void resetPID() {
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
}

// --- Funciones de Detección de Patrones (¡AJUSTAR UMBRALES Y LÓGICA!) ---

// Detecta Bifurcación en T (ej: 5 o 6 sensores ven negro)
bool detectTJunction(int values[]) {
  int blackCount = 0;
  for(int i = 0; i < NUM_SENSORS; i++) {
    if (values[i] > SENSOR_THRESHOLD_BLACK) 
      {
        blackCount++;
      }
  }
  // Ajustar el número mínimo de sensores negros para considerar T (ej. 5 o 6)
  return (blackCount >= 5); 
}

// Detecta Giro Brusco a la Izquierda (ej: 3-4 izq negros, 2 der blancos)
bool detectSharpLeftTurn(int values[]) {
  // Ajustar índices y umbrales según sea necesario
  return (values > SENSOR_THRESHOLD_BLACK &&
          values[1] > SENSOR_THRESHOLD_BLACK &&
          values[2] > SENSOR_THRESHOLD_BLACK &&
          values[3] < SENSOR_THRESHOLD_WHITE &&
          values[4] < SENSOR_THRESHOLD_WHITE); 
}

// Detecta Giro Brusco a la Derecha (ej: 3-4 der negros, 2 izq blancos)
bool detectSharpRightTurn(int values[]) {
  // Ajustar índices y umbrales según sea necesario
  return (values[5] > SENSOR_THRESHOLD_BLACK &&
          values[3] > SENSOR_THRESHOLD_BLACK &&
          values[4] > SENSOR_THRESHOLD_BLACK &&
          values < SENSOR_THRESHOLD_WHITE &&
          values[1] < SENSOR_THRESHOLD_WHITE);
}

// Detecta si se ha perdido la línea (ej: todos los sensores ven blanco)
bool detectLineLost(int values[]) {
    int whiteCount = 0;
    for(int i = 0; i < NUM_SENSORS; i++) {
        // Usar un umbral un poco más alto que el blanco puro puede ser más robusto
        if (values[i] < SENSOR_THRESHOLD_WHITE + 50) whiteCount++; 
    }
    return (whiteCount == NUM_SENSORS); // Todos ven blanco
}

// --- Funciones de Maniobra ---

// Maneja una bifurcación T (ejemplo: siempre gira a la izquierda)
void handleTJunction_TurnLeft() {
  Serial.println("T Junction Detectada - Girando Izquierda");
  stopMotors(); // Parar brevemente
  delay(100); 

  // Opcional: Avanzar un poco para centrar el robot en la T
  moveForwardBasic(baseSpeed / 2); // Avanzar a velocidad reducida
  delay(turnDelay); // Ajustar este delay experimentalmente
  stopMotors();
  delay(100);

  // Ejecutar giro a la izquierda basado en sensores
  turnLeftSharp_SensorBased(); 

  // El control PID se reanudará en el siguiente ciclo del loop()
}

// Gira bruscamente a la izquierda hasta realinearse con la línea
void turnLeftSharp_SensorBased() {
  Serial.println("Giro Brusco Izquierda");
  // Girar sobre el eje (Motor Izq Atrás, Motor Der Adelante)
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
  analogWrite(ENA, turnSpeed); // Usar velocidad de giro definida
  analogWrite(ENB, turnSpeed);

  // Esperar hasta que los sensores centrales (2 y 3) detecten negro
  // O hasta que se detecte un patrón centrado (ej. 0 0 1 1 0 0)
  // ¡Esta condición de salida es CRÍTICA y necesita ajuste!
  while(!(analogRead(sensorPins[2]) > SENSOR_THRESHOLD_BLACK && analogRead(sensorPins[5]) > SENSOR_THRESHOLD_BLACK) ) {
     // Podría añadirse un timeout aquí para evitar bucles infinitos si no encuentra la línea
     delay(1); // Pequeño delay para no sobrecargar
  }
  
  // Opcional: Continuar girando un poco más o parar inmediatamente
  stopMotors();
  delay(50); // Pequeña pausa para estabilizar
}

// Gira bruscamente a la derecha hasta realinearse con la línea
void turnRightSharp_SensorBased() {
   Serial.println("Giro Brusco Derecha");
  // Girar sobre el eje (Motor Izq Adelante, Motor Der Atrás)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
  analogWrite(ENA, turnSpeed); // Usar velocidad de giro definida
  analogWrite(ENB, turnSpeed);

  // Esperar hasta que los sensores centrales (2 y 3) detecten negro
  while(!(analogRead(sensorPins[2]) > SENSOR_THRESHOLD_BLACK && analogRead(sensorPins[5]) > SENSOR_THRESHOLD_BLACK) ) {
     delay(1);
  }
  stopMotors();
  delay(50);
}

// Detiene ambos motores (freno activo)
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
}

// Mueve el robot hacia adelante a una velocidad fija (sin control PID)
void moveForwardBasic(int speed) { 
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); 
  analogWrite(ENB, speed);
}