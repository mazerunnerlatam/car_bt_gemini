Robot Seguidor de Línea Avanzado con Arduino: Navegación de Giros y Bifurcaciones1. IntroducciónBienvenido a la guía para construir un robot seguidor de línea avanzado. Este proyecto se centra en el desarrollo de un vehículo autónomo capaz de navegar por una pista definida por una línea negra sobre un fondo blanco. El objetivo principal es no solo seguir la línea, sino también manejar características complejas de la pista, específicamente giros de 90 grados y bifurcaciones en T, utilizando una placa Arduino, seis sensores infrarrojos (IR) y dos motores de corriente continua (DC) [User Query].Los componentes de hardware clave que se utilizarán incluyen una placa Arduino (como el popular Arduino Uno, adecuado para principiantes 1), dos motores DC (comúnmente motores amarillos con caja reductora ), un controlador de motor L298N para gestionar la potencia de los motores 1, y un conjunto de seis sensores de reflectancia IR para detectar la línea.5Para lograr una navegación precisa y fiable, exploraremos diversas estrategias de control. Comenzaremos con un control proporcional básico, utilizaremos un método de promedio ponderado para determinar la posición de la línea con mayor precisión, y avanzaremos hacia la implementación de un controlador PID (Proporcional-Integral-Derivativo) para un rendimiento superior y estable.7 Además, se detallará la lógica específica necesaria para detectar y maniobrar a través de giros cerrados y bifurcaciones.11Este informe está estructurado para guiarle a través de cada paso del proceso:
Ensamblaje de Hardware y Conexiones: Detalla los componentes necesarios y cómo conectarlos correctamente.
Comprensión de los Sensores IR: Explica el funcionamiento de los sensores y las técnicas para leer sus datos.
Control de Motores DC con Driver L298N: Describe cómo comandar los motores usando el driver L298N.
Algoritmo Central de Seguimiento de Línea: Presenta el método de promedio ponderado y el control proporcional.
Implementación PID: Introduce el control PID para un rendimiento avanzado.
Navegación de Características Complejas: Detalla la lógica para giros de 90° y bifurcaciones en T.
Ejemplos de Código Arduino: Proporciona código funcional y comentado.
Solución de Problemas y Ajuste: Ofrece consejos para diagnosticar problemas y optimizar el rendimiento.
Conclusión: Resume los logros y sugiere posibles mejoras futuras.
2. Ensamblaje de Hardware y ConexionesUn ensamblaje correcto y conexiones fiables son fundamentales para el éxito del robot seguidor de línea. Esta sección detalla los componentes necesarios y los pasos para montarlos y conectarlos.2.1. Lista Detallada de Componentes
Placa Arduino: Se recomienda Arduino Uno por su facilidad de uso y disponibilidad de pines 1, aunque otras placas como Arduino Mega también son compatibles.14
Chasis del Robot: Puede ser de diversos materiales como madera contrachapada 15, acrílico 16, impreso en 3D 5, o un kit prefabricado.16 Es importante considerar el peso total del robot, ya que un menor peso permite una mayor aceleración y velocidad.
Motores DC (x2): Típicamente motores de hobby con caja reductora para obtener un par adecuado a bajas velocidades.1 Es crucial notar que los motores idénticos pueden tener ligeras variaciones en su rendimiento, lo que podría causar que el robot se desvíe ligeramente en tramos rectos; esto puede requerir ajustes en el software.22
Ruedas (x2): Deben coincidir con los ejes de los motores.19 La tracción es un factor importante a considerar para evitar deslizamientos.
Rueda Loca (x1): Proporciona un tercer punto de apoyo para la estabilidad y permite giros suaves.19
Controlador de Motor L298N: Un módulo popular y económico para controlar dos motores DC.1 Aunque es funcional, el L298N utiliza tecnología bipolar H-Bridge, que introduce una caída de voltaje significativa (a menudo 2V o más) entre la fuente de alimentación y los motores.24 Esta pérdida de voltaje reduce la eficiencia y puede ser particularmente notable con motores de bajo voltaje (por ejemplo, 5V o 6V), limitando su par y velocidad máxima. Los controladores basados en MOSFET son una alternativa mucho más eficiente, pero el L298N es adecuado para este proyecto de nivel intermedio debido a su bajo costo y amplia disponibilidad.
Sensores IR (x6): Se requiere un conjunto de 6 sensores de reflectancia IR. Puede ser un módulo integrado o sensores individuales (tipo TCRT5000).6 Se prefieren sensores con salida analógica para implementar algoritmos avanzados como PID o promedio ponderado.8
Fuente de Alimentación: Un paquete de baterías adecuado. Opciones comunes incluyen 6 pilas AA (proporcionando 9V si son alcalinas) 2, una batería LiPo de 7.4V 30, o una batería de 12V.1 El L298N requiere un voltaje para los motores entre 5V y 35V, y una alimentación lógica de 5V.3 Es crucial entender el jumper del regulador de 5V en el L298N.4 Evite usar baterías PP3 de 9V para alimentar los motores, ya que no pueden suministrar la corriente necesaria durante mucho tiempo.28
Cables Jumper: Para realizar las conexiones entre los componentes.16
Opcional: Interruptor de encendido/apagado 1, tornillos, tuercas, separadores para el montaje.2
2.2. Pasos de Ensamblaje
Montar Motores y Ruedas: Fije los motores al chasis y acople las ruedas a los ejes.15 Asegúrese de que los motores estén alineados correctamente para que el robot avance recto.15
Montar Rueda Loca: Fije la rueda loca en la parte delantera o trasera del chasis, según el diseño, para proporcionar estabilidad.
Montar Componentes Electrónicos: Fije la placa Arduino, el módulo L298N y el soporte de la batería al chasis.15 Considere la distribución del peso para mantener el centro de masa bajo y cerca de las ruedas motrices, lo que mejora la estabilidad y la tracción.
Montar Sensores IR: Fije los 6 sensores IR en la parte delantera del chasis, orientados hacia abajo.5 La altura óptima sobre la superficie suele ser de unos 3 mm (o 1/8 de pulgada).31 La separación entre los sensores y su distancia respecto al eje de las ruedas influyen en la capacidad del robot para detectar la línea y anticipar los giros.
2.3. Instrucciones de CableadoEs fundamental realizar las conexiones eléctricas correctamente. Se recomienda verificar cada conexión antes de alimentar el circuito. Un diagrama visual sería muy útil en un informe final.
Motores al L298N: Conecte los terminales del motor izquierdo a las salidas OUT1 y OUT2 del L298N, y los del motor derecho a OUT3 y OUT4.1 Si un motor gira en la dirección incorrecta después de cargar el código, simplemente invierta los cables de ese motor en los terminales del L298N.15 Si se utilizan 4 motores, conecte los pares de cada lado en paralelo, invirtiendo la polaridad de uno de los motores de cada par para que giren en la misma dirección.32
L298N a Arduino:

Control de Dirección: Conecte los pines IN1, IN2, IN3 e IN4 del L298N a pines digitales del Arduino.1 Estos pines controlan la dirección de giro de cada motor.
Control de Velocidad: Conecte los pines ENA y ENB del L298N a pines PWM del Arduino (p. ej., 3, 5, 6, 9, 10, 11 en Uno).1 Estos pines permiten controlar la velocidad de cada motor mediante modulación por ancho de pulso (PWM). Algunos ejemplos omiten estos pines y utilizan jumpers en el L298N , pero el control PWM es esencial para algoritmos como PID y para un movimiento suave.
Tierra (GND): Conecte un pin GND del L298N a un pin GND del Arduino para establecer una referencia de voltaje común.1
Alimentación Lógica L298N (5V): Este punto requiere atención especial debido al jumper "5V EN" o similar en el módulo L298N 4:

Si el voltaje de la batería del motor es > 12V O si alimenta el Arduino por separado (p. ej., USB): RETIRE el jumper. En este caso, debe suministrar 5V al pin de entrada de 5V del L298N (puede tomarlo del pin 5V del Arduino).
Si el voltaje de la batería del motor es <= 12V: MANTENGA el jumper puesto. El L298N generará su propia alimentación lógica de 5V a partir de la batería del motor. El pin 5V del L298N actuará como una salida de 5V que opcionalmente puede usar para alimentar el Arduino.4
Precaución: Nunca conecte el pin 5V del Arduino al pin 5V del L298N si el jumper está puesto, ya que estaría conectando dos fuentes de 5V, lo que podría dañar los componentes.34




Sensores IR a Arduino:

Alimentación: Conecte el pin VCC de todos los sensores al pin 5V del Arduino.1 Conecte el pin GND de todos los sensores a un pin GND del Arduino.1
Salida Analógica: Conecte los pines de salida (OUT, S, A0, etc., según el sensor) de los 6 sensores IR a 6 pines de entrada analógica del Arduino (A0 a A5).5 Aunque algunos ejemplos básicos usan entradas digitales 1, las entradas analógicas son preferibles para los algoritmos avanzados que se abordarán.


Fuente de Alimentación: Conecte el terminal positivo (+) de la batería al pin de entrada de voltaje del motor del L298N (a menudo etiquetado como 12V, VMS o VCC) y el terminal negativo (-) de la batería al pin GND del L298N.1 Si desea, puede intercalar un interruptor en el cable positivo de la batería.1
2.4. Tabla de Configuración de Pines (Ejemplo para Arduino Uno)Para facilitar el cableado y la programación, la siguiente tabla resume una posible asignación de pines. Es crucial verificar esta tabla con sus conexiones reales.Pin ArduinoComponentePin ComponenteFunción/Descripción9L298NENAVelocidad Motor A (Izquierdo) - PWM8L298NIN1Dirección Motor A - 17L298NIN2Dirección Motor A - 26L298NENBVelocidad Motor B (Derecho) - PWM5L298NIN3Dirección Motor B - 14L298NIN4Dirección Motor B - 2A0Sensor IR ArrayS1_OUTSalida Analógica Sensor 1 (Más Izq.)A1Sensor IR ArrayS2_OUTSalida Analógica Sensor 2A2Sensor IR ArrayS3_OUTSalida Analógica Sensor 3A3Sensor IR ArrayS4_OUTSalida Analógica Sensor 4A4Sensor IR ArrayS5_OUTSalida Analógica Sensor 5A5Sensor IR ArrayS6_OUTSalida Analógica Sensor 6 (Más Der.)5VL298N / SensoresVCC / +5VAlimentación 5VGNDL298N / SensoresGNDTierra ComúnVINBatería (+)L298N VCC Mot.Alimentación Motores (p.ej. 7.4V-12V)GNDBatería (-)L298N GNDTierra Batería3. Comprensión de los Sensores IR para Seguimiento de LíneaLos sensores infrarrojos (IR) son los "ojos" del robot seguidor de línea. Entender cómo funcionan y cómo interpretar sus lecturas es esencial para desarrollar un algoritmo de control eficaz.3.1. Principio de FuncionamientoEl principio se basa en la reflectividad diferencial de las superficies claras y oscuras a la luz infrarroja.2 Las superficies blancas o claras tienden a reflejar una gran cantidad de luz IR, mientras que las superficies negras u oscuras tienden a absorberla.2Cada módulo sensor IR típicamente contiene:
Un LED IR (Emisor): Emite un haz de luz infrarroja invisible hacia la superficie debajo del robot.2
Un Fotodiodo o Fototransistor (Receptor): Detecta la cantidad de luz infrarroja que es reflejada por la superficie.2
La intensidad de la luz reflejada captada por el receptor genera una señal eléctrica (generalmente un voltaje) que indica si el sensor está sobre una superficie clara u oscura.23.2. Lectura de Datos de los Sensores (6 Sensores)Con seis sensores montados en la parte frontal, el robot obtiene una visión más detallada de la posición de la línea. Para este proyecto, se recomienda encarecidamente utilizar las salidas analógicas de los sensores.

Lectura Analógica:

Conexión: Conecte las salidas de los seis sensores a los pines de entrada analógica del Arduino (A0 a A5).5
Función: Utilice la función analogRead(pin) en el código Arduino para leer el valor de voltaje en cada pin analógico.8
Valores: Esta función devuelve un valor entero entre 0 y 1023, que representa el voltaje medido (0V corresponde a 0, 5V corresponde a 1023 en un Arduino típico).2
Interpretación: La relación entre el valor leído y el color de la superficie (blanco/negro) depende del diseño específico del módulo sensor. En muchos módulos comunes (como los basados en TCRT5000 con una configuración específica), una superficie blanca (alta reflexión) puede dar un valor analógico bajo (cercano a 0), mientras que una superficie negra (baja reflexión) puede dar un valor analógico alto (cercano a 1023 o VCC).27 Sin embargo, en otros diseños o configuraciones, esta relación puede invertirse.10 Es crucial verificar experimentalmente cómo responden sus sensores específicos colocando el robot sobre superficies blancas y negras y observando los valores leídos (por ejemplo, usando el Monitor Serie del IDE de Arduino).
Ventaja: La lectura analógica proporciona información sobre la intensidad de la reflexión, no solo un sí/no. Esto permite determinar con mayor precisión qué tan centrado está el robot sobre la línea o cuánto se ha desviado, lo cual es fundamental para algoritmos suaves y precisos como el promedio ponderado y el PID.7



Lectura Digital (Alternativa Menos Recomendada):

Funcionamiento: Algunos módulos IR incluyen un comparador (como el LM393) y un potenciómetro. El potenciómetro ajusta un umbral de voltaje. La salida digital (DO) del módulo será HIGH o LOW dependiendo de si el voltaje del sensor está por encima o por debajo de este umbral.26
Conexión: Conectaría las salidas DO de los 6 sensores a 6 pines digitales del Arduino.
Función: Usaría digitalRead(pin) para leer HIGH o LOW.20
Interpretación: HIGH podría significar línea negra y LOW línea blanca, o viceversa, dependiendo del módulo y el ajuste del potenciómetro.3
Desventaja: Proporciona solo información binaria (encima/fuera de la línea), lo que dificulta la implementación de control proporcional fino y PID, pudiendo resultar en un movimiento más brusco o "zigzagueante".42 Para los objetivos de este proyecto (manejar giros complejos y bifurcaciones), la lectura analógica es la opción preferida.


3.3. Calibración de SensoresLa calibración es un paso crítico para obtener lecturas fiables y consistentes de los sensores IR analógicos.
¿Por qué calibrar?

Luz Ambiental: La luz del entorno puede afectar las lecturas del sensor.31
Altura del Sensor: La distancia entre el sensor y la superficie influye en la cantidad de luz reflejada captada.31
Variaciones entre Sensores: Incluso sensores del mismo modelo pueden tener ligeras diferencias en su respuesta.9
Superficie: Las características exactas del blanco y negro de la pista pueden variar.


Método de Calibración (Analógica):

Implementar Rutina: Añada una rutina de calibración en la función setup() del código Arduino.
Movimiento: Durante esta rutina (p. ej., durante 5-10 segundos), mueva manualmente el conjunto de sensores del robot lentamente de lado a lado, barriendo completamente la línea negra sobre el fondo blanco.9
Registro: Para cada uno de los 6 sensores, registre los valores analógicos mínimos y máximos leídos durante el barrido.9 Almacene estos 12 valores (un mínimo y un máximo por sensor).
Uso: Estos valores calibrados pueden usarse de varias maneras:

Normalización: Escalar las lecturas futuras de cada sensor a un rango fijo (p. ej., 0 a 1000) usando sus valores min/max calibrados. Esto compensa las diferencias entre sensores. La biblioteca QTRSensors de Pololu a menudo realiza esto.7
Umbral Dinámico: Calcular un umbral para cada sensor (p. ej., el punto medio mid = (min_calibrado + max_calibrado) / 2) para convertir la lectura analógica en un valor binario (0 o 1) si es necesario para ciertas lógicas.27




Calibración (Digital): Si se usan salidas digitales, la calibración implica ajustar manualmente el potenciómetro de cada módulo. Coloque el sensor sobre la superficie blanca y ajuste el potenciómetro hasta que el LED indicador (si existe) o la salida digital cambie de estado justo en el borde de la línea negra.2 Este método es menos flexible y más sensible a cambios en las condiciones.
3.4. Posibles Problemas
Ruido/Interferencia: Las lecturas analógicas pueden ser susceptibles al ruido eléctrico.43 Promediar varias lecturas consecutivas puede ayudar a suavizar los datos.43
Luz Ambiental: Cambios fuertes en la luz ambiental pueden afectar las lecturas. Intentar proteger los sensores de la luz directa puede ser útil.17
Interferencia Cruzada (Crosstalk): La luz emitida por un sensor podría ser detectada por el receptor de un sensor adyacente, especialmente si están muy juntos. Tubos o separadores alrededor de cada par emisor/receptor pueden minimizar esto.43
4. Control de Motores DC con Driver L298NEl módulo L298N actúa como intermediario entre las señales de bajo voltaje del Arduino y la mayor potencia requerida por los motores DC. Permite controlar tanto la velocidad como la dirección de dos motores de forma independiente.14.1. Descripción General del L298NEste módulo integra un chip L298N, que contiene dos circuitos H-Bridge completos.4 Los pines clave son:
Entradas de Alimentación: VCC (o similar) para la alimentación de los motores (5V-35V) y GND.1
Entrada/Salida Lógica de 5V: Para alimentar la lógica interna del L298N y/o el Arduino (ver Sección 2.3).1
Pines de Control de Dirección (IN1, IN2, IN3, IN4): Pines digitales que determinan la dirección de giro de cada motor.1
Pines de Habilitación (ENA, ENB): Permiten activar/desactivar cada motor y controlar su velocidad mediante PWM.1
Salidas de Motor (OUT1, OUT2, OUT3, OUT4): Terminales donde se conectan los motores.1
4.2. Control de DirecciónUn H-Bridge permite invertir la polaridad del voltaje aplicado al motor, cambiando así su dirección de giro. Se controla mediante los pines IN:
Motor A (conectado a OUT1/OUT2):

Adelante: IN1 = HIGH, IN2 = LOW.4
Atrás: IN1 = LOW, IN2 = HIGH.4
Parar/Frenar: IN1 = LOW, IN2 = LOW (o HIGH/HIGH).


Motor B (conectado a OUT3/OUT4):

Adelante: IN3 = HIGH, IN4 = LOW.4
Atrás: IN3 = LOW, IN4 = HIGH.4
Parar/Frenar: IN3 = LOW, IN4 = LOW (o HIGH/HIGH).


En Arduino, esto se implementa usando la función digitalWrite(pin, estado) en los pines conectados a IN1-IN4.4.3. Control de Velocidad (PWM)La velocidad del motor se controla aplicando una señal PWM a los pines de habilitación ENA y ENB del L298N, que deben estar conectados a pines PWM del Arduino.1
Función: analogWrite(pin_ENA_o_ENB, valor_velocidad).1
Valor: valor_velocidad es un entero entre 0 y 255.3

0: Motor detenido (o velocidad mínima).
255: Velocidad máxima.
Valores intermedios: Velocidad proporcional.


La técnica PWM ajusta el "ciclo de trabajo" (duty cycle) de la señal, cambiando el voltaje promedio aplicado al motor y, por lo tanto, su velocidad.4.4. Funciones Básicas de Control de Motores (Código Arduino)Es útil crear funciones reutilizables para controlar el movimiento del robot. Aquí hay ejemplos conceptuales (los números de pin deben coincidir con su configuración de la Sección 2.4):C++// Definición de pines (Ejemplo - ¡Ajustar según su cableado!)
#define ENA 9   // PWM Motor Izquierdo (A)
#define IN1 8
#define IN2 7
#define ENB 6   // PWM Motor Derecho (B)
#define IN3 5
#define IN4 4

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors(); // Asegurar que los motores estén parados al inicio
}

// Mover ambos motores hacia adelante a una velocidad dada (0-255)
void moveForward(int speed) {
  digitalWrite(IN1, HIGH); // Motor Izquierdo Adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Motor Derecho Adelante
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); // Velocidad Motor Izquierdo
  analogWrite(ENB, speed); // Velocidad Motor Derecho
}

// Mover ambos motores hacia atrás a una velocidad dada (0-255)
void moveBackward(int speed) {
  digitalWrite(IN1, LOW);  // Motor Izquierdo Atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Motor Derecho Atrás
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Girar a la izquierda (pivotando sobre el centro)
// Motor Izquierdo atrás, Motor Derecho adelante
void turnLeft(int speed) {
  digitalWrite(IN1, LOW);  // Motor Izquierdo Atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // Motor Derecho Adelante
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Girar a la derecha (pivotando sobre el centro)
// Motor Izquierdo adelante, Motor Derecho atrás
void turnRight(int speed) {
  digitalWrite(IN1, HIGH); // Motor Izquierdo Adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Motor Derecho Atrás
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Detener ambos motores (freno activo)
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); // También se puede poner velocidad 0
  analogWrite(ENB, 0);
}

void loop() {
  // Ejemplo de uso:
  moveForward(150);
  delay(2000);
  turnLeft(100);
  delay(1000);
  stopMotors();
  delay(5000);
}
4.5. Uso de Bibliotecas (Alternativa)Existen bibliotecas como L298N.h 46 que simplifican el control de los motores. Esta biblioteca encapsula las operaciones digitalWrite y analogWrite en funciones más intuitivas. Para controlar dos motores, se utiliza la variante L298NX2.h.46Ejemplo de uso con L298NX2.h:C++#include <L298NX2.h>

// Definir pines Arduino conectados al L298N
#define ENA_PIN 9
#define IN1A_PIN 8
#define IN2A_PIN 7
#define ENB_PIN 6
#define IN1B_PIN 5
#define IN2B_PIN 4

// Crear instancia para dos motores
L298NX2 motors(ENA_PIN, IN1A_PIN, IN2A_PIN, ENB_PIN, IN1B_PIN, IN2B_PIN);

void setup() {
  // La biblioteca maneja la configuración de pines
}

void loop() {
  // Mover Motor A (Izquierdo) adelante a velocidad 200
  motors.setSpeedA(200);
  motors.forwardA();
  delay(1000);

  // Mover Motor B (Derecho) atrás a velocidad 150
  motors.setSpeedB(150);
  motors.backwardB();
  delay(1000);

  // Detener ambos motores
  motors.stop();
  delay(2000);
}
El uso de bibliotecas puede acelerar el desarrollo, pero comprender el control directo de pines es valioso para entender los principios subyacentes y para la depuración avanzada.5. Algoritmo Central de Seguimiento de Línea con 6 SensoresCon los sensores y motores bajo control, el siguiente paso es implementar la lógica que utiliza las lecturas de los sensores para guiar al robot a lo largo de la línea. Un método eficaz para 6 sensores analógicos es el cálculo de la posición mediante promedio ponderado, combinado con un control proporcional.5.1. Lectura del Conjunto de 6 SensoresPrimero, es necesario leer los valores analógicos de los 6 sensores (conectados a A0-A5) en cada ciclo del bucle principal (loop()). Estos valores se pueden almacenar en un array:C++int sensorPins = {A0, A1, A2, A3, A4, A5}; // Pines analógicos para los 6 sensores
int sensorValues; // Array para almacenar las lecturas

void readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Aquí se podría aplicar la calibración/normalización si se implementó
  }
}
5.2. Cálculo de la Posición/Error - Método del Promedio PonderadoEste método proporciona una estimación continua de la posición de la línea negra en relación con el centro del conjunto de sensores, lo cual es mucho más informativo que simples estados discretos (como "sensor izquierdo activado").10

Asignar Pesos: A cada sensor se le asigna un peso numérico que representa su posición. Los sensores más alejados del centro reciben pesos de mayor magnitud.9 Una posible asignación para 6 sensores, donde el centro ideal de la línea está entre el sensor 2 y 3, podría ser:

Sensor 0 (más a la izquierda): -5
Sensor 1: -3
Sensor 2: -1
Sensor 3: 1
Sensor 4: 3
Sensor 5 (más a la derecha): 5
(Estos valores pueden escalarse, por ejemplo, a -2500, -1500, -500, 500, 1500, 2500 para obtener un rango mayor, similar a como lo hacen algunas bibliotecas 7).



Calcular Promedio Ponderado: Se calcula una suma ponderada de las lecturas de los sensores y se divide por la suma total de las lecturas (para normalizar).8 Antes de esto, es útil convertir las lecturas analógicas crudas (0-1023) en una medida de "cuán negro" está detectando cada sensor (por ejemplo, escalando después de la calibración para que 0 represente blanco puro y 1000 negro puro).

C++long weightedSum = 0;
long sumOfReadings = 0;
int weights = {-5, -3, -1, 1, 3, 5}; // O pesos escalados

readSensors(); // Obtener las lecturas actuales en sensorValues
                // Asumimos que sensorValues ya están calibrados/escalados
                // donde un valor más alto significa más negro

for (int i = 0; i < 6; i++) {
  weightedSum += (long)sensorValues[i] * weights[i];
  sumOfReadings += sensorValues[i];
}

int position = 0; // Valor por defecto si no se detecta línea
if (sumOfReadings > 0) { // Evitar división por cero si todos leen 0 (blanco)
  position = weightedSum / sumOfReadings;
}
// 'position' ahora contiene la posición estimada de la línea.
// 0 = perfectamente centrado (entre sensor 2 y 3)
// Negativo = línea desviada a la izquierda
// Positivo = línea desviada a la derecha

Error: En este contexto, el valor position calculado representa directamente el error del robot con respecto a la posición deseada (el centro, que corresponde a una position de 0). Por lo tanto, error = position.
5.3. Control Proporcional (P-Control)El control proporcional ajusta la dirección del robot aplicando una corrección que es directamente proporcional al error calculado.9

Calcular Ajuste: correction = Kp * error

error es la position calculada anteriormente.
Kp es la Ganancia Proporcional, una constante que debe ser ajustada experimentalmente (tuning). Determina la fuerza de la reacción al error.



Ajustar Velocidad de Motores: La corrección se utiliza para modificar la velocidad base de los motores. Si el error es positivo (línea a la derecha), el robot debe girar a la derecha (motor derecho más lento, motor izquierdo más rápido). Si el error es negativo (línea a la izquierda), debe girar a la izquierda (motor izquierdo más lento, motor derecho más rápido).8

C++int baseSpeed = 150; // Velocidad base cuando el robot va recto (ajustable)
float Kp = 0.5;      // Ganancia Proporcional (¡Necesita ajuste!)

// Calcular error (position) como se mostró antes
int error = position;

// Calcular corrección
int correction = Kp * error;

// Calcular velocidades finales para cada motor
int rightMotorSpeed = baseSpeed - correction;
int leftMotorSpeed = baseSpeed + correction;

// Limitar las velocidades al rango PWM válido (0-255)
rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

// Aplicar velocidades a los motores (usando funciones como las de la Sección 4.4)
// Asumiendo que moveForward ajusta velocidades individuales si se modifica:
// setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
// setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
// O usando funciones directas:
analogWrite(ENA, leftMotorSpeed);
analogWrite(ENB, rightMotorSpeed);
// Asegurarse de que la dirección esté configurada para adelante
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
Este enfoque P con promedio ponderado proporciona un seguimiento de línea mucho más suave que los métodos basados en lógica digital simple, pero aún puede presentar oscilaciones a altas velocidades o en curvas cerradas.76. Estrategia de Control Avanzada: Implementación PIDPara lograr un rendimiento superior, especialmente a altas velocidades y en pistas complejas, se recomienda implementar un controlador PID (Proporcional-Integral-Derivativo). El PID mejora el control proporcional al considerar no solo el error actual, sino también los errores pasados y la tendencia futura del error.76.1. ¿Por qué PID?
Suavidad y Estabilidad: Reduce las oscilaciones (zigzagueo) comunes en el control P simple.7
Precisión: Puede eliminar el error de estado estacionario (cuando el robot sigue la línea pero ligeramente desviado).
Rendimiento: Permite al robot seguir la línea de manera más rápida y fiable, incluso en curvas cerradas.49
6.2. Componentes PID ExplicadosEl controlador PID calcula una salida de control basada en tres términos:
Proporcional (P): Reacciona a la magnitud del error actual (error = setpoint - position). Es el componente principal de la corrección.7 Una ganancia Kp alta acelera la respuesta pero puede causar inestabilidad.7
Pterm​=Kp​×error
Integral (I): Suma los errores pasados a lo largo del tiempo. Este término ayuda a eliminar el error residual (estado estacionario) que el control P por sí solo podría no corregir.8 Si la ganancia Ki es demasiado alta, puede causar "wind-up" (acumulación excesiva) y overshoot (sobrepasar la línea).8 En algunos seguidores de línea simples, este término se omite si el error de estado estacionario no es un problema significativo.9
Iterm​=Ki​×∑error×Δt (En código: integral += error; I_term = Ki * integral;)
Derivativo (D): Reacciona a la tasa de cambio del error (rateOfChange = (error - previousError) / deltaTime). Actúa como un freno, anticipando el comportamiento futuro del error y amortiguando las oscilaciones.7 Mejora la estabilidad y reduce el overshoot.8 Una ganancia Kd demasiado alta puede amplificar el ruido de los sensores y causar vibraciones o inestabilidad.18
Dterm​=Kd​×Δterror−previousError​ (En código: derivative = error - lastError; D_term = Kd * derivative;)
6.3. Cálculo PIDEn cada ciclo del loop():
Lea los sensores y calcule la position actual (usando promedio ponderado, Sección 5.2).
Calcule el error = 0 - position (asumiendo que el setpoint es 0).
Calcule el término Integral (acumulando el error).
Calcule el término Derivativo (usando el error actual y el lastError del ciclo anterior).
Calcule la salida PID total: PID_output = (Kp * error) + (Ki * integral) + (Kd * derivative).8
Actualice lastError = error para el siguiente ciclo.
6.4. Aplicación de la Salida PID a los MotoresLa PID_output se usa para ajustar la velocidad de los motores de manera similar al control P:C++int baseSpeed = 150; // Ajustable
float Kp = 0.6;      // ¡Necesita ajuste!
float Ki = 0.001;    // ¡Necesita ajuste! (Puede empezar en 0)
float Kd = 0.8;      // ¡Necesita ajuste!

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float PID_output = 0;

// Dentro del loop():
// 1. Leer sensores y calcular 'position' (promedio ponderado)
//...
error = 0 - position; // Calcular error (Setpoint = 0)

integral += error; // Acumular error para el término I
// Opcional: Limitar el término integral (anti-windup)
// if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
// else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

derivative = error - lastError; // Calcular cambio en el error para el término D

PID_output = (Kp * error) + (Ki * integral) + (Kd * derivative); // Calcular salida PID

lastError = error; // Guardar error actual para el próximo ciclo

// Calcular velocidades finales
int rightMotorSpeed = baseSpeed - PID_output;
int leftMotorSpeed = baseSpeed + PID_output;

// Limitar velocidades (0-255)
rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

// Aplicar velocidades a los motores (usando ENA, ENB y IN1-IN4)
//... (similar al ejemplo P-Control)
6.5. Ajuste de Constantes PID (Tuning)El ajuste (tuning) de las ganancias Kp, Ki y Kd es la parte más crítica y a menudo requiere prueba y error.8 No hay valores universales; dependen de la mecánica del robot, velocidad, sensores, superficie, etc..18Guía Sistemática de Ajuste:GananciaEfecto al AumentarPaso de AjusteObjetivo de ObservaciónKpRespuesta más rápida, Mayor oscilación1. Poner Ki=0, Kd=0. Empezar Kp bajo. Aumentar Kp hasta oscilación estable.Encontrar punto de oscilación. Reducir Kp un poco.KiReduce error estacionario, Puede aumentar el overshoot2. Mantener Kp. Empezar Ki=0. Aumentar Ki lentamente.Robot se centra perfectamente en la línea.KdAmortigua oscilación, Mejora estabilidad, Sensible al ruido3. Mantener Kp, Ki. Empezar Kd bajo. Aumentar Kd lentamente.Reducir/eliminar oscilación, respuesta estable.Consejos para el Ajuste:
Realice los ajustes en la pista real donde competirá el robot.
Ajuste una constante a la vez.
Realice cambios pequeños en los valores.
Utilice la comunicación Serial para imprimir los valores de error, P, I, D y las velocidades de los motores, lo que ayuda a entender el comportamiento.28
Considere usar un módulo Bluetooth para ajustar las constantes en tiempo real desde un teléfono o PC, lo que acelera mucho el proceso.18
7. Navegación de Características Complejas de la Pista (Giros de 90° y Bifurcaciones en T)Mientras que el control PID es excelente para seguir líneas curvas y rectas, los giros muy cerrados (90°) y las intersecciones (como las bifurcaciones en T) a menudo requieren una lógica específica adicional. El algoritmo PID estándar podría perder la línea o comportarse de manera impredecible en estas situaciones.117.1. Necesidad de Lógica EspecíficaEl valor de error del PID puede volverse muy grande en un giro cerrado, pero el controlador no distingue inherentemente esta situación de estar simplemente muy desviado de una línea recta. Las bifurcaciones presentan patrones de sensores que el PID básico no está diseñado para interpretar. Por lo tanto, se necesita detectar estas características específicas y activar maniobras predefinidas, pausando temporalmente el control PID normal.7.2. Detección y Ejecución de Giros de 90 Grados
Detección (Patrones de Sensores): Un giro brusco se puede identificar analizando el estado de los 6 sensores. Busque patrones donde varios sensores consecutivos en un lado detecten negro mientras que los del otro lado detectan blanco.

Ejemplo Giro a la Derecha (90°): Los sensores 0, 1, 2 (izquierda) leen blanco (valor bajo/calibrado), mientras que los sensores 3, 4, 5 (derecha) leen negro (valor alto/calibrado). O quizás un patrón como o.13 La clave es que la línea "desaparece" del centro y aparece fuertemente en un lado.
Ejemplo Giro a la Izquierda (90°): Patrón inverso, p. ej., o.
Diferenciación: Para distinguir un giro de 90° de una simple desviación grande, se puede requerir que el patrón persista durante unas pocas lecturas o que un número mínimo de sensores (p. ej., 3 o 4) en el lado del giro detecten negro.13


Ejecución:

Pausar PID: Desactivar temporalmente el cálculo y la aplicación de la salida PID a los motores.
Realizar Giro: Ejecutar una maniobra de giro brusco. Hay dos enfoques principales:

Giro Temporizado: Hacer girar los motores en direcciones opuestas (giro de pivote) o uno parado y otro girando (giro de barrido) durante un tiempo fijo (delay()).11 Este método requiere un ajuste preciso del tiempo, que puede variar con el voltaje de la batería.
Giro Basado en Sensores: Hacer girar los motores bruscamente (p. ej., pivote) y continuar girando hasta que los sensores detecten que el robot se ha realineado con el nuevo segmento de línea (p. ej., hasta que los sensores centrales vuelvan a detectar negro, o se alcance un patrón ``).51 Este método es generalmente más robusto y adaptable.


Reanudar PID: Una vez completado el giro, reactivar el control PID normal.


Control de Motores para Giro Brusco: Utilizar funciones como turnLeft(velocidad_giro) y turnRight(velocidad_giro) (ver Sección 4.4) con una velocidad adecuada para el giro.6


7.3. Detección y Manejo de Bifurcaciones en T
Detección (Patrón de Sensores): Una bifurcación en T (o una intersección en '+') se caracteriza típicamente porque todos (o casi todos) los sensores detectan la línea negra simultáneamente.12 El patrón ideal sería ``.

Fiabilidad: La detección puede ser complicada. Si el robot entra en la T ligeramente girado, podría activar la lógica de giro de 90° antes de reconocer la T.13 Para mejorar la fiabilidad:

Requerir que el patrón "todo negro" se mantenga durante varias lecturas consecutivas.
Verificar que el robot estaba razonablemente centrado antes de detectar el patrón "todo negro".
Considerar el uso de sensores adicionales más separados o en diferentes ángulos (aunque esto va más allá del alcance de 6 sensores frontales).13




Manejo: Una vez detectada una T, el robot necesita una estrategia para decidir qué camino tomar:

Giro Predeterminado: Programar al robot para que siempre gire a la izquierda (o siempre a la derecha) en cada T encontrada. Esta es la estrategia más simple.
Parar: Simplemente detener el robot al llegar a la T.12
Ruta Preprogramada: Si la pista es conocida (como en algunas competiciones), se puede usar un contador de intersecciones o una secuencia almacenada para decidir el giro en cada T específica.6 Por ejemplo: "En la primera T, gira a la izquierda; en la segunda T, sigue recto (si es posible); en la tercera T, gira a la derecha".
Algoritmo de Exploración: Para laberintos desconocidos, se podrían implementar algoritmos como la regla de la mano izquierda/derecha 11 (esto añade complejidad significativa).


Ejecución:

Pausar PID: Similar a los giros de 90°.
Maniobra Opcional: Puede ser útil avanzar una pequeña distancia fija después de detectar la T para asegurarse de que el centro de rotación del robot esté sobre la intersección antes de girar.12
Ejecutar Giro Decidido: Realizar el giro elegido (izquierda 90°, derecha 90°, o continuar recto si esa es la decisión y la pista lo permite) usando una maniobra temporizada o basada en sensores.
Reanudar PID: Volver al seguimiento de línea normal.


7.4. Tabla de Lógica para Giros y Bifurcaciones (Ejemplo)Esta tabla resume posibles patrones de sensores (simplificados a Blanco/Negro después de aplicar umbral o calibración) y las acciones correspondientes. 'B' indica Negro (línea detectada), 'W' indica Blanco.CaracterísticaPatrón Sensores (S0-S5)DescripciónAcción del RobotRecta / Curva Suave``CentradoControl PID NormalRecta / Curva Suave``Lig. IzquierdaControl PID Normal (Corregirá a la derecha)Recta / Curva Suave``Lig. DerechaControl PID Normal (Corregirá a la izquierda)Giro 90° Izquierda``Detectando giro brusco a la izq.Pausar PID, Ejecutar Giro Izquierda 90° (hasta centrar), Reanudar PIDGiro 90° Derecha``Detectando giro brusco a la der.Pausar PID, Ejecutar Giro Derecha 90° (hasta centrar), Reanudar PIDBifurcación en T``Detectando intersecciónPausar PID, Avanzar un poco (opcional), Decidir y Ejecutar Giro T, Reanudar PIDLínea Perdida[W, W, W, W, W, W]Ningún sensor detecta líneaPausar PID, Buscar Línea (p. ej., barrido izq/der) o PararNota: Los patrones exactos (B vs W) dependen de la calibración y la lógica específica (si B corresponde a valor alto o bajo). La implementación debe ser robusta frente a lecturas intermedias o ruidosas.8. Ejemplos Completos de Código ArduinoA continuación, se presentan esqueletos de código que ilustran cómo implementar las estrategias discutidas. Estos códigos deben adaptarse a su hardware específico (pines) y requieren un ajuste cuidadoso de las constantes (velocidad base, Kp, Ki, Kd, umbrales de detección, tiempos de giro).8.1. Estructura General del CódigoUn programa típico de seguidor de línea en Arduino tendrá la siguiente estructura:C++// --- Inclusiones de Bibliotecas (si se usan) ---
// #include <L298NX2.h> // Ejemplo si se usa biblioteca para motores

// --- Definiciones de Pines ---
// (Definir pines para ENA, IN1, IN2, ENB, IN3, IN4 y pines de sensores A0-A5)
//... ver Tabla 2.4...

// --- Constantes Globales ---
const int NUM_SENSORS = 6;
int baseSpeed = 150; // Velocidad base
//... otras constantes (Kp, Ki, Kd, umbrales, etc.)

// --- Variables Globales ---
int sensorValues;
//... variables para PID (error, lastError, integral)
//... variables de estado (ej. isTurning, junctionDetected)

// --- Funciones Auxiliares ---
// void readSensors() {... }
// int calculatePosition() {... } // Calcula error/posición (promedio ponderado)
// void moveForward(int speedL, int speedR) {... } // Controla ambos motores
// void turnLeftSharp() {... } // Maniobra de giro 90 izq
// void turnRightSharp() {... } // Maniobra de giro 90 der
// void handleTJunction() {... } // Lógica para bifurcaciones T
// void stopMotors() {... }
// void runPID() {... } // Calcula y aplica corrección PID

// --- Función setup() ---
void setup() {
  Serial.begin(9600); // Para depuración
  // Configurar pinMode para pines de motor
  //...
  // Opcional: Ejecutar rutina de calibración de sensores
  // calibrateSensors();
  // Esperar inicio (ej. presionar un botón)
}

// --- Función loop() ---
void loop() {
  readSensors();
  int position = calculatePosition();

  // --- Lógica de Detección de Giros/Bifurcaciones ---
  if (detectSharpLeftTurn(sensorValues)) {
    turnLeftSharp();
  } else if (detectSharpRightTurn(sensorValues)) {
    turnRightSharp();
  } else if (detectTJunction(sensorValues)) {
    handleTJunction();
  } else if (detectLineLost(sensorValues)) {
    // searchForLine() o stopMotors();
  } else {
    // --- Control Normal de Seguimiento de Línea (PID o Proporcional) ---
    runPID(position); // O runProportionalControl(position);
  }
}

// --- Implementación de Funciones Auxiliares ---
//... (implementar todas las funciones declaradas arriba)
// bool detectSharpLeftTurn(int values) { /* lógica de patrón */ return false; }
// bool detectSharpRightTurn(int values) { /* lógica de patrón */ return false; }
// bool detectTJunction(int values) { /* lógica de patrón */ return false; }
// bool detectLineLost(int values) { /* lógica de patrón */ return false; }
//...
8.2. Código Ejemplo 1: Control Proporcional + Promedio Ponderado + Lógica Giros/TEste ejemplo esboza la implementación usando control proporcional simple con lógica específica para giros y bifurcaciones.C++// --- Definiciones y Constantes (Adaptar) ---
#define ENA 9
#define IN1 8
#define IN2 7
#define ENB 6
#define IN3 5
#define IN4 4
int sensorPins = {A0, A1, A2, A3, A4, A5};
const int NUM_SENSORS = 6;
int sensorValues;
int weights = {-5, -3, -1, 1, 3, 5}; // Pesos para promedio ponderado

int baseSpeed = 120;
float Kp = 0.4; // Ganancia Proporcional (¡AJUSTAR!)

// Umbrales para detección (¡AJUSTAR según calibración!)
const int SENSOR_THRESHOLD_BLACK = 600; // Ejemplo: valor analógico > 600 es negro
const int SENSOR_THRESHOLD_WHITE = 400; // Ejemplo: valor analógico < 400 es blanco

void setup() {
  Serial.begin(9600);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
  // Añadir calibración si es necesario
  delay(1000); // Esperar antes de empezar
}

void loop() {
  readSensors();
  int position = calculatePosition();

  if (detectTJunction(sensorValues)) {
    handleTJunction_TurnLeft(); // Ejemplo: siempre gira a la izquierda
  } else if (detectSharpLeftTurn(sensorValues)) {
    turnLeftSharp_SensorBased();
  } else if (detectSharpRightTurn(sensorValues)) {
    turnRightSharp_SensorBased();
  } else if (detectLineLost(sensorValues)) {
     stopMotors(); // O implementar búsqueda
  } else {
    // Control Proporcional Normal
    runProportionalControl(position);
  }
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Serial.print(sensorValues[i]); Serial.print("\t"); // Depuración
  }
  // Serial.println(); // Depuración
}

int calculatePosition() {
  long weightedSum = 0;
  long sumOfReadings = 0; // Usar lecturas umbralizadas o normalizadas aquí
  bool lineDetected = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = sensorValues[i];
    // Simplificación: considerar negro si > umbral
    if (reading > SENSOR_THRESHOLD_BLACK) {
        weightedSum += (long)1000 * weights[i]; // Usar un valor fijo para 'negro'
        sumOfReadings += 1000;
        lineDetected = true;
    }
  }

  if (!lineDetected) {
    // Si no se detecta línea, devolver un error grande basado en la última posición
    // o un valor especial. Aquí devolvemos 0 por simplicidad.
    return 0;
  }
  return weightedSum / sumOfReadings;
}

void runProportionalControl(int error) {
  int correction = Kp * error;
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Asumiendo que IN1/IN3=HIGH, IN2/IN4=LOW es adelante
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

// --- Funciones de Detección y Maniobra (Implementación Simplificada) ---

bool detectTJunction(int values) {
  // Detecta si todos los sensores (o los 4 centrales) ven negro
  int blackCount = 0;
  for(int i = 0; i < NUM_SENSORS; i++) {
    if (values[i] > SENSOR_THRESHOLD_BLACK) blackCount++;
  }
  return (blackCount >= 5); // Umbral ajustable (5 o 6)
}

bool detectSharpLeftTurn(int values) {
  // Detecta si los 3-4 sensores más a la izquierda ven negro y los de la derecha blanco
  return (values > SENSOR_THRESHOLD_BLACK &&
          values > SENSOR_THRESHOLD_BLACK &&
          values > SENSOR_THRESHOLD_BLACK &&
          values < SENSOR_THRESHOLD_WHITE &&
          values < SENSOR_THRESHOLD_WHITE);
}

bool detectSharpRightTurn(int values) {
  // Detecta si los 3-4 sensores más a la derecha ven negro y los de la izquierda blanco
  return (values > SENSOR_THRESHOLD_BLACK &&
          values > SENSOR_THRESHOLD_BLACK &&
          values > SENSOR_THRESHOLD_BLACK &&
          values < SENSOR_THRESHOLD_WHITE &&
          values < SENSOR_THRESHOLD_WHITE);
}

bool detectLineLost(int values) {
    int whiteCount = 0;
    for(int i = 0; i < NUM_SENSORS; i++) {
        if (values[i] < SENSOR_THRESHOLD_WHITE) whiteCount++;
    }
    return (whiteCount == NUM_SENSORS); // Todos ven blanco
}

void handleTJunction_TurnLeft() {
  Serial.println("T Junction Detected - Turning Left");
  // 1. Avanzar un poco para centrar la T (opcional, ajustar distancia/tiempo)
  moveForwardBasic(80); // Función simple para mover adelante
  delay(150); // Ajustar tiempo/distancia

  // 2. Girar a la izquierda 90 grados
  turnLeftSharp_SensorBased(); // Usar la misma función de giro 90

  // 3. Reanudar seguimiento (se hará en el siguiente ciclo del loop)
}

void turnLeftSharp_SensorBased() {
  Serial.println("Sharp Left Turn");
  // Girar a la izquierda (pivote) hasta que los sensores centrales detecten la línea
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Izq Atrás
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Der Adelante
  analogWrite(ENA, 100); // Velocidad de giro (ajustar)
  analogWrite(ENB, 100);

  // Esperar hasta que los sensores 2 y 3 detecten negro (o un patrón centrado)
  while(!(analogRead(sensorPins) > SENSOR_THRESHOLD_BLACK && analogRead(sensorPins) > SENSOR_THRESHOLD_BLACK) ) {
     // Podría añadirse un timeout para evitar bucles infinitos
     delay(1);
  }
  // Opcional: Continuar girando un poco más o parar inmediatamente
  stopMotors();
  delay(50); // Pequeña pausa
}

void turnRightSharp_SensorBased() {
   Serial.println("Sharp Right Turn");
  // Girar a la derecha (pivote) hasta que los sensores centrales detecten la línea
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Izq Adelante
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Der Atrás
  analogWrite(ENA, 100); // Velocidad de giro (ajustar)
  analogWrite(ENB, 100);

  while(!(analogRead(sensorPins) > SENSOR_THRESHOLD_BLACK && analogRead(sensorPins) > SENSOR_THRESHOLD_BLACK) ) {
     delay(1);
  }
  stopMotors();
  delay(50);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void moveForwardBasic(int speed) { // Función simple sin P-control
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

Este código es ilustrativo y requiere pruebas y ajustes exhaustivos.8.3. Código Ejemplo 2: Control PID + Lógica Giros/TEste ejemplo integra el control PID en lugar del control proporcional simple.C++// --- Definiciones y Constantes (Igual que Ejemplo 1, más constantes PID) ---
//... (pines, sensores, pesos, baseSpeed, umbrales)...
float Kp = 0.5; // ¡AJUSTAR!
float Ki = 0.001; // ¡AJUSTAR! (Empezar en 0 si se desea)
float Kd = 0.7; // ¡AJUSTAR!

// Variables PID
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

void setup() {
  //... (Igual que Ejemplo 1)...
}

void loop() {
  readSensors();
  // No calculamos 'position' directamente aquí si la lógica de detección
  // usa los valores crudos o umbralizados.

  // --- Lógica de Detección Prioritaria ---
  if (detectTJunction(sensorValues)) {
    handleTJunction_TurnLeft();
    resetPID(); // Resetear PID después de maniobra especial
  } else if (detectSharpLeftTurn(sensorValues)) {
    turnLeftSharp_SensorBased();
    resetPID();
  } else if (detectSharpRightTurn(sensorValues)) {
    turnRightSharp_SensorBased();
    resetPID();
  } else if (detectLineLost(sensorValues)) {
     stopMotors();
     resetPID();
  } else {
    // --- Control PID Normal ---
    int position = calculatePosition(); // Calcular posición para PID
    runPID(position);
  }
}

// --- Funciones readSensors, calculatePosition, detección y maniobras ---
//... (Implementar igual o similar al Ejemplo 1)...

void runPID(int position) {
  error = 0 - position; // Error es la desviación del centro (setpoint 0)

  integral += error;
  // Añadir anti-windup si es necesario

  derivative = error - lastError;

  float PID_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  lastError = error;

  // Calcular y limitar velocidades
  int leftSpeed = baseSpeed + PID_output;
  int rightSpeed = baseSpeed - PID_output;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Aplicar velocidades (dirección adelante)
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Imprimir valores para depuración/ajuste (opcional)
  // Serial.print("Err:"); Serial.print(error);
  // Serial.print(" PID:"); Serial.print(PID_output);
  // Serial.print(" LSpd:"); Serial.print(leftSpeed);
  // Serial.print(" RSpd:"); Serial.println(rightSpeed);
}

void resetPID() {
  // Resetear variables PID para evitar saltos después de maniobras
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;
}

// --- Implementar resto de funciones auxiliares ---
//... (stopMotors, moveForwardBasic, etc.)...
Este código también es ilustrativo y necesita adaptación y ajuste.8.4. Presentación del Código
Comentarios: Añada comentarios claros explicando la lógica de cada sección, el propósito de las variables y las constantes.55
Formato: Utilice la función "Auto Format" (Ctrl+T) en el IDE de Arduino para mantener el código legible y ordenado.41
Modularidad: Divida el código en funciones lógicas (leer sensores, calcular error, controlar motores, detectar giros, etc.) para mejorar la organización y la reutilización.
Bibliotecas: Si utiliza bibliotecas externas (como L298N.h o QTRSensors.h), asegúrese de incluirlas correctamente (#include <LibraryName.h>) y consulte su documentación para el uso adecuado.7
GitHub: Considere alojar su código en un repositorio de GitHub para control de versiones y compartirlo con la comunidad.14
9. Solución de Problemas y Ajuste de RendimientoIncluso con un diseño y código cuidadosos, es probable que surjan problemas durante las pruebas. Esta sección aborda problemas comunes y ofrece consejos para optimizar el rendimiento del robot.9.1. Problemas Comunes y Soluciones
Robot No se Mueve:

Alimentación: Verifique que la batería esté cargada y conectada correctamente. Mida el voltaje. Asegúrese de que la batería pueda suministrar suficiente corriente para los motores.28
Cableado: Revise todas las conexiones, especialmente GND común entre Arduino, L298N y sensores. Verifique los pines de motor y control.36
L298N: Compruebe el estado del jumper 5V EN.4 Asegúrese de que los pines ENA y ENB estén recibiendo una señal PWM (si se usan para velocidad) y no estén permanentemente en LOW.
Código: Verifique que las funciones de motor se estén llamando correctamente y que los pines estén definidos como OUTPUT.


Motores Giran en Dirección Incorrecta:

Solución Física: Invierta los dos cables de conexión del motor problemático en los terminales OUT del L298N.15
Solución Software: Invierta la lógica de los pines IN para ese motor en el código (p. ej., si HIGH/LOW era adelante, cámbielo a LOW/HIGH).16


Movimiento Errático / No Sigue la Línea:

Lecturas de Sensores: Use Serial.print() para mostrar los valores leídos por cada sensor en el Monitor Serie del IDE.2 Verifique si responden correctamente al blanco y negro.
Calibración: Realice o ajuste la calibración de los sensores.2 Los umbrales incorrectos son una causa común de problemas.
Altura/Alineación Sensores: Asegúrese de que los sensores estén a la altura correcta (aprox. 3mm) y alineados perpendicularmente a la dirección de avance.31
Luz Ambiental: Pruebe en diferentes condiciones de iluminación; la luz solar directa o sombras pueden interferir.31 Considere añadir protecciones laterales a los sensores.17
Lógica del Algoritmo: Revise cuidadosamente la lógica de cálculo de error y la aplicación de la corrección a los motores. Un error en el signo o la fórmula puede causar comportamiento incorrecto.


Robot Oscila (Zigzaguea) - Control PID:

Ajuste PID: Es el problema más común con PID. Siga la guía de ajuste (Sección 6.5).7 Generalmente, implica reducir Kp o aumentar Kd.8


Robot Demasiado Lento/Rápido:

Velocidad Base: Ajuste la variable baseSpeed en el código.3
Batería: Una batería baja reducirá la velocidad.
Caída de Voltaje L298N: Recuerde que el L298N consume voltaje, por lo que los motores no recibirán el voltaje completo de la batería.24


Motores Desbalanceados (Robot se Desvía en Recta):

Mecánica: Verifique que no haya fricción desigual en las ruedas o motores.
Ajuste Software: Introduzca una pequeña compensación constante en las velocidades de los motores en el código. Si el robot se desvía a la derecha, aumente ligeramente la velocidad base del motor derecho o disminuya la del izquierdo.22


Giros Inexactos (90°/T):

Giros Temporizados: Ajuste el valor delay() de la maniobra.11
Giros Basados en Sensores: Revise las condiciones de los sensores que determinan el final del giro. Podrían ser demasiado estrictas o laxas.51 Ajuste la velocidad de giro.
Ajuste PID (si aplica): Si el PID interviene demasiado pronto o tarde, ajuste la lógica de detección o las constantes PID.


Detección de Bifurcaciones Falla:

Patrón de Sensores: Revise el patrón `` (o similar). ¿Se detecta de forma fiable? Ajuste los umbrales.13
Robustez: Considere requerir que el patrón se mantenga durante varias lecturas o añadir condiciones adicionales (como estar centrado previamente) para evitar falsos positivos.13


9.2. Consejos para Optimizar el Rendimiento
Ajuste Fino PID: Dedique tiempo a ajustar Kp, Ki y Kd para la pista y velocidad deseadas. Es un proceso iterativo clave.
Velocidad Adaptativa: Para un rendimiento avanzado, considere variar baseSpeed. Aumentarla en tramos rectos y reducirla antes de curvas cerradas. Esto podría requerir lógica adicional basada en patrones de sensores (p. ej., si el error es consistentemente bajo durante un tiempo, aumentar velocidad) o el uso de un giroscopio/IMU.9
Minimizar Peso: Un robot más ligero acelera y gira más rápido. Use materiales ligeros para el chasis y componentes compactos.
Maximizar Tracción: Use ruedas con buen agarre para evitar deslizamientos, especialmente durante giros rápidos.
Optimizar Código: Evite el uso excesivo de delay() dentro del bucle principal de control, ya que detiene la ejecución e impide reacciones rápidas. Use técnicas basadas en millis() para tareas temporizadas no bloqueantes. Asegúrese de que el ciclo del loop() sea lo más rápido posible.
10. ConclusiónEste informe ha proporcionado una guía detallada para construir y programar un robot seguidor de línea avanzado utilizando Arduino, 6 sensores IR y un controlador L298N, con la capacidad de navegar por giros de 90 grados y bifurcaciones en T. Se han cubierto los aspectos fundamentales del hardware, el funcionamiento y lectura de los sensores IR, el control de motores DC, y diversas estrategias de control, desde el proporcional básico hasta el PID avanzado.Se ha enfatizado la importancia de la lectura analógica de los sensores para implementar algoritmos como el promedio ponderado, que proporciona una estimación continua del error necesaria para un control suave. La implementación del controlador PID, aunque requiere un ajuste cuidadoso de las constantes Kp, Ki y Kd, ofrece una mejora significativa en estabilidad y rendimiento en comparación con métodos más simples. Además, se ha detallado la lógica específica basada en patrones de sensores para detectar y ejecutar maniobras en giros cerrados y bifurcaciones en T, superando las limitaciones del control de seguimiento de línea estándar.La calibración de los sensores y el ajuste (tuning) del algoritmo de control son pasos cruciales que requieren experimentación y paciencia para lograr un rendimiento óptimo. La solución de problemas es una parte integral del proceso de desarrollo en robótica.Este proyecto sirve como una excelente plataforma para aprender sobre sistemas de control, integración de sensores y programación de microcontroladores. A partir de aquí, existen numerosas vías para la mejora y la exploración:
Navegación de Laberintos: Implementar algoritmos de exploración y mapeo para resolver laberintos de forma autónoma.11
Evasión de Obstáculos: Añadir sensores ultrasónicos 35 o IR de distancia para detectar y evitar obstáculos en la pista.
Control de Movimiento Preciso: Utilizar encoders en los motores para medir la distancia recorrida y la velocidad de las ruedas con precisión, permitiendo giros más exactos y control de odometría.25
Estabilidad Mejorada: Incorporar una Unidad de Medición Inercial (IMU) o giroscopio para obtener datos de orientación, lo que puede ayudar a realizar giros más suaves o mantener una dirección recta con mayor precisión.9
Control y Ajuste Inalámbrico: Integrar un módulo Bluetooth para controlar el robot o ajustar los parámetros PID en tiempo real desde un dispositivo móvil u ordenador.18
Eficiencia Energética: Reemplazar el L298N por un controlador de motor basado en MOSFET para reducir las pérdidas de energía y mejorar la eficiencia.
Se anima al lector a experimentar con las configuraciones, ajustar los parámetros y explorar estas mejoras para profundizar en el fascinante campo de la robótica autónoma.# Robot Seguidor de Línea Avanzado con Arduino: Navegación de Giros y Bifurcaciones1. IntroducciónBienvenido a la guía para construir un robot seguidor de línea avanzado. Este proyecto se centra en el desarrollo de un vehículo autónomo capaz de navegar por una pista definida por una línea negra sobre un fondo blanco. El objetivo principal es no solo seguir la línea, sino también manejar características complejas de la pista, específicamente giros de 90 grados y bifurcaciones en T, utilizando una placa Arduino, seis sensores infrarrojos (IR) y dos motores de corriente continua (DC) [User Query].Los componentes de hardware clave que se utilizarán incluyen una placa Arduino (como el popular Arduino Uno, adecuado para principiantes 1), dos motores DC (comúnmente motores amarillos con caja reductora ), un controlador de motor L298N para gestionar la potencia de los motores 1, y un conjunto de seis sensores de reflectancia IR para detectar la línea.5Para lograr una navegación precisa y fiable, exploraremos diversas estrategias de control. Comenzaremos con un control proporcional básico, utilizaremos un método de promedio ponderado para determinar la posición de la línea con mayor precisión, y avanzaremos hacia la implementación de un controlador PID (Proporcional-Integral-Derivativo) para un rendimiento superior y estable.7 Además, se detallará la lógica específica necesaria para detectar y maniobrar a través de giros cerrados y bifurcaciones.11Este informe está estructurado para guiarle a través de cada paso del proceso:
Ensamblaje de Hardware y Conexiones: Detalla los componentes necesarios y cómo conectarlos correctamente.
Comprensión de los Sensores IR: Explica el funcionamiento de los sensores y las técnicas para leer sus datos.
Control de Motores DC con Driver L298N: Describe cómo comandar los motores usando el driver L298N.
Algoritmo Central de Seguimiento de Línea: Presenta el método de promedio ponderado y el control proporcional.
Implementación PID: Introduce el control PID para un rendimiento avanzado.
Navegación de Características Complejas: Detalla la lógica para giros de 90° y bifurcaciones en T.
Ejemplos de Código Arduino: Proporciona código funcional y comentado.
Solución de Problemas y Ajuste: Ofrece consejos para diagnosticar problemas y optimizar el rendimiento.
Conclusión: Resume los logros y sugiere posibles mejoras futuras.
2. Ensamblaje de Hardware y ConexionesUn ensamblaje correcto y conexiones fiables son fundamentales para el éxito del robot seguidor de línea. Esta sección detalla los componentes necesarios y los pasos para montarlos y conectarlos.2.1. Lista Detallada de Componentes
Placa Arduino: Se recomienda Arduino Uno por su facilidad de uso y disponibilidad de pines 1, aunque otras placas como Arduino Mega también son compatibles.14
Chasis del Robot: Puede ser de diversos materiales como madera contrachapada 15, acrílico 16, impreso en 3D 5, o un kit prefabricado.16 Es importante considerar el peso total del robot, ya que un menor peso permite una mayor aceleración y velocidad.
Motores DC (x2): Típicamente motores de hobby con caja reductora para obtener un par adecuado a bajas velocidades.1 Es crucial notar que los motores idénticos pueden tener ligeras variaciones en su rendimiento, lo que podría causar que el robot se desvíe ligeramente en tramos rectos; esto puede requerir ajustes en el software.22
Ruedas (x2): Deben coincidir con los ejes de los motores.19 La tracción es un factor importante a considerar para evitar deslizamientos.
Rueda Loca (x1): Proporciona un tercer punto de apoyo para la estabilidad y permite giros suaves.19
Controlador de Motor L298N: Un módulo popular y económico para controlar dos motores DC.1 Aunque es funcional, el L298N utiliza tecnología bipolar H-Bridge, que introduce una caída de voltaje significativa (a menudo 2V o más) entre la fuente de alimentación y los motores.24 Esta pérdida de voltaje reduce la eficiencia y puede ser particularmente notable con motores de bajo voltaje (por ejemplo, 5V o 6V), limitando su par y velocidad máxima. Los controladores basados en MOSFET son una alternativa mucho más eficiente, pero el L298N es adecuado para este proyecto de nivel intermedio debido a su bajo costo y amplia disponibilidad.
Sensores IR (x6): Se requiere un conjunto de 6 sensores de reflectancia IR. Puede ser un módulo integrado o sensores individuales (tipo TCRT5000).6 Se prefieren sensores con salida analógica para implementar algoritmos avanzados como PID o promedio ponderado.8
Fuente de Alimentación: Un paquete de baterías adecuado. Opciones comunes incluyen 6 pilas AA (proporcionando 9V si son alcalinas) 2, una batería LiPo de 7.4V 30, o una batería de 12V.1 El L298N requiere un voltaje para los motores entre 5V y 35V, y una alimentación lógica de 5V.3 Es crucial entender el jumper del regulador de 5V en el L298N.4 Evite usar baterías PP3 de 9V para alimentar los motores, ya que no pueden suministrar la corriente necesaria durante mucho tiempo.28
Cables Jumper: Para realizar las conexiones entre los componentes.16
Opcional: Interruptor de encendido/apagado 1, tornillos, tuercas, separadores para el montaje.2
2.2. Pasos de Ensamblaje
Montar Motores y Ruedas: Fije los motores al chasis y acople las ruedas a los ejes.15 Asegúrese de que los motores estén alineados correctamente para que el robot avance recto.15
Montar Rueda Loca: Fije la rueda loca en la parte delantera o trasera del chasis, según el diseño, para proporcionar estabilidad.
Montar Componentes Electrónicos: Fije la placa Arduino, el módulo L298N y el soporte de la batería al chasis.15 Considere la distribución del peso para mantener el centro de masa bajo y cerca de las ruedas motrices, lo que mejora la estabilidad y la tracción.
Montar Sensores IR: Fije los 6 sensores IR en la parte delantera del chasis, orientados hacia abajo.5 La altura óptima sobre la superficie suele ser de unos 3 mm (o 1/8 de pulgada).31 La separación entre los sensores y su distancia respecto al eje de las ruedas influyen en la capacidad del robot para detectar la línea y anticipar los giros.
2.3. Instrucciones de CableadoEs fundamental realizar las conexiones eléctricas correctamente. Se recomienda verificar cada conexión antes de alimentar el circuito. Un diagrama visual sería muy útil en un informe final.
Motores al L298N: Conecte los terminales del motor izquierdo a las salidas OUT1 y OUT2 del L298N, y los del motor derecho a OUT3 y OUT4.1 Si un motor gira en la dirección incorrecta después de cargar el código, simplemente invierta los cables de ese motor en los terminales del L298N.15 Si se utilizan 4 motores, conecte los pares de cada lado en paralelo, invirtiendo la polaridad de uno de los motores de cada par para que giren en la misma dirección.32
L298N a Arduino:

Control de Dirección: Conecte los pines IN1, IN2, IN3 e IN4 del L298N a pines digitales del Arduino.1 Estos pines controlan la dirección de giro de cada motor.
Control de Velocidad: Conecte los pines ENA y ENB del L298N a pines PWM del Arduino (p. ej., 3, 5, 6, 9, 10, 11 en Uno).1 Estos pines permiten controlar la velocidad de cada motor mediante modulación por ancho de pulso (PWM). Algunos ejemplos omiten estos pines y utilizan jumpers en el L298N , pero el control PWM es esencial para algoritmos como PID y para un movimiento suave.
Tierra (GND): Conecte un pin GND del L298N a un pin GND del Arduino para establecer una referencia de voltaje común.1
Alimentación Lógica L298N (5V): Este punto requiere atención especial debido al jumper "5V EN" o similar en el módulo L298N 4:

Si el voltaje de la batería del motor es > 12V O si alimenta el Arduino por separado (p. ej., USB): RETIRE el jumper. En este caso, debe suministrar 5V al pin de entrada de 5V del L298N (puede tomarlo del pin 5V del Arduino).
Si el voltaje de la batería del motor es <= 12V: MANTENGA el jumper puesto. El L298N generará su propia alimentación lógica de 5V a partir de la batería del motor. El pin 5V del L298N actuará como una salida de 5V que opcionalmente puede usar para alimentar el Arduino.4
Precaución: Nunca conecte el pin 5V del Arduino al pin 5V del L298N si el jumper está puesto, ya que estaría conectando dos fuentes de 5V, lo que podría dañar los componentes.34




Sensores IR a Arduino:

Alimentación: Conecte el pin VCC de todos los sensores al pin 5V del Arduino.1 Conecte el pin GND de todos los sensores a un pin GND del Arduino.1
Salida Analógica: Conecte los pines de salida (OUT, S, A0, etc., según el sensor) de los 6 sensores IR a 6 pines de entrada analógica del Arduino (A0 a A5).5 Aunque algunos ejemplos básicos usan entradas digitales 1, las entradas analógicas son preferibles para los algoritmos avanzados que se abordarán.


Fuente de Alimentación: Conecte el terminal positivo (+) de la batería al pin de entrada de voltaje del motor del L298N (a menudo etiquetado como 12V, VMS o VCC) y el terminal negativo (-) de la batería al pin GND del L298N.1 Si desea, puede intercalar un interruptor en el cable positivo de la batería.1
2.4. Tabla de Configuración de Pines (Ejemplo para Arduino Uno)Para facilitar el cableado y la programación, la siguiente tabla resume una posible asignación de pines. Es crucial verificar esta tabla con sus conexiones reales.Pin ArduinoComponentePin ComponenteFunción/Descripción9L298NENAVelocidad Motor A (Izquierdo) - PWM8L298NIN1Dirección Motor A - 17L298NIN2Dirección Motor A - 26L298NENBVelocidad Motor B (Derecho) - PWM5L298NIN3Dirección Motor B - 14L298NIN4Dirección Motor B - 2A0Sensor IR ArrayS1_OUTSalida Analógica Sensor 1 (Más Izq.)A1Sensor IR ArrayS2_OUTSalida Analógica Sensor 2A2Sensor IR ArrayS3_OUTSalida Analógica Sensor 3A3Sensor IR ArrayS4_OUTSalida Analógica Sensor 4A4Sensor IR ArrayS5_OUTSalida Analógica Sensor 5A5Sensor IR ArrayS6_OUTSalida Analógica Sensor 6 (Más Der.)5VL298N / SensoresVCC / +5VAlimentación 5VGNDL298N / SensoresGNDTierra ComúnVINBatería (+)L298N VCC Mot.Alimentación Motores (p.ej. 7.4V-12V)GNDBatería (-)L298N GNDTierra Batería3. Comprensión de los Sensores IR para Seguimiento de LíneaLos sensores infrarrojos (IR) son los "ojos" del robot seguidor de línea. Entender cómo funcionan y cómo interpretar sus lecturas es esencial para desarrollar un algoritmo de control eficaz.3.1. Principio de FuncionamientoEl principio se basa en la reflectividad diferencial de las superficies claras y oscuras a la luz infrarroja.2 Las superficies blancas o claras tienden a reflejar una gran cantidad de luz IR, mientras que las superficies negras u oscuras tienden a absorberla.2Cada módulo sensor IR típicamente contiene:
Un LED IR (Emisor): Emite un haz de luz infrarroja invisible hacia la superficie debajo del robot.2
Un Fotodiodo o Fototransistor (Receptor): Detecta la cantidad de luz infrarroja que es reflejada por la superficie.2
La intensidad de la luz reflejada captada por el receptor genera una señal eléctrica (generalmente un voltaje) que indica si el sensor está sobre una superficie clara u oscura.23.2. Lectura de Datos de los Sensores (6 Sensores)Con seis sensores montados en la parte frontal, el robot obtiene una visión más detallada de la posición de la línea. Para este proyecto, se recomienda encarecidamente utilizar las salidas analógicas de los sensores.

Lectura Analógica:

Conexión: Conecte las salidas de los seis sensores a los pines de entrada analógica del Arduino (A0 a A5).5
Función: Utilice la función analogRead(pin) en el código Arduino para leer el valor de voltaje en cada pin analógico.8
Valores: Esta función devuelve un valor entero entre 0 y 1023, que representa el voltaje medido (0V corresponde a 0, 5V corresponde a 1023 en un Arduino típico).2
Interpretación: La relación entre el valor leído y el color de la superficie (blanco/negro) depende del diseño específico del módulo sensor. En muchos módulos comunes (como los basados en TCRT5000 con una configuración específica), una superficie blanca (alta reflexión) puede dar un valor analógico bajo (cercano a 0), mientras que una superficie negra (baja reflexión) puede dar un valor analógico alto (cercano a 1023 o VCC).27 Sin embargo, en otros diseños o configuraciones, esta relación puede invertirse.10 Es crucial verificar experimentalmente cómo responden sus sensores específicos colocando el robot sobre superficies blancas y negras y observando los valores leídos (por ejemplo, usando el Monitor Serie del IDE de Arduino).
Ventaja: La lectura analógica proporciona información sobre la intensidad de la reflexión, no solo un sí/no. Esto permite determinar con mayor precisión qué tan centrado está el robot sobre la línea o cuánto se ha desviado, lo cual es fundamental para algoritmos suaves y precisos como el promedio ponderado y el PID.7



Lectura Digital (Alternativa Menos Recomendada):

Funcionamiento: Algunos módulos IR incluyen un comparador (como el LM393) y un potenciómetro. El potenciómetro ajusta un umbral de voltaje. La salida digital (DO) del módulo será HIGH o LOW dependiendo de si el voltaje del sensor está por encima o por debajo de este umbral.26
Conexión: Conectaría las salidas DO de los 6 sensores a 6 pines digitales del Arduino.
Función: Usaría digitalRead(pin) para leer HIGH o LOW.20
Interpretación: HIGH podría significar línea negra y LOW línea blanca, o viceversa, dependiendo del módulo y el ajuste del potenciómetro.3
Desventaja: Proporciona solo información binaria (encima/fuera de la línea), lo que dificulta la implementación de control proporcional fino y PID, pudiendo resultar en un movimiento más brusco o "zigzagueante".42 Para los objetivos de este proyecto (manejar giros complejos y bifurcaciones), la lectura analógica es la opción preferida.


3.3. Calibración de SensoresLa calibración es un paso crítico para obtener lecturas fiables y consistentes de los sensores IR analógicos.
¿Por qué calibrar?

Luz Ambiental: La luz del entorno puede afectar las lecturas del sensor.31
Altura del Sensor: La distancia entre el sensor y la superficie influye en la cantidad de luz reflejada captada.31
Variaciones entre Sensores: Incluso sensores del mismo modelo pueden tener ligeras diferencias en su respuesta.9
Superficie: Las características exactas del blanco y negro de la pista pueden variar.


Método de Calibración (Analógica):

Implementar Rutina: Añada una rutina de calibración en la función setup() del código Arduino.
Movimiento: Durante esta rutina (p. ej., durante 5-10 segundos), mueva manualmente el conjunto de sensores del robot lentamente de lado a lado, barriendo completamente la línea negra sobre el fondo blanco.9
Registro: Para cada uno de los 6 sensores, registre los valores analógicos mínimos y máximos leídos durante el barrido.9 Almacene estos 12 valores (un mínimo y un máximo por sensor).
Uso: Estos valores calibrados pueden usarse de varias maneras:

Normalización: Escalar las lecturas futuras de cada sensor a un rango fijo (p. ej., 0 a 1000) usando sus valores min/max calibrados. Esto compensa las diferencias entre sensores. La biblioteca QTRSensors de Pololu a menudo realiza esto.7
Umbral Dinámico: Calcular un umbral para cada sensor (p. ej., el punto medio mid=(mincalibrado​+maxcalibrado​)/2) para convertir la lectura analógica en un valor binario (0 o 1) si es necesario para ciertas lógicas.27




Calibración (Digital): Si se usan salidas digitales, la calibración implica ajustar manualmente el potenciómetro de cada módulo. Coloque el sensor sobre la superficie blanca y ajuste el potenciómetro hasta que el LED indicador (si existe) o la salida digital cambie de estado justo en el borde de la línea negra.2 Este método es menos flexible y más sensible a cambios en las condiciones.
3.4. Posibles Problemas
Ruido/Interferencia: Las lecturas analógicas pueden ser susceptibles al ruido eléctrico.43 Promediar varias lecturas consecutivas puede ayudar a suavizar los datos.43
Luz Ambiental: Cambios fuertes en la luz ambiental pueden afectar las lecturas. Intentar proteger los sensores de la luz directa puede ser útil.17
Interferencia Cruzada (Crosstalk): La luz emitida por un sensor podría ser detectada por el receptor de un sensor adyacente, especialmente si están muy juntos. Tubos o separadores alrededor de cada par emisor/receptor pueden minimizar esto.43
4. Control de Motores DC con Driver L298NEl módulo L298N actúa como intermediario entre las señales de bajo voltaje del Arduino y la mayor potencia requerida por los motores DC. Permite controlar tanto la velocidad como la dirección de dos motores de forma independiente.14.1. Descripción General del L298NEste módulo integra un chip L298N, que contiene dos circuitos H-Bridge completos.4 Los pines clave son:
Entradas de Alimentación: VCC (o similar) para la alimentación de los motores (5V-35V) y GND.1
Entrada/Salida Lógica de 5V: Para alimentar la lógica interna del L298N y/o el Arduino (ver Sección 2.3).1
Pines de Control de Dirección (IN1, IN2, IN3, IN4): Pines digitales que determinan la dirección de giro de cada motor.1
Pines de Habilitación (ENA, ENB): Permiten activar/desactivar cada motor y controlar su velocidad mediante PWM.1
Salidas de Motor (OUT1, OUT2, OUT3, OUT4): Terminales donde se conectan los motores.1
4.2. Control de DirecciónUn H-Bridge permite invertir la polaridad del voltaje aplicado al motor, cambiando así su dirección de giro. Se controla mediante los pines IN:
Motor A (conectado a OUT1/OUT2):

Adelante: IN1 = HIGH, IN2 = LOW.4
Atrás: IN1 = LOW, IN2 = HIGH.4
Parar/Frenar: IN1 = LOW, IN2 = LOW (o HIGH/HIGH).


Motor B (conectado a OUT3/OUT4):

Adelante: IN3 = HIGH, IN4 = LOW.4
Atrás: IN3 = LOW, IN4 = HIGH.4
Parar/Frenar: IN3 = LOW, IN4 = LOW (o HIGH/HIGH).


En Arduino, esto se implementa usando la función digitalWrite(pin, estado) en los pines conectados a IN1-IN4.4.3. Control de Velocidad (PWM)La velocidad del motor se controla aplicando una señal PWM a los pines de habilitación ENA y ENB del L298N, que deben estar conectados a pines PWM del Arduino.1
Función: analogWrite(pin_ENA_o_ENB, valor_velocidad).1
Valor: valor_velocidad es un entero entre 0 y 255.3

0: Motor detenido (o velocidad mínima).
255: Velocidad máxima.
Valores intermedios: Velocidad proporcional.


La técnica PWM ajusta el "ciclo de trabajo" (duty cycle) de la señal, cambiando el voltaje promedio aplicado al motor y, por lo tanto, su velocidad.4.4. Funciones Básicas de Control de Motores (Código Arduino)Es útil crear funciones reutilizables para controlar el movimiento del robot. Aquí hay ejemplos conceptuales (los números de pin deben coincidir con su configuración de la Sección 2.4):C++// Definición de pines (Ejemplo - ¡Ajustar según su cableado!)
#define ENA 9   // PWM Motor Izquierdo (A)
#define IN1 8
#define IN2 7
#define ENB 6   // PWM Motor Derecho (B)
#define IN3 5
#define IN4 4

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors(); // Asegurar que los motores estén parados al inicio
}

// Mover ambos motores hacia adelante a una velocidad dada (0-255)
void moveForward(int speed) {
  digitalWrite(IN1, HIGH); // Motor Izquierdo Adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Motor Derecho Adelante
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); // Velocidad Motor Izquierdo
  analogWrite(ENB, speed); // Velocidad Motor Derecho
}

// Mover ambos motores hacia atrás a una velocidad dada (0-255)
void moveBackward(int speed) {
  digitalWrite(IN1, LOW);  // Motor Izquierdo Atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Motor Derecho Atrás
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Girar a la izquierda (pivotando sobre el centro)
// Motor Izquierdo atrás, Motor Derecho adelante
void turnLeft(int speed) {
  digitalWrite(IN1, LOW);  // Motor Izquierdo Atrás
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // Motor Derecho Adelante
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Girar a la derecha (pivotando sobre el centro)
// Motor Izquierdo adelante, Motor Derecho atrás
void turnRight(int speed) {
  digitalWrite(IN1, HIGH); // Motor Izquierdo Adelante
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Motor Derecho Atrás
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Detener ambos motores (freno activo)
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); // También se puede poner velocidad 0
  analogWrite(ENB, 0);
}

void loop() {
  // Ejemplo de uso:
  moveForward(150);
  delay(2000);
  turnLeft(100);
  delay(1000);
  stopMotors();
  delay(5000);
}
4.5. Uso de Bibliotecas (Alternativa)Existen bibliotecas como L298N.h 46 que simplifican el control de los motores. Esta biblioteca encapsula las operaciones digitalWrite y analogWrite en funciones más intuitivas. Para controlar dos motores, se utiliza la variante L298NX2.h.46Ejemplo de uso con L298NX2.h:C++#include <L298NX2.h>

// Definir pines Arduino conectados al L298N
#define ENA_PIN 9
#define IN1A_PIN 8
#define IN2A_PIN 7
#define ENB_PIN 6
#define IN1B_PIN 5
#define IN2B_PIN 4

// Crear instancia para dos motores
L298NX2 motors(ENA_PIN, IN1A_PIN, IN2A_PIN, ENB_PIN, IN1B_PIN, IN2B_PIN);

void setup() {
  // La biblioteca maneja la configuración de pines
}

void loop() {
  // Mover Motor A (Izquierdo) adelante a velocidad 200
  motors.setSpeedA(200);
  motors.forwardA();
  delay(1000);

  // Mover Motor B (Derecho) atrás a velocidad 150
  motors.setSpeedB(150);
  motors.backwardB();
  delay(1000);

  // Detener ambos motores
  motors.stop();
  delay(2000);
}
El uso de bibliotecas puede acelerar el desarrollo, pero comprender el control directo de pines es valioso para entender los principios subyacentes y para la depuración avanzada.5. Algoritmo Central de Seguimiento de Línea con 6 SensoresCon los sensores y motores bajo control, el siguiente paso es implementar la lógica que utiliza las lecturas de los sensores para guiar al robot a lo largo de la línea. Un método eficaz para 6 sensores analógicos es el cálculo de la posición mediante promedio ponderado, combinado con un control proporcional.5.1. Lectura del Conjunto de 6 SensoresPrimero, es necesario leer los valores analógicos de los 6 sensores (conectados a A0-A5) en cada ciclo del bucle principal (loop()). Estos valores se pueden almacenar en un array:C++int sensorPins = {A0, A1, A2, A3, A4, A5}; // Pines analógicos para los 6 sensores
int sensorValues; // Array para almacenar las lecturas

void readSensors() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Aquí se podría aplicar la calibración/normalización si se implementó
  }
}
5.2. Cálculo de la Posición/Error - Método del Promedio PonderadoEste método proporciona una estimación continua de la posición de la línea negra en relación con el centro del conjunto de sensores, lo cual es mucho más informativo que simples estados discretos (como "sensor izquierdo activado").10

Asignar Pesos: A cada sensor se le asigna un peso numérico que representa su posición. Los sensores más alejados del centro reciben pesos de mayor magnitud.9 Una posible asignación para 6 sensores, donde el centro ideal de la línea está entre el sensor 2 y 3, podría ser:

Sensor 0 (más a la izquierda): -5
Sensor 1: -3
Sensor 2: -1
Sensor 3: 1
Sensor 4: 3
Sensor 5 (más a la derecha): 5
(Estos valores pueden escalarse, por ejemplo, a -2500, -1500, -500, 500, 1500, 2500 para obtener un rango mayor, similar a como lo hacen algunas bibliotecas 7).



Calcular Promedio Ponderado: Se calcula una suma ponderada de las lecturas de los sensores y se divide por la suma total de las lecturas (para normalizar).8 Antes de esto, es útil convertir las lecturas analógicas crudas (0-1023) en una medida de "cuán negro" está detectando cada sensor (por ejemplo, escalando después de la calibración para que 0 represente blanco puro y 1000 negro puro).

C++long weightedSum = 0;
long sumOfReadings = 0;
int weights = {-5, -3, -1, 1, 3, 5}; // O pesos escalados

readSensors(); // Obtener las lecturas actuales en sensorValues
                // Asumimos que sensorValues ya están calibrados/escalados
                // donde un valor más alto significa más negro

for (int i = 0; i < 6; i++) {
  weightedSum += (long)sensorValues[i] * weights[i];
  sumOfReadings += sensorValues[i];
}

int position = 0; // Valor por defecto si no se detecta línea
if (sumOfReadings > 0) { // Evitar división por cero si todos leen 0 (blanco)
  position = weightedSum / sumOfReadings;
}
// 'position' ahora contiene la posición estimada de la línea.
// 0 = perfectamente centrado (entre sensor 2 y 3)
// Negativo = línea desviada a la izquierda
// Positivo = línea desviada a la derecha

Error: En este contexto, el valor position calculado representa directamente el error del robot con respecto a la posición deseada (el centro, que corresponde a una position de 0). Por lo tanto, error = position.
5.3. Control Proporcional (P-Control)El control proporcional ajusta la dirección del robot aplicando una corrección que es directamente proporcional al error calculado.9

Calcular Ajuste: correction=Kp​×error

error es la position calculada anteriormente.
Kp​ es la Ganancia Proporcional, una constante que debe ser ajustada experimentalmente (tuning). Determina la fuerza de la reacción al error.



Ajustar Velocidad de Motores: La corrección se utiliza para modificar la velocidad base de los motores. Si el error es positivo (línea a la derecha), el robot debe girar a la derecha (motor derecho más lento, motor izquierdo más rápido). Si el error es negativo (línea a la izquierda), debe girar a la izquierda (motor izquierdo más lento, motor derecho más rápido).8

C++int baseSpeed = 150; // Velocidad base cuando el robot va recto (ajustable)
float Kp = 0.5;      // Ganancia Proporcional (¡Necesita ajuste!)

// Calcular error (position) como se mostró antes
int error = position;

// Calcular corrección
int correction = Kp * error;

// Calcular velocidades finales para cada motor
int rightMotorSpeed = baseSpeed - correction;
int leftMotorSpeed = baseSpeed + correction;

// Limitar las velocidades al rango PWM válido (0-255)
rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

// Aplicar velocidades a los motores (usando funciones como las de la Sección 4.4)
// Asumiendo que moveForward ajusta velocidades individuales si se modifica:
// setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
// setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
// O usando funciones directas:
analogWrite(ENA, leftMotorSpeed);
analogWrite(ENB, rightMotorSpeed);
// Asegurarse de que la dirección esté configurada para adelante
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
Este enfoque P con promedio ponderado proporciona un seguimiento de línea mucho más suave que los métodos basados en lógica digital simple, pero aún puede presentar oscilaciones a altas velocidades o en curvas cerradas.76. Estrategia de Control Avanzada: Implementación PIDPara lograr un rendimiento superior, especialmente a altas velocidades y en pistas complejas, se recomienda implementar un controlador PID (Proporcional-Integral-Derivativo). El PID mejora el control proporcional al considerar no solo el error actual, sino también los errores pasados y la tendencia futura del error.76.1. ¿Por qué PID?
Suavidad y Estabilidad: Reduce las oscilaciones (zigzagueo) comunes en el control P simple.7
Precisión: Puede eliminar el error de estado estacionario (cuando el robot sigue la línea pero ligeramente desviado).
Rendimiento: Permite al robot seguir la línea de manera más rápida y fiable, incluso en curvas cerradas.49
6.2. Componentes PID ExplicadosEl controlador PID calcula una salida de control basada en tres términos:
Proporcional (P): Reacciona a la magnitud del error actual (error=setpoint−position). Es el componente principal de la corrección.7 Una ganancia Kp​ alta acelera la respuesta pero puede causar inestabilidad.7
Pterm​=Kp​×error
Integral (I): Suma los errores pasados a lo largo del tiempo. Este término ayuda a eliminar el error residual (estado estacionario) que el control P por sí solo podría no corregir.8 Si la ganancia Ki​ es demasiado alta, puede causar "wind-up" (acumulación excesiva) y overshoot (sobrepasar la línea).8 En algunos seguidores de línea simples, este término se omite si el error de estado estacionario no es un problema significativo.9
Iterm​=Ki​×∑error×Δt (En código: integral += error; I_term = Ki * integral;)
Derivativo (D): Reacciona a la tasa de cambio del error (rateOfChange=(error−previousError)/Δt). Actúa como un freno, anticipando el comportamiento futuro del error y amortiguando las oscilaciones.7 Mejora la estabilidad y reduce el overshoot.8 Una ganancia Kd​ demasiado alta puede amplificar el ruido de los sensores y causar vibraciones o inestabilidad.18
Dterm​=Kd​×Δterror−previousError​ (En código: derivative = error - lastError; D_term = Kd * derivative;)
6.3. Cálculo PIDEn cada ciclo del loop():
Lea los sensores y calcule la position actual (usando promedio ponderado, Sección 5.2).
Calcule el error=0−position (asumiendo que el setpoint es 0).
Calcule el término Integral (acumulando el error).
Calcule el término Derivativo (usando el error actual y el lastError del ciclo anterior).
Calcule la salida PID total: PIDoutput​=(Kp​×error)+(Ki​×integral)+(Kd​×derivative).8
Actualice lastError = error para el siguiente ciclo.
6.4. Aplicación de la Salida PID a los MotoresLa PID_output se usa para ajustar la velocidad de los motores de manera similar al control P:C++int baseSpeed = 150; // Ajustable
float Kp = 0.6;      // ¡Necesita ajuste!
float Ki = 0.001;    // ¡Necesita ajuste! (Puede empezar en 0)
float Kd = 0.8;      // ¡Necesita ajuste!

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float PID_output = 0;

// Dentro del loop():
// 1. Leer sensores y calcular 'position' (promedio ponderado)
//...
error = 0 - position; // Calcular error (Setpoint = 0)

integral += error; // Acumular error para el término I
// Opcional: Limitar el término integral (anti-windup)
// if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
// else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

derivative = error - lastError; // Calcular cambio en el error para el término D

PID_output = (Kp * error) + (Ki * integral) + (Kd * derivative); // Calcular salida PID

lastError = error; // Guardar error actual para el próximo ciclo

// Calcular velocidades finales
int rightMotorSpeed = baseSpeed - PID_output;
int leftMotorSpeed = baseSpeed + PID_output;

// Limitar velocidades (0-255)
rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

// Aplicar velocidades a los motores (usando ENA, ENB y IN1-IN4)
//... (similar al ejemplo P-Control)
6.5. Ajuste de Constantes PID (Tuning)El ajuste (tuning) de las ganancias Kp​, Ki​ y Kd​ es la parte más crítica y a menudo requiere prueba y error.8 No hay valores universales; dependen de la mecánica del robot, velocidad, sensores, superficie, etc..18Guía Sistemática de Ajuste:GananciaEfecto al AumentarPaso de AjusteObjetivo de ObservaciónKp​Respuesta más rápida, Mayor oscilación1. Poner Ki​=0,Kd​=0. Empezar Kp​ bajo. Aumentar Kp​ hasta oscilación estable.Encontrar punto de oscilación. Reducir Kp​ un poco.Ki​Reduce error estacionario, Puede aumentar el overshoot2. Mantener Kp​. Empezar Ki​=0. Aumentar Ki​ lentamente.Robot se centra perfectamente en la línea.Kd​Amortigua oscilación, Mejora estabilidad, Sensible al ruido3. Mantener Kp​,Ki​. Empezar Kd​ bajo. Aumentar Kd​ lentamente.Reducir/eliminar oscilación, respuesta estable.Consejos para el Ajuste:
Realice los ajustes en la pista real donde competirá
