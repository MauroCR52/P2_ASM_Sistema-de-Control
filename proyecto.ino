// PROYECTO: Control PID de Puerta con Ultrasónico
// AUTOR: Jorge Gutierrez Vindas
// CURSO: Análisis de Señales Mixtas

// --- 1. DEFINICIÓN DE PINES ---
const int PIN_TRIG = 4;
const int PIN_ECHO = 5;
const int ENA = 10;  // PWM L298N
const int IN1 = 8;
const int IN2 = 9;

// --- 2. VARIABLES GLOBALES PID Y SISTEMA ---
long prevT = 0;           // Tiempo previo
float posPrev = 0;        // Posición (distancia) previa
float vFilt = 0;          // Velocidad Filtrada (IMPORTANTE)
float vPrev = 0;          // Velocidad previa para el filtro

float eintegral = 0;      // Acumulador del error integral

// Parámetros del Filtro Pasa Bajas (Ajustable)
float alpha = 0.85; 

// Parámetros PID (Ajustar probando)
float Kp = 10.0;   // Ganancia Proporcional
float Ki = 0.5;    // Ganancia Integral

// Estado del sistema
bool sistemaActivo = false; // Para encender/apagar el PID
float targetSpeed = 0;      // Velocidad objetivo (cm/s)
int selectorVelocidad = 0;  // 0=Baja, 1=Media, 2=Alta

void setup() {
  Serial.begin(115200); // Usar 115200 baudios
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("Sistema Iniciado (Motor Invertido).");
  Serial.println("'1','2','3': Seleccionar Velocidad");
  Serial.println("'e': Encender Control | 'a': Apagar Control");
}

void loop() {
  // --- A. LECTURA DE COMANDOS SERIAL (Usuario) ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'e') sistemaActivo = true;
    if (cmd == 'a') sistemaActivo = false;
    if (cmd == '1') { selectorVelocidad = 0; targetSpeed = 10.0; } // 10 cm/s
    if (cmd == '2') { selectorVelocidad = 1; targetSpeed = 20.0; } // 20 cm/s
    if (cmd == '3') { selectorVelocidad = 2; targetSpeed = 30.0; } // 30 cm/s
  }

  // --- B. MEDICIÓN DE TIEMPO (Delta T) ---
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6; // Tiempo en segundos
  prevT = currT;

  // --- C. OBTENER POSICIÓN (Sensor Ultrasónico) ---
  float distanciaActual = leerUltrasonico();
  
  // --- D. CALCULAR VELOCIDAD (Derivada) ---
  float velocityRaw = (distanciaActual - posPrev) / deltaT;
  posPrev = distanciaActual;

  // Invertimos el signo para que acercarse al sensor sea velocidad positiva
  velocityRaw = -velocityRaw; 

  // --- E. FILTRO PASA BAJAS ---
  vFilt = alpha * vFilt + (1.0 - alpha) * velocityRaw;

  // --- F. LÓGICA DE FRENADO AUTOMÁTICO ---
  float setPoint = targetSpeed;
  // Aumenta el 15.0 si ves que choca con el sensor
  if (distanciaActual < 15.0) { 
    setPoint = 0; // Frenar
  }

  // --- G. CÁLCULO PID ---
  float u = 0; 
  
  if (sistemaActivo) {
    float e = setPoint - vFilt;
    eintegral = eintegral + e * deltaT;
    
    // Anti-windup simple
    if(setPoint == 0) eintegral = 0; 

    u = (Kp * e) + (Ki * eintegral);
  } else {
    u = 0;
    eintegral = 0;
  }

  // --- H. ACTUADOR (Motor Driver) ---
  int dir = 1;
  if (u < 0) dir = -1; 
  
  int pwr = (int) fabs(u); 
  if (pwr > 255) pwr = 255; 
  
  // Zona muerta para evitar zumbidos
  if (pwr < 40 && pwr > 0) pwr = 45; 

  setMotor(dir, pwr);

  // --- I. MONITOREO ---
  Serial.print("Target:"); Serial.print(setPoint);
  Serial.print(" Velocidad:"); Serial.print(vFilt);
  Serial.print(" Distancia:"); Serial.println(distanciaActual);
  
  delay(50); 
}

// Función auxiliar para leer sensor
float leerUltrasonico() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); 
  if (duration == 0) return posPrev; 
  
  return duration * 0.034 / 2.0; 
}

// --- FUNCIÓN MODIFICADA (MOTOR INVERTIDO) ---
void setMotor(int dir, int pwmVal) {
  analogWrite(ENA, pwmVal);
  
  if (dir == 1) { // Avanzar (Hacia el sensor)
    // ANTES: HIGH, LOW. AHORA: LOW, HIGH (Invertido)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (dir == -1) { // Retroceder (Alejarse)
    // ANTES: LOW, HIGH. AHORA: HIGH, LOW (Invertido)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else { // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
