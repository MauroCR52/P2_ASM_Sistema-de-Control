
// --- 1. CALIBRACIÓN (Igual para ambos lados) ---
// Como ahora tenemos un sensor en cada meta, la lógica es simétrica.
const float DIST_META = 6.0;      // Distancia final al sensor para detenerse (cm)
const float DIST_RAMPA = 40.0;    // Distancia donde empieza a frenar suave (cm)

// --- 2. DEFINICIÓN DE PINES ---
// Sensor 1: Lado de CIERRE
const int TRIG_C = 4;
const int ECHO_C = 5;

// Sensor 2: Lado de APERTURA (NUEVO)
const int TRIG_A = 6; 
const int ECHO_A = 7; 

// Motor L298N
const int ENA = 10;
const int IN1 = 8;
const int IN2 = 9;

// --- 3. VARIABLES SISTEMA ---
long prevT = 0;
float posPrev = 0;
float vFilt = 0;
float eintegral = 0;

// Filtro y PID
float alpha = 0.85; 
float Kp = 12.0;   
float Ki = 0.8;    

// Control de Modos
char modoActual = 's'; // 's'=Stop, 'c'=Cerrar, 'o'=Abrir
float targetSpeed = 25.0; 
bool resetLectura = true; // Para evitar saltos al cambiar de sensor

void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_A, OUTPUT); pinMode(ECHO_A, INPUT);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("SISTEMA DOBLE SENSOR LISTO");
  Serial.println("Comandos: 'c'=Cerrar, 'o'=Abrir, 's'=Stop");
}

void loop() {
  // --- A. COMANDOS ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Al cambiar de modo, activamos bandera para resetear la lectura anterior
    // y evitar picos de velocidad falsos.
    if (cmd == 'c') { modoActual = 'c'; eintegral = 0; resetLectura = true; } 
    if (cmd == 'o') { modoActual = 'o'; eintegral = 0; resetLectura = true; } 
    if (cmd == 's') { modoActual = 's'; setMotor(0,0); } 
    
    if (cmd == '1') targetSpeed = 15.0;
    if (cmd == '2') targetSpeed = 25.0;
    if (cmd == '3') targetSpeed = 35.0;
  }

  // --- B. TIEMPO ---
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  // --- C. SELECCIÓN DE SENSOR Y LECTURA ---
  float dist = 0;
  
  if (modoActual == 'c') {
    // Si estamos cerrando, leemos el Sensor de Cierre
    dist = leerSensor(TRIG_C, ECHO_C);
  } 
  else if (modoActual == 'o') {
    // Si estamos abriendo, leemos el Sensor de Apertura
    dist = leerSensor(TRIG_A, ECHO_A);
  }
  else {
    // Si estamos en Stop, leemos cualquiera para no dejar la variable vacía
    dist = leerSensor(TRIG_C, ECHO_C);
  }

  // Reset de seguridad al cambiar de sensor
  if (resetLectura) {
    posPrev = dist;
    vFilt = 0;
    resetLectura = false;
  }

  // --- D. CÁLCULO VELOCIDAD ---
  // AHORA ES SIMÉTRICO: En ambos casos nos ACERCAMOS al sensor activo.
  // Por tanto, la distancia disminuye. (Actual - Previa) será negativo.
  float vRaw = (dist - posPrev) / deltaT;
  posPrev = dist;

  // Invertimos el signo para que la velocidad sea positiva hacia adelante
  vRaw = -vRaw; 

  // --- E. FILTRO ---
  vFilt = alpha * vFilt + (1.0 - alpha) * vRaw;

  // --- F. GENERACIÓN DE SETPOINT (RAMPA) ---
  float setPoint = 0;

  if (modoActual != 's') {
    // Lógica universal para ambos lados
    if (dist < DIST_META) {
      setPoint = 0; // Llegamos
    }
    else if (dist < DIST_RAMPA) {
      // Regla de tres simple para frenado suave
      float factor = (dist - DIST_META) / (DIST_RAMPA - DIST_META);
      setPoint = targetSpeed * factor;
    }
    else {
      setPoint = targetSpeed; // Velocidad crucero
    }
  }

  // --- G. PID ---
  float u = 0;
  if (modoActual != 's' && setPoint > 0) {
    float e = setPoint - vFilt;
    eintegral += e * deltaT;
    if (setPoint == 0) eintegral = 0; 
    u = (Kp * e) + (Ki * eintegral);
  }

  // --- H. ACTUADOR ---
  int pwr = (int) fabs(u);
  if (pwr > 255) pwr = 255;
  if (pwr < 45 && pwr > 0) pwr = 50; // Zona muerta

  if (modoActual == 'c') setMotor(1, pwr);      
  else if (modoActual == 'o') setMotor(-1, pwr); 
  else setMotor(0, 0);

  // --- I. VISUALIZACIÓN ---
  // Escalamos PWM para ver bonito en el Plotter
  float pwm_grafica = pwr / 8.0; 
  Serial.print("Min:0 Max:40"); 
  Serial.print(" Target:"); Serial.print(setPoint);
  Serial.print(" Velocidad:"); Serial.print(vFilt);
  Serial.print(" PWM_Escalado:"); Serial.println(pwm_grafica);

  delay(50);
}

// --- FUNCIÓN GENÉRICA PARA LEER CUALQUIER SENSOR ---
float leerSensor(int pinTrig, int pinEcho) {
  digitalWrite(pinTrig, LOW); delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH); delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  
  long duration = pulseIn(pinEcho, HIGH, 30000); 
  if (duration == 0) return posPrev; // Si falla, devuelve anterior
  return duration * 0.034 / 2.0; 
}

void setMotor(int dir, int pwmVal) {
  analogWrite(ENA, pwmVal);
  if (dir == 1) { // CERRAR
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  }
  else if (dir == -1) { // ABRIR
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
}