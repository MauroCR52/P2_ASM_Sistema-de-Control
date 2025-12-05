// PROYECTO: Control PID Puerta - ABRIR y CERRAR
// AUTOR: Jorge Gutierrez Vindas
// ESTADO: Bidireccional + Gráficas Corregidas

// --- DEFINICIÓN DE PINES ---
const int PIN_TRIG = 4;
const int PIN_ECHO = 5;
const int ENA = 10;
const int IN1 = 8;
const int IN2 = 9;

// --- VARIABLES SISTEMA ---
long prevT = 0;
float posPrev = 0;
float vFilt = 0;
float eintegral = 0;

// Filtro y PID
float alpha = 0.85; 
float Kp = 12.0;   // Ajustado un poco más fuerte
float Ki = 0.8;    

// Control de Modos
char modoActual = 's'; // 's'=Stop, 'c'=Cerrar, 'o'=Abrir
float targetSpeed = 25.0; // Velocidad base (cm/s)

// Variables para gráficas
float pwm_grafica = 0; 

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("MIN:0 MAX:40"); // Truco para fijar escala inicial en plotter
}

void loop() {
  // --- A. COMANDOS ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c') { modoActual = 'c'; eintegral = 0; } // CERRAR
    if (cmd == 'o') { modoActual = 'o'; eintegral = 0; } // ABRIR
    if (cmd == 's') { modoActual = 's'; setMotor(0,0); } // STOP
    
    // Ajuste velocidad al vuelo
    if (cmd == '1') targetSpeed = 15.0;
    if (cmd == '2') targetSpeed = 25.0;
    if (cmd == '3') targetSpeed = 35.0;
  }

  // --- B. TIEMPO Y SENSOR ---
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  float dist = leerUltrasonico();

  // --- C. CÁLCULO VELOCIDAD ---
  // Velocidad cruda = cambio de distancia / tiempo
  float vRaw = (dist - posPrev) / deltaT;
  posPrev = dist;

  // AJUSTE DE SIGNO SEGÚN MODO
  // Si estamos en 'CERRAR', acercarse es velocidad "positiva" para el PID
  // Si estamos en 'ABRIR', alejarse es velocidad "positiva" para el PID
  if (modoActual == 'c') {
     vRaw = -vRaw; 
  }
  // En modo 'o', vRaw ya es positivo al alejarse, no se toca.

  // --- D. FILTRO ---
  vFilt = alpha * vFilt + (1.0 - alpha) * vRaw;

  // --- E. GENERACIÓN DE SETPOINT (RAMPAS) ---
  float setPoint = 0;

  if (modoActual == 'c') { 
    // === MODO CERRAR (Hacia el sensor) ===
    // Meta: Llegar a 5cm. Empezar a frenar en 30cm.
    float distFinal = 5.0;
    float distRampa = 30.0;
    
    if (dist < distFinal) setPoint = 0;
    else if (dist < distRampa) {
      float factor = (dist - distFinal) / (distRampa - distFinal);
      setPoint = targetSpeed * factor;
    }
    else setPoint = targetSpeed;
  }
  else if (modoActual == 'o') {
    // === MODO ABRIR (Alejarse del sensor) ===
    // Meta: Llegar a 90cm. Empezar a frenar en 60cm.
    float distFinal = 90.0;
    float distRampa = 60.0;

    if (dist > distFinal) setPoint = 0;
    else if (dist > distRampa) {
      // Regla de 3 inversa: entre más cerca de 90, más lento
      float factor = (distFinal - dist) / (distFinal - distRampa);
      setPoint = targetSpeed * factor;
    }
    else setPoint = targetSpeed;
  }
  else {
    setPoint = 0; // Modo Stop
  }

  // --- F. PID ---
  float u = 0;
  if (modoActual != 's' && setPoint > 0) {
    float e = setPoint - vFilt;
    eintegral += e * deltaT;
    if (setPoint == 0) eintegral = 0; // Reset al frenar
    u = (Kp * e) + (Ki * eintegral);
  }

  // --- G. ACTUADOR ---
  int pwr = (int) fabs(u);
  if (pwr > 255) pwr = 255;
  // Zona muerta para arranque suave
  if (pwr < 45 && pwr > 0) pwr = 50; 

  // Dirección física del motor
  // NOTA: Revisa si tu motor abre o cierra correctamente con estos.
  // Si va al revés en 'o', invierte los IN1/IN2 dentro del if.
  if (modoActual == 'c') setMotor(1, pwr);      // 1 = Acercar
  else if (modoActual == 'o') setMotor(-1, pwr); // -1 = Alejar
  else setMotor(0, 0);

  // --- H. GRÁFICAS LIMPIAS ---
  // Escalamos el PWM para que se vea bien junto a la velocidad (0-30)
  // 255 / 8 = 31 aprox.
  pwm_grafica = pwr / 8.0; 

  // IMPRIMIR SOLO LO NECESARIO
  // "Min" y "Max" fuerzan al plotter a quedarse quieto y no bailar
  Serial.print("Min:0 Max:40"); 
  Serial.print(" Target:"); Serial.print(setPoint);
  Serial.print(" Velocidad:"); Serial.print(vFilt);
  Serial.print(" PWM_Escalado:"); Serial.println(pwm_grafica);

  delay(50);
}

// --- AUXILIARES ---
float leerUltrasonico() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); 
  if (duration == 0) return posPrev; 
  return duration * 0.034 / 2.0; 
}

void setMotor(int dir, int pwmVal) {
  analogWrite(ENA, pwmVal);
  if (dir == 1) { // CERRAR (Acercar) -> Ajustar si está al revés
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  }
  else if (dir == -1) { // ABRIR (Alejar) -> Ajustar si está al revés
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  }
}
