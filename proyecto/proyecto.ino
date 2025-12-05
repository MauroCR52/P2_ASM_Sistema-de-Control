// PROYECTO: Control PID Puerta - VERSIÓN FINAL ENTREGABLE
// CUMPLE CON: PID, Doble Sensor, Comparación Lazo Abierto/Cerrado, Test Escalón

// --- 1. CONFIGURACIÓN ---
const float META_CIERRE  = 6.0;   
const float RAMPA_CIERRE = 40.0;  

const float META_ABRIR   = 10.0;   
const float RAMPA_ABRIR  = 50.0;  

// --- 2. PINES ---
const int TRIG_C = 4; const int ECHO_C = 5;
const int TRIG_A = 6; const int ECHO_A = 7;
const int ENA = 10;
const int IN1 = 8;
const int IN2 = 9;

// --- 3. VARIABLES GLOBALES ---
long prevT = 0;
float posPrev = 0;
float vFilt = 0;
float eintegral = 0;
char modoActual = 's'; 
float targetSpeed = 25.0; 
bool nuevoModo = true; 

// ESTADO DEL CONTROLADOR (Requisito del PDF)
bool usarPID = true; // true = PID Activo, false = Lazo Abierto (Voltaje fijo)

// PID
float alpha = 0.85; 
float Kp = 12.0;   
float Ki = 0.8;    

// --- 4. FUNCIONES AUXILIARES ---
void moverMotor(int dir, int pwmVal) {
  analogWrite(ENA, pwmVal);
  if (dir == 1) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); } 
  else if (dir == -1) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); } 
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
}

float leerSoloEsteSensor(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000); 
  if (duration == 0) return posPrev; 
  return duration * 0.034 / 2.0; 
}

// --- 5. SETUP ---
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_A, OUTPUT); pinMode(ECHO_A, INPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  
  Serial.println("--- SISTEMA FINAL LISTO ---");
  Serial.println("Comandos Normales: 'c'=Cerrar, 'o'=Abrir, 's'=Stop");
  Serial.println("Velocidades: '1', '2', '3'");
  Serial.println("COMPARACION (Requisito):");
  Serial.println("  'y' -> Activar PID (Movimiento suave)");
  Serial.println("  'n' -> Desactivar PID (Lazo Abierto/Brusco)");
  Serial.println("MODELADO:");
  Serial.println("  't' -> Test Escalon (Dispara motor 1 seg para graficar)");
}

// --- 6. LOOP ---
void loop() {
  // A. LEER COMANDOS
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Comandos de Movimiento
    if (cmd == 'c') { modoActual = 'c'; nuevoModo = true; Serial.println("CMD: CERRAR"); }
    if (cmd == 'o') { modoActual = 'o'; nuevoModo = true; Serial.println("CMD: ABRIR"); }
    if (cmd == 's') { modoActual = 's'; moverMotor(0,0); Serial.println("CMD: STOP"); }
    
    // Ajuste Velocidad
    if (cmd == '1') targetSpeed = 15.0;
    if (cmd == '2') targetSpeed = 20.0;
    if (cmd == '3') targetSpeed = 35.0;

    // --- REQUISITO: ENCENDER/APAGAR CONTROLADOR  ---
    if (cmd == 'y') { usarPID = true; Serial.println("CONTROLADOR: ON (PID Activo)"); }
    if (cmd == 'n') { usarPID = false; Serial.println("CONTROLADOR: OFF (Lazo Abierto)"); }

    // --- REQUISITO: RESPUESTA AL ESCALON [cite: 47] ---
    if (cmd == 't') { ejecutarTestEscalon(); }
  }

  // B. CONTROL DE TIEMPO
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  // C. EJECUTAR RUTINA
  if (modoActual == 'c') rutinaCerrar(deltaT);
  else if (modoActual == 'o') rutinaAbrir(deltaT);
  else { moverMotor(0, 0); vFilt = 0; eintegral = 0; }
  
  delay(50); 
}

// --- 7. RUTINAS DE CONTROL ---

void rutinaCerrar(float deltaT) {
  float dist = leerSoloEsteSensor(TRIG_C, ECHO_C);
  
  if (nuevoModo) { posPrev = dist; vFilt = 0; eintegral = 0; nuevoModo = false; return; }

  // Calculo velocidad
  float vRaw = (dist - posPrev) / deltaT;
  posPrev = dist;
  vRaw = -vRaw; // Invertir signo
  vFilt = alpha * vFilt + (1.0 - alpha) * vRaw;

  // Si NO usamos PID (Lazo Abierto), mandamos PWM fijo
  if (!usarPID) {
     int pwmFijo = 120; // Valor fijo para probar sin control
     if (dist < META_CIERRE) pwmFijo = 0; // Seguridad mínima
     moverMotor(1, pwmFijo);
     Serial.print("LAZO_ABIERTO -> Dist: "); Serial.println(dist);
     return;
  }

  // Si usamos PID (Código Normal)
  float setPoint = 0;
  if (dist < META_CIERRE) setPoint = 0; 
  else if (dist < RAMPA_CIERRE) {
    float factor = (dist - META_CIERRE) / (RAMPA_CIERRE - META_CIERRE);
    setPoint = targetSpeed * factor;
  } else setPoint = targetSpeed;

  ejecutarPIDyMotor(setPoint, deltaT, 1);
  Serial.print("PID_CERRANDO -> Target:"); Serial.print(setPoint); 
  Serial.print(" Vel:"); Serial.println(vFilt);
}

void rutinaAbrir(float deltaT) {
  float dist = leerSoloEsteSensor(TRIG_A, ECHO_A);

  if (nuevoModo) { posPrev = dist; vFilt = 0; eintegral = 0; nuevoModo = false; return; }

  float vRaw = (dist - posPrev) / deltaT;
  posPrev = dist;
  vRaw = -vRaw; 
  vFilt = alpha * vFilt + (1.0 - alpha) * vRaw;

  // Lazo Abierto
  if (!usarPID) {
     int pwmFijo = 120; 
     if (dist < META_ABRIR) pwmFijo = 0; 
     moverMotor(-1, pwmFijo);
     Serial.print("LAZO_ABIERTO -> Dist: "); Serial.println(dist);
     return;
  }

  // PID Normal
  float setPoint = 0;
  if (dist < META_ABRIR) setPoint = 0; 
  else if (dist < RAMPA_ABRIR) {
    float factor = (dist - META_ABRIR) / (RAMPA_ABRIR - META_ABRIR);
    setPoint = targetSpeed * factor;
  } else setPoint = targetSpeed;

  ejecutarPIDyMotor(setPoint, deltaT, -1);
  Serial.print("PID_ABRIENDO -> Target:"); Serial.print(setPoint); 
  Serial.print(" Vel:"); Serial.println(vFilt);
}

void ejecutarPIDyMotor(float setPoint, float deltaT, int direction) {
  float u = 0;
  if (setPoint > 0) {
    float e = setPoint - vFilt;
    eintegral += e * deltaT;
    if (setPoint == 0) eintegral = 0;
    u = (Kp * e) + (Ki * eintegral);
  }
  int pwr = (int) fabs(u);
  if (pwr > 255) pwr = 255;
  if (pwr < 60 && pwr > 0) pwr = 60; 
  if (setPoint == 0) pwr = 0;
  moverMotor(direction, pwr);
}

// --- 8. FUNCION ESPECIAL: TEST ESCALÓN (MODELADO) ---
void ejecutarTestEscalon() {
  Serial.println("INICIANDO TEST ESCALON (3 SEGUNDOS)...");
  Serial.println("Poniendo PWM al maximo (255) de golpe.");
  
  // Aseguramos estar parados
  moverMotor(0,0); delay(1000);
  
  long tInicio = millis();
  float posPreviaTest = leerSoloEsteSensor(TRIG_C, ECHO_C); // Usamos sensor cierre
  long prevT_test = micros();

  // Ejecutamos por 2 segundos
  while (millis() - tInicio < 2000) {
    long currT = micros();
    float deltaT = ((float)(currT - prevT_test)) / 1.0e6;
    prevT_test = currT;

    // Leemos sensor
    float dist = leerSoloEsteSensor(TRIG_C, ECHO_C);
    
    // Calculamos velocidad
    float vRaw = (dist - posPreviaTest) / deltaT;
    posPreviaTest = dist;
    vRaw = -vRaw; // Positivo al acercarse

    // Filtramos un poco solo para ver la grafica
    // Nota: Para modelado a veces se prefiere vRaw, pero vFilt se ve mejor
    static float vFiltTest = 0;
    vFiltTest = 0.85 * vFiltTest + 0.15 * vRaw;

    // APLICAMOS ESCALÓN (Voltaje Fijo Máximo)
    moverMotor(1, 255); 

    // Imprimir para Plotter
    Serial.print("Escalon:255"); 
    Serial.print(" Velocidad:"); Serial.println(vFiltTest);
    
    // Seguridad Anti-Choque
    if (dist < 10.0) { moverMotor(0,0); break; }
    
    delay(20);
  }
  moverMotor(0,0);
  Serial.println("TEST FINALIZADO.");
  modoActual = 's'; // Volver a stop
}