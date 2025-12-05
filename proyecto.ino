// --- DEFINICIÓN DE PINES ---
const int enA = 10;  // Pin PWM (Velocidad)
const int in1 = 8;   // Dirección 1
const int in2 = 9;   // Dirección 2

// Variable para guardar lo que recibimos del PC
char comando; 

void setup() {
  // Configuramos pines
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Iniciamos la comunicación con la PC
  Serial.begin(9600);
  
  // Establecemos velocidad fija (200 de 255)
  analogWrite(enA, 200); 

  // Mensaje de bienvenida
  Serial.println("Sistema Listo.");
  Serial.println("Escribe 'a' para Izquierda, 'd' para Derecha, 's' para Stop");
}

void loop() {
  // 1. Verificamos si la PC nos mandó algo
  if (Serial.available() > 0) {
    
    // 2. Leemos el carácter
    comando = Serial.read(); 

    // 3. Decidimos qué hacer según la letra recibida
    if (comando == 'a') {
      // --- IZQUIERDA ---
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      Serial.println("Girando Izquierda <--");
    }
    else if (comando == 'd') {
      // --- DERECHA ---
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      Serial.println("Girando Derecha -->");
    }
    else if (comando == 's') {
      // --- STOP ---
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      Serial.println("Motor Detenido [X]");
    }
  }
  // Aquí no hay delays, el motor se queda en el último estado
  // hasta que reciba una orden nueva.
}