/*
  Comprobación del funcionamiento del Encoder: Girar, pulsar.

  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/
const uint8_t PIN_ENC_A   = 3;
const uint8_t PIN_ENC_B   = 2;
const uint8_t PIN_BUTTON  = 5;   // Pulsador a GND con INPUT_PULLUP

volatile int encoderValue = 0;
int lastEncoded = 0;

unsigned long lastButtonTime = 0;
bool lastButtonState = HIGH;

// *****************************************************************************
void setup() {
  Serial.begin(9600);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), updateEncoder, CHANGE);

  Serial.println("Iniciando test de encoder...");
}

// *****************************************************************************
void loop() {
  // --- Lectura del botón con antirrebote ---
  bool buttonState = digitalRead(PIN_BUTTON);
  unsigned long now = millis();

  if (buttonState == LOW && lastButtonState == HIGH && (now - lastButtonTime) > 200) {
    Serial.println("Pulsado");
    lastButtonTime = now;
  }
  lastButtonState = buttonState;

  delay(10);
}

// *****************************************************************************
void updateEncoder() {
  int MSB = digitalRead(PIN_ENC_A);
  int LSB = digitalRead(PIN_ENC_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Secuencia cuadratura — sentido invertido
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;

  lastEncoded = encoded;

  Serial.print("Encoder: ");
  Serial.println(encoderValue);
}
