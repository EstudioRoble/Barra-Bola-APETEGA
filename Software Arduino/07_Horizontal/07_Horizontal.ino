/*
  Calibración Horizontal
  --------------------------------------------------
  Gira el encoder para mover el servo (μs)
  Pulsa el botón para guardar el valor en EEPROM (addr 16, float)
  
  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/

#include <EEPROM.h>
#include <Encoder.h>
#include <Adafruit_TiCoServo.h>

// ---------------------- Pines ----------------------
const uint8_t PIN_ENC_A   = 3;
const uint8_t PIN_ENC_B   = 2;
const uint8_t PIN_SERVO   = 9;
const uint8_t PIN_BUTTON  = 5;      // Pulsador a GND con INPUT_PULLUP

// ---------------------- EEPROM ----------------------
const int EEPROM_ADDR_REST_US = 16; // [16..19] float reposo (μs)

// ---------------------- Servo y encoder ----------------------
const int SERVO_US_MIN    = 1000; // 550;
const int SERVO_US_MAX    = 2000; // 2550;
const int DEFAULT_REST_US = 1500; // 1510;   // Valor típico de reposo
const int STEP_US         = 1;      // Ajuste por click

Adafruit_TiCoServo servo;
Encoder enc(PIN_ENC_A, PIN_ENC_B);

// ---------------------- Estado ----------------------
long  encRaw = 0;
long  lastDetents = 0;
float restUs = DEFAULT_REST_US;

bool  buttonLast = HIGH;    // INPUT_PULLUP
unsigned long lastButtonChangeMs = 0;
const unsigned long DEBOUNCE_MS = 40;

// ---------------------- Funciones ----------------------
void loadRestFromEEPROM() {
  float stored;
  EEPROM.get(EEPROM_ADDR_REST_US, stored);
  if (stored >= SERVO_US_MIN && stored <= SERVO_US_MAX) {
    restUs = stored;
  } else {
    restUs = DEFAULT_REST_US;
  }
}

void saveRestToEEPROM() {
  EEPROM.put(EEPROM_ADDR_REST_US, restUs);
  Serial.print(F("Valor guardado en EEPROM (us): "));
  Serial.println(restUs, 0);
}

// ---------------------- Setup ----------------------
void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println(F("\n[Calibrar Horizontal]"));

  loadRestFromEEPROM();

  servo.attach(PIN_SERVO);
  servo.writeMicroseconds((int)restUs);

  Serial.print(F("Reposo inicial (us): "));
  Serial.println(restUs, 1);
  Serial.println(F("Gira el encoder para ajustar. Pulsa el botón para guardar."));
}

// ---------------------- Loop ----------------------
void loop() {
  // --- Encoder ---
  encRaw = enc.read();
  long detents = encRaw / 4;  // Divide por 4 si el encoder da 4 pasos por click
  if (detents != lastDetents) {
    long delta = detents - lastDetents;
    lastDetents = detents;

    restUs -= delta * STEP_US;
    restUs = constrain(restUs, SERVO_US_MIN, SERVO_US_MAX);

    servo.writeMicroseconds((int)restUs);

    Serial.print(F("Reposo= "));
    Serial.print(restUs, 0);
    Serial.println(F("us"));
  }

  // --- Botón ---
  bool b = digitalRead(PIN_BUTTON);
  unsigned long now = millis();

  if (b != buttonLast && (now - lastButtonChangeMs) > DEBOUNCE_MS) {
    lastButtonChangeMs = now;

    if (b == LOW) { // pulsado
      saveRestToEEPROM();
    }
    buttonLast = b;
  }
}
