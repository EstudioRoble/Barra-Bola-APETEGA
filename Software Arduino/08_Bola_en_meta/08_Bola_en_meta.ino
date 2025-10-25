/*
  Control PID de Barra y Bola

  Con el Serial Monitor envía:
  p10 -> para asignar 10 a p
  d10 -> para asignar 10 a p
  i0.5 -> para asignar 0.5 a i
  g -> Muestra valores actuales de p, d, i.

  Gira el encoder para modificar la meta.
  Pulsa el encoder para asignar la meta a la posición actual de la bola.
  
  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Adafruit_TiCoServo.h>

// *****************************************************************************
#define SENSORPIN A0 //Pin Analogico donde esta conectada la señal del Sensor de distancia
#define NEOPIN    4  // neoPixels
#define PIN_SERVO 9  // Servo
const uint8_t PIN_ENC_A   = 3;
const uint8_t PIN_ENC_B   = 2;
const uint8_t PIN_BUTTON  = 5;   // a GND con INPUT_PULLUP

int measure; // Lo que mide el sensor. Son ADCs.
float unfilteredDist; // measure antes del filtro. Son cm.
float velocidadSinFiltrar;
float kdist = 0.6; // 0.3; // ema Gain para dist
float kvel = 0.3; // ema Gain para velocidad
float posBola; // despues de filtro ema

float calPoint[] = {0, 4, 8, 12, 16, 20, 23, 27, 31};
float calValue[] = {969, 711, 553, 448, 377, 329, 305, 276, 252};
const int nCalPoints = sizeof(calPoint) / sizeof(calPoint[0]);

#define NUMPIXELS      32
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

uint32_t azul = pixels.Color(0, 0, 10);
uint32_t blanco = pixels.Color(255, 255, 255);
uint32_t verde   = pixels.Color(0, 255, 0);

// Control del tiempo
unsigned long lastLoopTime = 0;
const unsigned long TIEMPO_ENTRE_CICLOS = 50; // ms (20 Hz)

float ultimaPosBola;
float velocidad;   // cm/s

Adafruit_TiCoServo servo;

// Dirección EEPROM del valor de reposo (μs) [16..19]
const int EEPROM_ADDR_REST_US = 16;

// Servo
const int SERVO_US_MIN    = 1000;
const int SERVO_US_MAX    = 2000;
float reposo = 1510;      // valor de reposo (μs), se carga de EEPROM

// PID
float p = 10;
float d = 10;
float i = 0.5;
float cuantoMasLejosMasCuestaAbajo;
float cuantoMasRapidaMasCuestaArriba;
float iAcumulado  = 0.0;
float distAmeta, correcion;

// Anti-windup sencillo (solo integra en banda)
int Rint = 0.9;  // cm
int Rext = 3;  // cm

// --- Meta controlado por encoder ---
volatile long encDelta = 0;         // ticks acumulados del encoder
float meta = 12.0f;                 // cm, variable
const float SP_STEP_CM = 0.5f;      // cm por tick del encoder
const float SP_MIN_CM  = 0.0f;      // límites de la meta
const float SP_MAX_CM  = 31.0f;     // (calPoint[0]..calPoint[nCalPoints-1])

// Al pulsar el botón del encoder, engancha la meta a la distancia actual
unsigned long lastBtnMs = 0;
const unsigned long BTN_DEBOUNCE_MS = 120;

volatile unsigned long lastEncMicros = 0;

// --- Serial command parser (P/D/I<float>) ---
char  cmdBuf[24];
uint8_t cmdLen = 0;

void parseCommand(const char* s) {
  // saltar espacios iniciales
  while (*s == ' ' || *s == '\t') ++s;
  if (*s == '\0') return;

  char k = toupper(*s);        // 'P' 'D' 'I'
  if (k != 'P' && k != 'D' && k != 'I' && k != 'G' && k != '?') {
    Serial.println(F("ERR: use P<val>, D<val>, I<val>, G para ver."));
    return;
  }
  if (k == 'G' || k == '?') {  // G or ? -> print gains
    Serial.print(F("p=")); Serial.print(p, 3);
    Serial.print(F("  d=")); Serial.print(d, 3);
    Serial.print(F("  i=")); Serial.println(i, 3);
    return;
  }

  // avanzar al comienzo del número: saltar letra, espacios y '=' opcional
  ++s;
  while (*s == ' ' || *s == '\t' || *s == '=') ++s;

  // convertir a float (usa punto decimal)
  float v = atof(s);
  if (isnan(v) || isinf(v)) {
    Serial.println(F("ERR: valor invalido"));
    return;
  }

  switch (k) {
    case 'P': p = v; break;
    case 'D': d = v; break;
    case 'I': i = v; break;
  }

  // eco de confirmación
  Serial.print(F("OK "));
  Serial.print(k); Serial.print(F("="));
  Serial.println(v, 3);
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        parseCommand(cmdBuf);
        cmdLen = 0;
      }
    } else {
      if (cmdLen < sizeof(cmdBuf) - 1) {
        cmdBuf[cmdLen++] = c;
      } else {
        // overflow: reset buffer
        cmdLen = 0;
      }
    }
  }
}


// ISR sencilla: en cualquier cambio de A, leemos B para el sentido.
// Si A == B -> +1; si A != B -> -1. Con pequeño guard de rebotes por tiempo.
void isrEncA() {
  unsigned long now = micros();
  if (now - lastEncMicros < 500) return; // ~0.5 ms de guarda anti-rebote
  lastEncMicros = now;

  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  if (a == b) encDelta++; else encDelta--;
}

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL); // AREF conectado a 3.3V

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(255);

  // Poblamos calValue[] con datos leidos de la EEPROM
  for (int i = 0; i < nCalPoints; i++) {
    EEPROM.get(20 + i * 4, calValue[i]);
    // Mostramos puntos de calibración
    if (0) {
      Serial.print(i);
      Serial.print("\t");
      Serial.print(calPoint[i]);
      Serial.print("\t");
      Serial.println(calValue[i]);
    }
  }
  delay(5); // pequeño settle de AREF
  measure = analogRead(SENSORPIN);
  unfilteredDist = cm(measure);
  posBola = unfilteredDist;  // semilla del filtro
  ultimaPosBola = posBola;        // semilla consistente para velocidad

  float stored;
  EEPROM.get(EEPROM_ADDR_REST_US, stored);
  if (stored >= SERVO_US_MIN && stored <= SERVO_US_MAX) {
    reposo = stored;
  }
  servo.attach(PIN_SERVO);
  servo.writeMicroseconds((int)reposo);

  if (0) {
    Serial.print("reposo (EEPROM): "); Serial.println(reposo);
    Serial.println("posBola,vel,distAmeta,correcion(us)");
  }  

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
}

int readADC(int pin) {
  long acc = 0;
  const int N = 8;
  for (int i = 0; i < N; ++i) acc += analogRead(pin);
  return (int)(acc / N);
}

float cm(int adc) {
  int hi = (int)max(calValue[0], calValue[nCalPoints - 1]);
  int lo = (int)min(calValue[0], calValue[nCalPoints - 1]);
  adc = constrain(adc, lo, hi);

  // Endpoints por si cae justo en el límite
  if (adc >= calValue[0])                  return calPoint[0];
  if (adc <= calValue[nCalPoints - 1])     return calPoint[nCalPoints - 1];

  // Busca el tramo (tabla descendente en ADC)
  for (int i = 0; i < nCalPoints - 1; ++i) {
    float hiADC = calValue[i];
    float loADC = calValue[i + 1];         // menor que hiADC
    if (adc <= hiADC && adc >= loADC) {
      float t = (adc - hiADC) / (loADC - hiADC); // t in [0,1]
      return calPoint[i] + t * (calPoint[i + 1] - calPoint[i]);
    }
  }

  // Fallback imposible, pero por seguridad:
  return calPoint[nCalPoints - 1];
}

// Dibuja todos en azul, la posición en blanco y el meta en verde
void renderDistance(float dist_cm, float sp_cm) {
  for (int i = 0; i < NUMPIXELS; ++i) pixels.setPixelColor(i, azul); // fondo
  int idxDist = (int)roundf(constrain(dist_cm, 0.0f, (float)NUMPIXELS - 1));
  int idxSP   = (int)roundf(constrain(sp_cm,   0.0f, (float)NUMPIXELS - 1));
  pixels.setPixelColor(idxDist, blanco);
  pixels.setPixelColor(idxSP, verde);
  pixels.show();
}

float ema(float unfilteredDist, float posBola, float k) {
  return k * unfilteredDist + (1 - k) * posBola; // Filtro ema
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if (now - lastLoopTime < TIEMPO_ENTRE_CICLOS) return;
  lastLoopTime += TIEMPO_ENTRE_CICLOS;

  // ---- Medición y filtrado ----
  measure = readADC(SENSORPIN);
  unfilteredDist = cm(measure);
  posBola = ema(unfilteredDist, posBola, kdist);

  // ---- LEE TICKS DEL ENCODER Y AJUSTA META ----
  long delta;
  noInterrupts(); 
  delta = encDelta; 
  encDelta = 0; 
  interrupts();

  if (delta != 0) {
    meta += delta * SP_STEP_CM;
    if (meta < SP_MIN_CM) meta = SP_MIN_CM;
    if (meta > SP_MAX_CM) meta = SP_MAX_CM;
  }

  // Botón: engancha meta a la distancia actual
  if (digitalRead(PIN_BUTTON) == LOW) {
    if (now - lastBtnMs > BTN_DEBOUNCE_MS) {
      meta = posBola;  // “pega” el objetivo a la bola
      if (meta < SP_MIN_CM) meta = SP_MIN_CM;
      if (meta > SP_MAX_CM) meta = SP_MAX_CM;
      lastBtnMs = now;
    }
  }

  renderDistance(posBola, meta);

  // ---- Velocidad (cm/s) ----
  velocidadSinFiltrar = (posBola - ultimaPosBola) / (TIEMPO_ENTRE_CICLOS / 1000.0f);
  ultimaPosBola = posBola;
  velocidad = ema(velocidadSinFiltrar, velocidad, kvel);

  // ---- PID (posición servo en μs relativa a reposo) ----
  distAmeta = posBola - meta;

  // Integral
  if (abs(distAmeta) >= Rint && abs(distAmeta) <= Rext) {
    iAcumulado += distAmeta * i;
  } else {
    iAcumulado = 0; // Si salimos de las bandas, reseta.
  }

  cuantoMasLejosMasCuestaAbajo = p*distAmeta;
  cuantoMasRapidaMasCuestaArriba = d*velocidad;

  correcion = cuantoMasLejosMasCuestaAbajo + cuantoMasRapidaMasCuestaArriba + iAcumulado;

  // Escribir al servo manteniendo límites seguros
  float posServo = reposo + correcion;
  if (posServo < SERVO_US_MIN) posServo = SERVO_US_MIN;
  if (posServo > SERVO_US_MAX) posServo = SERVO_US_MAX;
  servo.writeMicroseconds((int)posServo);

  // Telemetría (para graficar fácil)
  if(0){
    Serial.print(posBola);      Serial.print(",");
    Serial.print(velocidad);  Serial.print(",");
    Serial.print(distAmeta);     Serial.print(",");
    Serial.println(correcion);     // μs relativos
    }
}