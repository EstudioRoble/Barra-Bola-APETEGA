/*
  Comprobación de la calibración del sensor.

  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

// ===== Config =====
#define SENSORPIN A0          // Pin analógico del sensor
#define NEOPIN    4           // NeoPixels
#define NUMPIXELS 32
#define DEBUG 0               // Pon 1 para ver la tabla de calibración por Serial

// ===== Estado =====
int   measure;                // Lo que mide el sensor (ADCs)
float unfilteredDist;         // Distancia sin filtro (cm)
float distFiltered = 0.0f;    // Distancia filtrada (cm)
const float alpha = 0.70f;    // Ganancia de la EMA (0..1) suaviza la "sombra"

// ===== Calibración =====
// calPoint: centímetros
// calValue: cuentas ADC (de mayor a menor)
float calPoint[] = {0, 4, 8, 12, 16, 20, 23, 27, 31};
float calValue[] = {969, 711, 553, 448, 377, 329, 305, 276, 252};
const int nCalPoints = sizeof(calPoint) / sizeof(calPoint[0]);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

// Colores
uint32_t azul    = pixels.Color(0, 0, 3);
uint32_t blanco  = pixels.Color(10, 10, 10);

// ===== Utilidades =====
float cm_from_adc(int adc)
{
  // Limitar a rango de calibración (OJO: último índice es nCalPoints-1)
  adc = constrain(adc, (int)calValue[nCalPoints - 1], (int)calValue[0]);

  // Buscar tramo donde cae 'adc' (calValue es monótono decreciente)
  for (int i = 0; i < nCalPoints - 1; ++i) {
    float hiADC = calValue[i];
    float loADC = calValue[i + 1];
    if (adc <= hiADC && adc >= loADC) {
      float t = (adc - hiADC) / (loADC - hiADC);           // t in [0,1]
      return calPoint[i] + t * (calPoint[i + 1] - calPoint[i]);
    }
  }
  // Si por redondeos no entra en el bucle, devuelve el último punto.
  return calPoint[nCalPoints - 1];
}

void renderDistance(float dist_cm)
{
  // Fondo azul
  for (int i = 0; i < NUMPIXELS; ++i) pixels.setPixelColor(i, azul);

  // Posición blanca (clamp y redondeo al LED más cercano)
  int idx = (int)round(dist_cm);
  idx = constrain(idx, 0, NUMPIXELS - 1);
  pixels.setPixelColor(idx, blanco);

  pixels.show();
}

// ===== Setup / Loop =====
void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL); // AREF conectado a 3.3V

  pixels.begin();
  pixels.setBrightness(255);

  // Cargar (opcional) valores de calibración desde EEPROM (float = 4B)
  // Si no has grabado antes esos floats, deja tal cual tu tabla por defecto.
  for (int i = 0; i < nCalPoints; i++) {
    float v;
    EEPROM.get(20 + i * 4, v);
    // Opcional: solo aceptar si parece "creíble"
    if (isfinite(v) && v > 50 && v < 1024) {
      calValue[i] = v;
    }
  }

#if DEBUG
  for (int i = 0; i < nCalPoints; i++) {
    Serial.print(i); Serial.print("\t");
    Serial.print(calPoint[i]); Serial.print("\t");
    Serial.println(calValue[i]);
  }
  Serial.println();
#endif

  // Inicializa el filtro con el primer valor realista
  measure = analogRead(SENSORPIN);
  unfilteredDist = cm_from_adc(measure);
  distFiltered = unfilteredDist;
}

void loop() {
  measure = analogRead(SENSORPIN);            // RAW data
  unfilteredDist = cm_from_adc(measure);      // cm sin filtrar

  // Suavizado EMA
  distFiltered += alpha * (unfilteredDist - distFiltered);

  // Render
  renderDistance(distFiltered);

  // Telemetría
  Serial.print((int)distFiltered);
  Serial.print("\t");
  Serial.println(distFiltered, 2);

  delay(50); // ~20 Hz
}
