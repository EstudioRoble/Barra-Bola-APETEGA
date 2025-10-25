/*
  Comprobación del funcionamiento de los Neopixeles.

  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/

#include <Adafruit_NeoPixel.h>

// *****************************************************************************
#define NEOPIN    4 // neoPixels

#define NUMPIXELS      32
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

uint32_t azul = pixels.Color(0, 0, 3);
uint32_t blanco = pixels.Color(10, 10, 10);

int p = 0; // Posición del pixel blanco

void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(255);
}

void loop() {
  // Fondo azul
  for (int i = 0; i < NUMPIXELS; ++i)
    pixels.setPixelColor(i, azul);

  // Punto blanco en la posición actual
  pixels.setPixelColor(p, blanco);
  pixels.show();

  // Avanza a la siguiente posición
  p++;
  if (p >= NUMPIXELS) p = 0;  // Reinicia al llegar al final

  delay(200);
}
