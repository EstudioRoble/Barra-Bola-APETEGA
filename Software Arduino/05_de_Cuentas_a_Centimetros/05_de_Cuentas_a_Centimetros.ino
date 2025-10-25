/*
  Calibración con 9 puntos del sensor.
  Coloca la bola sobre el neopixel encendido y pulsa el encoder.

  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

// *****************************************************************************
#define SENSORPIN 0 // Pin Analogico donde esta conectada la señal del Sensor de distancia
#define NEOPIN    4 // neoPixels
#define BUTTONPIN 5 // Botón del Encoder


int measure; // Lo que mide el sensor. Son ADCs.
float unfilteredDist; // measure después de calibracion

float calPoint[] = {0, 4, 8, 12, 16, 20, 23, 27, 31};
float calValue[] = {969, 711, 553, 448, 377, 329, 305, 276, 252};
const int nCalPoints = sizeof(calPoint) / sizeof(calPoint[0]);

#define NUMPIXELS      32
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

uint32_t azul = pixels.Color(0, 0, 3);
uint32_t blanco = pixels.Color(10, 10, 10);

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL); // AREF conectado a 3.3V

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(255);

  pinMode(BUTTONPIN, INPUT_PULLUP);
}

void loop() {
  for (int point = 0; point < nCalPoints ; point++) { // /2 xq un int ocupa 2 bytes
    pixels.clear();
    pixels.setPixelColor(calPoint[point], blanco);
    pixels.show();
    while (digitalRead(BUTTONPIN) == HIGH) {
      // Espero que coloquen la bola y aprieta el boton
    }
        // Tomamos n lecturas, las promediamos => calRead
    int n = 100; // Nº de lecturas tomadas para calcular calRead
    long calRead = 0;
    for (int i = 0; i < n; i++) {
      int reading = analogRead(A0);
      calRead += reading;
      delay(10);
    }
    calRead = calRead / n;
    Serial.print("#");
    Serial.print(point);
    Serial.print(":\t");
    Serial.print(int(calPoint[point]));
    Serial.print(" cm tienen ");
    Serial.print(calRead);
    Serial.println(" cuentas");

    //Almacenamos en la EEPROM a partir de la posicion 20
    /* Mapa de Memoria/home/atreyu/Escritorio/APETEGA/Firmware/_201_BB_Inicializamos_EEPROM_Quita_nan/_201_BB_Inicializamos_EEPROM_Quita_nan.ino
    [ 0 -  3] SetPoints   [Sin usar en esta version]
    [ 4 - 15] kp, kd, ki  [Sin usar en esta version]
    [16 - 19] Valor Horizontal de Reposo
    [20 - 59] Lecturas del sensor en calPoint    */
    EEPROM.put(20 + point * 4, float(calRead));
    calValue[point] = calRead;
    while (digitalRead(BUTTONPIN) == LOW) {
      // Espero que suelte el boton
    }
  }
  Serial.println("Calibración finalizada.");
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, azul);
  pixels.show();
  while (true) {
    // Bucle infinito: no vuelve a ejecutar loop()
    delay(1000);
  }
}