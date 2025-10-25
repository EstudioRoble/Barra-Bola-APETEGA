/*
  Comprobación del funcionamiento del Sensor de distancia.

  bajo Licencia Creative Commons Atribución-CompartirIgual 4.0 Internacional.
  https://creativecommons.org/licenses/by-sa/4.0/

  octubre de 2025
  por Angel Espeso
  ESTUDIO ROBLE
  https://roble.uno
*/

#define sensorPin 0 //Pin Analogico donde esta conectada la señal del Sensor de distancia

int measure; // Lo que mide el sensor. Son ADCs.

void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL); // AREF conectado a 3.3V
}

void loop() {
  measure = analogRead(sensorPin); // RAW data
  Serial.println(measure);
  delay(200);
}