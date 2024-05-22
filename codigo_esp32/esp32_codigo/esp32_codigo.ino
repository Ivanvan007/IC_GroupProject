//Código para verificar se o ESP32 funciona
#include <SoftwareSerial.h>

// Definir os pinos para a comunicação serial
const int rxPin = 0;
const int txPin = 1;

// Inicializar a comunicação serial com o ESP32
SoftwareSerial espSerial(rxPin, txPin);

void setup() {
  // Inicializar a comunicação serial com o computador
  Serial.begin(9600);
  while (!Serial) {
    ; // Esperar pela inicialização da serial
  }

  // Inicializar a comunicação serial com o ESP32
  espSerial.begin(115200);
  delay(1000); // Tempo para o ESP32 inicializar

  Serial.println("Testando comunicação com ESP32...");

  // Enviar o comando AT ao ESP32
  espSerial.println("AT");
}

void loop() {
  // Verificar se há dados do ESP32
  if (espSerial.available()) {
    while (espSerial.available()) {
      char c = espSerial.read();
      Serial.write(c); // Enviar a resposta para o monitor serial
    }
  }

  // Verificar se há dados do monitor serial
  if (Serial.available()) {
    while (Serial.available()) {
      char c = Serial.read();
      espSerial.write(c); // Enviar os dados para o ESP32
    }
  }
}
