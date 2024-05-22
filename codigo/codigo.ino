//////////////////////////////////////////////////
/*  "#include <Arduino.h>" é necessário para o código correr no CLion.
    Se o código for corrido no Arduino IDE, esta linha TEM OBRIGATORIAMENTE DE SER RETIRADA!
*/
//Para cada include tem de se INSTALAR a biblioteca procurando pelo nome em "libraries"
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Wire.h>
#include "RTClib.h"
#include <Servo.h>

//Definição dos pinos
//pinos analógicos
const int pinSensorTemperatura = A0;
const int pinPotenciometro = A1;
const int pinSensorChama = A2;
const int pinLDR = A3;   // Pino analógico para o Sensor de Luz LDR
//const int joystickX = A4; // Eixo X
//const int waterLevelPin = A5; // Pino do sensor de nível de água


//pinos digitais
#define DHTPIN 2 // Pino de dados do sensor DHT11 (pino digital 2)
const int pinLEDVermelho = 3;
const int pinLEDVerde = 4;
const int pinLEDAzul = 6;  // Pino PWM para o LED azul
const int pinBuzzer = 5;
const int buttonPin = 7; // Pino digital para o botão
const int pinRed = 8;
const int pinGreen = 9;
const int pinBlue = 10;


//Outras configurações
int lastButtonState = LOW;  // Estado anterior do botão
int currentButtonState;     // Estado atual do botão

int redValue = 0;   // Intensidade do LED vermelho
int greenValue = 0; // Intensidade do LED verde
int blueValue = 0;  // Intensidade do LED azul
//int lightLevel;     // Nível de luminosidade lido pelo LDR

int servoPos = 90; // Posição inicial do servo

//Definições restantes do sensor DHT
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); //Originalmente: DHT dht(DHTPIN, DHTTYPE);



// Configuração do display LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); //O LCD tem (e é) de ser ligado aos pinos analógicos A4 e A5

// Configuração do RTC
RTC_DS3231 rtc;


//Servo motor
// Cria um objeto servo para controlar o servo motor
Servo myServo;

// Valores lidos dos sensores
int valorPotenciometro = 0;
int valorSensorChama = 0;

//int n = 15; //relacionado com o filtro da temperatura //tinhamos começado por 7

float voltage = 0;

float tempC;

void setup() {
    //Inicialização dos pinos
    pinMode(pinPotenciometro, INPUT);
    pinMode(pinSensorChama, INPUT);
    pinMode(pinBuzzer, OUTPUT);
    pinMode(pinLEDVermelho, OUTPUT);
    pinMode(pinLEDVerde, OUTPUT);
    pinMode(pinLEDAzul, OUTPUT);
    pinMode(buttonPin, INPUT);

    lcd.print("Servo Position:");

    lcd.print("Nivel de Agua:");


    // Anexa o servo ao pino D9
    myServo.attach(9);
    myServo.write(servoPos); // Posição inicial do servo
    
    dht.begin();
    lcd.init();                      // Inicializa o LCD
    lcd.backlight();                 // Acende a luz de fundo
    
    if (!rtc.begin()) {
        Serial.println("Não foi possível encontrar o RTC");
    }
    if (rtc.lostPower()) {
        Serial.println("RTC perdeu a hora, ajuste novamente!");
        // Quando o RTC perdeu a hora, defina a hora corrente assim:
        // Formato: DateTime(ano, mês, dia, hora, minuto, segundo);
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));//rtc.adjust(DateTime(2024, 5, 13, 12, 14, 25)); //ISTO AINDA NÃO DÁ BEM
    }
    
    
    // Ajusta o RTC para uma data e hora específica
    //rtc.adjust(DateTime(2024, 5, 13, 12, 14, 25));

    // Iniciar comunicação serial para depuração
    Serial.begin(9600);
}

void loop() {

    //1. Estação Meteorológica - //FUNCIONA

    // Leitura do sensor de temperatura LM35
    //float voltage = analogRead(pinSensorTemperatura) * 5.0 / 1023.0;
    //float tempC = voltage * 100.0;
    /*
    if(analogRead(pinSensorTemperatura)!= voltage)
    {
      voltage = (voltage * n + analogRead(pinSensorTemperatura))/ (n+1); //filtro temperatura
      tempC = voltage;
      n++;
    }
    */



    // Leitura do DHT11

    float humidity = dht.readHumidity();
    float tempDHT = dht.readTemperature();

    // Leitura do RTC
    DateTime now = rtc.now();

    // Mostrar dados no LCD
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    //lcd.print(tempC); //temperatura do sensor de temp
    //lcd.print("/");
    lcd.print(tempDHT); //temperatura do sensorDHT
    lcd.print(" C");
    
    // Get humidity event and print its value.
    /*
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity);
    lcd.print("%");
    */
    // Mostra a data e a hora
    
    lcd.setCursor(0, 1);  // Ajuste se necessário dependendo do seu display
    lcd.print(now.day(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.year(), DEC);
    lcd.print(' ');
    
    //lcd.setCursor(0, 1);  // Ajuste se necessário dependendo do seu display
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    //lcd.print(':');
    //lcd.print(now.second(), DEC);
    
    
    //2. Sistema de Alarme de Segurança //FUNCIONA
    valorPotenciometro = analogRead(pinPotenciometro);
    valorSensorChama = analogRead(pinSensorChama);

    Serial.print("Valor potenciometro: ");
    Serial.println(valorPotenciometro);

    // Simulação de detecção de "presença" com potenciômetro
    if (valorPotenciometro > 512) { // Ajuste este valor conforme a sensibilidade desejada
        digitalWrite(pinLEDVerde, HIGH);
        //tone(pinBuzzer, 1000, 200); // Emitir um som breve //FUNCIONA É SÓ DESCOMENTAR
        delay(2000); // Tempo ativo do LED verde
        digitalWrite(pinLEDVerde, LOW);
    }
    Serial.print("Valor sensor chama: ");
    Serial.println(valorSensorChama);
    // Detecção de chama - FUNCIONA
    if (valorSensorChama > 300) { // Ajuste este valor conforme a sensibilidade do sensor de chama
        digitalWrite(pinLEDVermelho, HIGH);
        //tone(pinBuzzer, 2000, 300); // Emitir um som mais longo e agudo para fogo
        delay(2000); // Tempo ativo do LED vermelho
        digitalWrite(pinLEDVermelho, LOW);
    }

    //3. Controlador de Iluminação Automatizado. //CÓDIGO PRONTO, FALTA TESTAR
    
    //lightLevel = analogRead(pinLDR); // Lê o valor do LDR
    currentButtonState = digitalRead(buttonPin);

    // Verifica se o botão foi pressionado
    if (currentButtonState == HIGH && lastButtonState == LOW) {
      changeColor(); // Altera a cor do LED
      delay(150);    // Debounce do botão
    }
    lastButtonState = currentButtonState; // Atualiza o estado do botão
    Serial.print("lastbuttonstate:");
    Serial.println(lastButtonState);
    Serial.print("currentButtonState:");
    Serial.println(currentButtonState);
    /*
    if (currentButtonState == LOW) {
      //adjustLight(lightLevel); // Ajusta a cor do LED de acordo com a luminosidade
    }
    */
    // Atualiza as cores do LED RGB
    analogWrite(pinRed, redValue);
    analogWrite(pinGreen, greenValue);
    analogWrite(pinBlue, blueValue);


    
    //4. Interface de Jogos ou Controle Robótico
    /*
    int xValue = analogRead(joystickX); // Lê o eixo X do joystick
    xValue = map(xValue, 0, 1023, 0, 180); // Mapeia o valor para o ângulo do servo (0 a 180)

    if (abs(xValue - servoPos) > 2) { // Se a mudança for significativa
      servoPos = xValue;
      myServo.write(servoPos); // Move o servo
      updateLCD(servoPos); // Atualiza o display LCD
    }
    */
    //5. Sistema de Monitoramento de Níveis de Água

    //6. Automação Residencial com Controle Remoto IR

    // Aguarda um segundo antes de atualizar o display
    delay(2000);
}

void adjustLight(int light) {
  // Converte o nível de luz em valores PWM para o LED
  int brightness = map(light, 0, 1023, 0, 255);
  redValue = brightness;
  greenValue = brightness;
  blueValue = brightness;
}

void changeColor() {
  // Muda a cor do LED entre algumas cores predefinidas
  if (redValue == 255) {
    // Vermelho para Verde
    redValue = 0;
    greenValue = 255;
    blueValue = 0;
  } else if (greenValue == 255) {
    // Verde para Azul
    redValue = 0;
    greenValue = 0;
    blueValue = 255;
  } else {
    // Azul para Vermelho
    redValue = 255;
    greenValue = 0;
    blueValue = 0;
  }
}

void updateLCD(int position) {
  lcd.setCursor(0, 1);
  lcd.print("Angle: ");
  lcd.print(position);
  lcd.print("    "); // Limpa caracteres extras
}
