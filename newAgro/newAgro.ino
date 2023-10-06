/**************************************************
          Projeto:New-Agro;
          
          Hardware: ESP32;
          Plataforma: IDE ARDUINO;
          Placa: DOIT ESP32 DEVKIT V1;

          Sensores: Ultrassonico, Higrometro, DRH22, LDR; PH;
          Elaborado por: Nadyne Barbieri e Gustavo Bizon;   
          
          Código: C++
          Versão: 1.0
          Data: //

***************************************************/
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL2JbDDPn3_"
#define BLYNK_TEMPLATE_NAME "NewAgro"
#define BLYNK_AUTH_TOKEN "IgxA1ctSG41kA3JuM7Ov2iD3PZRqGL4y"

#include <Ultrasonic.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

char ssid[] = "NADYNE_2G";
char pass[] = "Naoseiasenha";

//ATUADORES
#define alerta 2
#define piezo 5
#define bomba 12
#define led 13
#define peltier 15
#define releSPDT 18
#define motorDC 19
#define microBomba 21
#define solenoideAgua 22
#define solenoideNutr 23

//SENSORES
#define higrometro 33
#define DHTPIN 32
#define luz 35
#define ph 34
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);
Ultrasonic ultrasonicAgua(14, 27);
Ultrasonic ultrasonicNutr(26, 25);

BlynkTimer timer; //CONECTA A NUVEM 
 BLYNK_WRITE(V0) {
  int value =  param.asInt();
  digitalWrite(motorDC, value);  
}

void temperaturaUmidade(){
  float h = dht.readHumidity();
  float t = dht.readTemperature(); 
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V3, h);

  if (isnan(h) || isnan(t)) {
    Serial.println("Falha na leitura");
    return;
  }

  if (dht.readHumidity() <33) {
    digitalWrite(piezo, HIGH);
  } else {
    digitalWrite(piezo, LOW);
  }

  if (dht.readTemperature() > 24){
    digitalWrite(peltier, HIGH);
    delay(10000);
    digitalWrite(peltier,LOW);
  }

  if(dht.readTemperature() < 18) {
    digitalWrite(releSPDT , HIGH);
    digitalWrite(peltier, HIGH);
    delay(10000);
    digitalWrite(peltier, LOW);
    digitalWrite(releSPDT ,LOW);
  }
}

void ReservatorioNutr(){
  
  int distanceNutr = ultrasonicNutr.read();
  Serial.print("nutrientes: ");
  Serial.println(distanceNutr);
  delay(1000);
 
  if(distanceNutr > 100){ //aterar o valor de alcance aqui
    digitalWrite(alerta, HIGH);
    delay(1000);//falta definir tempo que valvula vai ficar ligada
    digitalWrite(alerta, LOW);
  
    digitalWrite(solenoideNutr, HIGH);
    delay(1000);//falta definir tempo que valvula vai ficar ligada
    digitalWrite(solenoideNutr, LOW);
    //delay(100);
    digitalWrite(motorDC, HIGH);
    delay(100); // definir tempo que o motor vai ficar ligado aqui 
    digitalWrite(motorDC, LOW);
  }
  Blynk.virtualWrite(V4, distanceNutr);
}

void ReservatorioAgua(){
  int distanceAgua = ultrasonicAgua.read();
  //Serial.print("Agua: ");
 // Serial.println(distanceAgua);
  Blynk.virtualWrite(V5, distanceAgua);

  if(distanceAgua > 100){ //aterar o valor de alcance aqui
    digitalWrite(solenoideAgua, HIGH);
    delay(5000);//falta definir tempo ficara ligado
    digitalWrite(solenoideAgua, LOW);
    digitalWrite(microBomba, HIGH);
    delay(1000);//falta definir tempo ficara ligado
    digitalWrite(microBomba, LOW);
  } else {
      digitalWrite(solenoideAgua, LOW);
      digitalWrite(microBomba, LOW);
    }

  //Higrometro

  int umidade = analogRead(higrometro);
  int Porcento = map(umidade, 1023, 0, 0, 100);

  Serial.print(Porcento);
  Serial.println("%");

 

  if(Porcento <= (-200))// alterar aqui depois dos testes 
  {
    Serial.println("Irrigando...");
    digitalWrite(bomba, HIGH);
    } else {
    digitalWrite(bomba, LOW);
    }
    delay(100);

}
//LUMINOSIDADE 
void luminosidade(){
int ldrValor = analogRead(luz); 
 
if (ldrValor>= 800) {
  digitalWrite(led,HIGH);
} else {digitalWrite(led,LOW);
  Serial.println(ldrValor);
  delay(100);
  }
  Blynk.virtualWrite(V1, ldrValor);
}

void setup(){
  //Serial.begin(115200);
  pinMode(bomba, OUTPUT);
  pinMode(peltier, OUTPUT);
  pinMode(piezo, OUTPUT);
  pinMode(releSPDT, OUTPUT);
  pinMode(motorDC, OUTPUT);
  pinMode(microBomba, OUTPUT);
  pinMode(solenoideAgua, OUTPUT);
  pinMode(solenoideNutr, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(alerta, OUTPUT);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  dht.begin();
  timer.setInterval(1000L, temperaturaUmidade);
  timer.setInterval(1000L, ReservatorioAgua);
  timer.setInterval(1000L, ReservatorioNutr);
  timer.setInterval(1000L, luminosidade); 
}

void loop(){
  Blynk.run();
  timer.run();
}
