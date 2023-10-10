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
#include <PID_v1.h>

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

// Define as constantes do PID (ajuste conforme necessário)
float Temperatura;
float SetPointTemp;
float SetPointUmid;
double kp = 1.0;  // Ganho Proporcional
double ki = 0.1;  // Ganho Integral
double kd = 0.01; // Ganho Derivativo
double setpoint = SetPointTemp; // Valor desejado (setpoint)
double input, output;

// Crie um objeto PID para a temperatura
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);


void temperaturaUmidade(){
  // Inicialize o PID
  myPID.SetMode(AUTOMATIC); // Inicia o PID em modo automático
  myPID.SetOutputLimits(0, 255); // Limita a saída entre 0 e 255 (para PWM, por exemplo)

  SetPointUmid = Blynk.virtualWrite(v7); // umidade 
  SetPointTemp = Blynk.virtualWrite(v6); // tempeeratura
  
  float h = dht.readHumidity();
  float t = dht.readTemperature(); 
  Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V3, h);
  Temperatura = dht.readHumidity();

  if (dht.readHumidity() < SetPointUmid) {
    digitalWrite(piezo, HIGH);
  } else {
    digitalWrite(piezo, LOW);
  }

  if (Temperatura < SetPointTemp){
    if (Temperatura > SetPointTemp-3){
    digitalWrite(releSPDT, LOW);
    digitalWrite(peltier, HIGH);
    }
     if (Temperatura <= SetPointTemp-3){
      input = Temperatura;
      myPID.Compute(); //calcula o controle PID
      analogWrite(peltier,output);
    }
  }

  if (Temperatura > SetPointTemp){
    if (Temperatura < SetPointTemp+3){
    digitalWrite(releSPDT , HIGH);
    digitalWrite(peltier, HIGH);
    }
      if (Temperatura >= SetPointTemp+3){
      input = Temperatura;
      myPID.Compute(); //calcula o controle PID
      analogWrite(peltier,output);
  }
}
}

void reservatorioNutr(){
  
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

void reservatorioAgua(){
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
  int porcento = map(umidade, 1023, 0, 0, 100);

  Serial.print(porcento);
  Serial.println("%");

 

  if(porcento <= (-200))// alterar aqui depois dos testes 
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


//pH
void fph(){
float valor_calibracao = 21.34 ;   // Fator de calibração
 
int contagem = 0;           // Variável de contagem
float soma_tensao = 0;      // Variável para soma de tensão
float media = 0;            // Variável que calcula a media
float entrada_34;           // Variável de leitura do pino A0
float tensao;               // Variável para conversão em tensão
unsigned long tempo;        // Float tempo

  soma_tensao = 0;   // Inicia soma_tensão em 0
  contagem = 0;      // Inicia a contagem em 0

 while (contagem < 10) {                   // Executa enquanto contagem menor que 10
    tempo = millis();                       // Define o tempo em microssegundos
    entrada_34 = analogRead(ph);            // Lê a entrada analógica
    tensao = (entrada_34 * 5.0) / 1024.0;   // Converte em tensão, o valor lido
    soma_tensao = (soma_tensao + tensao);   // Soma a tensão anterior com a atual
    contagem++;                             // Soma 1 à variável de contagem
    delay(100);                             // Aguarda para próxima leitura
  }

 media = soma_tensao / 10; 

float valor_pH = -5.70 * media + valor_calibracao;    // Calcula valor de pH
   float valor = analogRead(18);
  if (valor > 7)  {
  //manda mensagem para diminuir o ph
  }
  else {
    if (valor < 5) {
      // manda mensagem para aumentar o ph
    }
  }
  Serial.println(valor_pH);
  delay(1000);                    // Aguarda para próxima leitura
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
  timer.setInterval(1000L, reservatorioAgua);
  timer.setInterval(1000L, reservatorioNutr);
  timer.setInterval(1000L, luminosidade); 
  timer.setInterval(1000L, fph);
}

void loop(){
  Blynk.run();
  timer.run();
}
