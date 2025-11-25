#include <ps4.h>
#include <PS4Controller.h>
#include <ps4_int.h>
#include <ESP32Servo.h>
//#include "thingProperties.h"

/*static const int signal1 = 15;
static const int signal2 = 4;
static const int signal3 = 5;
static const int signal4 = 19;
static const int signal5 = 21;
static const int signal6 = 23;*/
float L1=151.7f, L2 = 143.7f;
int o0 = 0, o1 = 0, o2 = 0, o3 = 0;
static const int servosPins[6] = {15,4,5,19,21,23};
int X,Y,Z;
int a[6] = {0,0,0,0,0,0};
int g[6] = {0,0,0,0,0,0};
int velocidade = 20;
float C;
unsigned long tempo;
Servo servos[6];

void setup() {
  Serial.begin(9600);  
  delay(1500); 

  PS4.begin("dc:97:ba:e7:68:7c");//Inicia configuração com o controle a partir do endereço MAC
  Serial.print("Iniciando conexão com o controle...");
  while(!PS4.isConnected()){
    Serial.print('.');
    delay(250);
  }

  PS4.setLed(0, 100, 0);//Configura led para a cor verde
  PS4.sendToController();//Envia dado para configurar no controle
  
  tempo = millis();
  for(int i = 0; i < 6; ++i) {
    if(!servos[i].attach(servosPins[i])) {
      Serial.print("Servo ");
      Serial.print(i);
      Serial.println("attach error");
    }
  }
  
  g[4]=90;
  g[0]=90;
//  initProperties();
  C = 180 / 3.1415f;
//  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
//  setDebugMessageLevel(2);
//  ArduinoCloud.printDebugInfo();
}

void setServos() 
{
  if(millis()-tempo<velocidade){return;} 
  tempo = millis();
  if(a[0]!=g[0]) // base
  {
    int a1 = a[0]+(g[0]-a[0])/abs(g[0]-a[0]);
    servos[0].write(a1);  
    a[0]=a1;
  }
  if(a[1]!=g[1]) // arm
  {
    int a1 = a[1]+(g[1]-a[1])/abs(g[1]-a[1]);
    servos[1].write(a1);  
    servos[2].write(180-a1+9);  
    a[1]=a1;
  }
  if(a[3]!=g[3]) // forearm
  {
    int a1 = a[3]+(g[3]-a[3])/abs(g[3]-a[3]);
    servos[3].write(a1);  
    a[3]=a1;
  }
  if(a[4]!=g[4]) // pulso
  {
    int a1 = a[4]+(g[4]-a[4])/abs(g[4]-a[4]);
    servos[4].write(a1); 
    servos[5].write(180-a1); 
    a[4]=a1;
  }
  
}

void loop() {
//  ArduinoCloud.update();
  if(PS4.isConnected()){
    //Valore do analógico -127 ~ 127
    int LStickX = PS4.LStickX();
    int LStickY = PS4.LStickY();
    int RStickX = PS4.RStickX();
    int RStickY = PS4.RStickY();

    int motorBase = map(RStickX, -127, 127, -180, 180);//Mapea dados para a base do motor
    int motorBraco = map(RStickY, -127, 127, -180, 180);//Mapea dados para o braço do motor
    int motorCotovelo = map(LStickX, -127, 127, -180, 180);//Mapea dados para a base do motor//Mapea dados para o cotovelo do motor
    int motorPulso = map(LStickY, -127, 127, -180, 180);//Mapea dados para a base do motor//Mapea dados para o pulso do motor
   
    if( 100 < motorBase || motorBase < -100){
      while(g[0] < 180 && 100 < motorBase){
        motorBase = map(PS4.RStickX(), -127, 127, -180, 180);
        if(100 > motorBase) break;
        g[0]++;
        delay(200);
      }
      
      while(g[0] > 0 && motorBase < -100){
        motorBase = map(PS4.RStickX(), -127, 127, -180, 180);
        if(motorBase > -100) break;
        g[0]--;
        delay(200);
      }
    }

    if( 100 < motorBraco || motorBraco < -100){
      while(g[1] < 155 && 100 < motorBraco){
        motorBraco = map(PS4.RStickY(), -127, 127, -180, 180);
        if(100 > motorBraco) break;
        g[1]++;
        delay(200);
      }
      
      while(g[1] > -25 && motorBraco < -100){
        motorBraco = map(PS4.RStickY(), -127, 127, -180, 180);
        if(motorBraco > -100) break;
        g[1]--;
        delay(200);
      }
    }

    if( 100 < motorCotovelo || motorCotovelo < -100){
      while(g[3] < 150 && 100 < motorCotovelo){
        motorCotovelo = map(PS4.LStickX(), -127, 127, -180, 180);
        if(100 > motorCotovelo) break;
        g[3]++;
        delay(200);
      }
      
      while(g[3] > -30 && motorCotovelo < -100){
        motorCotovelo = map(PS4.LStickX(), -127, 127, -180, 180);
        if(motorCotovelo > -100) break;
        g[3]--;
        delay(200);
      }
    }

    if( 100 < motorPulso || motorPulso < -100){
      while(g[4] < 180 && 100 < motorPulso){
        motorPulso = map(PS4.LStickY(), -127, 127, -180, 180);
        if(100 > motorPulso) break;
        g[4]++;
        delay(200);
      }
      
      while(g[4] > 0 && motorPulso < -100){
        motorPulso = map(PS4.LStickY(), -127, 127, -180, 180);
        if(motorPulso > -100) break;
        g[4]--;
        delay(200);
      }
    }

    Serial.println("---------------------------");
    Serial.println(g[0]);
    Serial.println(g[1]);
    Serial.println(g[3]);
    Serial.println(g[4]);
    Serial.println("===========================");
  }else{
    //setServos(); 
  }
  
  //angulos();
  delay(1000);
}

/*void angulos()
{
   float xy = sqrt(X*X + Y*Y);
  
   g[0] = atan2(y,x) * C;
   //g[1] = teta1(xy) -25;
   g[1] = constrain(g[1], 25, 205);
  g[0] = constrain(g[0], 0, 180);
   float Sen = (Z - L1 * sin(g[1] / C)) / L2;
   float Cos = (xy - L1 * cos(g[1] / C)) / L2;
   int aSin = asin(Sen) * C;
   int aCos = acos(Cos) * C;

    Serial.print("base = ");
   Serial.print(g[0]);
   Serial.print(", braco = ");
   Serial.print(g[1]);
   g[3] = aSin;
   g[3] = constrain(g[3], -25, 155);
   Serial.print(", antebraço = ");
   Serial.println(g[3]);
   
}
int teta1(float xy)
{
  for(int i=0;i<180;i++)
  {
    float result = sqrt(pow(xy - L1*cos(i / C), 2) + pow(Z - L1*sin(i / C), 2)) - L2;
    if(result < 1 and result > -1)
    {
      return i;
    }
  }
  return 0;
}
void onSpeedChange()  
{
 // velocidade = 500/speed;
}
void onBaseChange()  
{
  //g[0] = base;
}
void onArmChange()  
{
  //25 ate 205
  //g[1] = arm-25;
}
void onForearmChange() 
{
  // -25 ate 155
  //g[3] = forearm+25;
}
void onPulsofowardChange()  
{
  //g[4] = pulsofoward;
}

void onO0Change()  {
  g[0] = o0;
}
void onO1Change()  {
  //155 = 0; 
  // -25 ate 155
  g[1] =   155  - o1;
}
void onO2Change()  {
  // -30 150
  g[3] = o2 + 30;
}
void onO3Change()  {
  g[4] = o3;
}
void onO4Change()  {
  // Add your code here to act upon O4 change
}*/
