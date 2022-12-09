#include <Servo.h>
#include <SPI.h>
#include <Encoder.h>
#include"task.h"
#include"timer_group_struct.h"
#include"timer_group_reg.h"

#define ESP32_PC_MEASURE_RESULT 0xFF
#define PC_ESP32_MEASURE_REQUEST 0xFE
#define PC_ESP32_SEND_MATERIAL 0xFD
#define PC_ESP32_SEND_PERMISSION 0xFC

#define PC_PERMIT 0xFB
#define PC_NOT_PERMIT 0xFA

//DA出力のためのピン指定
#define SCLK 18
#define MOSI 23
#define MISO 19
#define CS 5

//sin関数のため
#define PIE 3.141592653589793
#define TIME_US 1000000.0  //これで割ればμsをsに直Wせる

//ループのため
#define REFRESH_TIME_US 100  //ループの一周の秒数 単位はμ ループは10kHzにするために1/10000,100μs

#define AMP_RUB 240.0
#define AMP_WOO 150.0
#define AMP_ALU 300.0

#define DEC_RUB 60.0
#define DEC_WOO 80.0
#define DEC_ALU 90.0

#define FRQ_RUB 30.0
#define FRQ_WOO 100.0
#define FRQ_ALU 300.0

struct contact {
  int flag;
  float vel;
};

long time_us;
long time_sin;
long time_sin_0;

//sin関数の為
int material = 3;
float A[4] = {0, AMP_RUB, AMP_WOO, AMP_ALU};
float B[4] = {0, DEC_RUB, DEC_WOO, DEC_ALU};
float F[4] = {0, FRQ_RUB, FRQ_WOO, FRQ_ALU};

unsigned char snd[4];

char PCPermission = PC_PERMIT;

contact myContact;

Encoder myEnc(16, 17);
Servo myservo;

void DAout(char channel, float voltage);
contact ReadContact(void);
void SetVibration(int isContact);
void feedTheDog();
void task0(void* param);

void setup() {
  Serial.begin(115200);
  
  SPI.begin(SCLK, MISO, MOSI, CS);
  SPI.setFrequency(20000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setHwCs(true);
  
  myservo.attach(2);

  myContact.flag = 0;

  DAout(0, 5);

  xTaskCreatePinnedToCore(task0,"Task0",4096,NULL,1,NULL,0);
}

void loop() {
  time_us = micros();
  
  myContact = ReadContact(myContact);
  SetVibration(myContact);
  while ((micros() - time_us) < REFRESH_TIME_US); //1ループの秒数をそろえる
}


void DAout(char channel, float voltage) {
  short spiData, DA;
  DA = (voltage / 5.0) * 4095;
  spiData = ((channel << 15) | (0x07 << 12) | (DA & 0x0FFF));
  SPI.transfer16(spiData);
}

contact ReadContact(contact oldContact) {
  contact temp = oldContact;
  int arrayNum = 51;
  static int orientation[51];
  static int i = 0;
  int kazu = arrayNum-1;
  orientation[i] = myEnc.read();

  
  snd[0] = ESP32_PC_MEASURE_RESULT;
  snd[1] = orientation[i]>>8;
  snd[2] = orientation[i]&0xF0;
  snd[3] = orientation[i]&0x0F;
  
  if(i>=kazu){
    temp.vel = -(orientation[i] - orientation[i-kazu]) / (REFRESH_TIME_US/ TIME_US * kazu)/2000;
    if(temp.vel>1)temp.vel=1;
  }else{
    temp.vel = -(orientation[i] - orientation[arrayNum-kazu + i]) / (REFRESH_TIME_US/ TIME_US * kazu)/2000;
    if(temp.vel>1)temp.vel=1;
  }
  if (orientation[i] <= 0) temp.flag = 1;
  if (orientation[i] > 20 && temp.flag == 1)temp.flag = 0;
  i++;
  if(i==arrayNum-1)i=0;
  return temp;
}

void SetVibration(contact newCnt) {
  static contact oldCnt;
  static int t_us;
  float A_v;
  static float vel;
  if (oldCnt.flag == 0 && newCnt.flag == 1 && PCPermission == PC_PERMIT) {
    t_us = 0;
    time_sin_0 = micros();
    vel = newCnt.vel;
  }
  t_us = micros() - time_sin_0;
  A_v = ((A[material] * vel * exp(- B[material] * t_us / TIME_US) * sin(2.0 * PIE * F[material] * t_us / TIME_US)) / 300.0) * 2.5  + 2.5;
  DAout(0, A_v);
  oldCnt = newCnt;
}

void task0(void* param){
  char rcv;
  while(1){
    if(Serial.available() > 0){
      rcv = Serial.read();
      if(rcv == PC_ESP32_MEASURE_REQUEST){
        Serial.write(snd,4);  
        Serial.flush();
      } 
      if(rcv == PC_ESP32_SEND_MATERIAL){
        material = Serial.read()-48;
        myservo.write(60 * material); 
        Serial.flush();
      } 
      if(rcv == PC_ESP32_SEND_PERMISSION){
        PCPermission = Serial.read();
        Serial.flush();
      }
    }
    vTaskDelay(100);
  }
}
