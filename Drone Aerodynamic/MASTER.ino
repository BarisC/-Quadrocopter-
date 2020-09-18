#include <SoftwareSerial.h>
#define joystickX A0
#define joystickY A1
#define potPin A2

SoftwareSerial BTserial(8,9);
int switchPin = 7;
int deger1,deger2,deger3,deger4;
int veri[4], startB,stopB;
byte data[12];
int i;

void baglan(){
  BTserial.write("AT+CON3CA30891A6DC"); delay(500);
}
void gonder(){

  
  
  deger1 = analogRead(A0);
  veri[0] = map(deger1,0,1023,1000,2000);
  Serial.print(">Gönderilen Deger1: "); Serial.println(veri[0],DEC);
  
  deger2 = analogRead(A1);
  veri[1] = map(deger2,0,1023,1000,2000);
  Serial.print(">Gönderilen Deger2: "); Serial.println(veri[1],DEC);
  
  deger3 = analogRead(A2);
  veri[2] = map(deger3,0,1023,1000,2000);
  Serial.print(">Gönderilen Deger3: "); Serial.println(veri[2],DEC);

  if(digitalRead(switchPin)){
    //deger4 = 1000;
    veri[3] = 2000; //map(deger4,0,1023,1000,2000);
    Serial.print(">Gönderilen Deger4: "); Serial.println(veri[3],DEC);
  }                                  
  else
  {
  //  deger4 = 0;
    veri[3] = 1000;//map(deger4,0,1023,1000,2000);
    Serial.print(">Gönderilen Deger4: "); Serial.println(veri[3],DEC);
  }
  
  Serial.println("------------------------------");
  
  data[0] = (startB & 0xff);
  data[1] = ((startB >> 8) & 0xff);
  
  data[2] = ((veri[0]) & 0xff);
  data[3] = ((veri[0] >> 8) & 0xff);
  
  data[4] = ((veri[1]) & 0xff);
  data[5] = ((veri[1] >> 8) & 0xff);
  
  data[6] = ((veri[2]) & 0xff);
  data[7] = ((veri[2] >> 8) & 0xff);
  
  data[8] = ((veri[3]) & 0xff);
  data[9] = ((veri[3] >> 8) & 0xff);
  
  data[10] = (stopB & 0xff);
  data[11] = ((stopB >> 8) & 0xff);
  
  BTserial.write(data, sizeof(data));
  i += 1;
}
void setup() {
  Serial.begin(38400);
  BTserial.begin(38400);
  baglan();
  pinMode(switchPin, INPUT);
  startB = 5000;
  stopB = 4500;
  i = 0;
}
void loop() {
    gonder();
}
