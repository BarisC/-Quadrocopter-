#include <SoftwareSerial.h>
SoftwareSerial BTserial(8,9);

int d1,d2,d3;
byte data1[2],data2[2],data3[2],startB[2],stopB[2];
int startB1,stopB1;

void veriAl(){

  if(BTserial.available() == 10){

    d1 = 0;
    d2 = 0;
    d3 = 0;

    while(BTserial.available()){

      BTserial.readBytes(startB,2);
      startB1 = (startB[1] << 8);
      startB1 = (startB1 | startB[0]);

      if(startB1 == 5000){

        BTserial.readBytes(data1,2);
        BTserial.readBytes(data2,2);
        BTserial.readBytes(data3,2);

        BTserial.readBytes(stopB,2);
        stopB1 = (stopB[1] << 8);
        stopB1 = (stopB1 | stopB[0]);

        if(stopB1 == 4500){

          d1 = (data1[1] << 8);
          d1 = (d1 | data1[0]);

          d2 = (data2[1] << 8);
          d2 = (d2 | data2[0]);

          d3 = (data3[1] << 8);
          d3 = (d3 | data3[0]);

          Serial.print("> Deger1: "); Serial.print(d1);
          Serial.print("> Deger2: "); Serial.print(d2);
          Serial.print("> Deger3: "); Serial.println(d3);
        }
        else{
          stopB1 = 0; stopB[0] = 0; stopB[1] = 0;
          break;
        }
      }
      else{
        startB1 = 0; startB[0] = 0; startB[1] = 0;
        break;
      }
    }    
  }

//  Serial.print("data1 0: "); Serial.println(data1[0]);
//  Serial.print("data1 1: "); Serial.println(data1[1]);
//  Serial.print("data2 0: "); Serial.println(data2[0]);
//  Serial.print("data2 1: "); Serial.println(data2[1]);
//  Serial.print("data3 0: "); Serial.println(data3[0]);
//  Serial.print("data3 1: "); Serial.println(data3[1]); 
}

void setup() {
  
  Serial.begin(9600);
  BTserial.begin(9600);
  startB1 = 0;
  stopB1 = 0;
}

void loop() {
  
  veriAl();
}
