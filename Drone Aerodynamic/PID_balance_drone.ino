#include <Wire.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <SoftwareSerial.h>
SoftwareSerial BTserial(8,9);
Servo right_prop;
Servo left_prop;
Servo back_left;
Servo back_right;

SimpleKalmanFilter simpleKalmanFilter(250, 50, 0.01);
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

SimpleKalmanFilter simpleKalmanFilter2(250, 50, 0.01);
const long SERIAL_REFRESH_TIME2 = 100;
long refresh_time2;


int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float PID2, pwmFront, pwmBack, errorY, previous_errorY;
float pid_pY=0;
float pid_iY=0;
float pid_dY=0;
/////////////////PID Y CONSTANTS////////////////////////////////////////////////////////
double kpY=1;//5;
double kiY=0.002;//0.003
double kdY=2.5;
////////////////////////////////////////////////////////////////////////////////////////


float PID, pwmleftFront, pwmrightFront, pwmleftBack, pwmrightBack, errorX, previous_errorX;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID X CONSTANTS////////////////////////////////////////////////////////
double kp=2;//5;
double ki=0.001;//0.003
double kd=2;
////////////////////////////////////////////////////////////////////////////////////////

float desired_angle = 0;

int thro;

int dongu;
int dizi[3];
int deger1,deger2,deger3;
int veri[4], hiz = 0;
int d1,d2,d3,d4;
byte data1[2],data2[2],data3[2],data4[2],startB[2],stopB[2];
int startB1,stopB1;
byte data[10];
int sayac;

void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(38400);
  BTserial.begin(38400);
  startB1 = 0;
  stopB1 = 0;
  sayac = 0;
  right_prop.attach(5);
  left_prop.attach(4);
  back_left.attach(3);
  back_right.attach(2);

  time = millis();
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  back_right.writeMicroseconds(1000);
  back_left.writeMicroseconds(1000);
  delay(5000); /*Give some delay, 7s, to have time to connect
//                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {

    thro = analogRead(A2);
    int thro2 = map(thro, 100, 1023, 1000, 2000);
    //Serial.println(thro2);
/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // Önceki zamanı saklıyoruz.
    time = millis();  // Şimdiki zaman.
    elapsedTime = (time - timePrev) / 1000; //Zaman farkı. Ms cinsinden kullandığımız için 1000'e bölüyoruz.

     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 

     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     Wire.beginTransmission(0x68);
     Wire.write(0x43); //Gyro data first adress
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,4,true); //Just 4 registers
   
     Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
     Gyr_rawY=Wire.read()<<8|Wire.read();
     Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
     Gyro_angle[1] = Gyr_rawY/131.0;
     Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle--- */
     Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

    
    errorY = Total_angle[1] - desired_angle;
    if(errorY < 1 && errorY > -1)
      errorY = 0;
    pid_pY = kpY*errorY;
    
    if(-3 < errorY < 3)
     {
      pid_iY = pid_iY+(kiY*errorY);
     }
     
    pid_dY = kdY*((errorY - previous_errorY)/elapsedTime);
    PID2 = pid_pY + pid_iY + pid_dY;

    
    //Serial.print(pid_pY); Serial.print("   "); Serial.print(pid_iY); Serial.print("   "); Serial.println(pid_dY); //Serial.print("              ");
 
    
    errorX = Total_angle[0] - (desired_angle - 2);
    if(errorX < 1 && errorX > -1)
      errorX = 0;
    pid_p = kp*errorX; //P değeri bir sabittir.
    if(-3 <errorX <3)
     {
      pid_i = pid_i+(ki*errorX); 
     }
    pid_d = kd*((errorX - previous_errorX)/elapsedTime);
    PID = pid_p + pid_d;



   int pidKalman = simpleKalmanFilter.updateEstimate(PID); //Kalman filtresi sonucu.
    if (millis() > refresh_time) 
    {
      refresh_time = millis() + SERIAL_REFRESH_TIME;
    }


   int pidKalman2 = simpleKalmanFilter2.updateEstimate(PID2); //Kalman filtresi sonucu.
    if (millis() > refresh_time2) 
    {
      refresh_time2 = millis() + SERIAL_REFRESH_TIME2;
    }    




   //Serial.print(Total_angle[1]); Serial.print("   "); Serial.println(Total_angle[0]);
   PID2 = pidKalman2;
   PID = pidKalman;
   if(PID < -1000)
   {
     PID=-1000;
   }
   if(PID > 1000)
   {
     PID=1000;
   }

   if(PID2 < -1000)
   {
     PID2=-1000;
   }
   if(PID2 > 1000)
   {
     PID2=1000;
   }


/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value */
   pwmleftBack = thro2 + PID - PID2;
   pwmrightBack = thro2 - PID - PID2;
   pwmleftFront = thro2 + PID + PID2;
   pwmrightFront= thro2 - PID + PID2;

  //Serial.print(" Y AÇISI: "); Serial.print(Total_angle[1]); Serial.print("  ERRORY: "); Serial.print(errorY); Serial.print(" PIDY: "); 
    Serial.print(PID2);
  //Serial.print("              X AÇISI: "); Serial.print(Total_angle[0]); Serial.print("  ERRORX: "); Serial.print(errorX); Serial.print(" PIDX: "); 
    Serial.print(PID);
   
  
//Right
   if(pwmrightBack < 1000)
   {
     pwmrightBack= 1000;
   }
   if(pwmrightBack > 2000)
   {
     pwmrightBack=2000;
   }
   //Left
   if(pwmleftBack < 1000)
   {
     pwmleftBack= 1000;
   }
   if(pwmleftBack > 2000)
   {
     pwmleftBack=2000;
   }

   if(pwmrightFront < 1000)
   {
     pwmrightFront= 1000;
   }
   if(pwmrightFront > 2000)
   {
     pwmrightFront=2000;
   }
   //Left
   if(pwmleftFront < 1000)
   {
     pwmleftFront= 1000;
   }
   if(pwmleftFront > 2000)
   {
     pwmleftFront=2000;
   }

   Serial.print(pwmleftBack); Serial.print("        "); Serial.print(pwmrightBack); //Serial.print("    PID: "); Serial.print(PID); Serial.print("     PID2: "); Serial.print(PID2);
   Serial.print(" X : "); Serial.print(errorX); Serial.print(" Y : "); Serial.println(errorY);
/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse */
  left_prop.writeMicroseconds(pwmleftFront);
  right_prop.writeMicroseconds(pwmrightFront);
  back_left.writeMicroseconds(pwmleftBack);
  back_right.writeMicroseconds(pwmrightBack);
  previous_errorX = errorX; //Önceki hata değerini kaydediyoruz.
  previous_errorY = errorY;

  //Serial.print(pwmleftFront); Serial.print("   "); Serial.print(pwmrightFront);  Serial.print("   "); Serial.print(pwmleftBack); Serial.print("   "); Serial.println(pwmrightBack);
 
}//end of loop void

int veriAlhiz()
{

  if(BTserial.available() >= 12){
    d1 = 0;
    d2 = 0;
    d3 = 0;
    d4 = 0;
    while(BTserial.available())
    {
      BTserial.readBytes(startB,2);
      startB1 = (startB[1] << 8);
      startB1 = (startB1 | startB[0]);
      if(startB1 == 5000)
      {
        BTserial.readBytes(data1,2);
        BTserial.readBytes(data2,2);
        BTserial.readBytes(data3,2);
        BTserial.readBytes(data4,2);
        
        BTserial.readBytes(stopB,2);
        stopB1 = (stopB[1] << 8);
        stopB1 = (stopB1 | stopB[0]);
        
        if(stopB1 == 4500)
        {
          d1 = (data1[1] << 8);
          d1 = (d1 | data1[0]);
 
          d2 = (data2[1] << 8);
          d2 = (d2 | data2[0]);

          d3 = (data3[1] << 8);
          d3 = (d3 | data3[0]);

          d4 = (data4[1] << 8);
          d4 = (d4 | data4[0]);
          return d3;
          dizi[0] = d1;
          dizi[1] = d2;
          dizi[2] = d3;
          //Serial.print("Dizi -> "); Serial.print(dizi[2]); Serial.print("    ");
        }
        else
        {
          stopB1 = 0; stopB[0] = 0; stopB[1] = 0;
          return 0; 
        }
        }
      else 
      {
        startB1 = 0; startB[0] = 0; startB[1] = 0;
        return 0;
      }
    }    
  
  
  
 }
 //else
 //{
   //     d1=dizi[0];
     //   d2=dizi[1];
     //   d3=dizi[2];
 // }
 




  
}
