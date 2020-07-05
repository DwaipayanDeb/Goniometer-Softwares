#include <Wire.h>  // Library header for I2C communication
#include <I2Cdev.h>// Library header for MPU6050 communication
#include "MPU6050.h"// Library header for MPU6050 communication
MPU6050 accelgyro1(0x68);// FOR GY-521 WITH AD0 PIN AT 'LOW' STATE (HERE INCINENCE ANGLE MEASURERER)
MPU6050 accelgyro2(0x69);// FOR GY-521 WITH AD0 PIN AT 'HIGH' STATE(HERE EMERGENCE ANGLE MEASURERER)
int16_t ax1, ay1, az1, gx1, gy1, gz1;
double arx1, ary1, arz1,tmp1;
double sumary1=0.0, result1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;
double arx2, ary2, arz2,tmp2;
double sumary2=0.0, result2;
char anglei[10],anglee[10],laserval1[10],laserval2[10], temperature1[10], temperature2[10]; 
String serialin;
String inString ="";

int i,j=1,intensity1=0,intensity2=10,sendDelay=0,sysDelay=1000,milliNow=0,M=0,N=0,P=0,R=0,L=0,K=0,Cint,Dint;
double acclSampleSize=50.0;

    const int stepPin1 = A1; // Stepper motor
    const int dirPin1 = A0; // Stepper motor
    const int stepPin2 = A3; // Stepper motor
    const int dirPin2 = A2; // Stepper motor
    const int resPin1=3; // Stepper motor
    const int resPin2=2; // Stepper motor

    const int resetPin=12; // Reset Arduino
void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  accelgyro1.initialize();
  accelgyro2.initialize();

  pinMode(stepPin1,OUTPUT); 
  pinMode(dirPin1,OUTPUT);
  pinMode(resPin1,OUTPUT);
  pinMode(stepPin2,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  pinMode(resPin2,OUTPUT);
  pinMode(resetPin,OUTPUT);
}
byte x = 0;

void loop() {
 milliNow=millis();

//Here this device will read values send from computer software through serial port.

  if (Serial.available()>0){  
    M=0;
    N=0;
    P=0;  
    sysDelay=00;

   serialin=Serial.readString();
   if (serialin.substring(0,1)=="L") {   // LASER ON TIME IN ms
  L=serialin.substring(1,2).toInt();
  if (L==0) {
    intensity1=0;
    sendDelay=0;
  }
  else if(L==1){
     intensity1=255;
     sendDelay=0;
  }
  else if (L==2){
    sendDelay=5000;
  }
  }
    if (serialin.substring(2,3)=="R") {
  R=serialin.substring(3,4).toInt();
  }
 if (serialin.substring(4,5)=="M") {
  M=round(1.667*serialin.substring(5,8).toInt()); 
  N=0;
     stepper1(1,R,M);  //(res,dir,step)
  }
 else if (serialin.substring(4,5)=="N"){
  N=serialin.substring(5,8).toInt();
  M=0;
    stepper2(1,R,N);
  }
   else if (serialin.substring(4,5)=="P"){
  P=serialin.substring(5,8).toInt();
  N=0;
  M=0;
    stepper1(1,0,round(1.667*P)); 
    stepper2(1,1,P);
  }

 }
 
 else { 
 //Here collects acceleromter & gyro readings
  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  accelgyro2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  tmp1=((accelgyro1.getTemperature()/340.0)+36.53); // tmp is the temperature of the accelerometer sensor on MPU6050. 
  tmp2=((accelgyro2.getTemperature()/340.0)+36.53); // tmp is the temperature of the accelerometer sensor on MPU6050. 
  // calculate accelerometer angles
  arx1 = (180/3.141592) * atan(ax1 / sqrt(square(ay1) + square(az1))); 
  ary1 = (180/3.141592) * atan(ay1 / sqrt(square(ax1) + square(az1)));
  arz1 = (180/3.141592) * atan(sqrt(square(ay1) + square(ax1)) / az1);

  arx2 = (180/3.141592) * atan(ax2 / sqrt(square(ay2) + square(az2))); 
  ary2 = (180/3.141592) * atan(ay2 / sqrt(square(ax2) + square(az2)));
  arz2 = (180/3.141592) * atan(sqrt(square(ay2) + square(ax2)) / az2);

   if (j<=acclSampleSize){     //Averages 500 readings for Y axis angles 
   sumary1=sumary1+ary1;
   sumary2=sumary2+ary2;
   j=j+1;
   }
   else{
   result1=sumary1/acclSampleSize;
   result2=sumary2/acclSampleSize;
   sumary1=0.0;
   sumary2=0.0;
   j=1;
   delay(sendDelay);
   delay(sysDelay);
   ary1=(ary1*(-1)+1.5+.006)/1.04;   // Here -1 is multiplied for direction convension. The term 1.5 is due to offset value during calibration. Terms 0.006 and 1.04 are due to calibration constants obtained experimentally. 
   ary2=(ary2*(-1)-1.1+0.33)/0.95;  //Here -1 is multiplied for direction convension. The term -1.1 is due to offset value during calibration. Terms 0.33 and 0.95 are due to calibration constants obtained experimentally. 
   Serial.print("I");
   Serial.println(ary1);
   Serial.print("E");
   Serial.println(ary2);
   Serial.print("C");
    Wire.requestFrom(1, 6);    // request 6 bytes from slave device #1
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
 //   inString += c;
  }
 // Cint=inString.toInt();
 // inString="";
  Serial.println();
 Serial.print("D");
      Wire.requestFrom(2, 6);    // request 6 bytes from slave device #2
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
   Serial.print(c);         // print the character
 //   inString += c;
  }
  //Dint=inString.toInt();  
   Serial.println();
//   Serial.print(Cint/Dint);
//   inString="";
   dtostrf(ary1, 5, 2, anglei); //convers the float or integer to a string. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array)
   dtostrf(tmp1, 2, 0, temperature1);// same "
   dtostrf(ary2, 5, 2, anglee); //convers the float or integer to a string. (floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, empty array)
   dtostrf(tmp2, 2, 0, temperature2);// same "
    
  Wire.beginTransmission(1); // transmit to Arduino NANO #1 for display on LCD
   Wire.write("i=");        // sends two bytes one by one
    Wire.write(anglei);  // adding five bytes one by one to above
    Wire.write ("deg ");  // adding four bytes one by one to above
    Wire.write ("T=");// adding two bytes one by one to above
    Wire.write (temperature1);// adding two bytes one by one to above
    Wire.write ("C");// adding one byte to above
    Wire.write(intensity1);// adding one integer byte(0 to 255) to above
  Wire.endTransmission();    // stop transmitting
  Wire.beginTransmission(2); // transmit to Arduino NANO #2 for display on LCD
   Wire.write("e=");        // sends two bytes one by one
    Wire.write(anglee);  // adding five bytes one by one to above
    Wire.write ("deg ");  // adding four bytes one by one to above
    Wire.write ("T=");// adding two bytes one by one to above
    Wire.write (temperature2);// adding two bytes one by one to above
    Wire.write ("C");// adding one byte to above
    Wire.write(intensity2);// adding one integer byte(0 to 255) to above
  Wire.endTransmission();    // stop transmitting
  
 //Wire.beginTransmission(2); // transmit to device #2
  // Wire.write(65);        // sends five bytes
 // Wire.endTransmission();    // stop transmitting
   }
 }
  //delay(500);
}
void stepper1(int res, int dir, int steps){
        digitalWrite(resPin1,res); // Resolution 1/4 step for HIGH
        digitalWrite(dirPin1,dir); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin1,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin1,LOW); 
        delayMicroseconds(500); 
       delay(10); // Slowing rotation
      }
 
}
void stepper2(int res, int dir, int steps){
        digitalWrite(resPin2,res); // Resolution 1/4 step for HIGH
        digitalWrite(dirPin2,dir); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
      for(int x = 0; x < steps; x++) {
        digitalWrite(stepPin2,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin2,LOW); 
        delayMicroseconds(500);
        delay(10); // Slowing rotation
      }   
}

void Reset(){
 
}