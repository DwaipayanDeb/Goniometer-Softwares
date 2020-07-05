#include <Wire.h>
#include "LiquidCrystal.h"
#include "RTClib.h"

#define TSL_FREQ_PIN 2 // output use digital pin2 for interrupt
float fD = 0.1; // Dark frequency
float Re = 1; // Irradiance responsivity (Here it has been set to 1 because in final calculation of relative flux this will be canelled out)
float Ee=0; // Irradiance
int timing = 1000; // in milliseconds
volatile unsigned long pulse_cnt = 0;
RTC_DS1307 RTC; // define the Real Time Clock object

LiquidCrystal lcd(10,9,8,7,6,5);

int laserpin=12, AD0pin=A3, intensity=0; //intensity is laser intensity, 
char flux[10]; //flux is detector value
void setup() {
  RTC.begin();  // Start Real Time Clock
  pinMode(TSL_FREQ_PIN, INPUT);
  pinMode(13,OUTPUT);  //LED pin
  Wire.begin(1);                // 'This device' here joins i2c bus with address #1
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Serial.begin(9600);   // start serial for output
  lcd.begin(16,2);
  pinMode(AD0pin, OUTPUT);
  digitalWrite(AD0pin,LOW); // MPU6050 I2C address 0x68 for LOW and 0x69 for HIGH
  pinMode(laserpin, OUTPUT);
 
}

void loop() {
  analogWrite(laserpin,intensity);
 
  pulse_cnt=0;
  uint32_t millis1 = millis();
  DateTime time1 = RTC.now();
  
  attachInterrupt(0, add_pulse, RISING);
  delay(timing);
  detachInterrupt(0);

  uint32_t millis2 = millis();
  DateTime time2 = RTC.now();
  unsigned long finalcnt=pulse_cnt;  
  dtostrf(finalcnt/(timing/1000.0), 6, 0, flux);

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {

  
       lcd.clear();     // here first line is being displayed
       lcd.setCursor(0,0);
       lcd.write("L=");
       lcd.print(intensity);
       lcd.write(" F=");
       lcd.print(flux);
       lcd.setCursor(0,1);
//   Serial.println(Wire.available());


 int i=1;
  while (0< Wire.available()) { 

 if(i<=16){               // reading first 16 bytes
    char c = Wire.read(); // receive byte as a character
    lcd.write(c);
 //   Serial.print(c);         // print the character
  }
else{                      // reading last byte of total 17 bytes
      intensity = Wire.read(); // receive byte as a character

//    lcd.write(c);
 //   Serial.print(intensity);         // print the character
}
i++;
  }

}
void requestEvent() {
  digitalWrite(13,HIGH);
  delay(500);             //LED flash indicating data sent
  digitalWrite(13,LOW);
  
  Wire.write(flux); // respond with message of 6 bytes
  // as expected by master
}

void add_pulse() {
  // increase pulse count
  pulse_cnt++;
  //Serial.println(pulse_cnt);
  return;
}