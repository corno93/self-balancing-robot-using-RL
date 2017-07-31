//~~~~~~~~~~~~~~~~~~~~~~ data from arduino to pi
//#include <Time.h>
//#include <TimeLib.h>
//
//void setup()
//{
//  Serial.begin(9600);
//  setTime(12,0,0,1,1,11);
//}
//
//void loop()
//{
//  Serial.print("Im sending a message");
//  Serial.print("hors");
//  printDigits(minute());
//  Serial.println();
//  delay(5000);
//}
//
//void printDigits(int digits)
//{
//  Serial.print(":");
//  if (digits < 10)
//  {
//    Serial.print("0");
//   
//  }
//  Serial.print(digits);
//}


//int n;
//void setup() {
//  // put your setup code here, to run once:
//  Serial.begin(9600);
//  
//
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//  if (Serial.available())
//  {
//    n = Serial.read();
//  }
//  if (n == 1)
//  {
//    Serial.print("hello Rpi");
//  }
//  else
//  {
//    Serial.print("smoke weed erryday");
//  }
//
//}
//~~~~~~~~~~~~~~~~~~~~~~ 


//~~~~~~~~~~~~~~~~~~~~~~ data from pi to arduino
//int n;
//const int ledPin = 13;
//void setup()
//{
//  pinMode(ledPin,OUTPUT);
//  Serial.begin(9600);
//  n = 1; 
//}
//
//void loop()
//{
//  if (Serial.available()){
//    n = Serial.read() - '0';
//  }
//  digitalWrite(ledPin,HIGH);
//  delay(n*100);
//  digitalWrite(ledPin,LOW);
//  delay(n*100);
//}


//~~~~~~~~~~~~~~~~~~~~~~  from pi to arduino to initiate motors ....WORKS

#define USBCON //uses Tx1 (see SabertoothSimplified.h)

#include <SabertoothSimplified.h>

char n = 64;
char cmd;
void setup()
{
   SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
   Serial.begin(9600);

}

void loop()
{
  if (Serial.available())
  {
       n = Serial.read();


       Serial1.write(n);  
  }
}



