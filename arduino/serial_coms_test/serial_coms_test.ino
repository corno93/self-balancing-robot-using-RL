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


/*
#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define M1pin 12
#define M2pin 11
#define ledPin 13

#include <SabertoothSimplified.h>

char n = 64;
char cmd;
void setup()
{
   SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    Serial.begin(9600);
    pinMode(ledPin, OUTPUT);
    pinMode(M1pin, OUTPUT);
    pinMode(M2pin, OUTPUT);


}

void loop()
{
 /* if (Serial.available())
  {
       n = Serial.read();
        digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging freq
        analogWrite(M1pin, n);  
        analogWrite(M2pin, n);
    //   Serial1.write(n);  
  }
}



void serialEvent() {
  while (Serial.available()) {
     digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging freq

  }
}


*/
#define ledPin 13


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int counter = 0;
int databuff[8];
int RPM_ref_m2, RPM_ref_m1;


void setup() {
      pinMode(ledPin, OUTPUT);

  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

}

void loop() {
  // print the string when a newline arrives:
  



}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() 
{
  while (Serial.available()) 
  {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging 
    // get the new byte:
    databuff[counter] = (int)Serial.read();;
    counter++;
    if (counter == 8)
    {
      counter = 0;
      int m1_sign = databuff[0]; 
      int m1_hund = databuff[1] - '0'; 
      int m1_tens = databuff[2] - '0'; 
      int m1_ones = databuff[3] - '0';
      int m2_sign = databuff[4]; 
      int m2_hund = databuff[5] - '0'; 
      int m2_tens = databuff[6] - '0'; 
      int m2_ones = databuff[7] - '0';
      RPM_ref_m1 = m1_hund*100 + m1_tens*10 + m1_ones;
      RPM_ref_m2 = m2_hund*100 + m2_tens*10 + m2_ones;
      if (m1_sign == '-')
      {
        RPM_ref_m1 = -RPM_ref_m1;
      }
      if (m2_sign == '-')
      {
         RPM_ref_m2 = -RPM_ref_m2;
      }
      Serial.println(RPM_ref_m1);      Serial.println(RPM_ref_m2);
      databuff[8] = {0};
    }
  }
}


