

//#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
#define IN_BUFF_SIZE 8
#define OUT_BUFF_SIZE 8
//#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"
#include "WheelController.h"

#define M1pin 12
#define M2pin 11
#define STOP 126  //was 128 at some point...
#define BAUD_RATE 115200

char dataStringR1[OUT_BUFF_SIZE] = {0};
char dataStringR2[OUT_BUFF_SIZE] = {0};


//DT VARIBALES
unsigned long start = 0;
unsigned long end_ = 0;


int saturation(int cmd);

//OBJECT INSTANCES
WheelController wheelCtrl1(0x00003550,0x00001500,0x00000020);
WheelController wheelCtrl2(0x00003550,0x00001500,0x00000020);


//INTERRUPT VARIABLES
int timer3_counter;
int ISR3_counter=0;

//ENCODER VARIABLES
signed long encoder1count = 0;
signed long encoder2count = 0;

//PID VARIABLES:
boolean PID_flag = false;
int RPM_ref_m1 = 0;
int RPM_actual_m1;
int RPM_ref_m2 = 0;
int RPM_actual_m2;
int PID_count;

//COMS VARIBALES:
int counter = 0;
int databuff[IN_BUFF_SIZE];

void setup() {
    Serial.begin(BAUD_RATE);      // Serial com for data input
    Serial1.begin(BAUD_RATE);     // Serial com for data output
  //  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    initEncoders();       //Serial.println("Encoders Initialized...");  
    clearEncoderCount();  //Serial.println("Encoders Cleared...");
    pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup(); //encoders read and RPM calcs
 // timer4_interrupt_setup(); //increase rpm ref every 5 secs (debugging)

 // PWM SETTINGS (set timer 1 to 8 prescale to get a PWM with freq 3921.16)
  TCCR1B = TCCR1B & B11111000 | B00000001; 

  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);

    analogWrite(M1pin, STOP);  
    analogWrite(M2pin, STOP);


    
    interrupts();



}

void loop() {



    if (PID_flag)
     {
        PID_flag = false;
        analogWrite(M1pin, wheelCtrl1.tick(RPM_actual_m1, RPM_ref_m1));
        analogWrite(M2pin, wheelCtrl2.tick(-RPM_actual_m2, RPM_ref_m2));
        sprintf(dataStringR1,"R1%d",RPM_actual_m1); // convert a value to hexa 
        Serial1.println(dataStringR1);
        dataStringR1[OUT_BUFF_SIZE] = {0};
        sprintf(dataStringR2,"R2%d",RPM_actual_m2); // convert a value to hexa 
        Serial1.println(dataStringR2);
        dataStringR2[OUT_BUFF_SIZE] = {0};
     }


}




void timer3_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer3_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer3_counter = 3036;    // 3036 gives 0.5Hz ints at 256
  //timer3_counter = 59286;   //10hz ints at 256 prescale
  //timer3_counter = 40536;   //40536: 10hz ints at 64 prescale
  //timer3_counter = 45536;     //100hz at 8 prescale
    timer3_counter = 55536;   //200Hz at 8 prescale
  
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B &=~7; //clear
  TCCR3B |= (1 << CS11);    //8 prescaler
  //TCCR3B |= (1 << CS12);    // 256 prescaler 
 //TCCR3B |= (1 << CS11);    // 64 prescaler 
 //TCCR3B |= (1 << CS10);    // 64 prescaler 
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}


  
ISR(TIMER3_OVF_vect)        // interrupt service routine at 100Hz
{

  TCNT3 = timer3_counter;   // preload timer

  encoder1count = readEncoder(1); 
  encoder2count = readEncoder(2);

  ISR3_counter++;

  if (ISR3_counter >=2)   //at 100Hz
  {    

    ISR3_counter = 0;
    RPM_actual_m1 = (encoder1count*6000)/1920;
    RPM_actual_m2 = (encoder2count*6000)/1920;
//    sprintf(dataStringR1,"R1%d",RPM_actual_m1); // convert a value to hexa 
//    Serial.println(dataStringR1);
//    dataStringR1[OUT_BUFF_SIZE] = {0};
//    sprintf(dataStringR2,"R2%d",RPM_actual_m2); // convert a value to hexa 
//    Serial.println(dataStringR2);
//    dataStringR2[OUT_BUFF_SIZE] = {0};
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(); 
    PID_flag = true;

  }

}


void serialEvent() 
{
  while (Serial.available()) 
  {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging 
    // get the new byte:
    databuff[counter] = (int)Serial.read();
    counter++;
    if (counter == IN_BUFF_SIZE)
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
      
      // re-initalise PID for new reference command
      //wheelCtrl1.pid.init();
      //wheelCtrl2.pid.init();
      if (m1_sign == '-')
      {
        RPM_ref_m1 = -RPM_ref_m1;
      }
      if (m2_sign == '-')
      {
         RPM_ref_m2 = -RPM_ref_m2;
      }
      //Serial.println(RPM_ref_m1);      Serial.println(RPM_ref_m2);
      databuff[IN_BUFF_SIZE] = {0};
    }
  }
}
