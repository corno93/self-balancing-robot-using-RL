

//#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
//#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"
#include "WheelController.h"

#define M1pin 12
#define M2pin 11


//DT VARIBALES
unsigned long start = 0;
unsigned long end_ = 0;


int saturation(int cmd);

//OBJECT INSTANCES
//SabertoothSimplified ST;
// TODO: test derivative term for stability
// Can't really inspect derivative term without a bigger P
//PID motor1(0x0000A000,0x00000200,0x00000100);
WheelController wheelCtrl1(0x0000B000,0x00000200,0x00000100);
WheelController wheelCtrl2(0x0000A000,0x00000200,0x00000100);

//PID motor2(0,0,0);


//INTERRUPT VARIABLES
int timer4_counter;
int timer3_counter;
int ISR3_counter=0,ISR4_counter;

//ENCODER VARIABLES
signed long encoder1count = 0;
signed long encoder2count = 0;

//PID VARIABLES:
boolean PID_flag = true;
int RPM_ref_m1;
int RPM_actual_m1;
int RPM_ref_m2;
int RPM_actual_m2;
int PID_count;

//COMS VARIBALES:
boolean new_data = false;
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup() {
    Serial.begin(9600);      // Serial com for data output
  //  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    initEncoders();       //Serial.println("Encoders Initialized...");  
    clearEncoderCount();  //Serial.println("Encoders Cleared...");
    pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup(); //encoders read and RPM calcs
 // timer4_interrupt_setup(); //increase rpm ref every 5 secs (debugging)

 // PWM SETTINGS
   TCCR2B &=~7;    //clear
  TCCR2B |= 2;  //set to prescale of 8 (freq 4000Hz)
  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);
  
    interrupts();
  //  RPM_ref_m1 = 60;
  //  RPM_ref_m2 = -60;
    new_data = true;
    analogWrite(M1pin, 128);  
    analogWrite(M2pin, 128);
      inputString.reserve(200);


  /* char str[]  = "-123&-46";
   char* cmd = strtok(str,"&");
   int RPM_ref_m1 = atoi(cmd);
   cmd = strtok (NULL, "&");
   int RPM_ref_m2 = atoi(cmd);

  Serial.println(RPM_ref_m1);    Serial.println(RPM_ref_m2);    */
  



}

void loop() {
  fixed_point_t pid_output;
  unsigned char motor_cmd = 0;
  int time_elapsed= 0;
  int start, end_;
  int n;

     //Serial.print(RPM_actual_m1);Serial.print("-");Serial.println(RPM_actual_m2);

       if (PID_flag)
     {
        start = micros();
        PID_flag = false;

        analogWrite(M1pin, wheelCtrl1.tick(RPM_actual_m1, RPM_ref_m1));
        analogWrite(M2pin, wheelCtrl2.tick(-RPM_actual_m2, RPM_ref_m2));

        // time elapsed debug
        end_ = micros() - start;
        //Serial.println(end_);
        
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
  timer3_counter = 40536;   //40536: 10hz ints at 64 prescale
  //timer3_counter = 45536;     //100hz at 8 prescale
  
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

  if (ISR3_counter >=2)   //at 50Hz
  {    

    ISR3_counter = 0;
    RPM_actual_m1 = (encoder1count*3000)/1920;
    RPM_actual_m2 = (encoder2count*3000)/1920;
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(); 
    PID_flag = true;

  }

}


void serialEvent()
{
  String data;
  int inData[2];

  while (Serial.available())
  {
        digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging freq  

    for (int n = 0; n < 2;n++)
    {
      inData[n] = Serial.read();
    }

  //  data = Serial.read();
  RPM_ref_m1 = inData[0];
    RPM_ref_m2 = inData[1];

  

    

    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:

  }
}
   
