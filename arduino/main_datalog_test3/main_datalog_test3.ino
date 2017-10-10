

//#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
#define IN_BUFF_SIZE 8
//#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"
#include "WheelController.h"

#define M1pin 12
#define M2pin 11
#define STOP 126
#define CMD_FREQ 1

//DT VARIBALES
unsigned long start = 0;
unsigned long end_ = 0;


//OBJECT INSTANCES
WheelController wheelCtrl1(0x00003500,0x00000500,0x00000010);
WheelController wheelCtrl2(0x00003550,0x00001500,0x00000020);

//PID motor2(0,0,0);


//INTERRUPT VARIABLES
int timer4_counter;
int timer3_counter;
int ISR3_counter=0,ISR4_counter;

//ENCODER VARIABLES
signed long encoder1count = 0;
signed long encoder2count = 0;

//PID VARIABLES:
boolean PID_flag = false;
int RPM_ref_m1;
int RPM_actual_m1;
int RPM_ref_m2;
int RPM_actual_m2;
int PID_count;

//COMS VARIBALES:
boolean new_data = false;
String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int counter = 0;
int databuff[IN_BUFF_SIZE];


//DATA LOGGING VARIBALES
int data_cntr;
int *data_ptr_M1;
int *data_ptr_M2;
#define DATA_LOG_BUFF 600
int ISR4_cntr = 0;



void setup() {
    Serial.begin(9600);      // Serial com for data output
  //  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    initEncoders();       //Serial.println("Encoders Initialized...");  
    clearEncoderCount();  //Serial.println("Encoders Cleared...");
    pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup(); //encoders read and RPM calcs
    timer4_interrupt_setup(); 

 // PWM SETTINGS
//   TCCR2B &=~7;    //clear
//  TCCR2B |= 2;  //set to prescale of 8 (freq 4000Hz)
  TCCR1B = TCCR1B & B11111000 | B00000001; 



  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);
  
    RPM_ref_m1 = 0;
    RPM_ref_m2 = 0;
    analogWrite(M1pin, STOP);  
    analogWrite(M2pin, STOP);

    // 5 second delay to let motors come to complete rest
   // delay(50000);
    interrupts();

          data_ptr_M1 = (int *) malloc(DATA_LOG_BUFF);

    inputString.reserve(200);

    // set malloc
   // data_ptr_M1 = (int *) malloc(DATA_LOG_BUFF);
  //  data_ptr_M2 = (int *) malloc(DATA_LOG_BUFF);
    data_cntr = 0;
    *(data_ptr_M1+data_cntr) = 9990;
  //  *(data_ptr_M2+data_cntr) = 9990;
    data_cntr++;


}

void loop() {
  fixed_point_t pid_output;
  unsigned char motor_cmd = 0;
  int time_elapsed= 0;
  int start, end_;
  int n;


       if (PID_flag)
     {
      //  start = micros();
        PID_flag = false;

        analogWrite(M1pin, wheelCtrl1.tick(RPM_actual_m1, RPM_ref_m1));
        analogWrite(M2pin, wheelCtrl2.tick(-RPM_actual_m2, RPM_ref_m2));

        // time elapsed debug
    //    end_ = micros() - start;
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
  //timer3_counter = 40536;   //40536: 10hz ints at 64 prescale
  timer3_counter = 45536;     //100hz at 8 prescale
  // timer3_counter = 55536;     //200Hz at 8 prescale
 //  timer3_counter = 25536;     //400Hz at 1 prescale

  TCNT3 = timer3_counter;   // preload timer
  TCCR3B &=~7; //clear
  TCCR3B |= (1 << CS11);    //8 prescaler
  //TCCR3B |= (1 << CS12);    // 256 prescaler 
 //TCCR3B |= (1 << CS11);    // 64 prescaler 
 //TCCR3B |= (1 << CS10);    // 64 prescaler 
 //  TCCR3B |= (1 << CS10);    //1 prescaler

  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}


  
ISR(TIMER3_OVF_vect)        // main interrupt service routine 
{

  TCNT3 = timer3_counter;   // preload timer


  encoder1count = readEncoder(1); 
  encoder2count = readEncoder(2);

  ISR3_counter++;

  if (ISR3_counter >=2)   
  {    

    ISR3_counter = 0;
    RPM_actual_m1 = (encoder1count*3000)/1920;
    RPM_actual_m2 = (encoder2count*3000)/1920;
    
    *(data_ptr_M1 + data_cntr) = RPM_actual_m1;
  //  *(data_ptr_M2 + data_cntr) = RPM_actual_m2;
    data_cntr++;

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
    //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging 
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
      wheelCtrl1.pid.init();
      wheelCtrl2.pid.init();
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

// TESTER INTERRUPT
void timer4_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer3_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer3_counter = 3036;    // 3036 gives 0.5Hz ints at 256
  //timer3_counter = 59286;   //10hz ints at 256 prescale
  timer4_counter = 49911;   //49911: 4hz ints at 256 prescale
  //timer3_counter = 45536;     //100hz at 8 prescale
  
  TCNT4 = timer4_counter;   // preload timer
  TCCR4B &=~7; //clear
 // TCCR4B |= (1 << CS11);    //8 prescaler
  TCCR4B |= (1 << CS12);    // 256 prescaler 
 //TCCR3B |= (1 << CS11);    // 64 prescaler 
 //TCCR3B |= (1 << CS10);    // 64 prescaler 
  TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt
}


  
ISR(TIMER4_OVF_vect)        // interrupt service routine at 4Hz
{

  TCNT4 = timer4_counter;   // preload timer
  //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging 
  ISR4_cntr++;

  if(ISR4_cntr == CMD_FREQ*1)
  {
    *(data_ptr_M1+data_cntr) = 9991;
 //   *(data_ptr_M2+data_cntr) = 9991;
    data_cntr++;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    RPM_ref_m1 = 0;
    RPM_ref_m2 = 0;
  }else if (ISR4_cntr == CMD_FREQ*2)
  {
    *(data_ptr_M1+data_cntr) = 9992;
  //  *(data_ptr_M2+data_cntr) = 9992;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = 30;
    RPM_ref_m2 = 30;
  }else if (ISR4_cntr == CMD_FREQ*3)
  {
    *(data_ptr_M1+data_cntr) = 9993;
   // *(data_ptr_M2+data_cntr) = 9993;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = -10;  //100 rpm doesnt want to work...
    RPM_ref_m2 = -10;
  }else if (ISR4_cntr == CMD_FREQ*4)
  {
    *(data_ptr_M1+data_cntr) = 9994;
  //  *(data_ptr_M2+data_cntr) = 9994;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
  //  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging 

    RPM_ref_m1 = 15;
    RPM_ref_m2 = 15;
  }else if(ISR4_cntr == CMD_FREQ*5)
  {
    *(data_ptr_M1+data_cntr) = 9995;
  //  *(data_ptr_M2+data_cntr) = 9995;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = -45;
    RPM_ref_m2 = -45;
  }else if(ISR4_cntr == CMD_FREQ*6)
  {
    *(data_ptr_M1+data_cntr) = 9996;
  //  *(data_ptr_M2+data_cntr) = 9996;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = 10;
    RPM_ref_m2 = 10;
  }else if(ISR4_cntr == CMD_FREQ*7)
  {
    *(data_ptr_M1+data_cntr) = 9997;
  //  *(data_ptr_M2+data_cntr) = 9997;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = 60;
    RPM_ref_m2 = 60;
  }else if(ISR4_cntr == CMD_FREQ*8)
  {
    *(data_ptr_M1+data_cntr) = 9998;
  //  *(data_ptr_M2+data_cntr) = 9998;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = 0;
    RPM_ref_m2 = 0;
    }else if(ISR4_cntr == CMD_FREQ*9)
  {
    *(data_ptr_M1+data_cntr) = 9910;
  //  *(data_ptr_M2+data_cntr) = 9998;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();
    data_cntr++;
    RPM_ref_m1 = -40;
    RPM_ref_m2 = -40;
    }else if(ISR4_cntr == CMD_FREQ*10)
  {
    *(data_ptr_M1+data_cntr) = 9900;
  //  *(data_ptr_M2+data_cntr) = 9900;
    RPM_ref_m1 = 0;
    RPM_ref_m2 = 0;
    wheelCtrl1.pid.init();
    wheelCtrl2.pid.init();

     //analogWrite(M1pin, STOP);
     //analogWrite(M2pin, STOP);
  
    // free array
    //print arrays to serial
    Serial.println("RPM for Motor 1: ");
    for (int i = 0; i < DATA_LOG_BUFF; i++)
    {
      Serial.println(*(data_ptr_M1 + i));
    }
    
//    Serial.println("RPM for Motor 2: ");
//    for (int i = 0; i < DATA_LOG_BUFF; i++)
//    {
//      Serial.println(*(data_ptr_M2 + i));
//    }
    Serial.println("Finished");

    free(data_ptr_M1);
   // free(data_ptr_M2);

   // noInterrupts();           // disable all interrupts
  }

}

   
