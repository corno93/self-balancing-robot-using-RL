

#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"

SabertoothSimplified ST;
PID motor1(15,2,3);




int timer1_counter;
int timer3_counter;
int RPM_actual_m2 = 0;
boolean PID = true;
int ISR3_counter=0;

signed long encoder1count = 0;
signed long encoder2count = 0;

//PID VARIABLES:
int RPM_ref_m1;
int RPM_actual_m1;
int PID_count;
int integral, error_prev;



void setup() {
    Serial.begin(9600);      // Serial com for data output
    SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    initEncoders();       Serial.println("Encoders Initialized...");  
    clearEncoderCount();  Serial.println("Encoders Cleared...");
    pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup();
    interrupts();
    
}

void loop() {
  // put your main code here, to run repeatedly:
  int pid_cmd;
  Serial1.write(70);
  RPM_ref_m1 = 50;


  while(1)
  {
       Serial.print("The actual rpm is ");Serial.println(RPM_actual_m1);
       if (PID)
       {
        pid_cmd = motor1.updatePID(RPM_actual_m1, RPM_ref_m1);
        Serial1.write(pid_cmd);

       }
       

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
  //timer3_counter = 3036;   // 3036 gives 0.5Hz ints
  timer3_counter = 62411;   // 62411 gives 10Hz ints//34286 gives 1Hz ints
  
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}


  
ISR(TIMER3_OVF_vect)        // interrupt service routine at 100Hz
{
  TCNT3 = timer3_counter;   // preload timer

  encoder1count = readEncoder(1); 
 // encoder2count = readEncoder(2);

  ISR3_counter++;
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging freq


  if (ISR3_counter >=2)   //at 50Hz
  {    

    ISR3_counter = 0;
    RPM_actual_m1 = (encoder1count*300)/1920;
    encoder1count = 0;
    clearEncoderCount(); // Serial.println("Encoders Cleared...");
    PID_count++;
    if (PID_count >=2)
    {
      PID = true;
    }
      
  }

}



int PID::updatePID(int actual, int ref){
  int error, error_kp,error_ki, error_kd, pid_cmd_serial, pid_cmd, derivative;
  
    error = ref - actual;  //get RPM error
    
    error_kp = (kp*error)/10; //proportional

    integral+=error*0.2;  //integral
    error_ki = (ki*integral);

    derivative = (error - error_prev) /dt; 
    error_kd = kd*(derivative/10);

    error_prev = error;   //save next previous error

    //debugging...
    Serial.print("The error is ");Serial.println(error);
//    Serial.print("The error derivative is ");Serial.println(error_d);
    Serial.print("The integral is ");Serial.println(integral);

    pid_cmd = (error_kp + error_ki);// + error_kd;              //tuning tech at: http://robotsforroboticists.com/pid-control/
    Serial.print("The pid_cmd is ");Serial.println(pid_cmd);

  // Serial.print("motor1_rpmcmd ");Serial.println(motor1_rpmcmd);

    pid_cmd_serial = M1_rpm_to_serial(pid_cmd);
  //  Serial1.write(motor1_serialcmd);

//    Serial.print("pid_cmd_serial from PID: ");Serial.println(pid_cmd_serial);
   // Serial1.write(pid_cmd_serial);
           Serial.print("The pid_cmd_serial is ");Serial.println(pid_cmd_serial);

   return pid_cmd_serial;
}

