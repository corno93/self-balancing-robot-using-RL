

#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"

SabertoothSimplified ST;
PID motor1(5,5,3);
PID motor2(5,5,3);



int timer4_counter;
int timer3_counter;
boolean PID_flag = false;
int ISR3_counter=0,ISR4_counter;

signed long encoder1count = 0;
signed long encoder2count = 0;

//PID VARIABLES:
int RPM_ref_m1;
int RPM_actual_m1;
int RPM_ref_m2;
int RPM_actual_m2;
int PID_count;
int integral, error_prev;


//COMS VARIBALES:
boolean new_data = false;


void setup() {
    Serial.begin(9600);      // Serial com for data output
    SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    initEncoders();       Serial.println("Encoders Initialized...");  
    clearEncoderCount();  Serial.println("Encoders Cleared...");
    pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup();
 //   timer4_interrupt_setup();
    interrupts();
    RPM_ref_m1 = 90;
    RPM_ref_m2 = 90;
    new_data = true;

    
}

void loop() {
  // put your main code here, to run repeatedly:
  int serial_cmd;

  /*  if(new_data)
    {
      serial_cmd = motor1.M1_rpm_to_serial(RPM_ref_m1);
      Serial1.write(serial_cmd);
      new_data = false;
      serial_cmd = motor2.M2_rpm_to_serial(RPM_ref_m2);
      Serial1.write(serial_cmd);
    }*/
     Serial.print(RPM_actual_m1);Serial.print("-");Serial.println(RPM_actual_m2);


     if (PID_flag)
     {
        serial_cmd = motor1.updatePID(RPM_actual_m1, RPM_ref_m1,1);
        Serial1.write(serial_cmd);
        serial_cmd = motor2.updatePID(RPM_actual_m2, RPM_ref_m2,2);
        Serial1.write(serial_cmd);
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
  //timer3_counter = 59286;//10hz ints at 256 prescale
  timer3_counter = 40536;//40536: 10hz ints at 64 prescale
  
  TCNT3 = timer3_counter;   // preload timer
 // TCCR3B |= (1 << CS12);    // 256 prescaler 
 TCCR3B |= (1 << CS11);    // 64 prescaler 
 TCCR3B |= (1 << CS10);    // 64 prescaler 
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}


  
ISR(TIMER3_OVF_vect)        // interrupt service routine at 100Hz
{
  TCNT3 = timer3_counter;   // preload timer

  encoder1count = readEncoder(1); 
  encoder2count = readEncoder(2);

  ISR3_counter++;


  if (ISR3_counter >=2)   //at 5Hz
  {    
 // digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging freq

    ISR3_counter = 0;
    RPM_actual_m1 = (encoder1count*300)/1920;
    RPM_actual_m2 = (encoder2count*300)/1920;
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(); // Serial.println("Encoders Cleared...");
    PID_count++;
    if (PID_count >=2)
    {
      PID_flag = true;
      PID_count = 0;
    }
      
  }

}



int PID::updatePID(int actual, int ref, char motor){
  int error, error_kp,error_ki, error_kd, pid_cmd_serial, pid_cmd, derivative;
   //  delay(100); 

    error = ref - actual;  //get RPM error
    
    error_kp = (kp*error)/10; //proportional

    integral+=error*0.2;  //integral
    error_ki = (ki*integral)/10;

    derivative = (error - error_prev) / 0.2; 
    error_kd = (kd*derivative)/10;
    error_prev = error;   //save next previous error

    //debugging...
  //  Serial.print("The error is ");Serial.println(error);
  //  Serial.print("The kp error is ");Serial.println(error_kp);

   // Serial.print("The kp error is ");Serial.println(error_kp);

    //Serial.print("The error derivative is ");Serial.println(error_kd);
    //Serial.print("The integral is ");Serial.println(integral);

    pid_cmd = error_kp + error_ki;// + error_kd;              //tuning tech at: http://robotsforroboticists.com/pid-control/
 //   Serial.print("The pid_cmd is ");Serial.println(pid_cmd);

  // Serial.print("motor1_rpmcmd ");Serial.println(motor1_rpmcmd);

    if (motor == 1)
    {
      pid_cmd_serial = M1_rpm_to_serial(pid_cmd);
    }
    else if (motor == 2)
    {
      pid_cmd_serial = M2_rpm_to_serial(pid_cmd);

    }
  //  Serial1.write(motor1_serialcmd);

//    Serial.print("pid_cmd_serial from PID: ");Serial.println(pid_cmd_serial);
   // Serial1.write(pid_cmd_serial);
   //        Serial.print("The pid_cmd_serial is ");Serial.println(pid_cmd_serial);
   PID_flag = false;
   return pid_cmd_serial;
}

//INT 4 IS TIMING AND RPM_REF CHANGES FOR PLOTS.

void timer4_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer4_counter = 3036;   // preload timer 65536-16MHz/256/100Hz
//  timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
//  timer1_counter = 58000;//3037;//3037;//3036;//59286;//3036;//58500;//3036;//62411;//3036;//59286;     // preload timer 65536-16MHz/256/10Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  
  TCNT4 = timer4_counter;   // preload timer
  TCCR4B |= (1 << CS12);    // 256 prescaler 
  TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt
  
}


ISR(TIMER4_OVF_vect)        // interrupt service routine 
{
    // preload timer
      TCNT4 = timer4_counter; 
      ISR4_counter++;

      if (ISR4_counter >=5)
      {
        ISR4_counter = 0;
        RPM_ref_m1 = RPM_ref_m1 + 10;
        Serial.println("RPM inc");
        integral = 0;
      }
      if (RPM_ref_m1 >= 200)
      {
        Serial.println("RPM at 200 stop");
      }
      

  
}
