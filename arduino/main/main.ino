

#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
#define M1pin 12
#define M2pin 11
#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"

//OBJECT INSTANCES
SabertoothSimplified ST;
PID motor1(5,4,0);
PID motor2(0,0,0);


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


void setup() {
    Serial.begin(9600);      // Serial com for data output
    SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
    initEncoders();       Serial.println("Encoders Initialized...");  
    clearEncoderCount();  Serial.println("Encoders Cleared...");
    pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup(); //encoders read and RPM calcs
 // timer4_interrupt_setup(); //increase rpm ref every 5 secs (debugging)

 // PWM SETTINGS
   TCCR2B &=~7;    //clear
  TCCR2B |= 2;  //set to prescale of 8 (freq 4000Hz)
  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);
  
    interrupts();
    RPM_ref_m1 = -50;
    RPM_ref_m2 = 30;
    new_data = true;
    analogWrite(M1pin, 64);   
    
}

void loop() {
  int motor_cmd = 0;

  /*  if(new_data)  //imitate new data received over USB serial
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
        motor_cmd = motor1.updatePID(RPM_actual_m1, RPM_ref_m1,1);
        //Serial1.write(motor_cmd);
        analogWrite(M1pin, motor_cmd);    //M1: 1 full speed anti clockwise (+rpm), 127 stop, 255 full speed clockwise (-rpm)
        analogWrite(M2pin, 127);  //M2: 1 full speed  clockwise, 127 stop, 255 full speed anti clockwise

  //      motor_cmd = motor2.updatePID(RPM_actual_m2, RPM_ref_m2,2);
  //      Serial1.write(motor_cmd);
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
    clearEncoderCount(); 
    PID_flag = true;


   /* PID_count++;
    
    if (PID_count >=2)
    {
      PID_flag = true;
    }*/
      
  }

}



int PID::updatePID(int actual, int ref, char motor){
  int error, error_kp, error_ki, error_kd, pid_cmd_serial, pid_cmd, derivative, bias;

    error = ref - actual;             //get RPM error
    
    error_kp = (kp*error)/10;           //proportional

    integral_sum += error*0.2;          //integral error
    error_ki = (ki*integral_sum)/10;

    derivative = ((error - error_prev) /0.2);     //derivative error
    error_kd = (kd*derivative)/10;

    error_prev = error;   //save error

    pid_cmd = (error_kp + error_ki + error_kd + 127);          //tuning info at: http://robotsforroboticists.com/pid-control/
    Serial.print("pid cmd before saturation cuts: ");Serial.println(pid_cmd);


    //debugging...
      Serial.print("The error is ");Serial.println(error);
      Serial.print("The kp error is ");Serial.println(error_kp);
    Serial.print("The kd error  is ");Serial.println(error_kd);
    Serial.print("The derivative   is ");Serial.println(derivative);
    Serial.print("The ki error  is ");Serial.println(error_ki);
    Serial.print("The integral is ");Serial.println(integral_sum);

    //saturation for PWM control
    if (pid_cmd > 255)
    {
      pid_cmd = 255;
    }else if (pid_cmd < 1)
    {
      pid_cmd = 1;
    }


    //saturation for serial control
   /*   if (motor == 1)
    {
      if (pid_cmd > 127)
      {
        pid_cmd = 127;
      }
      else if (pid_cmd < 1)
      {
        pid_cmd = 1;
      }
    }
    else if (motor == 2)
    {
      if (pid_cmd > 255)
      {
        pid_cmd = 255;
      }
      else if (pid_cmd < 128)
      {
        pid_cmd = 128;
      }
    }*/


    
  /*    // translate rpm to serial (not sure if such a good idea...)(if using simplified serial)
    if (motor == 1)
    {
      pid_cmd_serial = M1_rpm_to_serial(pid_cmd);
    }
    else if (motor == 2)
    {
      pid_cmd_serial = M2_rpm_to_serial(pid_cmd);

    }*/
    //Serial.print("pid_cmd_serial from PID: ");Serial.println(pid_cmd_serial);

   PID_flag = false;
   return pid_cmd;
}

//int 4 is for debugging purposes. increment rpm ref by 10 every 5 secs
void timer4_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer4_counter = 3036;   // 
  
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
        motor1.integral_sum = 0;
        motor2.integral_sum = 0;


        
      }
      if (RPM_ref_m1 >= 200)
      {
        Serial.println("RPM at 200 stop");
      }
      

  
}
