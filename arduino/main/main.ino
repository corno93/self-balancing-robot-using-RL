

//#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13
#define M1pin 12
#define M2pin 11
//#include <SabertoothSimplified.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"


//DT VARIBALES
unsigned long start = 0;
unsigned long end_ = 0;


int saturation(int cmd);

//OBJECT INSTANCES
//SabertoothSimplified ST;
PID motor1(0x00006000,0x00000100,0);
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
  //  SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
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
    RPM_ref_m1 = 100;
    RPM_ref_m2 = 30;
    new_data = true;
    analogWrite(M1pin, 100);  
    analogWrite(M2pin, 128);


    fixed_point_t s1,s2,s3;

    if ((fixed_point_t)0x01000000 > (fixed_point_t)0x00000100) {
      Serial.println("fdsafdsa");
    }

    s1 = 0x00010000;
    s2 = 0x00100000;
    Serial.println(fp_mul(s1,s2), HEX);
    Serial.println(fp_saturate((fixed_point_t)0x01000000, (fixed_point_t)0x00FFFF00), HEX);

    //Serial1.write(72); 
}

void loop() {
  fixed_point_t pid_output;
  unsigned char motor_cmd = 0;
  fixed_point_t s1, s2, s3;
  int time_elapsed= 0;

 /* s1 = 0x00000033;
  s2 = 0x00000100;
  s3 = 0x00000A00;
  Serial.println(s1, HEX);
  Serial.println(s2, HEX);
  Serial.println(fp_mul(s1,s2), HEX);
  Serial.println(fp_mul(s1,s3), HEX);*/

     //Serial.print(RPM_actual_m1);Serial.print("-");Serial.println(RPM_actual_m2);

       if (PID_flag)
     {
        start = micros();
//      //delta time interrupt  test
//      Serial.println("ELPASED TIME IS");
//      time_elapsed = start - micros();
//      Serial.println(time_elapsed);
//      start = micros();
//      PID_flag = false;
        PID_flag = false;

      //  Serial.println(RPM_actual_m1);
        pid_output = motor1.updatePID(int16_fp(RPM_actual_m1), int16_fp(RPM_ref_m1), 1);
        // translate to integer
        // Serial.println(pid_output, HEX);
 //       Serial.print("other thing: "); Serial.println(*((unsigned char *)&pid_output + 1), HEX);

        // Saturate the PID output
        pid_output = fp_saturate(pid_output, 0x007FFF00);
        int32_t tmp = *(int *)((char *)&pid_output + 1);
        
        // motor_cmd = *((unsigned char *)&pid_output + 1) + (unsigned char)127;
        motor_cmd = (unsigned char)((tmp / 256) + 128);
        // motor_cmd = saturation(motor_cmd);
   //     Serial.print("motor cmd: ");Serial.println(motor_cmd);
        analogWrite(M1pin, motor_cmd);    //M1: 1 full speed anti clockwise (-rpm), 127 stop, 255 full speed clockwise (+rpm)
        end_ = micros() - start;
        
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
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);  //debugging freq

    ISR3_counter = 0;
    RPM_actual_m1 = (encoder1count*3000)/1920;
    RPM_actual_m2 = (encoder2count*3000)/1920;
    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(); 
    PID_flag = true;

  }

}



fixed_point_t PID::updatePID(fixed_point_t actual, fixed_point_t ref, char motor)
{
  // int error, error_kp, error_ki, error_kd, pid_cmd_serial, pid_cmd, derivative, bias;
 
    fixed_point_t error, error_kp, error_ki, error_kd, pid_cmd_serial, pid_cmd, derivative, bias;
    //Serial.println(actual, HEX);
    //Serial.println(ref, HEX);

    error = ref - actual;             //get RPM error
    
    error_kp = fp_mul(kp, error);
    //dt = dt - micros();
    //Serial.print("time difference is: ");Serial.println(dt);

    integral_sum += fp_mul(error, dt);          //integral error
    error_ki = fp_mul(ki, integral_sum);

    derivative = fp_mul(error - error_prev, dt_i);
    error_kd = fp_mul(kd, derivative);

    error_prev = error;   //save error
    //Serial.println("a");
    //Serial.println(error_kp, HEX);
    //Serial.println(error, HEX);
    //Serial.println(kp, HEX);
    // Serial.println(error_kp, HEX);

    return (error_kp + error_ki + error_kd);

    
    pid_cmd = (error_kp + error_ki + error_kd + 127);          //tuning info at: http://robotsforroboticists.com/pid-control/
    Serial.print("pid cmd before saturation cuts: ");Serial.println(pid_cmd);


    //debugging...
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


int saturation(int cmd)
{
 if (cmd > 255)
 {
  cmd = 255 ;
 }else if (cmd < 1)
 {
  cmd = 1;
 }
   return cmd;

}
