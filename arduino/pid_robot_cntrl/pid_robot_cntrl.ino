/*
 * Raw PID control. (ie, arduino subscribes to topic to receive motor PWM duty value)
 *                   
 */


#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <arduino_feedback/feedback.h>
#include "encoders.h"
#include "fixedpoint.h"
#include "PID.h"
#include "WheelController.h"
#include <SPI.h>


#define M1pin 12
#define M2pin 11
#define STOP 126  //was 128 at some point...
#define ledPin 13
#define KT 0.1552

char log_msg[50] = {0};

//INTERRUPT VARIABLES
int timer3_counter;
int ISR3_counter=0;

//ENCODER VARIABLES
signed long encoder1count = 0;
signed long encoder2count = 0;

//PID VARIABLES:
boolean publish_message = false;
int motor_PWM;
int RPM_actual_m1;
int RPM_ref_m2;
int RPM_actual_m2;
int PID_count;

//TORQUE CALC VARIABLES:
double raw_value_m1;
double raw_value_m2;
double voltage_m1;
double voltage_m2;
double amps_m1;
double amps_m2;

//DEBUGGING VARIABLES:
unsigned long start = 0;
unsigned long end_ = 0;

//CALLBACK FROM /rpm_cmd TOPIC
void pwm_cmdCb( const std_msgs::Int16& msg){
  //digitalWrite(13, HIGH-digitalRead(13));   
  motor_PWM = msg.data;

}

ros::NodeHandle  nh;
std_msgs::String str_msg;
arduino_feedback::feedback arduino_msg;
ros::Publisher arduino_chatter("arduino_data", &arduino_msg);
ros::Subscriber<std_msgs::Int16> sub("pwm_cmd", &pwm_cmdCb );



void setup()
{
  nh.initNode();
  nh.advertise(arduino_chatter);
  nh.subscribe(sub);
  
  initEncoders();       //Serial.println("Encoders Initialized...");  
  clearEncoderCount();  //Serial.println("Encoders Cleared...");
  pinMode(ledPin, OUTPUT);
  timer3_interrupt_setup(); //encoders read and RPM calcs
 // PWM SETTINGS (set timer 1 to 8 prescale to get a PWM with freq )
  TCCR1B = TCCR1B & B11111000 | B00000001; 
  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);

    analogWrite(M1pin, STOP);  
    analogWrite(M2pin, STOP);
    interrupts();


}

void loop()
{
   
  //update rpm reference commands
  nh.spinOnce();
  //sprintf(log_msg, "rpm_cmd: %d", RPM_ref_m1);
  //nh.loginfo(log_msg);
//  end_ = micros() - start;
//  sprintf(log_msg, "loop time in micros is: %d", end_);
//  nh.loginfo(log_msg);
//  start = micros();
  arduino_msg.reference_rpm = motor_PWM;
  
  digitalWrite(13, HIGH-digitalRead(13));   
  analogWrite(M1pin, motor_PWM);
  analogWrite(M2pin, motor_PWM);

  
  // compute PID gains and write to motors when ready
    if (publish_message)
     {
        publish_message = false;
        
        // get averaged ADC values for current in motor's armature
//        for (int i = 0; i < 5; i++)
//        {
//          raw_value_m1 += analogRead(A15);  //analog read takes 100 microseconds (0.0001s)
//          raw_value_m2 += analogRead(A14);
//        }
//        voltage_m1 = ((raw_value_m1 / 5) / 1023)*5000;
//        voltage_m2 = ((raw_value_m2 / 5) / 1023)*5000;
//        amps_m1 = ((voltage_m1 - 2500)/185);
//        amps_m2 = ((voltage_m2 - 2500)/185);
//        raw_value_m1 = 0;
//        raw_value_m2 = 0;  
//        arduino_msg.voltage_1 = voltage_m1;
//        arduino_msg.voltage_2 = voltage_m2;
//        arduino_msg.current_1 = amps_m1;
//        arduino_msg.current_2 = amps_m2;
//        arduino_msg.torque_1 = amps_m1 * KT;
//        arduino_msg.torque_2 = amps_m2 * KT;
    //    sprintf(log_msg, "current 1: %f", (arduino_msg.current_1));
    //   nh.loginfo(log_msg);
        
        // publish arduino data
        arduino_chatter.publish( &arduino_msg );
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
    arduino_msg.encoder1 = encoder1count;
    arduino_msg.encoder2 = encoder2count;
    arduino_msg.actual_rpm1 = RPM_actual_m1;
    arduino_msg.actual_rpm2 = RPM_actual_m2;

    encoder1count = 0;
    encoder2count = 0;
    clearEncoderCount(); 
    publish_message = true;

  }

}
