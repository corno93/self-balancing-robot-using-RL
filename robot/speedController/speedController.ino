/**
  Main script for the arduino microcontroller which acts as a speed controller.
  A reference RPM is published on the rpm_cmd topic from the higher level control loop.
  The arduino's job is to ensure it reaches and maintains this rpm value as quickly as possible. 
  It does this by running a PID loop and writing a new PWM to each motor at 100Hz. 
  This script also publishes data (encoder count and the actual RPM of each motor) to the topic arduino_data for data collection.

  @author Alex Cornelio

*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <arduino_feedback/feedback.hpp>
#include "encoders.hpp"
#include "fixedpoint.hpp"
#include "PID.hpp"
#include "wheelController.hpp"

#define M1pin 12  //motor 1 pin
#define M2pin 11  //motor 2 pin
#define STOP 128  // PWM value that will stop both motors

//wheel controllers. write the PID gains here
WheelController wheelCtrl1(0x00003550,0x00001500,0x00000020);
WheelController wheelCtrl2(0x00003550,0x00001500,0x00000020);

//global interrupt variables 
int timer3_counter;
int ISR3_counter=0;

//global encoder values
signed long encoder1count = 0;
signed long encoder2count = 0;

//global PID variables
boolean PID_flag = false;
int RPM_ref_m1;
int RPM_actual_m1;
int RPM_ref_m2;
int RPM_actual_m2;
int PID_count;

// global variables for interfacing with ROS
ros::NodeHandle  nh;
std_msgs::String str_msg;
arduino_feedback::feedback arduino_msg;
ros::Publisher arduino_chatter("arduino_data", &arduino_msg);
ros::Subscriber<std_msgs::Int16> sub("rpm_cmd", &rpm_cmdCb );

/**
  Handle callback from /rpm_cmd topic
*/
void rpm_cmdCb( const std_msgs::Int16& msg){
  digitalWrite(13, HIGH-digitalRead(13));   
  RPM_ref_m1 = msg.data;
  RPM_ref_m2 = RPM_ref_m1;
}

/**
  Setup speed controller. 
  This includes: interfacing with ROS, serial coms with chan 1, encoders, PWM pins for motors and interrupts on timer3.
  Initalise reference RPMs to 0
*/
void setup() {
  
    // subscribe to rpm_cmd topic and write to arduino_data topic
    nh.initNode();
    nh.advertise(arduino_chatter);
    nh.subscribe(sub);

    // Serial com for data output
    Serial.begin(9600);   

    // encoders   
    initEncoders();       
    clearEncoderCount();  
    timer3_interrupt_setup(); //encoders read and RPM calcs

    // PWM on timer 1
    TCCR1B = TCCR1B & B11111000 | B00000001; 

    // Motor config
    pinMode(M1pin, OUTPUT);
    pinMode(M2pin, OUTPUT);
    RPM_ref_m1 = 0;
    RPM_ref_m2 = 0;
    analogWrite(M1pin, STOP);  
    analogWrite(M2pin, STOP);

    pinMode(ledPin, OUTPUT);

    interrupts();

}

/**
  Main function.
  When the PID_flag is true, calculate the motor command with PID gains and write it to the PWM pins
*/
void loop() {

    //update rpm reference commands
    nh.spinOnce();
    arduino_msg.reference_rpm = RPM_ref_m1;

    if (PID_flag)
       {

          PID_flag = false;

          analogWrite(M1pin, wheelCtrl1.tick(RPM_actual_m1, RPM_ref_m1));
          analogWrite(M2pin, wheelCtrl2.tick(-RPM_actual_m2, RPM_ref_m2));
          
       }

    // publish arduino data
    arduino_chatter.publish( &arduino_msg );

}



/**
  Timer3 interrupt setup. 
  Set timer3 to have interrupts at every 0.005 seconds
*/
void timer3_interrupt_setup()
{
    // initialize timer3 
    noInterrupts();           // disable all interrupts
    TCCR3A = 0;
    TCCR3B = 0;

    // Set timer3_counter to the correct value for our interrupt interval
    timer3_counter = 55536;     //200Hz at 8 prescale

    TCNT3 = timer3_counter;   // preload timer
    TCCR3B &=~7; //clear
    TCCR3B |= (1 << CS11);    //8 prescaler

    TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}


/**
  ISR for timer 3. Read each encoders every 0.005 seconds and every 0.01 seconds, calculate the motor's actual RPM, reset the encoder counts and set the PID_flag to true.
*/
ISR(TIMER3_OVF_vect)        
{

  TCNT3 = timer3_counter;   // preload timer

  encoder1count = readEncoder(1); 
  encoder2count = readEncoder(2);

  ISR3_counter++;

  if (ISR3_counter >=2)    
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
    PID_flag = true;

  }

}



   
