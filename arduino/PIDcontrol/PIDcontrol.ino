//=========================HEADER=============================================================
/*
   Dual LS7366 Quadrature Counter Test Code
   AUTHOR: Jason Traud
   DATE: June 22, 2013
   
   This is a simple test program to read encoder counts
   collected by the LS7366 breakout board. The counts are
   then displayed in the Arduino's serial monitor at a 
   baud rate of 9600
   
   Hardware: Arduino Uno R3
   Powered 
   
   LS7366 Breakout    -------------   Arduino
   -----------------                    -------
            MOSI   -------------------   SDO (D11)
            MISO   -------------------   SDI (D12)
            SCK    -------------------   SCK (D13)
            SS1    -------------------   SS1 (D7)
            SS2    -------------------   SS2 (D8)
            GND    -------------------   GND
            VDD    -------------------   VCC (5.0V)
  //OUR PIN OUT! ! ! !
 LS7366 Breakout    -------------   Arduino
   -----------------                    -------
   S1                       7
   MOSI                       51
   MISO                       50
   SCLK                       52
   GND                        RAIL GND
   5V                         RAIL 5V
   I                          FLOAT
   B                          ENCODER WHITE
   A                          ENCODER YELLOW
   V                          ENCODER BLUE
   G                          ENCODER GREEN
  MOTOR S1                    TX1
   
     
      
   License: CCAv3.0 Attribution-ShareAlike (http://creativecommons.org/licenses/by-sa/3.0/)
   You're free to use this code for any venture. Attribution is greatly appreciated. 
//============================================================================================
*/
#define USBCON //uses Tx1 (see SabertoothSimplified.h)
#define ledPin 13

// Inclde the standard Arduino SPI Library, please ensure the SPI pins are
// connected properly for your Arduino version
#include <SPI.h>
#include <SabertoothSimplified.h>
#include <encoders.ino>

SabertoothSimplified ST;

//INTERRUPT STUFF
int timer1_counter;
int timer3_counter;
int RPM_actual_m2 = 0;
boolean PID = true;
int ISR3_counter=0;


//PID VARIABLES:
int RPM_ref_m1;
int RPM_actual_m1;
int integral;
float KP = 1.4, KD = 0.1, KI = 1;
float dt = 0.02, derivative;
int error_prev = 0;
int m1_cmd;
char PID_count;

  float error_kp, error_kd, error_ki, motor1_rpmcmd;
  int motor1_serialcmd;
  int error_m1,error_d;



// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;



int M1_rpm_to_serial(float rpm_cmd)
{
  int serial_cmd;
  serial_cmd = (rpm_cmd + 400.68)/6.3504;     //m1 eqn
  if (serial_cmd > 127)
  {
    serial_cmd = 127;
  }else if (serial_cmd < 1)
  {
    serial_cmd = 1;
  }
  return (serial_cmd);
}

void timer3_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer3_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer3_counter = 3036;   // 0.5Hz
  timer3_counter = 49911;   // preload timer 65536-16MHz/256/2Hz (results in 1 second interrupts)
  
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}


  
ISR(TIMER3_OVF_vect)        // interrupt service routine at 100Hz
{


  TCNT3 = timer3_counter;   // preload timer

  encoder1count = readEncoder(1); 
 // encoder2count = readEncoder(2);
  //Serial.println("int 3.1");

  ISR3_counter++;

  
  if (ISR3_counter >=2)   //at 50Hz
  {    
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);

    ISR3_counter = 0;
     RPM_actual_m1 = (encoder1count*60)/1920;
    encoder1count = 0;
   clearEncoderCount(); // Serial.println("Encoders Cleared...");
    PID_count++;
    if (PID_count >=2)
    {
      PID = false;
    }
      
  }

}


void setup() {
 Serial.begin(9600);      // Serial com for data output
 SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
 
 initEncoders();       Serial.println("Encoders Initialized...");  
 clearEncoderCount();  Serial.println("Encoders Cleared...");
   pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup(); 


}

void loop() {

 RPM_ref_m1 = 30;
 m1_cmd = M1_rpm_to_serial(RPM_ref_m1);
 Serial1.write(m1_cmd);
 Serial.print("The reference serial is: ");Serial.println(m1_cmd);

 //Serial1.write(55);  //motor 1: 1 is full reverse, 64 is stop and 127 is full forward
 //Serial1.write(180);   //motor 2: 128 is full reverse, 192 is stop and 255 is full forward

interrupts();

 while(1)
 {
  Serial.print("actual RPM is: ");Serial.println(RPM_actual_m1);

        if (PID)
                {
                  error_m1 = (RPM_ref_m1 - RPM_actual_m1);  //get RPM error
                  
                  error_kp = KP*error_m1; //proportional
              
                  integral+=error_m1*dt;  //integral
                  error_ki = KI*integral;
              
                  derivative = abs((error_m1 - error_prev))/ dt; //derivative (dt = 0.02 seconds since 50Hz)
                  error_kd = KD*derivative;
              
              
                  //debugging...
                  Serial.print("The error is ");Serial.println(error_m1);
                 // Serial.print("The error derivative is ");Serial.println(derivative);
                //  Serial.print("The integral is ");Serial.println(integral);
                 
                 error_prev = error_m1;   //save next previous error
              
                  motor1_rpmcmd = error_kp + error_ki; //+ error_kd;              //tuning tech at: http://robotsforroboticists.com/pid-control/
                // Serial.print("motor1_rpmcmd ");Serial.println(motor1_rpmcmd);
              
                  m1_cmd = M1_rpm_to_serial(motor1_rpmcmd);
                //  Serial1.write(motor1_serialcmd);
              
                  Serial.print("m1_cmd from PID: ");Serial.println(m1_cmd);
                 Serial1.write(m1_cmd);
              
                }
}


