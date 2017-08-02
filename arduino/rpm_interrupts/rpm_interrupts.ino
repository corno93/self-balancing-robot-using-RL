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

SabertoothSimplified ST;

//INTERRUPT STUFF
int RPM_actual_m1 = 0;
int timer1_counter;
int timer3_counter;
int RPM_actual_m2 = 0;
boolean PID = false;




// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;

void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);
  
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc2,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder) {
  
  // Initialize temporary variables for SPI read
  unsigned int count_1=0, count_2=0, count_3=0, count_4=0;
  long count_value=0;  
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  }
  
//  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;
  
  return count_value;
}

void clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
 // delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
  
//  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
//  // Write to DTR
  SPI.transfer(0x98);    
//  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
//  
 // delayMicroseconds(100);  // provides some breathing room between SPI conversations
//  
//  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation 
}

void timer1_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  timer1_counter = 58000;//3037;//3037;//3036;//59286;//3036;//58500;//3036;//62411;//3036;//59286;     // preload timer 65536-16MHz/256/10Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
}

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
    // preload timer
      TCNT1 = timer1_counter; 

  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);

  RPM_actual_m1 = (encoder1count*600)/1920;
  encoder1count = 0;
   Serial.print("RPM1: ");Serial.println(RPM_actual_m1);

     RPM_actual_m2 = (encoder2count*600)/1920;
  encoder2count = 0;
   Serial.print("RPM2: ");Serial.println(RPM_actual_m2);
   
    clearEncoderCount();  Serial.println("Encoders Cleared...");
  Serial.println("int 1");
PID = true;

  
}

void timer3_interrupt_setup()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer3_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  timer3_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer3_counter = 62411;   // preload timer 65536-16MHz/256/2Hz (results in 1 second interrupts)
  
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
  
}

ISR(TIMER3_OVF_vect)        // interrupt service routine 
{
  TCNT3 = timer3_counter;   // preload timer
  Serial.println("int 3");
   encoder1count = readEncoder(1); 
 encoder2count = readEncoder(2);
 Serial.print("Enc1: "); Serial.println(encoder1count); Serial.print(" Enc2: "); Serial.println(encoder2count);

}


void setup() {
 Serial.begin(9600);      // Serial com for data output
 SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
 
 initEncoders();       Serial.println("Encoders Initialized...");  
 clearEncoderCount();  Serial.println("Encoders Cleared...");
   pinMode(ledPin, OUTPUT);
    timer3_interrupt_setup();

  timer1_interrupt_setup();


}

void loop() {
// delay(100);
int a;
                // enable all interrupts

//  RPM_ref_m1 = 76;
a = 71;
 Serial1.write(a);  //motor 1: 1 is full reverse, 64 is stop and 127 is full forward
// Serial1.write(180);   //motor 2: 128 is full reverse, 192 is stop and 255 is full forward
 //delay(100);
interrupts();

 while(1)
 {
 // delay(100);
 //Serial1.write(127);  //motor 1: 1 is full reverse, 64 is stop and 127 is full forward
 //Serial1.write(255);   //motor 2: 128 is full reverse, 192 is stop and 255 is full forward
 // Retrieve current encoder counters

 
 } 
}


