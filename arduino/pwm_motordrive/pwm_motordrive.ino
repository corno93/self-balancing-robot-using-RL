
/*
scripts writes a PWM from pin 12
 pin 12 uses timer 1
PWM has been modified to a freq of 4kHz from using 
a prescale of 2 on timer 1

useful https://forum.arduino.cc/index.php?topic=72092.0
*/


#define M1pin 12
#define M2pin 11


void setup() {
  // put your setup code here, to run once:
  TCCR2B &=~7;    //clear
  TCCR2B |= 2;  //set to prescale of 8 (freq 4000Hz)
  pinMode(M1pin, OUTPUT);
  pinMode(M2pin, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(M1pin, 129);    //M1: 1 full speed anti clockwise, 127 stop, 255 full speed clockwise
  analogWrite(M2pin, 127);  //M2: 1 full speed  clockwise, 127 stop, 255 full speed anti clockwise


}
