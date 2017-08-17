


#ifndef HEADER_ENCODERS
  #define HEADER_ENCODERS

#include <SPI.h>
#include "Arduino.h"


       
  //VARIABLES:
  
  // Slave Select pins for encoders 1 and 2
  // Feel free to reallocate these pins to best suit your circuit
  const int slaveSelectEnc1 = 7;
  const int slaveSelectEnc2 = 8;
  
  // These hold the current encoder count.
  //signed long encoder1count = 0;
  //signed long encoder2count = 0;
  
  
  
  
  //PROTOTYPES:
  
  void initEncoders();
  long readEncoder(int encoder);
  void clearEncoderCount();

#endif

