/**
  Encoder functions declarations
  Credit goes to Jason Traud from SuperDroidRobotics for this code (https://github.com/SuperDroidRobots/Encoder-Buffer-Breakout)

*/


#ifndef HEADER_ENCODERS
  #define HEADER_ENCODERS

#include <SPI.h>
#include "Arduino.h"

  
  // Slave Select pins for encoders 1 and 2
  const int slaveSelectEnc1 = 7;
  const int slaveSelectEnc2 = 8;
    
    
  void initEncoders();
  long readEncoder(int encoder);
  void clearEncoderCount();

#endif

