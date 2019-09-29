// Jrk motor controller library
// Test, calibrate, and configure motors & controllers via USB using Pololu software
// Refer to reference user guide for configuring motors
// Baud rate must be set to 9600 && CRC should be DISABLED
#include <JrkG2.h> 
#include "BluetoothSerial.h"

//Jrk Driver side motor controller
//int driver = 11;
//int passenger = 12;

//JrkG2Serial jrk1(jrkG2Serial, 11); 
// Jrk Passenger side motor controller 
//JrkG2Serial jrk2(jrkG2Serial, 12);


void setup() {
  Serial.begin(9600);
  SerialBT.begin("SC2 Server");
  jrkSerial.begin(9600);
}

void loop() {
  
  if(Serial.available() >0) {
    String cmd = Serial.readString();

    if(cmd == "Auto close driver\n"){
      autoclose_driver();   
    }
    else if(cmd == "Auto close passenger\n") {
      autoclose_passenger();
    }
    else if(cmd == "Auto open driver\n"){
      autoopen_driver();
    }
    else if(cmd == "Auto open passenger\n"){
      autoopen_passenger();
    }
    else {
      Serial.println("Please input valid command");
    }   
  }
}

void autoclose_driver(){
  delay(1000);
  jrk1.setTarget(0);
  delay(1000);
}

void autoclose_passenger(){
  delay(1000);
  jrk2.setTarget(0);
  delay(1000)
}

void autoopen_driver(){
  delay(1000);
  jrk1.setTarget(4095);
  delay(1000);
}

void autoopen_passenger(){
  delay(1000);
  jrk1.setTarget(4095);
  delay(1000);
}
