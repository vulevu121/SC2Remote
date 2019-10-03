#include "BluetoothSerial.h"
#include <JrkG2.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const int DriverSwitch = 34;
const int PassengerSwitch = 39;
String inData;

#define jrkSerial Serial2

JrkG2Serial jrk1(jrkSerial, 11);
JrkG2Serial jrk2(jrkSerial, 12);

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(DriverSwitch, INPUT);
  pinMode(PassengerSwitch, INPUT);
  jrkSerial.begin(9600);
}

void loop() {
  while (SerialBT.available() >0) 
  {
    char received = SerialBT.read();
    inData += received;

    if (received == '\n')
    {
      Serial.println(inData);
      
      if(inData == "Auto close driver\n"){
        autoclose_driver();   
      }
      else if(inData == "Auto close passenger\n") {
        autoclose_passenger();
      }
      else if(inData == "Auto open driver\n"){
        autoopen_driver();
      }
      else if(inData == "Auto open passenger\n"){
        autoopen_passenger();
      }
      else if(inData == "Auto close both\n"){
        autoclose_both();
      }
      else if(inData == "Auto open both\n"){
        autoopen_both();
      }
      else {
        Serial.println("Please input valid command\n");
    }
    inData = ""; //Clear received buffer
    }
  }
  
// Driver side switch function (Two switches, one itnernl and one external). See schematic
  int Driver_Switch_State = digitalRead(DriverSwitch);
  if (Driver_Switch_State == HIGH) {
    // the function here will depend on the current getFeedback from the actuator. If the motor actuator
    // feedback is below the half actuated threshold then it should open, otherwise if it is past halfway
    // it will setTarget == 0 which is closed.
    Serial.println("Driver switch has been pressed, close or open door depending on feedback position");
    jrk1.setTarget(1750);
    delay(8000);
    jrk1.setTarget(2400);
  }
  else {
    Serial.println("Do nothing");
    delay(500);
  }

// Passenger side switch function (Two switches, one internal and one external). See schematic
  int Passenger_Switch_State = digitalRead(PassengerSwitch);
  if (Passenger_Switch_State == HIGH) {
    // the function here will depend on the current getFeedback from the actuator. If the motor actuator
    // feedback is below the half actuated threshold then it should open, otherwise if it is past halfway
    // it will setTarget == 0 which is closed.
    Serial.println("Passenger switch has been pressed, close or open door depending on feedback position");
    jrk2.setTarget(2400);
    delay(8000);
    jrk2.setTarget(1750);
  }
  else {
    Serial.println("Do Nothing");
  }
}

void autoclose_driver(){
  Serial.println("now closing driver door");
  jrk1.setTarget(2400);
  delay(500);
}

void autoclose_passenger(){
  Serial.println("now closing passenger door");
  jrk2.setTarget(2400);
  delay(3000);
  jrk2.setTarget(2200);
}

void autoopen_driver(){
  Serial.println("now opening driver door");
  delay(500);
  jrk1.setTarget(1600);
}

void autoopen_passenger(){
  Serial.println("now opening passenger door");
  delay(500);
  jrk2.setTarget(1600);
}

void autoopen_both(){
  Serial.println("Both doors are now opening");
  delay(500);
  jrk1.setTarget(1600);
  jrk2.setTarget(1600);
}

void autoclose_both(){
  Serial.println("Both doors are now closing");
  delay(500);
  jrk1.setTarget(2400);
  jrk2.setTarget(2400);
}

//Momentary Driver side switch
//While button press or serial command is true
//increment the setTarget position? 

//Momentary Passenger side switch
//While button press or serial command is true
//increment the setTarget position? 
