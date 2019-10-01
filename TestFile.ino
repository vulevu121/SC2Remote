#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const int DriverSwitch = 34;
const int PassengerSwitch = 39;
String inData;

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(DriverSwitch, INPUT);
  pinMode(PassengerSwitch, INPUT);
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
    delay(500);
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
    delay(500);
  }
  else {
    Serial.println("Do Nothing");
  }
}

void autoclose_driver(){
  delay(1000);
  Serial.println("now closing driver door");
}

void autoclose_passenger(){
  delay(1000);
  Serial.println("now closing passenger door");
}

void autoopen_driver(){
  delay(1000);
  Serial.println("now opening driver door");
}

void autoopen_passenger(){
  delay(1000);
  Serial.println("now opening passenger door");
}

// need functions for momentary button switches
