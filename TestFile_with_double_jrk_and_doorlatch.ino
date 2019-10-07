#include "BluetoothSerial.h" //Library for Bluetooth Serial communication
#include <JrkG2.h> // Library for the 18v27 Jrk Pololu motor controller

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define jrkSerial Serial2 //ESP32 Serial1 is used internally in the board, use Serial2 instead (GPIO 16,17) 

JrkG2Serial jrk1(jrkSerial, 11); //Jrk1 defines device number "11" - Driver
JrkG2Serial jrk2(jrkSerial, 12); //Jrk2 defines device number "12" - Passenger

BluetoothSerial SerialBT; 

const int DriverSwitch = 34; //Driver Door internal/external switch GPIO to ESP32 board
const int PassengerSwitch = 39; //Passenger Door internal/external switch GPIO to ESP32 board
const int DriverDoorLatch = 21; //Driver door actuator control, Pull Low to engage
const int PassengerDoorLatch = 14; //Passenger door actuator control, Pull Low to engage
String inData; //String variable name to take input from SerialBT command
enum ascii {K = 75, O = 79}; // ASCII format for unsigned int for SerialBT.write 

void setup() {
  Serial.begin(9600); //Initialize baud rate for serial comm
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(DriverSwitch, INPUT); //Define driver switch as an input
  pinMode(PassengerSwitch, INPUT); //Define passenger switch as an input
  pinMode(DriverDoorLatch, OUTPUT); //Define driver door latch as an ouput
  pinMode(PassengerDoorLatch, OUTPUT); //Define passenger door latch as an output
  digitalWrite(DriverDoorLatch,HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(PassengerDoorLatch,HIGH);  //Initial State to "High" relay "OFF"
  jrkSerial.begin(9600); //Initialize baud rate for bluetooth serial comm
}

void loop() {
  while (SerialBT.available() >0) //When serial bluetooh is available
  {
    char received = SerialBT.read(); //SerialBT receives one char
    inData += received; // all receive char will be parsed as an input

    if (received == '\n') //if received data returns new line ending, compare input for proper command functions
    {
      Serial.println(inData); //Print the received command
      
      if(inData == "Auto close driver\n"){
        autoclose_driver();   
      }
      else if(inData == "Auto close passenger\n") {
        autoclose_passenger();
      }
      else if(inData == "Auto open driver\n"){
        driver_door_latch_open();
        delay(1000);
        autoopen_driver();
        delay(3000);
        driver_door_latch_close();

      }
      else if(inData == "Auto open passenger\n"){
        passenger_door_latch_open();
        delay(1000);
        autoopen_passenger();
        delay(3000);
        passenger_door_latch_close();
      }
      else if(inData == "Auto close both\n"){
        autoclose_both();
      }
      else if(inData == "Auto open both\n"){
        driver_door_latch_open();
        passenger_door_latch_open();
        delay(1000);
        autoopen_both();
        delay(3000);
        driver_door_latch_close();
        passenger_door_latch_close();
      }
      else if(inData == "Manual driver open\n"){
        driver_door_latch_open();
        delay(1000);
        on_press_driver_open();
        delay(3000);
        driver_door_latch_close();
      }
      else if(inData == "Manual driver close\n"){
        on_press_driver_close();
      }
      else if (inData == "Manual driver stop\n"){
        on_release_driver();
      }
      else if (inData == "Manual passenger open\n"){
        driver_door_latch_open();
        delay(1000);
        on_press_passenger_open();
        delay(3000);
        driver_door_latch_close();
      }
      else if (inData == "Manual passenger close\n"){
        on_press_passenger_close();
      }
      else if (inData == "Manual passenger stop\n"){
        on_release_passenger();
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
    Serial.println("Do Nothing");
//  uint8_t msg[] = {86, 85, 32, 74,85, 73,67, 69}; // Message feedback to phone
//  SerialBT.write(msg, 8);
//  delay(3000);
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
jrk1.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto close driver has been initiated");
delay(500);
}

void autoclose_passenger(){
jrk2.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto close passenger door has been initiated");
delay(500);
}

void autoopen_driver(){
jrk1.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto open driver door has been initiated");
delay(500);
}

void autoopen_passenger(){
jrk2.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto open passenger door has been initiated");
delay(500);
}

void autoopen_both(){
jrk1.setTarget(1600);
jrk2.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto open both doors has been initiated");
delay(500);
}

void autoclose_both(){
jrk1.setTarget(2400);
jrk2.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto close both doors has been initiated");
delay(500);
}

void on_press_driver_open(){ //Command to open driver door manually
jrk1.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to open driver door manually initiated");
delay(500);
}

void on_press_driver_close(){ //Command to close driver door manually
jrk1.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to close driver door manually initiated");
delay(500);
}

void on_release_driver(){ //Command to stop the motor when button press is released for Driver
jrk1.stopMotor();
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to stop driver door manually initiated");
delay(500);
}

void on_press_passenger_open(){ //Command to open passenger door manually
jrk2.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to opem passenger door manually initiated");
delay(500);
}

void on_press_passenger_close(){ //Command to close passenger door manually 
jrk2.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to close passenger door manually initiated");
delay(500);
}

void on_release_passenger(){ //Command to stop the motor when button press is released for Passenger
jrk2.stopMotor();
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to stop passenger door manually initiated");
delay(500);
}

void driver_door_latch_open(){
  digitalWrite(DriverDoorLatch, LOW);
  Serial.println("Door Latch Open");
}

void driver_door_latch_close(){
  digitalWrite(DriverDoorLatch, HIGH);
  Serial.println("Door Latch Close");
}

void passenger_door_latch_open(){
  digitalWrite(PassengerDoorLatch, LOW);
  Serial.println("Passenger Latch Open");
}

void passenger_door_latch_close(){
  digitalWrite(PassengerDoorLatch, HIGH);
  Serial.println("Passenger Latch Close");
}
