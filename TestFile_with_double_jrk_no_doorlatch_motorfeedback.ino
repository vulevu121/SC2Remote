/*
 * Developed by Timothy Medina
 * KARMA Automotive LLC
 * EE Systems Integration & Validation
 * SC2 Vision automotic door control system
 */
 
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
const int hydraulic_up_relay = 33; // Relay to control hydraulic motor vehicle suspension up movement
const int hydraulic_down_relay = 27; // Relay to control hydraulic motor vehicle suspension down movement

String inData; //String variable name to take input from SerialBT command
enum ascii {F = 70, K = 75, O = 79, P = 50, E =45, N = 78, C = 67, L = 76, S=83}; // ASCII format for unsigned int for SerialBT.write 

void setup() {
  Serial.begin(9600); //Initialize baud rate for serial comm
  jrkSerial.begin(9600); //Initialize baud rate for bluetooth serial comm
  SerialBT.begin("SC2 Vision"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(DriverSwitch, INPUT); //Define driver switch as an input
  pinMode(PassengerSwitch, INPUT); //Define passenger switch as an input
  pinMode(DriverDoorLatch, OUTPUT); //Define driver door latch as an ouput
  pinMode(PassengerDoorLatch, OUTPUT); //Define passenger door latch as an output
  pinMode(hydraulic_up_relay, OUTPUT); //Define motor relay pin as an output
  pinMode(hydraulic_down_relay, OUTPUT); //Define motor relay pin as an output
  digitalWrite(hydraulic_up_relay, HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(hydraulic_down_relay, HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(DriverDoorLatch,HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(PassengerDoorLatch,HIGH);  //Initial State to "High" relay "OFF"
}

void loop() {
  while (SerialBT.available() >0) //When serial bluetooh is available
  {
    char received = SerialBT.read(); //SerialBT receives one char
    inData += received; // all receive char will be parsed as an input
    if (received == '\n') //if received data returns new line ending, compare input for proper command functions
    {
      Serial.println(inData); //Print the received command
      
//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
//-------------------------------------Conditions when certain commands are met -----------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
           
      //-----------Auto close driver door control -----------------------------------------------------------------------------
     
      if(inData == "Auto close driver\n")
      {
        Serial.println("Automatic close driver door");
        autoclose_driver();   
      }
  
      //-----------Auto close passenger door control --------------------------------------------------------------------------
     
      else if(inData == "Auto close passenger\n") 
      {
        Serial.println("Automatic close passenger door");
        autoclose_passenger();
      }
      
      //-----------Auto open driver door control-------------------------------------------------------------------------------
      else if(inData == "Auto open driver\n")
      {
        driver_door_latch_open();
        delay(2000);
        Serial.println("Automatic open driver door");
        autoopen_driver();
        delay(2000);  
        driver_door_latch_close(); 
       }
         
      //----------Auto open passenger door control-----------------------------------------------------------------------------
      else if(inData == "Auto open passenger\n")
      {
        passenger_door_latch_open();
        delay(2000);
        Serial.println("Automatic open passenger door");
        autoopen_passenger();
        delay(2000);
        passenger_door_latch_close();
      }
      
      //--------Auto close both door control-----------------------------------------------------------------------------------
      
      else if(inData == "Auto close both\n")
      {
        Serial.println("Automatic close both doors");
        autoclose_both();
        delay(500);
      }
      
      //-------Auto open both door control-------------------------------------------------------------------------------------
      else if(inData == "Auto open both\n")
      {
        driver_door_latch_open();
        passenger_door_latch_open();
        delay(2000);
        Serial.println("Automatic open both doors");
        autoopen_both();
        delay(1000);
        driver_door_latch_close();
        passenger_door_latch_close();
        delay(1000);
       
      }
      
      //------Manual open driver door control----------------------------------------------------------------------------------
      else if(inData == "Manual driver open\n")
      {
        passenger_door_latch_open();
        delay(1000);
        Serial.println("Manual open driver door");
        on_press_driver_open();
        delay(2000);     
      }
      //-----Manual close driver door control----------------------------------------------------------------------------------
      else if(inData == "Manual driver close\n")
      {
        Serial.println("Manual close driver door");
        on_press_driver_close();
        delay(500);
      }

      //-----Manual driver door stop control-----------------------------------------------------------------------------------
      else if (inData == "Manual driver stop\n")
      {
        Serial.println("Manual stop driver door");
        on_release_driver();
        delay(500);
      }

      //-----Manual open passenger door control--------------------------------------------------------------------------------
      
      else if (inData == "Manual passenger open\n")
      {
        Serial.println("Manual open passenger door");
        on_press_passenger_open();
        delay(500);
      }
        
      //-----Manual close passenger door control-------------------------------------------------------------------------------
      else if (inData == "Manual passenger close\n")
      {
        Serial.println("Manual close passenger door");
        on_press_passenger_close();
        delay(500);
      }

      //-----Manual passenger stop control-------------------------------------------------------------------------------------
      else if (inData == "Manual passenger stop\n")
      {
        Serial.println("Manual stop passenger door");
        on_release_passenger();
        delay(500);
      }

      //----Relay to initiate suspension going up motion-----------------------------------------------------------------------
      else if (inData == "Hydraulic Up\n")
      {
         hydraulic_up_direction();
      }

      //----Relay to initiate suspension going down motion---------------------------------------------------------------------
       
      else if (inData == "Hydraulic Down\n")
      {
        hydraulic_down_direction();
      }

      //----Stop hydraulic suspension movements--------------------------------------------------------------------------------
      
      else if (inData == "Hydraulic Stop\n")
      {
        hydraulic_stop();
      }

      else if (inData == "Door Status?\n")
      {
        door_status();
      }

      //-----Invalid command---------------------------------------------------------------------------------------------------
      else {
        Serial.println("Command not recoginized\n");
    }
    inData = ""; //Clear received buffer
    }
  }
  
// Driver side switch function (Two switches, one itnernal and one external). See schematic-------------------------------------
// Driver switch press logic control-------------------------------------------------------------------------------------------
  int Driver_Switch_State = digitalRead(DriverSwitch);
  if (Driver_Switch_State == HIGH)
  {
   uint16_t driver_feedback = jrk1.getScaledFeedback();
   if (driver_feedback > 3300)
   {
    Serial.println(driver_feedback);
    Serial.println("closing");
    Serial.println(Driver_Switch_State);
    autoclose_driver();
    delay(1000);
   }
   else if (driver_feedback < 3300)
   {
    Serial.println(driver_feedback);
    Serial.println("opening");
    Serial.println(Driver_Switch_State);
    autoopen_driver();
    delay(1000);
   }
  }
  else {
    Serial.println("Waiting");
    //driver_motor_stat();
    //jrk1.stopMotor();
    delay(500);
  }

// Passenger side switch function (Two switches, one internal and one external). See schematic---------------------------------
// Passenger switch press logic control----------------------------------------------------------------------------------------
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
    //delay(2000);
    //Serial.println("Do Nothing");
  }
  
}


//--------------------------------------------List of functions for main loop--------------------------------------------------

void autoclose_driver(){
jrk1.stopMotor();
delay(300);
jrk1.setTarget(128);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto close driver has been initiated");
delay(500);
}

void autoclose_passenger(){
jrk2.stopMotor();
delay(300);
jrk2.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto close passenger door has been initiated");
delay(500);
}

void autoopen_driver(){
jrk1.stopMotor();
delay(300);
jrk1.setTarget(3482);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto open driver door has been initiated");
delay(500);
}

void autoopen_passenger(){
jrk2.stopMotor();
delay(300);
jrk2.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto open passenger door has been initiated");
delay(500);
}

void autoopen_both(){
jrk1.stopMotor();
jrk2.stopMotor();
delay(300);
jrk1.setTarget(3482);
jrk2.setTarget(2400);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto open both doors has been initiated");
delay(500);
}

void autoclose_both(){
jrk1.stopMotor();
jrk2.stopMotor();
delay(300);
jrk1.setTarget(128);
jrk2.setTarget(1600);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to auto close both doors has been initiated");
delay(500);
}

void on_press_driver_open(){ //Command to open driver door manually
jrk1.setTarget(3482);
uint8_t msg[] = {'O', 'K'}; // Message feedback to phone
SerialBT.write(msg, 2);
Serial.println("Command to open driver door manually initiated");
delay(500);
}

void on_press_driver_close(){ //Command to close driver door manually
jrk1.setTarget(106);
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

//void driver_motor_stat()
//{
//  uint16_t driver_feedback = jrk1.getScaledFeedback();
//  if (driver_feedback > 3300){
//  uint8_t msg[] = {'O', 'P', 'E', 'N'};
//  SerialBT.write(msg,4);
//  }
//  else if (driver_feedback < 3300){
//  uint8_t msg[] = {'C', 'L', 'O', 'S', 'E'};
//  SerialBT.write(msg,5);
//  }
//}

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

void hydraulic_up_direction(){
  digitalWrite(hydraulic_down_relay, HIGH);
  delay(100);
  digitalWrite(hydraulic_up_relay, LOW);
  Serial.println("Opened motor down relay for safety, now moving UP");
  delay(100); 
}

void hydraulic_down_direction(){
  digitalWrite(hydraulic_up_relay, HIGH);
  delay(100);
  digitalWrite(hydraulic_down_relay, LOW);
  Serial.println("Opened motor up relay for safety, now moving DOWN");
  delay(100);
}

void hydraulic_stop(){
  digitalWrite(hydraulic_up_relay, HIGH);
  digitalWrite(hydraulic_down_relay, HIGH);
  Serial.println("Stop hydraulic suspension initiated");
  delay(100);
}

void door_status(){
  //uint16_t driver_feedback = jrk1.getScaledFeedback();
  float driver_feedback = 1200; // test value
  //uint16_t passenger_feedback = jrk2.getScaledFeedback();
  float passenger_feedback = 3400; // test value
  float driver_max_pos = 3400;
  float pass_max_pos = 3400;
  
  float scaled_driver_max_pos = ((driver_feedback/driver_max_pos) * 100);
  int scaled_driver_max_pos_int = round(scaled_driver_max_pos);

  float scaled_passenger_max_pos = ((passenger_feedback/pass_max_pos) * 100);
  int scaled_passenger_max_pos_int = round(scaled_passenger_max_pos);
  
  //Serial.println(scaled_driver_max_pos_int);
  
  char copy[25];
  String drvr_max_pos;
  
  drvr_max_pos = String(scaled_driver_max_pos_int); 
  //Serial.println(str1);
  drvr_max_pos.toCharArray(copy,25);
  //Serial.println(str1.length());
  //Serial.println(str1);
  uint8_t front_left[] = {'F','L'};
  SerialBT.write(front_left,2);
  for (int i=0; i < drvr_max_pos.length(); i++){
    char drvr = drvr_max_pos[i];
    uint8_t drvr_max_pos_val[] = {drvr};
    SerialBT.write(drvr_max_pos_val,1);
  }
  
  String psngr_max_pos;
  psngr_max_pos = String(scaled_passenger_max_pos_int);
  psngr_max_pos.toCharArray(copy,25);
  uint8_t front_right[] = {'F','R'};
  SerialBT.write(front_right,2);
  for (int i=0; i < psngr_max_pos.length(); i++){
    char psngr = psngr_max_pos[i];
    uint8_t psngr_max_pos_val[] = {psngr};
    SerialBT.write(psngr_max_pos_val,1);
  }
}
