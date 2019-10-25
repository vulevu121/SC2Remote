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
BluetoothSerial SerialBT; //Initialize BT Serial


const int DriverSwitch = 34; //Driver Door internal/external switch GPIO to ESP32 board 
const int PassengerSwitch = 39; //Passenger Door internal/external switch GPIO to ESP32 board
const int DriverDoorLatch = 21; //Driver door actuator control, Pull Low to engage
const int PassengerDoorLatch = 14; //Passenger door actuator control, Pull Low to engage
const int hydraulic_up_relay = 33; // Relay to control hydraulic motor vehicle suspension up movement
const int hydraulic_down_relay = 27; // Relay to control hydraulic motor vehicle suspension down movement
const int Driver_error_feedback = 32; //Input pin for Driver Motor error feedback. Will toggle high if system error is detected
const int Passenger_error_feedback = 15; //Input pin for Pass Motor error feedback. Will toggle high if system error is detected
bool driver_moving = false; //Set flag for driver_moving condition
bool passenger_moving = false; //Set flag for passenger_moving condition
const int Driver_open_target = 3400; //Variable to define set target for driver door open
const int Driver_close_target = 128; //Variable to define set target for driver door close
const int Passenger_open_target = 3400; //Variable to define set target for passenger open
const int Passenger_close_target = 128; //Variable to define set target for passenger close

String inData; //Input string from SerialBT command

void setup() //Set up and intialize values
{
  Serial.begin(9600); //Initialize baud rate for serial comm
  jrkSerial.begin(9600); //Initialize baud rate for bluetooth serial comm
  SerialBT.begin("SC2 Vision"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(DriverSwitch, INPUT); //Define driver switch as an input
  pinMode(PassengerSwitch, INPUT); //Define passenger switch as an input
  pinMode(Driver_error_feedback, INPUT); //Define driver error feedback as input
  pinMode(Passenger_error_feedback, INPUT); //Define passenger error feedback as input
  pinMode(DriverDoorLatch, OUTPUT); //Define driver door latch as an ouput
  pinMode(PassengerDoorLatch, OUTPUT); //Define passenger door latch as an output
  pinMode(hydraulic_up_relay, OUTPUT); //Define motor relay pin as an output
  pinMode(hydraulic_down_relay, OUTPUT); //Define motor relay pin as an output
  digitalWrite(hydraulic_up_relay, HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(hydraulic_down_relay, HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(DriverDoorLatch,HIGH); //Initial State to "High" relay "OFF"
  digitalWrite(PassengerDoorLatch,HIGH);  //Initial State to "High" relay "OFF"
}

void loop() 
{
 driver_resistance_detection(); //Constant check for additional resistance (Autostop if current exceeds normal condition)
 passenger_resistance_detection();//Constant check for additional resistance (Autostop if current exceeds normal condition)
 driver_moving_check(); //Constant check for driver moving flag
 passenger_moving_check(); //Constant check for driver moving flag
// cyclic_dummy_msg();
 door_status();
 while (SerialBT.available() >0) //While serial bluetooh is available, listen for defined commands
  {
   char received = SerialBT.read(); //SerialBT receives one char
   inData += received; // All receive char will be parsed and combined as an input 
   if (received == '\n') //if received data returns new line ending, compare input for proper command functions
   {
    Serial.println(inData); //Print the received command for serial monitor
 
//-------------------------------------Conditions when certain commands are met -----------------------------------------------
   
    //-----------Auto close driver door control -----------------------------------------------------------------------------
    if(inData == "Auto close driver\n")
    {
     autoclose_driver();   
    }
    //-----------Auto close passenger door control --------------------------------------------------------------------------
    else if(inData == "Auto close passenger\n") 
    {
     autoclose_passenger();
    }
    //-----------Auto open driver door control-------------------------------------------------------------------------------
    else if(inData == "Auto open driver\n")
    {
     autoopen_driver();
    }
//---------------Door latch functions---------Uncomment to use---------------------------------------------------------------   
//     uint16_t driver_feedback = jrk1.getScaledFeedback();
//     if (driver_feedback < 500)
//     {
//          driver_door_latch_open();
//          delay(2000);
//          autoopen_driver();
//          delay(2000);  
//          driver_door_latch_close();        
//        }
//        else if (driver_feedback > 500)
//        {
//          Serial.println("Driver door latch already open");
//          autoopen_driver();
//        }
//        else 
//        {
//          Serial.println("Driver system ERR");
//          jrk1.stopMotor();
//          uint8_t msg[] = {'E','R','R'}; // Message feedback to phone
//          SerialBT.write(msg, 3);
//        }
//       }    
  
      //----------Auto open passenger door control-----------------------------------------------------------------------------
     else if(inData == "Auto open passenger\n")
     {
      autoopen_passenger();
     }     
//---------------Door latch functions---------Uncomment to use---------------------------------------------------------------      
//      {
//        uint16_t passenger_feedback = jrk2.getScaledFeedback();
//        if (passenger_feedback < 500)
//        {
//          passenger_door_latch_open();
//          delay(2000);
//          autoopen_passenger();
//          delay(2000);  
//          passenger_door_latch_close();        
//        }
//        else if (passenger_feedback > 500)
//        {
//          Serial.println("Passenger door latch already open");
//          autoopen_passenger();
//        }
//        else 
//        {
//          Serial.println("Passenger system ERR");
//          jrk2.stopMotor();
//          uint8_t msg[] = {'E','R','R'}; // Message feedback to phone
//          SerialBT.write(msg, 3);
//        }
//      }
      
      //--------Auto close both door control-----------------------------------------------------------------------------------
      else if(inData == "Auto close both\n")
      {
       autoclose_both();
      }
      //-------Auto open both door control-------------------------------------------------------------------------------------
      else if(inData == "Auto open both\n")
      {
       autoopen_both();
      }

//---------------Door latch functions---------Uncomment to use---------------------------------------------------------------
//      {
//        uint16_t driver_feedback = jrk1.getScaledFeedback();
//        uint16_t passenger_feedback = jrk2.getScaledFeedback();
//        if (driver_feedback < 500 && passenger_feedback < 500)
//        {
//          driver_door_latch_open();
//          passenger_door_latch_open();
//          delay(1500);
//          autoopen_both();
//          delay(1000);
//          driver_door_latch_close();
//          passenger_door_latch_close();        
//        }
//
//        else if (driver_feedback >500 && passenger_feedback < 500)
//        {
//          Serial.println("Driver door latch already open, no need to open");
//          passenger_door_latch_open();
//          delay(1500);
//          autoopen_both();
//          delay(1000);
//          passenger_door_latch_close();
//        }
//
//        else if (driver_feedback < 500 && passenger_feedback >500)
//        {
//          Serial.println("Passenger door latch already open, no need to open");
//          driver_door_latch_open();
//          delay(1500);
//          autoopen_both();
//          delay(1000);
//          driver_door_latch_close();          
//        }
//        
//        else if (driver_feedback > 500 && passenger_feedback > 500)
//        {
//          Serial.println("Both door latch already open, no need to open");
//          autoopen_both();
//        }    
//          
//        else
//        {
//          jrk1.stopMotor();
//          jrk2.stopMotor();
//          Serial.println("Full system ERR");
//          uint8_t msg[] = {'E','R','R'}; // Message feedback to phone
//          SerialBT.write(msg, 3); 
//        }              
//      }
      
      //------Manual open driver door control----------------------------------------------------------------------------------
      else if(inData == "Manual open driver\n")
      {
       on_press_driver_open();
      }
      
//---------------Door latch functions---------Uncomment to use-----------------------------------------------------------------      
//      {
//        uint16_t driver_feedback = jrk1.getScaledFeedback();
//        if (driver_feedback < 500)
//        {
//          driver_door_latch_open();
//          delay(1500);
//          
//          delay(2000);
//          driver_door_latch_close();  
//        }
//        else if (driver_feedback > 500)
//        {
//          on_press_driver_open();
//        }
//        else
//        {
//          jrk1.stopMotor();
//          Serial.println("Driver system ERR");
//          uint8_t msg[] = {'E','R','R'}; // Message feedback to phone
//          SerialBT.write(msg, 3); 
//        }
//      }

      //-----Manual close driver door control----------------------------------------------------------------------------------
      else if(inData == "Manual close driver\n")
      {
       on_press_driver_close();
      }
      //-----Manual driver door stop control-----------------------------------------------------------------------------------
      else if (inData == "Manual stop driver\n")
      {
       on_release_driver();
      }
      //-----Manual open passenger door control--------------------------------------------------------------------------------
      else if (inData == "Manual open passenger\n")
      {
       on_press_passenger_open();
      }
      
//---------------Door latch functions---------Uncomment to use---------------------------------------------------------------      
//      {
//        uint16_t passenger_feedback = jrk2.getScaledFeedback();
//        if (passenger_feedback < 500)
//        {
//          passenger_door_latch_open();
//          delay(1500);
//          on_press_passenger_open();
//          delay(2000);
//          passenger_door_latch_close();  
//        }
//        else if (passenger_feedback > 500)
//        {
//          on_press_passenger_open();
//        }
//        else
//        {
//          jrk2.stopMotor();
//          Serial.println("Passenger system ERR");
//          uint8_t msg[] = {'E','R','R'}; // Message feedback to phone
//          SerialBT.write(msg, 3); 
//        }
//      }
        
      //-----Manual close passenger door control-------------------------------------------------------------------------------
      else if (inData == "Manual close passenger\n")
      {
       on_press_passenger_close();
      }
      //-----Manual passenger stop control-------------------------------------------------------------------------------------
      else if (inData == "Manual stop passenger\n")
      {
       on_release_passenger();
      }
      //----Relay to initiate suspension going up motion-----------------------------------------------------------------------
      else if (inData == "Hydraulic up\n")
      {
       hydraulic_up_direction();
      }
      //----Relay to initiate suspension going down motion---------------------------------------------------------------------
       
      else if (inData == "Hydraulic down\n")
      {
       hydraulic_down_direction();
      }
      //----Stop hydraulic suspension movements--------------------------------------------------------------------------------
      else if (inData == "Hydraulic stop\n")
      {
       hydraulic_stop();
      }
      else if (inData == "Door status?\n")
      {
       door_status();
      }
      else if (inData == "System status?\n")
      {
       error_feedback();
      }
      //-----Invalid command---------------------------------------------------------------------------------------------------
      else 
      {
       Serial.println("Command not recoginized\n");
      }
    inData = ""; //Clear received buffer
    }
  }

// Driver switch press logic control-------------------------------------------------------------------------------------------
  int Driver_Switch_State = digitalRead(DriverSwitch);
  if (Driver_Switch_State == HIGH)
  {
   Serial.println("Driver press detected");
   driver_door_switch();
  }
  else 
  {
   //Serial.println("Waiting");
  }
// Passenger switch press logic control----------------------------------------------------------------------------------------
  int Passenger_Switch_State = digitalRead(PassengerSwitch);
  if (Passenger_Switch_State == HIGH) 
  {
   Serial.println("Passenger press detected");
   passenger_door_switch();
  }
  else 
  {
   // Serial.println("Waiting");
  }
}

//--------------------------------------------List of functions for main loop--------------------------------------------------

//--------------------------------------------Door logic controls--------------------------------------------------------------
void autoclose_driver(){ //Command to auto close driver door
jrk1.stopMotor();  
delay(1000);
jrk1.setTarget(Driver_close_target);
feedbackmsg("Auto close driver ok\n");
Serial.println("Command to auto close driver has been initiated");
driver_moving = true;
delay(50);
}

void autoclose_passenger(){ //Command to autoclose passenger door
jrk2.stopMotor();
delay(1000);
jrk2.setTarget(Passenger_close_target);
feedbackmsg("Auto close passenger ok\n");
Serial.println("Command to auto close passenger door has been initiated");
passenger_moving = true;
delay(50);
}

void autoopen_driver(){ //Command to auto open driver door
jrk1.stopMotor();
delay(1000);
jrk1.setTarget(Driver_open_target);
feedbackmsg("Auto open driver ok\n");
Serial.println("Command to auto open driver door has been initiated");
driver_moving = true;
delay(50);
}

void autoopen_passenger(){ //Command to auto open passenger door
jrk2.stopMotor();
delay(1000);
jrk2.setTarget(Passenger_open_target);
feedbackmsg("Auto open passenger ok\n");
Serial.println("Command to auto open passenger door has been initiated");
passenger_moving = true;
delay(50);
}

void autoopen_both(){ //Command to auto open both doors
jrk1.stopMotor();
jrk2.stopMotor();
delay(1000);
jrk1.setTarget(Driver_open_target);
jrk2.setTarget(Passenger_open_target);
feedbackmsg("Auto open both ok\n");
Serial.println("Command to auto open both doors has been initiated");
driver_moving = true;
passenger_moving = true;
delay(50);
}

void autoclose_both(){ //Command to auto close both doors
jrk1.stopMotor();
jrk2.stopMotor();
delay(1000);
jrk1.setTarget(Driver_close_target);
jrk2.setTarget(Passenger_close_target);
feedbackmsg("Auto close both ok\n");
Serial.println("Command to auto close both doors has been initiated");
driver_moving = true;
passenger_moving = true;
delay(50);
}

void on_press_driver_open(){ //Command to open driver door manually
jrk1.setTarget(Driver_open_target);
feedbackmsg("Manual open driver ok\n");
Serial.println("Command to open driver door manually initiated");
driver_moving = true;
delay(50);
}

void on_press_driver_close(){ //Command to close driver door manually
jrk1.setTarget(Driver_close_target);
feedbackmsg("Manual close driver ok\n");
Serial.println("Command to close driver door manually initiated");
driver_moving = true;
delay(50);
}

void on_release_driver(){ //Command to stop the motor when button press is released for Driver
jrk1.stopMotor();
feedbackmsg("Manual stop driver ok\n");
Serial.println("Command to stop driver door manually initiated");
driver_moving = false;
delay(50);
}

void on_press_passenger_open(){ //Command to open passenger door manually
jrk2.setTarget(Passenger_open_target);
feedbackmsg("Manual open passenger ok\n");
Serial.println("Command to opem passenger door manually initiated");
passenger_moving = true;
delay(50);
}

void on_press_passenger_close(){ //Command to close passenger door manually 
jrk2.setTarget(Passenger_close_target);
feedbackmsg("Manual close passenger ok\n");
Serial.println("Command to close passenger door manually initiated");
passenger_moving = true;
delay(50);
}

void on_release_passenger(){ //Command to stop the motor when button press is released for Passenger
jrk2.stopMotor();
feedbackmsg("Manual stop passenger ok\n");
Serial.println("Command to stop passenger door manually initiated");
passenger_moving = false;
delay(50);
}

void driver_door_switch(){ //Command when driver door switch is pressed
 Serial.println(driver_moving);
 if (driver_moving)
 {
  jrk1.stopMotor();
  driver_moving = false;
 }
 else
 { 
  uint16_t driver_feedback = jrk1.getScaledFeedback();
  if (driver_feedback > 3300)
  {
   Serial.println("closing");
   autoclose_driver();
   driver_moving = true;
  }
  
  else if (driver_feedback < 3300)
  {
   Serial.println("opening");
   autoopen_driver();
   driver_moving = true;
  }
 }
}
void passenger_door_switch(){ //Command when passenger door switch is pressed
  if (passenger_moving)
  {
    jrk2.stopMotor();
    passenger_moving = false;
  }
  else{
    uint16_t passenger_feedback = jrk2.getScaledFeedback(); 
    if (passenger_feedback > 3300)
   {
    Serial.println("closing");
    autoclose_passenger();
    passenger_moving = true;
   }
   else if (passenger_feedback < 3300)
   {
    Serial.println("opening");
    autoopen_passenger();
    passenger_moving = true;
   }
  } 
}

//void driver_door_latch_open(){
//  digitalWrite(DriverDoorLatch, LOW);
//  Serial.println("Driver door latch open");
//}
//
//void driver_door_latch_close(){
//  digitalWrite(DriverDoorLatch, HIGH);
//  Serial.println("Driver door latch close");
//}
//
//void passenger_door_latch_open(){
//  digitalWrite(PassengerDoorLatch, LOW);
//  Serial.println("Passenger door latch open");
//}
//
//void passenger_door_latch_close(){
//  digitalWrite(PassengerDoorLatch, HIGH);
//  Serial.println("Passenger door latch close");
//}

//--------------------------------------------Hydraulic Pump Control--------------------------------------------------------------
void hydraulic_up_direction(){ //Hydraulic pump up direction command
  digitalWrite(hydraulic_down_relay, HIGH);
  delay(50);
  digitalWrite(hydraulic_up_relay, LOW);
  feedbackmsg("Hydraulic pump up ok\n");
  Serial.println("Opened pump relay down for safety, now moving UP");
  delay(50); 
}

void hydraulic_down_direction(){ //Hydraulic pump down direction command
  digitalWrite(hydraulic_up_relay, HIGH);
  delay(50);
  digitalWrite(hydraulic_down_relay, LOW);
  feedbackmsg("Hydraulic pump down ok\n");
  Serial.println("Opened pump relay up for safety, now moving DOWN");
  delay(50);
}

void hydraulic_stop(){ //Hydraulic pump stop command
  digitalWrite(hydraulic_up_relay, HIGH);
  digitalWrite(hydraulic_down_relay, HIGH);
  feedbackmsg("Hydraulic pump stop ok\n");
  Serial.println("Stop hydraulic pump initiated");
  delay(50);
}

//--------------------------------------------Door Status Feedback ---------------------------------------------------------------

void door_status(){
  //uint16_t driver_feedback = jrk1.getScaledFeedback();
  float driver_feedback = 1100; // test value
  //uint16_t passenger_feedback = jrk2.getScaledFeedback();
  float passenger_feedback = 2800; // test value
  float driver_max_pos = Driver_open_target;
  float pass_max_pos = Driver_open_target;
  float scaled_driver_max_pos = ((driver_feedback/driver_max_pos) * 100);
  int scaled_driver_max_pos_int = round(scaled_driver_max_pos);
  float scaled_passenger_max_pos = ((passenger_feedback/pass_max_pos) * 100);
  int scaled_passenger_max_pos_int = round(scaled_passenger_max_pos);
  char copy[25];
  String drvr_max_pos;
  drvr_max_pos = String(scaled_driver_max_pos_int); 
  String psngr_max_pos;
  psngr_max_pos = String(scaled_passenger_max_pos_int);
  feedbackmsg("DoorStatus:FL="+drvr_max_pos+','+"FR="+psngr_max_pos+"\n");
}

//--------------------------------------------System Error Feedback --------------------------------------------------------------
void error_feedback() 
{
  int Driver_feedback_state = digitalRead(Driver_error_feedback);
  int Passenger_feedback_state = digitalRead(Passenger_error_feedback);
  if (Driver_feedback_state == HIGH)
  {
    Serial.println("Driver side system error, please check system immidiately");
    feedbackmsg("Driver system error\n");
  }
  else
  {
    Serial.println("Driver side system in good condition");
    feedbackmsg("Driver system okay\n");
  }
  
  if (Passenger_feedback_state == HIGH)
  {
    Serial.println("Passenger side system error, please check system immidiately");
    feedbackmsg("Passenger system error\n");
  }
  else
  {
    Serial.println("Passenger side system in good condition");
    feedbackmsg("Passenger system okay\n");
  }
}

//--------------------------------------------Obstacle Detection control ---------------------------------------------------------

void driver_resistance_detection(){ //Constant check to measure current, auto stop if outside of normal load driver side
  uint16_t driver_current = jrk1.getCurrent();
  //Serial.println(driver_current);
  if (driver_current > 20000) //2A
  {
    jrk1.stopMotor();
    Serial.println("Driver door obstacle detected");
    feedbackmsg("Driver side obstacle detected\n");
  }
  else
  {   
  }
}
void passenger_resistance_detection(){ //Constant check to measure current, auto stop if outside of normal load passenger side
  uint16_t passenger_current = jrk2.getCurrent();
  //Serial.println(passenger_current);
  if (passenger_current > 20000) //4A
  {
    jrk2.stopMotor();
    Serial.println("Passenger door obstacle detected");
    feedbackmsg("Passenger side obstacle detected\n");
  }
  else
  { 
  }
}
//--------------------------------------------Moving Flag Check ------------------------------------------------------------------
void driver_moving_check(){ //Constant flag check for driver switch, will enable to use physical switch as stop button
   uint16_t driver_feedback = jrk1.getScaledFeedback(); 
   if (driver_feedback >= 3300)
   {
    driver_moving = false;
   }
   else if (driver_feedback >= 110 && driver_feedback <= 130)
   {
    driver_moving = false;
   }
}
void passenger_moving_check(){ //Constant flag check for driver switch, will enable to use physical switch as stop button
   uint16_t passenger_feedback = jrk2.getScaledFeedback(); 
   if (passenger_feedback >= 3300)
   {
    passenger_moving = false;
   }
   else if (passenger_feedback >= 110 && passenger_feedback <= 130)
   {
    passenger_moving = false;
   }
}

void feedbackmsg(String mymessage)
{
  char copy[50];
  mymessage.toCharArray(copy,50);
  for (int i = 0; i<mymessage.length(); i++){
    char message = mymessage[i];
    uint8_t message_send[] = {message};
    SerialBT.write(message_send, 1);
  }
}
