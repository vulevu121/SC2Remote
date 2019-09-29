void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() >0) {
    String input = Serial.readString();
    //String acp = Serial.readString();
    //String aod = Serial.readString();
    //String aop = Serial.readString();
    
    if(input == "Auto close driver\n"){
      autoclose_driver();   
    }
    else if(input == "Auto close passenger\n") {
      autoclose_passenger();
    }
    else if(input == "Auto open driver\n"){
      autoopen_driver();
    }
    else if(input == "Auto open passenger\n"){
      autoopen_passenger();
    }

    else {
      Serial.println("Please input valid command");
    }
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
