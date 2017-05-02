// 1.3 uploaded on Sep 13 2016 :: Updated to add frequency
// 2.0 uploaded on Sep 21 2016 :: Updated to add repeater
// 2.1 uploaded on Sep 22 2016 :: Updated to use only one sensor
// 2.2 uploaded on Feb 03 2017 :: Update to mysensors 2.1.1
// 3.0 uploaded on Feb 18 2017 :: Full gate control
// 3.1 uploaded on Mar 01 2017 :: Ready for testing
// 3.2 uploaded on Apr 25 2017 :: Add work mobile, add reason sensor
#define TINY_GSM_MODEM_A6
#define RF69_IRQ_NUM          4
#define MY_NODE_ID 2
#define MY_DEBUG
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY   RF69_868MHZ
#include <SPI.h>
#include <MySensors.h>
#include <TinyGsmClient.h>
#define EI_ARDUINO_INTERRUPTED_PIN
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#define SerialAT Serial2
#define SerialMon Serial
#define DELAY 1000
/////////////////// Gates ///////////////////////
#define LGC  9
#define RGO  10
#define LGO  11
#define RGC  12
//////////////////// Buttons ///////////////////
#define KEYPAD      A15
#define INTERCOM    A14
#define PHOTOCELLS  A13
#define KEYFOB      A12
#define WALLSWITCH  A11
#define REEDSWITCH  A10

TinyGsm modem(SerialAT);
MyMessage msg(1,V_TRIPPED);
MyMessage msg2(2,V_TEXT);
unsigned long const gatesTimeOpenClose = 28000;
unsigned long const gatesStayOpen = 90000;
unsigned long movingTime = 0, openTime = 0;

volatile bool keyPad = false, interCom = false, keyFob = false, reedSwitch = false;
bool gatesOpening = false, gatesClosing = false, gatesOpen = false, stringComplete = false, domoTrigger = false;
String inputString = "";
String reason = "";
int oldsec=0;

void PBChange() {
  volatile bool startup = true;
  if (startup && millis() < 7000) {
    startup = false;
    return;
  }
  //volatile static unsigned long last_interrupt_time = 0;
  //unsigned long interrupt_time = millis();
  //if (interrupt_time - last_interrupt_time > 50UL)  { // ignores interupts for 50milliseconds
    switch(arduinoInterruptedPin) {
      case KEYPAD:
        keyPad = true;
        return;
      case INTERCOM:
        interCom = true;
        return;
      case KEYFOB:
        keyFob = true;
        return;
      case REEDSWITCH:
        reedSwitch = true;
        return;
    }
  //}
  //last_interrupt_time = interrupt_time;
}

void openGates() {
  if (gatesOpening) {
    return;
  }
  if (gatesClosing) {
    unsigned long temp = millis() - movingTime; // temp is ms since gate was closing
    movingTime = millis() - (gatesTimeOpenClose - temp) + 1000;
    Serial.print("Gates were closing for ");
    Serial.print(temp/1000);
    Serial.println(" secs - reopening");
    stopGates();
  } else {
    movingTime = millis();
  }
  if (gatesOpen) { // Reset open timer
    Serial.println("Resetting timer");
    openTime = millis();
    return;
  }
  gatesOpening = true;
  send(msg2.set(reason.c_str()));
  reason="";
  Serial.println("Opening gate 1");
  digitalWrite(RGO,HIGH);
  delay(DELAY);
  Serial.println("Opening gate 2");
  digitalWrite(LGO,HIGH);
}
void stopGates() {
  Serial.println("Stopping gate 1");
  digitalWrite(LGC,LOW);
  digitalWrite(LGO,LOW);
  if (gatesClosing) {
    delay(1000);
  }
  Serial.println("Stopping gate 2");
  digitalWrite(RGC,LOW);
  digitalWrite(RGO,LOW);
  gatesOpen = gatesOpening?true:false;
  if (gatesOpen) {
    Serial.println("Gates are open");
    openTime = millis();
  }
  gatesOpening = false;
  gatesClosing = false;
}

void closeGates() {
  unsigned long wallOpen = millis();
  while(analogRead(WALLSWITCH) < 615) {
     if(millis() % 10000 == 0) {
       int sec = (millis() - wallOpen)/1000;
       if (sec != oldsec) {
        Serial.print("Wall switch engaged - leaving open (");
        Serial.print(sec);
        Serial.println(" secs)");
        oldsec = sec;
       }
     }
  }
  if (gatesClosing) {
    return;
  }
  movingTime = millis();
  gatesClosing = true;
  gatesOpen = false;
  Serial.println("Closing gate 1");
  digitalWrite(LGC,HIGH);
  delay(DELAY);
  Serial.println("Closing gate 2");
  digitalWrite(RGC,HIGH);
}

void CheckGates() {
  if (gatesOpening || gatesClosing) {
    if((millis() - movingTime) > gatesTimeOpenClose) {
      stopGates();
    } else {
      if (millis() % 2000 == 0)  {
        int secs = (millis() - movingTime) / 1000;
        if (secs != oldsec) {
          if (gatesOpening) {
            Serial.print("Opening : ");
          } else {
            Serial.print("Closing : ");
          }
          Serial.print((movingTime + gatesTimeOpenClose - millis()) / 1000);
          Serial.println(" seconds remaining");
          oldsec = secs;
        }
      }
    }
  }
  if (gatesClosing) {
    if (digitalRead(PHOTOCELLS) == HIGH) {
         Serial.print("Photocells have tripped");
         openGates();
    }
    if (digitalRead(REEDSWITCH) == LOW) {
       Serial.println("Gate close detected");
       stopGates();
    }
  }
  if (gatesOpen) {
    if((millis() - openTime) > gatesStayOpen) {
      closeGates();
    } else {
      if (millis() % 5000 == 0)  {
        int secs = (millis() - openTime) / 1000;
        if (secs != oldsec) {
          Serial.print("Opened for ");
          Serial.print(secs);
          Serial.print(" secs : Remaining : ");
          Serial.println((openTime + gatesStayOpen - millis()) / 1000);
          oldsec = secs;
        }
      }
    }
  }
}

void setup()  {
  Serial.println("Gate sketch started");
  inputString.reserve(200);
  digitalWrite(15,HIGH); // Power on for GSM
  SerialAT.begin(115200);
  SerialAT.write("AT+CLIP=1\r\n");
  for (int x=REEDSWITCH;x<=KEYPAD;x++) {
    if (x == WALLSWITCH) {
      pinMode(x, INPUT);
    } else {
      pinMode(x, INPUT_PULLUP);
    }
    if (x != PHOTOCELLS || x != WALLSWITCH) {
      enableInterrupt(x, PBChange, CHANGE);
    }
  }
  for (int x=9;x<13;x++) {
    pinMode(x,OUTPUT);
    digitalWrite(x,LOW);
  }
  pinMode(3,OUTPUT); // For the GSM power
  digitalWrite(3,HIGH);
  gatesOpen = (digitalRead(REEDSWITCH))==LOW?false:true;
  if (gatesOpen) {
    Serial.println("Detected gates open at start - closing");
    closeGates();
  }
}

void presentation() {
  sendSketchInfo("Front Gates", "3.2");
  present(1, S_DOOR, "Front Gates");
  present(2, S_INFO, "Gate Open Reason");
}

void loop() {
  if (keyFob) {
    Serial.println("Gate open request detected by Key Fob");
    reason="Opened by Key Fob";
    openGates();
    keyFob = false;
  }
  if (interCom) {
    Serial.println("Gate open request detected by Intercom");
    reason="Opened by Intercom";
    openGates();
    interCom = false;
  }
  if (keyPad) {
    Serial.println("Gate open request detected by key pad");
    reason="Opened by Key Pad";
    openGates();
    keyPad = false;
  }
  if(reedSwitch) {
    if (digitalRead(REEDSWITCH) == LOW) {
      Serial.println("Gate close detected");
      send(msg.set(1));
    } else {
      Serial.println("Gate open detected");
      send(msg.set(0));
    }
    reedSwitch = false;
  }
  if (stringComplete) {
    inputString.trim();
    if (inputString.length() > 0) {
      Serial.print("GSM DEBUG : ");
      Serial.println(inputString);
    }
    if(inputString.indexOf("0872241456") > 0 || inputString.indexOf("0876522241") > 0 || inputString.indexOf("0876255412") > 0) {
      String phoneNum = inputString.substring(inputString.indexOf("087"),inputString.indexOf("087")+10);
      Serial.print(phoneNum);
      Serial.println(" CALLING - Opening the gate");
      SerialAT.write("ATH \n\r");
      reason="Opened by " + phoneNum;
      openGates();
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  CheckGates();
}

void receive(const MyMessage &message) {
  if (message.type == V_STATUS && message.sensor == 1) {
     if (atoi(message.data) == 0) {
        Serial.println("Open requested from domoticz");
        reason = "Opened from domoticz";
        openGates();
     }
  }
}

void serialEvent2() {
  while (SerialAT.available()) {
    char inChar = (char)SerialAT.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
