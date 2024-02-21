#include "main.h"

#define BTN_PIN 14

static bool smpl_running = false;

static qos qos_mode = MODE_1;

static bool btn_confirm = false;

//Interrupt variables and functions
volatile bool btn_pressed = false;
volatile bool btn_released = false;

volatile int buttonState = LOW;
volatile int LastButtonState = HIGH;

volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 80; //ms

IRAM_ATTR void btn_press(){
  
  // Read the state of the button
  int reading = digitalRead(BTN_PIN);

  // Check if a debounce delay has passed since the last button change
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed
    if (reading != LastButtonState) {
      // Update the last debounce time
      lastDebounceTime = millis();
      
      // Update the button state
      buttonState = reading;

      // Check if the button was pressed or released
      if (buttonState == HIGH) {
        // Button was released
        btn_released = true;
      } else {
        // Button was pressed
        btn_pressed = true;
      }
    }
  }

  // Save the current button state as the last button state
  LastButtonState = reading;  
}

// Helper functions
message decode(uint8_t encoded){
  message data;

  data.cmd = (encoded >> 4) & 0xF;
  data.msg = encoded & 0xF;

  return data;
}

uint8_t encode(uint8_t cmd, uint8_t msg){
  return uint8_t (cmd << 4) | msg;
}


// Main Functions
void setup() {
  Serial.begin(9600);

  pinMode(BTN_PIN, INPUT_PULLUP);
  //Set interrupt for press
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btn_press, CHANGE);

  //Serial.write("device Started");
}

bool commandReceived = false;

void loop() {

  // check if data is in serial buffer
  if (Serial.available() > 0){
    uint8_t encoded = Serial.read();

    // decode message
    message msg = decode(encoded);

    // Filter out command types
    switch (msg.cmd){
      case INFO_CMD:
        //filter message for info
        switch (msg.msg)
        {
        case RDY_MSG:
          // if (!smpl_running) Serial.write(RDY_MSG_RTN);
          if (!smpl_running) Serial.write(encode(msg.cmd, RDY_MSG_RTN));
          break;
        case SMPL_STAT:
          // Serial.write(smpl_running ? SMPL_TRUE : SMPL_FALSE);
          Serial.write(smpl_running ? encode(INFO_CMD, SMPL_TRUE) : encode(INFO_CMD, SMPL_FALSE));
          break;
        case QOS_MSG:
          if (!smpl_running) Serial.write(qos_mode == MODE_1 ? encode(msg.cmd, QOS_MODE_1) : encode(msg.cmd, QOS_MODE_2));
          break;
        }
        break;
      case CTRL_CMD:
        switch (msg.msg)
        {
        case SMPL_START:
          if(!smpl_running){
            smpl_running = true;
            // return message
            Serial.write(encode(msg.cmd, SMPL_TRUE));
          } else {
            Serial.write(encode(msg.cmd,SMPL_FALSE));
          }
          break;

        case SMPL_STOP:
          if (smpl_running){
            smpl_running = false;
            // return message
            Serial.write(encode(msg.cmd, SMPL_TRUE));
          } else {
            Serial.write(encode(msg.cmd,SMPL_FALSE));
          }
          break;
        
        case QOS_MODE_1:
          qos_mode = MODE_1;
          //Return the status
          Serial.write(encode(msg.cmd, QOS_RTN_1));
          break;
        
        case QOS_MODE_2:
          qos_mode = MODE_2;
          //Return the active mode
          Serial.write(encode(msg.cmd, QOS_RTN_2));
          break;
        }
        
        break;
      case BTN_CMD:
        switch (msg.msg)
        {
        case BTN_CNFRM:
          btn_confirm = false;
          Serial.write(encode(msg.cmd, BTN_CNFRM));
          break;
        }
        break;
    }
  }

  // Send data to client over serial
  // check if sampling is running
  if (smpl_running){
    // Check interrupt flags
    // Button pressed message should always be transmitted first if both release and press are true

    //Check if button pressed
    if (btn_pressed){
      Serial.write(encode(BTN_CMD, BTN_RTN_ON));
      //Serial.println("pressed");
      btn_pressed = false;
    }

    //Dont include in else because we need both messages to send one after the other
    if (btn_released){
      Serial.write(encode(BTN_CMD, BTN_RTN_OFF));
      btn_released = false;
    }
  }


}
