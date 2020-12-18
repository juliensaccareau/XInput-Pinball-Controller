#include <XInput.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_MMA8451.h"
MMA8451 mma8451;

// Rumble
int i_pin_rumble_1 = 16;
int i_pin_rumble_2 = 10;
uint8_t rumbleValue;
static float xyz_g[3];
int x;
int y;

// Nudge
int i_debounce = 180;
int i_nudge_ms = 50;
int i_delta = 0;
int reset = 1;
unsigned long ul_t1;
unsigned long ul_millis;
bool b_nudge_started = false;
bool b_nudge_ended = false;
unsigned long ul_t_nudge_started;
int i_max_nudge = 0;
int i_contrained;
int i_level;
int i_increment;
bool b_nudge_left = false;
bool b_nudge_right = false;
bool b_nudge_bottom = false;
int i_value_accelerometer;
const int ci_min_nudge = 30;

// Plunger
int i_pin_pot_linear = A2;
int i_plunger;
unsigned long ul_t_plunger_deactivated;
bool b_plunger_active = false;
bool b_nudge_active = true;

// Flips
const int DIN_PIN_RIGHT_FLIP = 7;
const int DOUT_RIGHT_RELAY = 6;
const int DIN_PIN_LEFT_FLIP = 5;
const int DOUT_LEFT_RELAY = 4;

int i_right_flip;
int i_left_flip;
int i_prev_right_flip;
int i_prev_left_flip;
bool b_reset = false;
int i_debounce_solenoid = 370;
unsigned long ul_t_relay_activated;

// Buttons
const int DIN_PIN_LEFT_BUTTON = 14;
const int DIN_PIN_RIGHT_BUTTON = 15;
int i_left_button;
int i_right_button;



void setup() {
  XInput.setReceiveCallback(rumbleCallback);
  XInput.begin();
  mma8451.initialize();
  pinMode(DIN_PIN_RIGHT_FLIP, INPUT_PULLUP);
  pinMode(DIN_PIN_LEFT_BUTTON, INPUT_PULLUP);
  pinMode(DIN_PIN_RIGHT_BUTTON, INPUT_PULLUP);
  
  pinMode(DOUT_RIGHT_RELAY, OUTPUT);

  pinMode(i_pin_rumble_1, OUTPUT);
  pinMode(i_pin_rumble_2, OUTPUT);
  
  pinMode(DIN_PIN_LEFT_FLIP, INPUT_PULLUP);
  pinMode(DOUT_LEFT_RELAY, OUTPUT);

  XInput.setTriggerRange(0, 1023);
  XInput.setJoystickRange(0, 1023);
  
  Serial.begin(9600); 
}

void rumbleCallback(uint8_t packetType) {
  // If we have an LED packet (0x01), do nothing
  if (packetType == (uint8_t) XInputReceiveType::LEDs) {
    return;
  }else if (packetType == (uint8_t) XInputReceiveType::Rumble) {
   rumbleValue = XInput.getRumbleLeft() | XInput.getRumbleRight();
   digitalWrite(i_pin_rumble_1, rumbleValue);
   digitalWrite(i_pin_rumble_2, LOW); 
  }
}

void loop() {
 // Plunger
 i_plunger = map(analogRead(i_pin_pot_linear), 0, 880, 0, 512);
 XInput.setJoystick(JOY_RIGHT, 0, 512-i_plunger);

 // Prevent nudge false positive while plunger is bouncing
 if ((i_plunger > 100) && !b_plunger_active){
  b_plunger_active = true;
  b_nudge_active = false;
 }

 // Store time when pluger is released
 if ((i_plunger < 100) && b_plunger_active){
  ul_t_plunger_deactivated = millis();
  b_plunger_active = false;
 }

 // Reactivation nudge afer 100 ms last plunger action
 unsigned long ul_elapsed_time = millis() - ul_t_plunger_deactivated;
 if ((ul_elapsed_time > 100) && !b_plunger_active){
  b_nudge_active = true;
 }

 // Right flip
 i_right_flip = !digitalRead(DIN_PIN_RIGHT_FLIP);
 XInput.setButton(TRIGGER_RIGHT, i_right_flip);
 if (!i_prev_right_flip && i_right_flip){
   digitalWrite(DOUT_RIGHT_RELAY, 1);
   ul_t_relay_activated = millis();
 }

 // Left flip
 i_left_flip = !digitalRead(DIN_PIN_LEFT_FLIP);
 XInput.setButton(TRIGGER_LEFT, i_left_flip);
 if (!i_prev_left_flip && i_left_flip){
   digitalWrite(DOUT_LEFT_RELAY, 1);
   ul_t_relay_activated = millis();
 }

 // Deactivate nudge during coil activation to avoid false positive nudging
 if (millis() - ul_t_relay_activated < 100){
  b_nudge_active = false;
 }

 // Deactivate relay after 10 ms, avoid stick of coils 
 if (millis() - ul_t_relay_activated > 10){
   digitalWrite(DOUT_RIGHT_RELAY, 0);
   digitalWrite(DOUT_LEFT_RELAY, 0);
 }

 // Left button
 i_left_button = !digitalRead(DIN_PIN_LEFT_BUTTON);
 XInput.setButton(BUTTON_LB, i_left_button);

 // Right button
 i_right_button = !digitalRead(DIN_PIN_RIGHT_BUTTON);
 XInput.setButton(BUTTON_RB, i_right_button);

 // Store state of pins to activate coils on rising edge
 i_prev_right_flip = i_right_flip;
 i_prev_left_flip = i_left_flip;
 
 // Nudge
 if (b_nudge_active){
   nudge();
 }
}

void nudge(){
  mma8451.getMeasurement(xyz_g);
  x = xyz_g[0]*100;
  y = xyz_g[1]*100;
   
  // Calculate nudge direction
  if (((abs(x) > ci_min_nudge) || (y > ci_min_nudge)) && !b_nudge_started){
    ul_t_nudge_started = millis();
    if (abs(x) > 30){
      if (x > 0){
        b_nudge_left = true;
        i_max_nudge = x;
      }else{
        b_nudge_right = true;
        i_max_nudge = abs(x);
      }
    }else{
      b_nudge_bottom= true;
      i_max_nudge = y;
    }
    b_nudge_started = true;
  }
  
  // Check 30 first values from accelerometer to find the highest absolute value depending nudge direction
  if (b_nudge_started){
    if (millis()-ul_t_nudge_started < 30){
      if (b_nudge_bottom){
        i_value_accelerometer = xyz_g[1]*100;
      }else{
        i_value_accelerometer = abs(xyz_g[0]*100);
      }
      
      i_max_nudge = max(i_value_accelerometer, i_max_nudge);
    }else{
      b_nudge_ended = true;
      b_nudge_started = false;
    }
  }

  // Send XInput control to simulate nudge
  if (b_nudge_ended){
    // Tune values with empirism
    i_contrained = constrain(i_max_nudge,ci_min_nudge,100);
    i_level = map(i_contrained, 20, 100, 100, 512);
    i_increment = i_level / 10;

    if (b_nudge_left){
      for(int i = 0; i < i_level; i+=i_increment){
        XInput.setJoystick(JOY_LEFT,512 + i, 512);
      }
    
      for(int i = i_level; i > 0 ; i=i-i_increment){
        XInput.setJoystick(JOY_LEFT,512 + i, 512);
      }
      
      b_nudge_left = false;
    }else if (b_nudge_right){
      for(int i = 0; i <i_level; i+=i_increment){
        XInput.setJoystick(JOY_LEFT,512 - i, 512);
      }
    
      for(int i = i_level; i > 0 ; i=i-i_increment){
        XInput.setJoystick(JOY_LEFT,512 - i, 512);
      }
      
      b_nudge_right = false;
    }else{
      for(int i = 0; i <i_level; i+=i_increment){
        XInput.setJoystick(JOY_LEFT,512, 512 + i);
      }
    
      for(int i = i_level; i > 0 ; i=i-i_increment){
        XInput.setJoystick(JOY_LEFT,512, 512 + i);
      }
      
      b_nudge_bottom = false;
    }
    
    b_nudge_ended = false;
  }
}
