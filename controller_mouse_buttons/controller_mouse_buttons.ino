/*
  JoystickMouseControl

  Controls the mouse from a joystick on an Arduino Leonardo, Micro or Due.
  Uses a pushbutton to turn on and off mouse control, and a second pushbutton
  to click the left mouse button.

  Hardware:
  - 2-axis joystick connected to pins A0 and A1
  - pushbuttons connected to pin D2 and D3

  The mouse movement is always relative. This sketch reads two analog inputs
  that range from 0 to 1023 (or less on either end) and translates them into
  ranges of -6 to 6.
  The sketch assumes that the joystick resting values are around the middle of
  the range, but that they vary within a threshold.

  WARNING: When you use the Mouse.move() command, the Arduino takes over your
  mouse! Make sure you have control before you use the command. This sketch
  includes a pushbutton to toggle the mouse control state, so you can turn on
  and off mouse control.

  created 15 Sep 2011
  updated 28 Mar 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/JoystickMouseControl
*/

#include "Mouse.h"

// set pin numbers for switch, joystick axes, and LED:
const int xAxis = A0;         // joystick X axis
const int yAxis = A1;         // joystick Y axis
const int ledPin = 5;         // Mouse control LED

// parameters for reading the joystick:
//int range = 12;               // output range of X or Y movement
int range = 18;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = range / 4;    // resting threshold
int center = range / 2;       // resting position value

bool mouseIsActive = true;    // whether or not to control the mouse
int lastSwitchState = LOW;        // previous switch state


//button and debounce
#define bounce_interval       30
#define right_bounce_interval 30
#define btn_pin               2
#define right_btn_pin         3 

int btn_state;
int btn_read_state;
unsigned long btn_current_action_time;
unsigned long btn_last_action_time;

int right_btn_state;
int right_btn_read_state;
unsigned long right_btn_current_action_time;
unsigned long right_btn_last_action_time;


void setup() 
{
  Serial.begin(9600);
  pinMode(btn_pin, INPUT_PULLUP);
  pinMode(right_btn_pin, INPUT_PULLUP);

  pinMode(ledPin, OUTPUT);         // the LED pin
  // take control of the mouse:
  Mouse.begin();
}

void loop() {
  // read and scale the two axes:
  int xReading = xreadAxis(A0);
  int yReading = yreadAxis(A1);

  // if the mouse control state is active, move the mouse:
  if (mouseIsActive) {
    Mouse.move(xReading, yReading, 0);
  }

  btn_read_state = digitalRead(btn_pin);
  if(btn_read_state != btn_state) {
    btn_current_action_time = millis();
    if(btn_current_action_time - btn_last_action_time > bounce_interval) {
      btn_state = btn_read_state;
      btn_last_action_time = btn_current_action_time;
      if(btn_state == HIGH) {
        Mouse.release(MOUSE_LEFT);
      } else {
        Mouse.press(MOUSE_LEFT);
      }
    }
  }
   right_btn_read_state = digitalRead(right_btn_pin);
   if(right_btn_read_state != right_btn_state) {
     right_btn_current_action_time = millis();
     if(right_btn_current_action_time - right_btn_last_action_time > right_bounce_interval) {
       right_btn_state = right_btn_read_state;
       right_btn_last_action_time = right_btn_current_action_time;
       if(right_btn_state == HIGH) {
         Mouse.release(MOUSE_RIGHT);
       } else {
         Mouse.press(MOUSE_RIGHT);
       }
     }
   }

  delay(responseDelay);
}

/*
  reads an axis (0 or 1 for x or y) and scales the analog input range to a range
  from 0 to <range>
*/

int xreadAxis(int xReading) {
  // read the analog input:
  int reading = analogRead(xReading);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}

int yreadAxis(int yReading) {
  // read the analog input:
  int reading = analogRead(yReading);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 1023,0, 0, range);

  // if the output reading is outside from the rest position threshold, use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}
