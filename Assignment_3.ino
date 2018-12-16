// Edited from public code by James Reeve, 2018 

#include <TimerOne.h>
#include "engine.h" //Several librarys were required for this code, which I found and implemented.
arp a(C, 5, 2, 6, 200, c_harmonic, 0); //
bool button_pressed;
int ButtonVal;

#define baseNotepin 7
#define baseOctavepin 5
#define octaveShiftpin 1
#define stepspin 6
#define indelaypin 2
#define orderpin 0
#define modepin 4
#define syncinpin 3 //I reordered these to fit with my breadboard layout.

#define LEDPin 13

// Synchronization: choose one of two possible options:
//#define EXT_SYNC
#define INT_SYNC

void readPoties()
{
  unsigned i;
  a.setupArp(analogRead(baseNotepin), analogRead(baseOctavepin), analogRead(octaveShiftpin), analogRead(stepspin), analogRead(indelaypin), analogRead(orderpin), analogRead(modepin));
  
  // In my setup the buttons are connected to pins 6..12
  for (i=12;i>5;i--)
    if (!(digitalRead(i))) { button_pressed = true; ButtonVal = 13-i; return; }
}

void setup()
{
  a.midibegin();
  Timer1.initialize(200000);          // initialize timer1, and set a 1/10 second period
  Timer1.attachInterrupt(readPoties); // We will read potis and buttons values within timer interrupt
  
  // LED pin
  pinMode(LEDPin, OUTPUT);
  
  // Initialize pins for 2-pins pushbuttons with pullup enabled
  for (unsigned i=6;i<13;i++)
  {
    pinMode(i,INPUT_PULLUP);
    //pinMode(i, INPUT);
    //digitalWrite(i, 1);
  }
  button_pressed = false;
  ButtonVal = 1;
}

void loop()
{
    if (button_pressed)
    {
      a.setProgression(ButtonVal-1);
      button_pressed = false;
      
      // Switch on LED
      digitalWrite(LEDPin, HIGH);
      a.play();
      
      // Switch off LED
      digitalWrite(LEDPin, LOW);
    }
}
