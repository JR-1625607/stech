// Arduino step sequencer / synthesizer / noise box
// Copyright 2009 mechomaniac.com
// uses code from Atmega SID http://www.roboterclub-freiburg.de/atmega_sound/atmegaSID.html
// and Arduino Punk Console http://www.beavisaudio.com/projects/digital/ArduinoPunkConsole/

// Found and edited from mechomaniac.com orginal code by James Reeve, 2018. 
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
 
// Map all the input and output pins
#define AnalogInFrequency 1
#define AnalogInTempo 2
#define AnalogInDuration 0
#define DigitalOutSignal 11
#define DigitalInSave 10
#define DigitalInNext 13
#define DigitalInStartStop 12
#define DigitalOutLEDStart 2
 
#define MaxSteps 4 // should be a multiple of 8
#define MaxDuration 2000;
#define MinFrequency 200;
#define FrequencyMultiplier 3;
#define MinTempo 20;
 
#define INTERRUPT_PERIOD 512
#define FINT (F_CPU / INTERRUPT_PERIOD) // 16kHz?
#define FS (FINT)
 
// sine lookup table pre-calculated

// Below used to read prog_uchar PROGMEM due to the code being from an older version.
// I changed it to const unsigned char PROGMEM and fixed the probelm.
const unsigned char PROGMEM sinetable[256] = { // http://en.wikipedia.org/wiki/Lookup_table
        128,131,134,137,140,143,146,149,152,156,159,162,165,168,171,174,
        176,179,182,185,188,191,193,196,199,201,204,206,209,211,213,216,
        218,220,222,224,226,228,230,232,234,236,237,239,240,242,243,245,
        246,247,248,249,250,251,252,252,253,254,254,255,255,255,255,255,
        255,255,255,255,255,255,254,254,253,252,252,251,250,249,248,247,
        246,245,243,242,240,239,237,236,234,232,230,228,226,224,222,220,
        218,216,213,211,209,206,204,201,199,196,193,191,188,185,182,179,
        176,174,171,168,165,162,159,156,152,149,146,143,140,137,134,131,
        128,124,121,118,115,112,109,106,103,99, 96, 93, 90, 87, 84, 81, 
        79, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 
        37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10, 
        9,  8,  7,  6,  5,  4,  3,  3,  2,  1,  1,  0,  0,  0,  0,  0,  
        0,  0,  0,  0,  0,  0,  1,  1,  2,  3,  3,  4,  5,  6,  7,  8,  
        9,  10, 12, 13, 15, 16, 18, 19, 21, 23, 25, 27, 29, 31, 33, 35, 
        37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 
        79, 81, 84, 87, 90, 93, 96, 99, 103,106,109,112,115,118,121,124
};
 
// sequence arrays
int stepFrequencies[MaxSteps];
int stepDurations[MaxSteps];
 
// lookup table for output waveform
unsigned char wavetable[256];
 
unsigned int frequencyCoef = 100;
bool soundEnabled = true;
bool soundPWM = true; //start with square wave
unsigned int waveform = 0;
bool SoundOn = false;
 
// misc housekeeping
int pitchval = 1;
bool playing = true;
int lastPushedStep = -1;
int step = 0;
int direction = 1;
 
// button states
long playButtonTime = 0; // the last time the output pin was toggled
long debounce = 200; // the debounce time, increase if the output flickers
long buttonRepeatTime = 500;
int playButtonPrevReading = HIGH;
long nextButtonTime = 0;
int nextButtonPrevReading = HIGH;
long saveButtonTime = 0;
int saveButtonPrevReading = HIGH;
 
bool pressedPlay = false;
bool pressedNext = false;
bool pressedSave = false;
 
// Initialize the tempo
int tempo = 100;
 
// This is called at sampling freq to output 8-bit samples to PWM
ISR(TIMER1_COMPA_vect)
{
    static unsigned int phase0;
    static unsigned int sig0;
    static unsigned char flag = 0;
    static unsigned int tempphase;
 
    if (soundPWM)
    {
      tempphase = phase0 + frequencyCoef;
      sig0 = wavetable[phase0>>8];
      phase0 = tempphase;
      OCR2A = sig0; // output the sample
    } else { //square wave 
      flag ^= 1;
      digitalWrite(DigitalOutSignal, flag);
    }
}      
 
void SetupSquareSound()
{
  soundPWM = false;
}
 
void SetupPWMSound()
{
    // Set up Timer 2 to do pulse width modulation on the speaker pin.
    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));
    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);
    // Do non-inverting PWM on pin OC2A (p.155)
    // On the Arduino this is pin 11.
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
    TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
    // No prescaler (p.158)
    TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
    // Set initial pulse width to the first sample.
    OCR2A = 0;
    // Set up Timer 1 to send a sample every interrupt.
    cli();
    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = INTERRUPT_PERIOD;
    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);
    sei();
    soundPWM = true;
}
 
void StartSound()
{
  // Enable interrupt when TCNT1 == OCR1A (p.136)  
  cli();
  TIMSK1 |= _BV(OCIE1A);
  sei();
  SoundOn = true;
} 
 
void StopSound()
{
  cli();  
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~_BV(OCIE1A);
  sei();
  SoundOn = false;
}
 
void SetFrequency(unsigned int freq)
{
  if (soundPWM) {
    unsigned long templong = freq;
    frequencyCoef = templong * 65536 / FS;
  } else {
    unsigned long periode = F_CPU/(2*freq); //multiply by 2, because its only toggled once per cycle
    cli();
    OCR1A = periode;
  }
}
 
void SineWave()
{
   for (int i = 0; i < 256; ++i) {
      wavetable[i] = pgm_read_byte_near(sinetable + i);
   }
}
 
void SawtoothWave()
{
   for (int i = 0; i < 256; ++i) {
      wavetable[i] = i; // sawtooth
   }
}
 
void TriangleWave()
{
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = value;
    value -= 2;
  }
}
 
void setup()
{ 
  pinMode (DigitalInStartStop, INPUT);
  pinMode (DigitalInNext, INPUT);
  pinMode (DigitalInSave, INPUT);
  pinMode (DigitalOutSignal, OUTPUT);  
  for (int i = 0; i < 8; ++i)
  {
    pinMode (DigitalOutLEDStart + i, OUTPUT);
  }
  RandomSteps();
  SineWave();
  SetupPWMSound();
  waveform = random(4);
  SelectWaveform();
} 
 
void RandomSteps()
{
  randomSeed(analogRead(4));  
  // sometimes backwards direction
  int j = random(4);
  if (j == 1) {
    direction = -1;
  }
  for (int i = 0; i < MaxSteps; ++i) {
    stepFrequencies[i] = random(1024) * FrequencyMultiplier + MinFrequency;  
    stepDurations[i] = random(1024);
  }
}
 
void IncrementStep()
{
  step += direction;
  if (step >= MaxSteps) {
    step = 0;
  } else if (step < 0) {
    step = MaxSteps - 1;
  }
}
 
void CheckButtons()
{
  pressedPlay = false;
  pressedNext = false;
  pressedSave = false;
 
  long currentTime = millis();
  int reading = digitalRead (DigitalInStartStop);
  if (reading == LOW && playButtonPrevReading == HIGH && currentTime - playButtonTime > debounce) {
    pressedPlay = true;
    playButtonTime = currentTime;
  }
  playButtonPrevReading = reading;  
 
  reading = digitalRead (DigitalInNext);
  if (reading == LOW && ((nextButtonPrevReading == HIGH && currentTime - nextButtonTime > debounce) || (nextButtonPrevReading == LOW && currentTime - nextButtonTime > buttonRepeatTime))) {
    pressedNext = true;
    nextButtonTime = currentTime;
  }
  nextButtonPrevReading = reading;
 
  reading = digitalRead (DigitalInSave);
  if (reading == LOW && ((saveButtonPrevReading == HIGH && currentTime - saveButtonTime > debounce) || (saveButtonPrevReading == LOW && currentTime - saveButtonTime > buttonRepeatTime))){
    pressedSave = true;
    saveButtonTime = currentTime;
  }
  saveButtonPrevReading = reading;  
}
 
void SelectWaveform()
{
  if (waveform > 3) {
    waveform = 0;
  }
  if (waveform == 0) { // square wave has no PWM
    soundPWM = false;
  } else {
    soundPWM = true;
    SetupPWMSound();
    if (waveform == 1) {
      SineWave();
    } else if (waveform == 2) {
      SawtoothWave();
    } else {
      TriangleWave();
    }
  }
}
 
void loop() 
{
  // Main sequence loop    
 
  // show which step we're on 
  int iCurrentLED = DigitalOutLEDStart + (step % 8);
  digitalWrite (iCurrentLED, HIGH); 
 
  readPots();
  CheckButtons();    
 
  if (pressedPlay) {
    playing = !playing;
  }
  if (playing) { // playing 
    if (pressedNext) { //change sound
      waveform++;
      SelectWaveform();
      for (int i = 0; i < 8; ++i) {
        digitalWrite (DigitalOutLEDStart + i, HIGH);
      }
    } else if (pressedSave) {
      Siren();
    } 
 
    long noteDuration = (long)stepDurations[step] * (long)tempo / 1024;
    freqout (stepFrequencies[step], noteDuration);
    // Pause between steps
    long notePause = (long)tempo - noteDuration;
    delay (notePause); 
    digitalWrite (iCurrentLED, LOW);
    IncrementStep();
  } else { // paused so we can edit the sequence
    if (pressedNext) {
        IncrementStep();
        digitalWrite (iCurrentLED, LOW);
        iCurrentLED = DigitalOutLEDStart + (step % 8);
        digitalWrite (iCurrentLED, HIGH); 
        long noteDuration = (long)stepDurations[step] * (long)tempo / 1024;
        freqout (stepFrequencies[step], noteDuration);
    }
    if (pressedSave) {
        stepDurations[step] = analogRead (AnalogInDuration);  
        stepFrequencies[step] = analogRead (AnalogInFrequency) * FrequencyMultiplier + MinFrequency;  
        long noteDuration = (long)stepDurations[step] * (long)tempo / 1024;
        freqout (stepFrequencies[step], noteDuration);
    }
 }
} 
 
// Read the current values of the pots, called from the loop.
void readPots ()
{
    tempo = (analogRead (AnalogInTempo) * 2) + MinTempo;     
}
 
void LEDsOff()
{
  for (int i=0; i < 8; ++i) {
    digitalWrite (DigitalOutLEDStart + i, LOW);
  }
}
 
void LEDsOn()
{
  for (int i=0; i < 8; ++i) {
    digitalWrite (DigitalOutLEDStart + i, HIGH);
  }
}
 
void Siren()
{
  StartSound();
  int dir = 1;
  int freq = 200;
  do {
    int s = analogRead(AnalogInDuration) / 5;
    freq += s * dir;
    SetFrequency(freq);
    if (freq > (500 + analogRead(AnalogInFrequency) * 4) || freq < 200) {
      digitalWrite (DigitalOutLEDStart + random(8), HIGH);
      digitalWrite (DigitalOutLEDStart + random(8), LOW);
      dir *= -1;
    }
    delay(5);
    CheckButtons();
    if (pressedNext || pressedPlay) {
      freq = 200 + random(4000);
      LEDsOff();
    }
  } while (!pressedSave);
 
  freq = 4000;
  LEDsOff();
  int ledOn = 0;
  do {
    int s = analogRead(AnalogInDuration) / 5;
    freq -= s;
    SetFrequency(freq);
    if (freq < 200) {
        freq = 500 + analogRead(AnalogInFrequency) * 4;
        digitalWrite (DigitalOutLEDStart + ledOn, LOW);
        ledOn = random(8);
        digitalWrite (DigitalOutLEDStart + ledOn, HIGH);
    }
    delay(5);
    CheckButtons();
    if (pressedNext || pressedPlay) {
      freq = 200 + random(4000);
      LEDsOn();
    }
  } while (!pressedSave);
    LEDsOff();
    ledOn = 0;
  do {
    int dur = random(analogRead(AnalogInDuration));
    int freq = random(analogRead(AnalogInFrequency) * 4);
    SetFrequency(freq);
    digitalWrite (DigitalOutLEDStart + ledOn, LOW);
    ledOn = freq / 500;
    digitalWrite (DigitalOutLEDStart + ledOn, HIGH);
    delay(dur);
    CheckButtons();
  } while (!pressedSave && !pressedNext && !pressedPlay);
  StopSound();
 
}
 
void freqout(int freq, int t) 
{     
    SetFrequency(freq);
    StartSound();
    delay(t);
    StopSound();
}
