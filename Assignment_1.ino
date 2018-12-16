// Edited from public code by James Reeve, 2018 
const int val00 = A0;  // Potentiometer 1 - Harmonicity
const int val01 = A1;  // Potentiometer 2 - Duration
const int buttonPin = 2;     
int buttonState = 0;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.print(analogRead(val00));
  Serial.print(",");
  Serial.print(analogRead(val01));
  Serial.print(",");
  Serial.println(digitalRead(buttonState));
}
