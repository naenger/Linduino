/*
  Multiple tone player
 
 Plays multiple tones on multiple pins in sequence
 
 circuit:
 * 3 8-ohm speaker on digital pins 6, 7, and 8
 
 created 8 March 2010
 by Tom Igoe
 based on a snippet from Greg Borenstein

This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/Tone4
 
 */

//int tones[] = {262,294,330,349,392,440,494,523};
//int times[] = {200,200,200,200,200,200,200,200};
int numtones = 3;
int tones[] = {116,100,80,349,392,440,494,523};
int times[] = {600,600,1800};
int ptr = 0;

void setup() {
  noTone(8);           
  noTone(6);
  noTone(7);

}

void loop() {
  // play a note on pin 8
  tone(8, tones[ptr]);
  delay(times[ptr]);
  noTone(8);           
  delay(100);

  ptr++;
  ptr = (ptr >= numtones) ? 0 : ptr;
}
