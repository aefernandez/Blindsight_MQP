/* POT to LED test -> by Owen Mundy March 11, 2010
   from: http://itp.nyu.edu/physcomp/Labs/AnalogIn
—————————————————————*/
 
int potPin = 0;    // Analog input pin that the potentiometer is attached to
int potValue = 0;  // value read from the pot
int potVal = 0;
int led = 9;      // PWM pin that the LED is on.  n.b. PWM 0 is on digital pin 9
 
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  // declare the led pin as an output:
  pinMode(led, OUTPUT);
//  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
//  TCCR0B = _BV(CS00);
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
}
 
void loop() {
  potVal = analogRead(potPin); // read the pot value
  potValue = map(potVal, 0, 680, 0, 255);
  if(potValue>255){ 
    potValue = 255;
  }
  analogWrite(led, potValue);  // PWM the LED with the pot value (divided by 4 to fit in a byte)
  Serial.println(potValue);      // print the pot value back to the debugger pane
  Serial.print("  ");
  Serial.print(potVal);
  Serial.print("  ");
  //delay(10);                     // wait 10 milliseconds before the next loop
  
}
