/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Button
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;         // the number of the pushbutton pin
const int ledPin = LED_BUILTIN;  // the number of the LED pin
  // LED_BUILTIN is set to the correct LED pin independent of which board is used
long presses = -1;
// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  digitalWrite(ledPin, LOW);
}

void loop() {
  // read the state of the pushbutton value:
  readButton();

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    presses++;
    Serial1.println(presses);
    Serial.println(presses);
    delay(100);
    while (buttonState == HIGH)
    {
        readButton();
        if (buttonState == LOW)
        {
          delay(200); // Prevent the button being pressed more often than is reasonable - as might occur by accident
        }
    }
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
  delay(100);
}

void readButton()
{
  buttonState = digitalRead(buttonPin);
}
