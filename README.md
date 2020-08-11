#include <SoftwareSerial.h> // Allows communication with Bluetooth
#include <HCSR04.h>
#include <L298N.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    5

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 42
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial mySerial(0, 1); // RX, TX
int data;


int inputPin = 10;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status

const unsigned int IN1 = 7;
const unsigned int IN2 = 8;
const unsigned int EN = 6;
const unsigned int IN3 = 9;
const unsigned int IN4 = 4;
const unsigned int ENB = 3;
const unsigned int trigPin = 13;
const unsigned int echoPin = 12;

int mode = 0; //mode 0 is bluetooth mode, mode 1 is obstacle-avoiding mode
boolean hasFlashed = false;

UltraSonicDistanceSensor distanceSensor(13, 12);  // Initialize sensor that uses digital pins 13 and 12.

L298N motor(EN, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

Servo servo1;

void setup() {
  // Open serial communications and wait for port to open:
  mySerial.begin(9600);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Initiated");
  motor.setSpeed(80); //set the speed of the motors, between 0-255
  motor2.setSpeed (81);
  pinMode(trigPin, OUTPUT);// set the trig pin to output (Send sound waves)
  pinMode(echoPin, INPUT);// set the echo pin to input (recieve sound waves)
  servo1.attach(11);
  servo1.write(115);
  pinMode(2, OUTPUT);
  pinMode(inputPin, INPUT);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();           // Turn OFF all pixels ASAP
  //strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}
void loop() {
  Serial.println ("Work already");

  while (mode == 0)    //changed if to while loop (RA)
  {
    if (!hasFlashed)
    {
      flashLED(247, 201, 69);
      hasFlashed = true;
    }
    if (mySerial.available()) {
      data = mySerial.read(); // define data as the num recieved from BT
      Serial.println(data);


    }
    if (data == 1) {
      forward_car();
    }
    else if (data == 2) {
      back_car();
    }
    else if (data == 3) {
      left_car();
    }
    else if (data == 4) {
      right_car();
    }
    else if (data == 5) {
      stop_car();
    }

    else if (data == 6) {
      mode = 1;
      hasFlashed = false;
      break;
    }
  }

  while (mode == 1) {

    if (!hasFlashed)
    {
      flashLED(117, 249, 76);
      hasFlashed = true;
    }

    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    int distance = distanceSensor.measureDistanceCm();
    motor.forward();
    motor2.forward();

    Serial.println (distance);

    if (distance <= 80 && distance > 15) {
      digitalWrite(2, LOW); // turn the LED on (HIGH is the voltage level)

      int red = map(distance, 0, 100, 255, 0);
      for (int n = 0; n < 42; n++)
      {
        strip.setPixelColor(n, red, 0, 0);
      }
      strip.show();
      motor.forward();
      motor2.forward();

    }
    else if (distance > 80 && distance < 400)
    { digitalWrite(2, LOW);
      for (int n = 0; n < 42; n++)
      {
        strip.setPixelColor(n, 0, 0, 0);
      }
      strip.show();
      Serial.println("No Lights are on! Go  ahead!");

      Serial.println ("No obstacle detected. going forward");
      motor.forward(); //if there's no obstacle ahead, Go Forward!
      motor2.forward();
      delay (100);

    }

    else if (distance <= 15 && distance > 0) {
      digitalWrite(2, HIGH);
      Serial.println("Red! Red! Turn Now");
      motor.stop();
      motor2.stop();
      delay(500);
      flashLED(255, 0, 0);
      motor.backward();
      motor2.backward(); //actually backward
      delay(1000);
      motor.stop();
      motor2.stop();
      delay(500);

      int leftdistance = lookLeft();
      int rightdistance = lookRight();


      if (leftdistance > rightdistance) {
        motor.forward();
        motor2.backward();
        delay(500);
        motor.stop();
        motor2.stop();
        delay(200);
        motor.forward();
        motor2.forward();
      }
      else {
        motor.backward(); //this means turn
        motor2.forward();
        delay(500);
        motor.stop();
        motor2.stop();
        delay(200);
        motor.forward();
        motor2.forward();

      }

    }


    if (mySerial.available()) {
      data = mySerial.read(); // define data as the num recieved from BT
      Serial.println(data);

    }

    if (data != 6) {
      mode = 0;
      hasFlashed = false;
      break;
    }

    val = digitalRead(inputPin);  // read input value
    if (val == HIGH) {            // check if the input is HIGH
      digitalWrite(2, HIGH);      // turn LED ON
      flashLED(0, 10, 120);
      mySerial.write(7);
      Serial.println("Blue is flashing");
      

      
      if (pirState == LOW) {
        // we have just turned on
        Serial.println("Person detected!");
        // We only want to print on the output change, not state
        pirState = HIGH;
      }
    } else {
      digitalWrite(2, LOW); // turn LED OFF
      if (pirState == HIGH) {
        // we have just turned of
        Serial.println("Ready for detection!");
        // We only want to print on the output change, not state
        pirState = LOW;
      }
    }


  }
}


void forward_car()
{ motor.forward();
  motor2.forward();

}

void back_car()
{ motor.backward();
  motor2.backward();
}

void left_car()
{ motor.forward();
  motor2.backward();
}

void right_car()
{ motor.backward();
  motor2.forward();
}

void stop_car()
{ motor.stop();
  motor2.stop();
}

void flashLED(int r, int g, int b) {

  for (int d = 0; d < 3; d++) {

    for (int n = 0; n < 42; n++)
    {
      strip.setPixelColor(n, r, g, b);
    }
    strip.show();
    delay(100);

    for (int n = 0; n < 42; n++)
    {
      strip.setPixelColor(n, 0, 0, 0);
    }
    strip.show();
    delay(100);

  }
}

int lookRight()
{
  servo1.write(50);
  delay(500);
  int distance = distanceSensor.measureDistanceCm();
  delay(100);
  servo1.write(115);
  return distance;
}

int lookLeft()
{
  servo1.write(170);
  delay(500);
  int distance = distanceSensor.measureDistanceCm();
  delay(100);
  servo1.write(115);
  return distance;
  delay(100);
}
