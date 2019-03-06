/* TODO 
 *  
 *  Code:
 *  - Connect to wifi, to adafruit (see example sketch code) and test
 *  
 *  
 *  
 *  
 *  
 *  Notes:
 *  
 *  
 */

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define DEBUG 1

const uint8_t pirPin = D7;        
const uint8_t sprayerPin = D5; 
const uint8_t LED1Pin = D1;
const uint8_t LED2Pin = D2;
const uint8_t buttonPin = D6;     
const uint8_t enabledLEDPin = LED_BUILTIN;

struct led_type {
  uint8_t pin;
  bool state;
};

led_type LED[2];


const int sprayDuration = 250;   // in milliseconds
const int buttonBounceDelay = 150; //in milliseconds

volatile bool button_pressed;
//volatile bool pir_triggered;
int LEDOnCount;

bool buttonLock = false;
//int remainingDisabledIterations;
//bool secondInterval;
//bool okToIdle;
//unsigned long int lastButtonPress;

// Adafruit IO credentials:
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "ma7801"
#define AIO_KEY         "98bc56ceffa54facb2458d5e4f27aae1"

// Wifi info
#define WLAN_SSID       "Pilsa_EXT"
#define WLAN_PASS       "MJLLAnder$729"

// Connection objects setup
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feed objects
Adafruit_MQTT_Publish logFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/cat-sprayer-log");
Adafruit_MQTT_Publish sprayButtonFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/cat-sprayer-spray-button");
Adafruit_MQTT_Publish LEDButtonFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/cat-sprayer-led-button");   // for testing



void blinkLED(byte LEDnum, int duration) {
  digitalWrite(LED[LEDnum - 1].pin, HIGH);
  delay(duration);
  digitalWrite(LED[LEDnum - 1].pin, LOW);
}

void onLED(byte LEDnum) {
  digitalWrite(LED[LEDnum - 1].pin, HIGH);
}

void offLED(byte LEDnum) {
  digitalWrite(LED[LEDnum - 1].pin, LOW);
}


void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  LED[0] = {LED1Pin, false};
  LED[1] = {LED2Pin, false};
  
  // Set up pins that aren't inputs
  pinMode(sprayerPin, OUTPUT);
  pinMode(LED[0].pin, OUTPUT);
  pinMode(LED[1].pin, OUTPUT);
  //pinMode(buttonPin, INPUT);

  // blink external LEDs to indicate power on
  for (byte i = 0; i < 2; i++) {
    blinkLED(1, 250);
    blinkLED(2, 250);
  }
  
  Serial.println("Serial test!");

  // Initialize flags, etc.
  //pir_triggered = false;
  LEDOnCount = 0;
  //button_pressed = false;
  //secondInterval = false;
  //okToIdle = false;
  //lastButtonPress = millis();

  // Start off disabled for 2 iterations ~= 16 seconds
  //remainingDisabledIterations = 2;  

  // Initialize pin states
  digitalWrite(pirPin, LOW);   //per HC-SR505 datasheet
  digitalWrite(sprayerPin, LOW);
  digitalWrite(LED[0].pin, LOW);
  digitalWrite(LED[1].pin, LOW);
  digitalWrite(buttonPin, LOW);
  //digitalWrite(enabledLEDPin, LOW);
}

void buttonHandler() {
  
  // Button handler
 
  // If the button has been released, release the "lock" on the button
  if (digitalRead(buttonPin) == LOW) {
    buttonLock = false;
  }

  delay(buttonBounceDelay);
  if (digitalRead(buttonPin) == HIGH && buttonLock == false) {
    
    #ifdef DEBUG
    Serial.println("Button pressed");
    #endif

    // Put a the button in a "lock" state so that this code doesn't get run again during the same button press
    buttonLock = true;

    // Prevent idle state
    //okToIdle = false;
    
    // Case 0: No LEDs lit
    if (LEDOnCount == 0) {
      // Turn on LED1
      onLED(1);
  
      LEDOnCount = 1;
      
      // Set "remainingDisabledIterations" to the disabledIterationsInteveral (acts as a flag and a counter)
      //remainingDisabledIterations = disabledIterationInterval;

    }
  
    // Case 1: Only LED1 is lit; restart disabled mode with double the iteration length:
    else if (LEDOnCount == 1) {    
      // Turn on LED2
      onLED(2);
      
      // Now 2 LEDs are on
      LEDOnCount = 2;
  
      //secondInterval = true;

      // Reset the iterations amount
      //remainingDisabledIterations = disabledIterationInterval;

    }
    else if (LEDOnCount == 2) {
    
      #ifdef DEBUG
      Serial.println("Turning off both LEDs - user cancelling disable");
      #endif

      //Turn off both LEDs
      offLED(1);
      offLED(2);

      // Reset flags / counters
      LEDOnCount = 0;
      //remainingDisabledIterations = 1;  // The "1" is for a delay after coming out of disabled mode
      //secondInterval = false;      
    }
  }

}


void loop() {

  buttonHandler();
  
  // If disabled
  /*
  if(remainingDisabledIterations > 0) {
    
    #ifdef DEBUG
    Serial.println("Sleeping for 8 seconds...");
    delay(SERIAL_DELAY);
    #endif

    //Delay to improve button operation
    delay(buttonDelay);
    
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

    #ifdef DEBUG
    Serial.println("Waking up!");
    delay(SERIAL_DELAY);
    #endif
    
    remainingDisabledIterations--;

    // If last iteration elapsed:
    if(remainingDisabledIterations == 0) {
      // If there is a second disable interval (i.e. user hit button twice)
      if(secondInterval) {
        secondInterval = false;
    
        // Turn off LED2
        digitalWrite(LED2Pin, LOW);

        // Indicate that only one LED is on now
        LEDOnCount = 1;
    
        // Reset the iteration count to disabledIterationsInterval
        remainingDisabledIterations = disabledIterationInterval;
    
      }
      
      // Otherwise, disabled period totally elapsed
      else {
        // Turn off LED1
        digitalWrite(LED1Pin, LOW);

        // Indicate now LEDs on
        LEDOnCount = 0;

        // Set Ok to power down flag
        okToIdle = true;
      }
    } 
  }
*/
  

/*
  // If motion detected
  if(pir_triggered) {

    // Reset flag
    pir_triggered = false;
    
    // Check if in disabled state; if so, don't spray, just repeat the loop()
    if (remainingDisabledIterations > 0) return;

    // Activate sprayer
    digitalWrite(sprayerPin, HIGH);
    
    #ifdef DEBUG
    Serial.println("Spray!");
    #endif

    #ifdef DEBUG
    digitalWrite(13, HIGH);
    #endif
    
    delay(sprayDuration);

    #ifdef DEBUG
    digitalWrite(13, LOW);
    #endif
    
    digitalWrite(sprayerPin, LOW);
  }

  // Enter a low power idle until interrupt activated
  if(okToIdle) {
    #ifdef DEBUG
    Serial.print("button_pressed=");
    Serial.println(button_pressed);
    Serial.print("millis()=");
    Serial.println(millis());
    Serial.println("Going to idle state...");
    delay(SERIAL_DELAY);
    #endif

    // Delay to improve button operation
    delay(350);
    
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    #ifdef DEBUG
    Serial.println("Coming out of idle state...");
    Serial.print("button_pressed=");
    Serial.println(button_pressed);
    Serial.print("millis()=");
    Serial.println(millis());
    delay(SERIAL_DELAY);
    #endif
  } 

  */
}

/*
void pirOnISR() {
  #ifdef DEBUG
  Serial.println("PIR trigger!");
  delay(SERIAL_DELAY);
  #endif

  pir_triggered = true;

  // Bug fix for button somehow getting triggered when either PIR is triggered or spray occurs:
  delay(350);
  button_pressed = false;
}

*/
