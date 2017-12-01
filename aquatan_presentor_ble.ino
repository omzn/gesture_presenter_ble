/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <MsTimer2.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define APDS9960_INT 6

#define KEY_UP_ARROW    "00-00-52-00-00-00-00-00"
#define KEY_DOWN_ARROW  "00-00-51-00-00-00-00-00"
#define KEY_LEFT_ARROW  "00-00-50-00-00-00-00-00"
#define KEY_RIGHT_ARROW "00-00-4F-00-00-00-00-00"
#define KEY_RELEASE     "00-00"

#define L_EYE  5
#define R_EYE  9

#define LED    13

#define VBAT A9

#define MAX_LED_POWER 127

#define EYE_MAX 127.0f
#define T_MAX 200

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
SparkFun_APDS9960 apds = SparkFun_APDS9960();

int isr_flag = HIGH;
int count = 0;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

boolean eye_left = OFF,eye_right = OFF;


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(LED, OUTPUT);
  pinMode(L_EYE, OUTPUT);
  pinMode(R_EYE, OUTPUT);
  analogWrite(L_EYE, 0);
  analogWrite(R_EYE, 0);

  pinMode(APDS9960_INT, INPUT_PULLUP);
  digitalWrite(APDS9960_INT, HIGH);

  digitalWrite(LED, HIGH);
  //while (!Serial);  // required for Flora & Micro
  delay(100);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Keyboard Example"));
  Serial.println(F("---------------------------------------"));

  delay(100);
  digitalWrite(LED, HIGH);

  if ( apds.init() ) {
    Serial.println(F("LAPDS-9960 initialization complete."));
  } else {
    error(F("LAPDS-9960 initialization fails."));
  }

  delay(100);
  digitalWrite(LED, LOW);

  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    Serial.println(F("Gesture sensor is now running"));
  } else {
    error(F("Failed to enable gesture sensor!"));
  }

  delay(100);
  digitalWrite(LED, HIGH);

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  Serial.println( F("OK!") );

  delay(100);
  digitalWrite(LED, LOW);

  if ( FACTORYRESET_ENABLE ) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  delay(100);
  digitalWrite(LED, HIGH);

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  Serial.println("Requesting Bluefruit info:");

  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name"));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Aquatan presentor" )) ) {
    error(F("Could not set device name?"));
  }

  delay(100);
  digitalWrite(LED, LOW);

  /* Enable HID Service */

  Serial.println(F("Enable HID Service (including Keyboard): "));

  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) ) {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  } else {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  delay(100);
  digitalWrite(LED, HIGH);

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));

  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  delay(100);
  digitalWrite(LED, LOW);

/*  float measuredvbat = analogRead(VBAT);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);

  MsTimer2::set(500, flashled); // 500ms period
  MsTimer2::start();  
  */
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
/*  float measuredvbat = analogRead(VBAT);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  if (measuredvbat < 3.7) {
    MsTimer2::set(500, flashled); // 500ms period
    MsTimer2::start();      
  }
*/
  isr_flag = digitalRead(APDS9960_INT);

  if ( count == 0 && isr_flag == LOW ) {
    handleGesture();
  }

  if (count > 0) {
    delay(200);
    sendBleKeyboardCode(KEY_RELEASE);
    delay(1000);
    eye(OFF, OFF);
    count = 0;
  }
  delay(30);
}

void handleGesture() {
  static unsigned long int pt = 0;
  if ( apds.isGestureAvailable() ) {
    unsigned long int t = millis();
    int g = apds.readGesture();
    if (t - pt > 1000) {
      pt = t;
      switch ( g ) {
        case DIR_UP:
          Serial.println(F("UP"));
          sendBleKeyboardCode(KEY_UP_ARROW);
          eye(ON, OFF);
          count++;
          break;
        case DIR_DOWN:
          Serial.println(F("DOWN"));
          sendBleKeyboardCode(KEY_DOWN_ARROW);
          eye(OFF, ON);
          count++;
          break;
        case DIR_LEFT:
          Serial.println(F("LEFT"));
          sendBleKeyboardCode(KEY_LEFT_ARROW);
          eye(ON, OFF);
          count++;
          break;
        case DIR_RIGHT:
          Serial.println(F("RIGHT"));
          sendBleKeyboardCode(KEY_RIGHT_ARROW);
          eye(OFF, ON);
          count++;
          break;
        case DIR_NEAR:
          Serial.println(F("NEAR"));
          // Keyboard.write('h');
          count++;
          break;
        case DIR_FAR:
          Serial.println(F("FAR"));
          count++;
          break;
        default:
          //Serial.println("NONE");
          break;
      }
    }
  }
}

void sendBleKeyboardCode(String keycode) {
  ble.print("AT+BleKeyboardCode=");
  ble.println(keycode);
  if (ble.waitForOK()) {
    Serial.println( F("OK!") );
  } else {
    Serial.println( F("FAILED!") );
  }
}

void eye(boolean left, boolean right) {
  if (left) {
    analogWrite(L_EYE,MAX_LED_POWER);
  } else {
    analogWrite(L_EYE,0);
  }
  if (right) {
    analogWrite(R_EYE,MAX_LED_POWER);
  } else {
    analogWrite(R_EYE,0);
  }
  
/*  eye_left = left;
  eye_right = right;
  MsTimer2::set(10, flash_eye); // 10ms period
  MsTimer2::start();        
  */
}


void flash_eye() {
  uint8_t output_L = 0;
  uint8_t output_R = 0;
  static uint16_t t = 0;
  
  if (eye_left == ON) {
    output_L = (sin( (float)t/T_MAX * PI )) * EYE_MAX;
  } else {
    output_L = 0;
  }
  if (eye_right == ON) {
    output_R = (sin( (float)t/T_MAX * PI )) * EYE_MAX;
  } else {
    output_R = 0;
  }
  analogWrite(L_EYE,output_L);
  analogWrite(R_EYE,output_R);  
  t++;    
  if (t >= T_MAX) {
    t = 0;
    MsTimer2::stop();      
    analogWrite(L_EYE,0);
    analogWrite(R_EYE,0);  
  }
}
