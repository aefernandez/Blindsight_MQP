/*
Test code for Blindsight: Ultrasonic.
This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
motors. The vibrating motors are controlled independently and receive the intensity information through a communication module.

Notes:
* This Version Implements:
* * Distance Ranging
* * Temperature Recalibration
* * Distance-Based Vibration Intensity
* * Wireless Comms
* * Device Pausing
* * Friendly Adjustments to Intensity and Sensitivity

Engineers: Alan Fernandez, Aatreya Chakravarti
Date: 03/26/2019
Worcester Polytechnic Institute (WPI)
Blindsight MQP '19

Sources used to write this code are listed below:
* NRF24L01 Implementation:            http://maniacbug.github.io/RF24/starping_8pde-example.html
*                                     https://medium.com/@benjamindavidfraser/arduino-nrf24l01-communications-947e1acb33fb
* Properly Building Arduino Library:  https://www.teachmemicro.com/create-arduino-library/
* MaxBotix Ultrasonic Sensor Setup:   https://www.maxbotix.com/tutorials1/031-using-multiple-ultrasonic-sensors.htm
* TMP36 Datasheet:                    https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf
* Adafruit TMP36 Notes:               https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor
* Low Power Mode:                     https://learn.sparkfun.com/tutorials/reducing-arduino-power-consumption/all
*/

/*******************************************/
/*** * Hardware Connection Definitions * ***/
/*******************************************/

int PAUSE_PIN       = 3;    // START/PAUSE button
int MOSFET_TRIGGER  = 4;    // MOSFET that powers Ultrasonic Sensors  
int SENSOR_PIN_ONE  = 5;    // Reads pulse from sensor 1
int SENSOR_PIN_TWO  = 6;    // Reads pulse from sensor 2
int CE_PIN          = 7;    // nRF24 Chip Enable Pin
int CSN_PIN         = 8;    // nRF24 Chip Select Pin
int TRIGGER_PIN_1   = 9;    // Used to trigger Sensor1 ranging
int TRIGGER_PIN_2   = 10;   // Triggers Sensor2 ranging
int TEMP_SENSOR_PIN = 14;   // Connected to the temperature sensor
int INTENSITY_POT   = 15;   // Adjusts Intensity
int SENSITIVITY_POT = 16;   // Adjusts Sensitivity


/*********************/
/*** * DEBUGGING * ***/
/*********************/

const char *timed_function_names[5] = {"start_ranging()", "read_sensor()", "print_sensor_datal()",
                                       "calculate_motor_intensity()", "update_nodes()"};
long timed_function_times[] = {0,0,0,0,0};

/* Precompiler Directives to exclude the parts of the code labeled false. Used for debugging. */
#define WIRELESS      true
#define RECALIBRATION false
#define USERINPUT     true

/***************************/
/*** * Library Imports * ***/
/***************************/

/* Import the necessary libraries to handle the Ultrasonic or Time of Flight (TOF) sensors and other support functions. */
#include <SPI.h>
#include <library.h>
#include <printf.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "LowPower.h"

Blindsight_Library BS;

/****************************/
/*** * Global Variables * ***/
/****************************/

int sensorList[]          = {SENSOR_PIN_ONE, SENSOR_PIN_TWO};   // Contains hwd pins associated to each sensor
int intensityList[]       = {0,0};                              // Contains current intensity level for the corresponding motor
int intensity_bool_list[] = {0,0};                              // Contains a flag (1) if the assigned intensity has changed
float pulseList[]         = {0,0};                              // Contains the actual pulses collected from the sensors

unsigned long button_last_read = millis();                      // last time buttons were polled
unsigned long last_temp_check = millis();                       // last time ambient temp was checked

float calibration_temperature = 0;                              // temperature at which device last calibrated

int delta_intensity = analogRead(INTENSITY_POT);                // Offset for the intensity. This number is subtracted from the intensity for each intensity bucket.
                                                                // Refer to USER INPUT section below for more information.
int delta_sensitivity = analogRead(SENSITIVITY_POT);

bool blindsight_running = true;

/********************************/
/*** * Communications Setup * ***/
/********************************/

#if WIRELESS

// Simple integer array to store node data { node_id, returned_data }
int remoteNodeData[2][2] = {{1, 1}, {2, 2}};

// setup radio pipe addresses for each sensor node
const byte nodeAddresses[2][5] = {
    {'N', 'O', 'D', 'E', '1'},
    {'N', 'O', 'D', 'E', '2'},
};

// set transmission cycle send rate - in milliseconds
#define SEND_RATE 1000
unsigned long lastSentTime = millis();
int masterSendCount = 1;

/* Create an NRF24 object */
RF24 radio(CE_PIN, CSN_PIN);
#endif


void setup() {
  Serial.begin(115200);
  pinMode(PAUSE_PIN, INPUT);
  pinMode(SENSOR_PIN_ONE, INPUT);
  pinMode(SENSOR_PIN_TWO, INPUT);

  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);

  #if USERINPUT 
  //pinMode(PAUSE_PIN, INPUT_PULLUP);
  //attachInterrupt(1, PAUSE_ISR, LOW); // attach hardware interrupt to pause pin on falling edge
  #endif

#if WIRELESS
  
  radio.begin();                    // begin radio object
  radio.setPALevel(RF24_PA_LOW);   // set power level of the radio
  radio.setDataRate(RF24_2MBPS);    // set RF datarate - lowest rate for longest range capability
  radio.setChannel(0x76);           // set radio channel to use - ensure all slaves match this
  radio.enableAckPayload();         // enable ack payload - each slave replies with sensor data using this feature
  printf_begin();
  radio.printDetails();             // Dump the configuration of the rf unit for debugging

/* Comms Config Done */
#endif
}

/*********************/
/*** * Main Loop * ***/
/*********************/

void loop() {
  // Do not allow the system to run until both modules are online
  //BS.check_modules_online();
  unsigned long last_pause = millis();
  bool pause_interrupt_set = false;
  
  while(blindsight_running){
    // Before re-attaching the interrupt to the PAUSE Pin:
    // Wait for 5 seconds and make sure the button is not being held down
    if(millis() - last_pause > 5000 && !pause_interrupt_set && digitalRead(PAUSE_PIN) == HIGH){
      attachInterrupt(1, PAUSE_ISR, LOW);
      pause_interrupt_set = true;
    }

    Serial.println("read");
    BS.read_sensor();
    Serial.println("Print");
    BS.print_sensor_data();
    Serial.println("calculate");
    BS.calculate_motor_intensity();
    Serial.println("update");
  #if WIRELESS
    BS.update_nodes();
  #endif
  
/*************************/
/*** * Recalibration * ***/
/*************************/
  
    // Ambient temperature is verified every TEMP_CHECK_INTERVAL minutes to determine if recalibration is necessary.
    // If abs(last_calibration_temperature - current_ambient_temperature) >= 5C then, according to MaxBotix, calibration is necessary.
  #if RECALIBRATION
    if (millis() - last_temp_check > TEMP_CHECK_INTERVAL) {
      //Serial.println("(temp) *** Temperature Check Running ***");
      //Serial.print("(temp) Last calibration temperature: ");
      //Serial.print((temp) calibration_temperature);
      //Serial.println("");
      if (BS.checkTemperature()) {
        //Serial.println("(temp) Recalibration necessary");
        BS.recalibrate_sensors();
        //Serial.println("(temp) Sensor recalibrated");
        delay(100);
      }
      last_temp_check = millis();
    }
  #endif
  
/**********************/
/*** * User Input * ***/
/**********************/
    
  #if USERINPUT
    /* Sensitivity and Intensity Adjustments
     * These globals offset the default intensity/sensitivity setting of the device. 
     * Sensitivity  refers to the distance at which the device begins to consider objects a threat (vibration begins).
     * Intensity    refers to the strength with which the motors vibrate.
     */
     
    // Sensitivity Pot
    // While the sensitivity is being adjusted, ignore all mid and low range readings and only vibrate for the max range. 
    // This should make it easier to select the range of the device. 
    if(abs(delta_sensitivity - analogRead(SENSITIVITY_POT)) > 6){
      unsigned long last_adjust_time = millis();
      
      while(millis() - last_adjust_time < 2000){
        int temp_sensitivity = analogRead(SENSITIVITY_POT);
        
        // if a change is made, reset the last_adjust time
        last_adjust_time = abs(delta_sensitivity - temp_sensitivity) > 6 ? millis() : last_adjust_time;

        // update past sensitivity setting 
        delta_sensitivity = abs(delta_sensitivity - temp_sensitivity) > 6 ? temp_sensitivity : delta_sensitivity;

        // calculate intensity and ignore the MID and LOW range obstacles - only vibrate for objects at the MAX range
        BS.calculate_motor_intensity(true);
        BS.update_nodes();
      }
    }
   //delta_sensitivity = analogRead(SENSITIVITY_POT);
   
    Serial.print("Sensitivity: ");
    Serial.println(delta_sensitivity);
    // Intensity Pot
    // Detect if user changed potentiometer. If he did then begin intensity adjustment.
    // This means vibrating the bands continuously so that the intensity may be set more easily
    // Stop the continuous vibrations after 2 seconds
    if(abs(delta_intensity - analogRead(INTENSITY_POT)) > 6){
      unsigned long last_adjust_time = millis();

      // stay in adjustment mode until the potentiometer value is not changed for >2seconds
      while(millis() - last_adjust_time < 2000){
        // read in intensity setting
        int temp_intensity = analogRead(INTENSITY_POT);
        
        // if a change is made, reset the last_adjust time
        last_adjust_time = abs(delta_intensity - temp_intensity) > 6 ? millis() : last_adjust_time;
        
        // update past intensity setting
        delta_intensity = abs(delta_intensity - temp_intensity) > 6 ? temp_intensity : delta_intensity;
        
        // send packet to bands to vibrate at new intensity at min period
        // activate the override parameter and use the default period
        int scaled_intensity = 255 - ((float)delta_intensity-0)/(1023-0)*255;

        // update the nodes 
        BS.update_nodes(true, scaled_intensity);
        Serial.print("Intensity: ");
        Serial.println(delta_intensity);
      }

      // reset the vibration setting at the nodes 
      BS.update_nodes(true, 0); 
    }


  #endif
  }

  /**************************/
  /*** * LOW POWER MODE * ***/ 
  /**************************/
  //Serial.println("Entering Low Power Mode");
  #if USERINPUT
    low_power_mode();
    blindsight_running != blindsight_running;
    //Serial.println("Exit Low Power Mode");
  #endif
}
