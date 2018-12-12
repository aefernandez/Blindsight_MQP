/*
Test code for the ultrasonic version of Blindsight.
This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
motors. The vibrating motors are controlled independently and received the intensity information through a communication module. 


Authors: Alan Fernandez
Date: 10/29/2018
Worcester Polytechnic Institute (WPI)
Blindsight MQP '19
*/

/*
 * Custom Library Imports
 *
 * Import the necessary libraries to handle the Ultrasonic or Time of Flight (TOF) sensors and other support functions.
 */

#include <Blindsight_Ultrasonic.h>
Blindsight_Ultrasonic BS;

/* 
 *  Globals!
 */
float intensity_multiplier = 0.5;
int blindsightActivated = 1;
long calibration_temp = 0;
long pulseList[] = {0, 0, 0};
int motor_list[] = {MOTOR_RIGHT, MOTOR_CENTER, MOTOR_LEFT};
int sensorList[] = {SENSOR_PIN_ONE, SENSOR_PIN_TWO, SENSOR_PIN_THREE};
unsigned long button_last_read = millis();
unsigned long last_temp_check = millis();

//extern int motor_list[] = {MOTOR_RIGHT, MOTOR_CENTER, MOTOR_LEFT};               // Contains motor pins

//extern int sensorList[] = {SENSOR_PIN_ONE, SENSOR_PIN_TWO, SENSOR_PIN_THREE};    // Contains sensor pins


/* Setup Function
 * Initial setup of the device
 */
void setup () {
  // Begin serial communication
  Serial.begin(9600);

  // Configure the sensors
  BS.sensorSetup(sensorList);

  // Configure all buttons
  pinMode(TRIGGER_PIN,OUTPUT);
}


/* Main Function
 * Main loop of the program.
 */
void loop () {
  // If the device is not paused then range. 
  if (blindsightActivated){
    BS.start_ranging();
    BS.read_sensor();
    BS.printall();
    // set_motor_intensity(pulseList);
  }
  
  // Handle button presses, debounce included
  if (button_last_read < millis()-50 && analogRead(sensitivity_increase) != 0){
    button_last_read = millis(); // debounce the button
    BS.buttonPress(0);
  }
  if (button_last_read < millis()-50 && analogRead(sensitivity_decrease) != 0){
    button_last_read = millis(); // debounce the button
    BS.buttonPress(1);
  }

  // Verify if enough time has elapsed to check ambient temperature
//  if (millis() - last_temp_check > TEMP_CHECK_INTERVAL * 60000){
//    if (checkTemperature(calibration_temp)){
//      recalibrate_sensors();
//    }
//    last_temp_check = millis();
//  }
  
}
