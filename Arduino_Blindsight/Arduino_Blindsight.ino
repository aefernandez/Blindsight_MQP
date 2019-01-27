/*
Test code for Blindsight: Ultrasonic.
This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
motors. The vibrating motors are controlled independently and received the intensity information through a communication module.

Notes:
* This Version Implements:
* * Distance Ranging
* * Temperature Recalibration
* * Distance-Based Vibration Intensity
* * Wireless Comms
* * Device Pausing
*
* Not Implemented Yet:
* * Reduction of Global Variables (for good practice)

Engineers: Alan Fernandez, Aatreya Chakravarti
Date: 12/13/2018
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

int PAUSE_PIN       = 3 ;       // START/PAUSE button
int SENSOR_PIN_ONE  = 5 ;       // Reads pulse from sensor 1
int SENSOR_PIN_TWO  = 6 ;       // Reads pulse from sensor 2
int CE_PIN          = 7 ;       // nRF24 Chip Enable Pin
int CSN_PIN         = 8 ;       // nRF24 Chip Select Pin
int TRIGGER_PIN     = 10;      // Used to trigger sensor ranging
int TEMP_SENSOR_PIN = 14;       // Connected to the temperature sensor
int INTENSITY_POT   = 15;      // Adjusts Intensity
int SENSITIVITY_POT = 16;      // Adjusts Sensitivity

#define NUMBER_OF_SENSORS   2       // Number of sensors connected to device

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

#if WIRELESS
#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"
#endif

#if USERINPUT
#include "LowPower.h"
#endif

/* Import the necessary libraries to handle the Ultrasonic or Time of Flight (TOF) sensors and other support functions. */
#include <library.h>
#include <printf.h>
/* Since the functions are contained within a class of the same name as the library, it is necessary to create
 * an instance of that class to access them.
 * The only change that is necessary to switch between Ultrasonic and TOF is the class name. Choose between:
 * Blindsight_Ultrasonic and Blindsight_TOF
 */
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
  pinMode(SENSOR_PIN_ONE, INPUT); 
  pinMode(SENSOR_PIN_TWO, INPUT);
  pinMode(5, INPUT);

  #if USERINPUT 
  pinMode(PAUSE_PIN, INPUT_PULLUP);
  attachInterrupt(1, PAUSE_ISR, LOW); // attach hardware interrupt to pause pin on falling edge
  #endif

#if WIRELESS
  
  radio.begin();                    // begin radio object
  radio.setPALevel(RF24_PA_LOW);    // set power level of the radio
  radio.setDataRate(RF24_250KBPS);  // set RF datarate - lowest rate for longest range capability
  radio.setChannel(0x66);           // set radio channel to use - ensure all slaves match this
  radio.setRetries(4, 5);           // set time between retries and max no. of retries
  radio.enableAckPayload();         // enable ack payload - each slave replies with sensor data using this feature
  printf_begin();
  radio.printDetails();             // Dump the configuration of the rf unit for debugging

/* Comms Config Done */
#endif
}
   
/*
 * Includes an array that stores the timing of each function:
 * timed_function_names = ["start_ranging()", "read_sensor()", "print_all()", "calculate_motor_intensity()", "update_nodes()"]
 * timed_function_times = [...corresponding timings...]
 */
void loop() {
  // Do not allow the system to run until both modules are online
  //BS.check_modules_online();
  unsigned long last_pause = millis();
  bool pause_interrupt_set = false;
  while(blindsight_running){
    // Before re-attaching the interrupt to the PAUSE Pin:
    // - Wait for 5 seconds and make sure the button is not being held down
    if(millis() - last_pause > 5000 && !pause_interrupt_set && digitalRead(3) == HIGH){
      attachInterrupt(1, PAUSE_ISR, LOW);
      pause_interrupt_set = true;
    }
    unsigned long start_t = millis();
    BS.start_ranging();
    unsigned long end_t = millis();
    timed_function_times[0] = end_t - start_t;
    
    start_t = millis(); 
    BS.read_sensor();
    end_t = millis();
    timed_function_times[1] = end_t - start_t;
    
    start_t = millis();
    BS.print_sensor_data();
    end_t = millis();
    timed_function_times[2] = end_t - start_t;
    
    start_t = millis();
    BS.calculate_motor_intensity();
    end_t = millis();
    timed_function_times[3] = end_t - start_t;
  
    BS.print_timing();
    
  #if WIRELESS
    if(millis() - lastSentTime >= SEND_RATE) {
      start_t = millis();
      
//      if(digitalRead(5) == HIGH){
//        //communicate sleep to node
//        intensityList[0] = 9999;
//        intensityList[1] = 9999;
//        BS.update_nodes();
//      }else{
        BS.update_nodes();
      //}
      end_t = millis();
      timed_function_times[4] = end_t - start_t;
      lastSentTime = millis();
    }
  #endif
  
    /**********************************/
    /*** * Recalibration Handling * ***/
    /**********************************/
  
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
  
    /*******************************/
    /*** * User Input Handling * ***/
    /*******************************/
    
  #if USERINPUT
    /* Sensitivity and Intensity Deltas
     * These globals offset the default intensity/sensitivity setting of the device. 
     * Sensitivity  refers to the distance at which the device begins to consider objects a threat (vibration begins).
     * Intensity    refers to the strength with which the motors vibrate.
     * 
     * These settings are adjusted the following way:
     
       else if(motor_intensity > MIN_DISTANCE){
        intensityList[i] = MAX_INTENSITY - delta_intensity;
  
     * The same applies to the sensitivity. Consult the library to see the code. 
     */
    // Sensitivity Pot
    delta_sensitivity = analogRead(SENSITIVITY_POT);
  
    // Intensity Pot
    delta_intensity = analogRead(INTENSITY_POT);
  #endif
  }

  /**************************/
  /*** * LOW POWER MODE * ***/ 
  /**************************/
  Serial.println("Entering Low Power Mode");
  #if USERINPUT
  low_power_mode();
  blindsight_running != blindsight_running;
  Serial.println("Exit Low Power Mode");
  #endif
}



/* NOTES
 * Currently working on adding low power mode to the device.
 * Added the MCU low power mode but missing the NRF low power mode. Thinking of adding a function that waits for the receiving modules to acknowledge a packet before allowing the device to start. 
 * This function would be called when the device is turned on and after each wake up. At turn on it serves to make sure that the device is not used without the modules working and on wake up from sleep it
 * serves to wait for the modules to wake up from their timed sleep before continuing. 
 * 
 * The modules need their low power to be configured. They are supposed to enter a watchdog timer controlled sleep. They will sleep for 8 seconds then check on a signal. Thus tt is important for the main module
 * to wait until the modules acknowledge the receipt of a packet before functioning. The main module has to be transmitting continuously for at max 9 seconds for this to occur. 
 * 
 * 
 */
