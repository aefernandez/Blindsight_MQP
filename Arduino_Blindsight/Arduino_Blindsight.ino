/*
Test code for Blindsight: Ultrasonic.
This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
motors. The vibrating motors are controlled independently and received the intensity information through a communication module. 

Notes:
* This Version Implements:
* * Distance Ranging
* * Temperature Recalibration
* * Distance-Based Vibration Intensity
* * Untested Wireless Comms
* 
* Not Implemented Yet:
* * Wireless Comms?
* * Reduction of Global Variables (for good practice)

Authors: Alan Fernandez
Date: 12/13/2018
Worcester Polytechnic Institute (WPI)
Blindsight MQP '19

Sources used to write this code are listed below:
* NRF24L01 Implementation:            http://maniacbug.github.io/RF24/starping_8pde-example.html
* Properly Building Arduino Library:  https://www.teachmemicro.com/create-arduino-library/
* MaxBotix Ultrasonic Sensor Setup:   https://www.maxbotix.com/tutorials1/031-using-multiple-ultrasonic-sensors.htm
* TMP36 Datasheet:                    https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf
* Adafruit TMP36 Notes:               https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor
*/

/**************************/
/* *** CODE EXCLUSION *** */
/**************************/  

/* Parts labeled false will be excluded from the compiled code. */
#define WIRELESS      false
#define RECALIBRATION true


/**********************************/
/* *** Custom Library Imports *** */
/**********************************/

/* Import the necessary libraries to handle the Ultrasonic or Time of Flight (TOF) sensors and other support functions. */
#include <Blindsight_Ultrasonic.h>


/* Since the functions are contained within a class of the same name as the library, it is necessary to create an instance of that class to access them. 
* The only change that is necessary to switch between Ultrasonic and TOF is the class name. Choose between:
* Blindsight_Ultrasonic and Blindsight_TOF
 */
Blindsight_Ultrasonic BS;
//Blindsight_TOF        BS;


/********************/
/* *** Globals! *** */
/********************/
float intensity_multiplier = 0.5;
int blindsightActivated = 1;
long calibration_temp = 0;
float pulseList[] = {0, 0, 0};
int motor_list[] = {MOTOR_LEFT, MOTOR_CENTER, MOTOR_RIGHT};
int sensorList[] = {SENSOR_PIN_ONE, SENSOR_PIN_TWO, SENSOR_PIN_THREE};
unsigned long button_last_read = millis();
unsigned long last_temp_check = millis();
char intensity_buckets[] = {0, 105, 195, 255};
int intensityList[] = {0,0,0};
float calibration_temperature = 0;


/********************************/ 
/* *** Communications Setup *** */
/********************************/
#if WIRELESS
/* Create an instance of the RF24 library to manage the NRF24L01 module */
RF24 radio(9,10);
const int role_pin = 7;

// Communications pipes
const uint64_t talking_pipes[5] = { 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0A5LL, 0xF0F0F0F096LL };
const uint64_t listening_pipes[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

// Address management
// Where in EEPROM is the address stored?
const uint8_t address_at_eeprom_location = 0;

// What is our address (SRAM cache of the address from EEPROM)
// Note that zero is an INVALID address.  The pong back unit takes address
// 1, and the rest are 2-6
uint8_t node_address;
#endif



/* Setup Function
 * Initial setup of the device
 */
void setup () {
  // Begin serial communication
  Serial.begin(9600);

  // Configure the sensors
  //BS.sensorSetup(sensorList);

  // Configure all buttons
  pinMode(TRIGGER_PIN,OUTPUT);
  
  pinMode(SENSOR_POWER_PIN,OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);


  /***************************/
  /* *** Configure Comms *** */
  /***************************/
  #if WIRELESS
  pinMode(role_pin, INPUT);
  digitalWrite(role_pin, HIGH);
  delay(20);

  // Read the address from EEPROM
  uint8_t reading = EEPROM.read(address_at_eeprom_location);
  
  // If it is in a valid range for node addresses, it is our
  // address.
  if ( reading >= 2 && reading <= 6 )
    node_address = reading;
  
  // Otherwise, it is invalid, so set our address AND ROLE to 'invalid'
  else
  {
    node_address = 0;
    role = role_invalid;
  }
  
  radio.begin();

  // Write on our talking pipe
  radio.openWritingPipe(talking_pipes[node_address-2]);
  // Listen on our listening pipe
  radio.openReadingPipe(1,listening_pipes[node_address-2]);

  radio.startListening();
  radio.printDetails();

  if ( role == role_invalid )
  {
    printf("\n\r*** NO NODE ADDRESS ASSIGNED *** Send 1 through 6 to assign an address\n\r");
  }
  /* Comms Config Done */
  #endif

  /* *** Get Calibration Temperature *** */
  calibration_temperature = (((analogRead(TEMP_SENSOR_PIN) * 5.0)/1024.0)-0.5)*100;
}


/* Main Function
 * Main loop of the program.
 */
void loop () {
  // If the device is not paused then range. 
  if (blindsightActivated){
    BS.start_ranging();
    BS.read_sensor();
    
    #if !WIRELESS
    BS.set_motor_intensity();
    #endif
    
    //BS.printall();
  }

  /*******************************/
  /* *** User Input Handling *** */
  /*******************************/
  if (button_last_read < millis()-50 && analogRead(sensitivity_increase) != 0){
    button_last_read = millis(); // debounce the button
    BS.buttonPress(0);
  }
  if (button_last_read < millis()-50 && analogRead(sensitivity_decrease) != 0){
    button_last_read = millis(); // debounce the button
    BS.buttonPress(1);
  }

  /**********************************/
  /* *** Recalibration Handling *** */
  /**********************************/
  #if RECALIBRATION
  // Verify if enough time has elapsed to check ambient temperature
  if (millis() - last_temp_check > TEMP_CHECK_INTERVAL * 6000){
    Serial.println("Temp check!");
    Serial.println(calibration_temperature);
    if (BS.checkTemperature()){
      BS.recalibrate_sensors();
      delay(100);
      Serial.println('Recalibrated Sensors');
    }
    last_temp_check = millis();
  }
  #endif 
  

  /***********************************/
  /* *** Communications Handling *** */
  /***********************************/
  #if WIRELESS
  // First, stop listening so we can talk.
  radio.stopListening();

  // Send the vibration intensity to the modules. A number between 0-255 for PWM.
  radio.write( &motor_intensity, sizeof(int) );

  // Listen for a reply from the modules to verify they are still active and responding properly.
  radio.startListening();

  /* Re-Implement as an if statement to avoid blocking the sensing */
  // Wait here until we get a response, or timeout (250ms)
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 250 )
      timeout = true;

  // Describe the results
  if ( timeout )
  {
    printf("Failed, response timed out.\n\r");
  }
  else
  {
    // Grab the response, compare, and send to debugging spew
    char status_bit;
    radio.read( &status_bit, sizeof(char) );

    // Spew it
    printf("Got response %c \n\r",status_bit);
  }
 #endif
   
}
