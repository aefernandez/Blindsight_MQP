/*
Test code for Blindsight: Ultrasonic Receivers.
This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
motors. The vibrating motors are controlled independently and received the intensity information through a communication module.

Notes:
* This Version Implements:
* * Distance Ranging
* * Temperature Recalibration
* * Distance-Based Vibration Intensity
* * Wireless Comms
*
* Not Implemented Yet:
* * Reduction of Global Variables (for good practice)

Authors: Alan Fernandez, Aatreya Chakravarti
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
*/


#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include "LowPower.h"

// chip select and RF24 radio setup pins
#define CE_PIN            7
#define CSN_PIN           8
#define MOTOR_PIN         6   // Motor 2 output
#define NODE_ID           0   // set to the appropriate number (0, 1, 2)

RF24 radio(CE_PIN,CSN_PIN);

uint8_t* pipe_num;  // Pipe in which the message is received

// setup radio pipe addresses for each slave node
const byte nodeAddress[2][5] = {
    {'N','O','D','E','1'},
    {'N','O','D','E','2'},
};

// simple integer array for each remote node data, in the form [node_id, returned_count]
int remoteNodeData[2][2] = {{1, 1,}, {2, 1}};

// integer to store count of successful transmissions
int dataFromMaster = 0;

// sleep mode flag
bool sleep_mode = false;

/* Function: setup
 *    Initialises the system wide configuration and settings prior to start
 */
void setup() {
  // setup serial communications for basic program display
  Serial.begin(115200);
  Serial.println("[*][*][*] Beginning nRF24L01+ ack-payload slave device program [*][*][*]");

  // ----------------------------- RADIO SETUP CONFIGURATION AND SETTINGS -------------------------//

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);          // set power level of the radio
  radio.setDataRate(RF24_250KBPS);        // set the transmit rate
  radio.setChannel(0x66);                 // set radio channel to use - ensure all slaves match this
  radio.openReadingPipe(1, nodeAddress[NODE_ID]);
  //radio.setAutoAck(1);                  // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.writeAckPayload(1, &remoteNodeData[NODE_ID], sizeof(remoteNodeData[NODE_ID]));
  radio.setRetries(0, 3);                 // Smallest time between retries, max no. of retries
  printf_begin();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();

  // --------------------------------------------------------------------------------------------//
}

void loop() {
  // transmit current preloaded data to master device if message request received
  while(!sleep_mode){
    radioCheckAndReply();
  }
  Serial.println("Entering Low Power Mode");
  //low_power_mode();
  sleep_mode = false;
}

/* Function: updateNodeData
 *    updates the count variable for the node and stores it in the nRF24L01+ radio
 *    preloaded ack payload ready for sending on next received message
 */
void updateNodeData(void){
  // increment node count - reset to 1 if exceeds 500
  if (remoteNodeData[NODE_ID][1] < 500) {
    remoteNodeData[NODE_ID][1]++;
  } else {
    remoteNodeData[NODE_ID][1] = 1;
  }
  // set the ack payload ready for next request from the master device
  radio.writeAckPayload(1, &remoteNodeData[NODE_ID], sizeof(remoteNodeData[NODE_ID]));
}


/* Function: radioCheckAndReply
 *    sends the preloaded node data over the nrf24l01+ radio when
 *    a message is received by the master
 */
void radioCheckAndReply(void){
  // check for radio message and send sensor data using auto-ack
  if ( radio.available()) {
    radio.read( &dataFromMaster, sizeof(dataFromMaster) );
    Serial.println("Received request from master - sending preloaded data.");
    Serial.print("The received count from the master was: ");
    Serial.println(dataFromMaster);
    Serial.print("Received from Pipe: ");
    Serial.println(*pipe_num);
    Serial.print("Reading from pipe: ");
    Serial.println("");
    Serial.println("--------------------------------------------------------");
    if(dataFromMaster <= 255){
      analogWrite(MOTOR_PIN, dataFromMaster);
    }else if(dataFromMaster == 9999){
      analogWrite(MOTOR_PIN, 0);
      sleep_mode = true;
    }
    // update the node count after sending ack payload - provides continually changing data
    updateNodeData();
  }
}

/* Low Power Mode Configuration
 * The node enters low power mode. The watch dog timer is set to interrupt every 8 seconds which wakes the device from sleep. Since the oscillator for the WDT is kept on, this is not the lowest
 * power saving mode. The node has to wake up periodically in order to receive the command to wake up from the master. On waking up the node checks for available packages from the master.
 * If a package is available, the node verifies that it is an intensity value and not the command to sleep. If it is an intensity value, it configures PWM and exits Low Power Mode.
 */
void low_power_mode(){
  while(!radio.available()){
    // Enter deep sleep for 8s (wake up on WDT interrupt) then continue execution
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  }
  radioCheckAndReply();
  sleep_mode = false;
}
