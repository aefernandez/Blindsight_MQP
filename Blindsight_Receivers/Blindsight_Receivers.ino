///*
//Test code for Blindsight: Ultrasonic Receivers.
//This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
//motors. The vibrating motors are controlled independently and received the intensity information through a communication module.
//
//Notes:
//* This Version Implements:
//* * Distance Ranging
//* * Temperature Recalibration
//* * Distance-Based Vibration Intensity
//* * Wireless Comms
//*
//* Not Implemented Yet:
//* * Reduction of Global Variables (for good practice)
//
//Authors: Alan Fernandez, Aatreya Chakravarti
//Date: 12/13/2018
//Worcester Polytechnic Institute (WPI)
//Blindsight MQP '19
//
//Sources used to write this code are listed below:
//* NRF24L01 Implementation:            http://maniacbug.github.io/RF24/starping_8pde-example.html
//*                                     https://medium.com/@benjamindavidfraser/arduino-nrf24l01-communications-947e1acb33fb
//* Properly Building Arduino Library:  https://www.teachmemicro.com/create-arduino-library/
//* MaxBotix Ultrasonic Sensor Setup:   https://www.maxbotix.com/tutorials1/031-using-multiple-ultrasonic-sensors.htm
//* TMP36 Datasheet:                    https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf
//* Adafruit TMP36 Notes:               https://learn.adafruit.com/tmp36-temperature-sensor/using-a-temp-sensor
//*/
//
//
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include "LowPower.h"

// chip select and RF24 radio setup pins
#define CE_PIN            7
#define CSN_PIN           8
#define MOTOR_PIN         6   // Motor 2 output
#define NODE_ID           1   // set to the appropriate number (0, 1, 2)

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

int pwm_setting = 0;
float previous_period = 0;
float period = 1;
float period_ms = 0;
unsigned long start_t;
unsigned long end_t;
float interrupts_per_period = 0;
int   number_of_interrupts = 0;
bool vibrate_on;
/* Function: setup
 *    Initialises the system wide configuration and settings prior to start
 */
void setup() {
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  // configure timer 2 to ~16ms interrupt
//  TIMSK2 = (TIMSK2 & B11111110) | 0x01;
//  TCCR2B = (TCCR2B & B11111000) | 0x07;
  
  // setup serial communications for basic program display
  Serial.begin(115200);
  Serial.println("[*][*][*] Beginning nRF24L01+ ack-payload slave device program [*][*][*]");

  // ----------------------------- RADIO SETUP CONFIGURATION AND SETTINGS -------------------------//

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);          // set power level of the radio
  radio.setDataRate(RF24_2MBPS);          // set the transmit rate
  radio.setChannel(0x76);                 // set radio channel to use - ensure all slaves match this
  radio.openReadingPipe(1, nodeAddress[NODE_ID]);
  //radio.setAutoAck(1);                  // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.writeAckPayload(1, &remoteNodeData[NODE_ID], sizeof(remoteNodeData[NODE_ID]));
  //radio.setRetries(0, 3);                 // Smallest time between retries, max no. of retries
  printf_begin();
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.startListening();
  // --------------------------------------------------------------------------------------------//

  
  // configure the timer to trigger when the count is 0xFF (0'd255).
  configure_interrupt();
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
  start_t = millis();
//  Serial.println(period);
//  Serial.println(vibrate_on);
  // check for radio message and send sensor data using auto-ack
  if ( radio.available()) {
    digitalWrite(4, HIGH);
    radio.read( &dataFromMaster, sizeof(dataFromMaster) );
    Serial.println("Received request from master - sending preloaded data.");
    Serial.print("The received count from the master was: ");
    Serial.println(dataFromMaster);
    Serial.print("Received from Pipe: ");
    Serial.println(*pipe_num);
    Serial.print("Reading from pipe: ");
    Serial.println("");
    Serial.println("--------------------------------------------------------");
    // obtain the period. The first 3 digits correspond to the PWM value, the last is the period divisor.
    // ie. 2554 -> PWM: 255, Period: 1/4
    
    period = 1.0 / (dataFromMaster % 10);
    period_ms = period * 1000;
    pwm_setting = (dataFromMaster - (dataFromMaster % 10)) / 10;
    vibrate_on = (period != 1)?true:false;
    
    //Serial.print("Received PWM: ");
//    Serial.println(pwm_setting);
//    Serial.print("Received Period: ");
//    Serial.println(period);
    
    if(dataFromMaster == 9999){
      analogWrite(MOTOR_PIN, 0);
      sleep_mode = true;
    }

    // removes the delay for the first pulse
    if(period != previous_period && period != 1.0){
      analogWrite(MOTOR_PIN, (pwm_setting > 105)?pwm_setting : 105);
      digitalWrite(3, HIGH);
      previous_period = period;
    }
    
    // update the node count after sending ack payload - provides continually changing data
    updateNodeData();
  }
//  Serial.print("time for all: ");
//  Serial.println(millis() - start_t);
  // this is for the logic analyzer
  digitalWrite(4, LOW);
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


// Configures the timer interrupt
void configure_interrupt(){
  OCR2A = 0xFF; // interrupt at 255
  TIMSK2 |= _BV(OCIE2A);
}

// Timer Interrupt Handler
// The intention is to pulse the motor n times per second. Where n = {4, 3, 2}.
// Since the ISR will trigger 1000 times per second, a counter is used to determine when to pulse the motor.
// It is also desired to pulse the motor with 10% duty cycle.
// The motor is set to HIGH when the count is < (1/10)(1000 / n)
// Otherwise the motor is set to LOW.
ISR(TIMER2_COMPA_vect){
  number_of_interrupts += 1;
  interrupts_per_period = 120*period;
  
  if(number_of_interrupts > interrupts_per_period){
    number_of_interrupts = 0;
    
//  }else if(previous_period != period && period != 1.0){
//    // put a floor at PWM=105 since the motors will not vibrate with a smaller PWM value. 
//    analogWrite(MOTOR_PIN, (pwm_setting > 105)?pwm_setting : 105);
//    digitalWrite(3, HIGH);
//    previous_period = period;
    
  }else if(period != 1.0 && number_of_interrupts < interrupts_per_period / 2){ // 10% duty cycle
    analogWrite(MOTOR_PIN, (pwm_setting > 105)?pwm_setting : 105);
    digitalWrite(3, HIGH);
    
  }else{
    analogWrite(MOTOR_PIN, 0);
    digitalWrite(3, LOW);
  }
}
//
