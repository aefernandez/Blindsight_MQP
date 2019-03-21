/*
 * This library contains the functions for Blindsight.
 *
 * Authors:
 *  Alan Fernandez, Aatreya Chakravarti
 *
 * Sources:
 * Bound Intensity to 0-255: https://stats.stackexchange.com/questions/281162/scale-a-number-between-a-range
 *
 * Worcester Polytechnic Institute (WPI)
 * WPI Blindsight MQP '19
 * 1/16/19
 */

#include "library.h"
#include <Arduino.h>
#include "LowPower.h"

/* Read Sensor Function
 * This function polls each sensor and retrieves distance data.
 */
void Blindsight_Library::read_sensor(){
  //Serial.println("read sensor function");
  // Iterate through each sensor in the list
  for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
//        Serial.print("read_sensor -- i: ");
//        Serial.print(i);
//        Serial.print(" sizeof(sList): ");
//        Serial.print(sizeof(sensorList));
//        Serial.print(" ");
    // Moving average with a window of two.
    // second sensor needs to get triggered to range
    start_ranging(i);

    float temp = pulseIn(sensorList[i], HIGH, 50000); //timeout of 50ms - optimized through testing

    // the pulseIn function may timeout before the sensor begins to range - ignore those data points
    if (temp != 0)
    {
      float newDist = (((temp + 38.447) / 5.4307) / 1000.0);
      // apply a weighted average of the last two pulses - this gives a disproportionate weight to low values
      // granting the device a greater sensitivity to smaller values that appear suddenly. This would be the case
      // when an object suddenly crosses in front of the user.
      pulseList[i] = (pulseList[i] > newDist) ? (pulseList[i] * 0.9 + newDist * 0.7) / 2 : (pulseList[i] + newDist) / 2;
    }
    Serial.print("Sensor: ");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(temp);
  }
}

/* Print Function
 * This function prints the distance data for all sensors. This is for debugging purposes.
 */
void Blindsight_Library::print_sensor_data(){
//  for(int i = 0; i < NUMBER_OF_SENSORS; i++){
//    Serial.print("[DBG] Sensor ");
//    Serial.print(i);
//    Serial.print(" ");
//    Serial.print(pulseList[i]);
//    Serial.print(" ");
//  }
  //Serial.println(pulseList[0]);
  //Serial.println("");
}

/* Start Ranging Function
 * This function needs to be called every time ranging needs to take place.
 * Params:
 *  sensor: the number of the sensor to trigger
 */
void Blindsight_Library::start_ranging(int sensor){
  //Serial.println("Start Ranging Function");
  if(sensor == 0){
    digitalWrite(TRIGGER_PIN_1,HIGH);
    delayMicroseconds(30);
    digitalWrite(TRIGGER_PIN_1,LOW);
  }else if (sensor == 1){
    digitalWrite(TRIGGER_PIN_2,HIGH);
    delayMicroseconds(30);
    digitalWrite(TRIGGER_PIN_2,LOW);
  }else{
    ;;
  }

}

/* Temperature Check Function
 * This function verifies the current temperature and determines whether a recalibration is necessary.
 * According to MaxBotix, a recalibration of the ultrasonic sensors is warranted when there is a 5C change in temperature.
 * Return:
 *  t_check: A flag to trigger the re-calibration.
 */
bool Blindsight_Library::check_temperature(){
  // Serial.println("Check Temperature Function");
  int t_check = false;

  // Read the temperature sensor
  float actual_temp = (((analogRead(TEMP_SENSOR_PIN) * 5.0)/1024.0)-0.5)*100;

  // Compare the last calibration temperature to current temperature
  // According to MaxBotix a change of +-5C justifies recalibration
  if (abs(actual_temp - calibration_temperature) >= 5.0){
    // Update calibration temperature
    calibration_temperature = actual_temp;
    // Return true to trigger recalibration
    t_check = true;
    // Serial.print('New Temperature ');
    // Serial.println(calibration_temperature);
  }
  return t_check;
}

/* Sensor Recalibration Function
 * This function cycles the sensors to recalibrate them.
 * The power up sequence takes 450ms according to the datasheet of the MX1020 sensors.
 * There is no delay added to wait for the 150ms calibration and first read sequence because the pulseIn
 * function in read_sensors() will wait until a pulse is received before continuing.
 */
void Blindsight_Library::recalibrate_sensors(){
  // Serial.println("Recalibrate Sensors Function");
  // digitalWrite(SENSOR_POWER_PIN, 0); // Turn off the sensors
  delay(50); // Small delay to allow sensors to power down
  // digitalWrite(SENSOR_POWER_PIN, 1); // Turn on the sensors
  delay(250); // Delay 250ms for the power-up delay
  // start_ranging(); // Trigger the first reading
  // read_sensor();
}

/* Calculate Motor Intensity Function
 * This function sets the PWM for the motors.
 * The distance for each sensor is scaled between 0-255 using the following formula:
 * scaled_intensity = (intensity - min_distance)/(max_distance - min_distance) * (upper_bound - lower_bound) + lower_bound
 *
 * Parameters
 *  sensorX_d: The distance measurement for the particular sensor
 */
void Blindsight_Library::calculate_motor_intensity(){

  //Serial.println("Calculate Motor Intensity Function");

  float range_sensitivity = 2.0 - ((float)delta_sensitivity-0)/(1023-0)*2.0;
  float MIN_DISTANCE = range_sensitivity / 3;
  float MED_DISTANCE = range_sensitivity / 3 * 2;
  float MAX_DISTANCE = range_sensitivity;
  int newIntensity;
  //The usable range of PWM for the motor is 105 - 255.
  int range_intensity = 255 - ((float)delta_intensity-0)/(1023-0)*255;

  for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
    // if intensity is changed then true
    char update_flag = 0;

    // Set the intensity of the motor
    // If the intensity is unchanged then do not do anything and set flag to 0 to stop unnecessarily updating nodes
    // Set the intensity of the motor, check first for high intensity given high priority
    if(pulseList[i] < MIN_DISTANCE){
      //Serial.println("High");
      newIntensity = range_intensity*10+4;
      if(intensityList[i] != newIntensity) {
        intensityList[i] = newIntensity;
        update_flag = 1;
      }

    }else if(pulseList[i] < MED_DISTANCE){
      //Serial.println("Mid");
      newIntensity = range_intensity*10+3;
      if(intensityList[i] != newIntensity) {
        intensityList[i] = newIntensity;
        update_flag = 1;
      }

    }else if(pulseList[i] < MAX_DISTANCE){
      //Serial.println("Low");
      newIntensity = range_intensity*10+2;
      if(intensityList[i] != newIntensity) {
        intensityList[i] = newIntensity;
        update_flag = 1;
      }

    }else{
      //Serial.println("No");
      newIntensity = NO_INTENSITY*10+1;
      if(intensityList[i] != newIntensity) {
        intensityList[i] = newIntensity;
        update_flag = 1;
      }
    }

    intensity_bool_list[i] = update_flag;

    // add the node ID at the end of the message
    //intensityList[i]*10+i;

//    Serial.print("Sensor Number: ");
//    Serial.println(i);
//    Serial.print("[DBG] delta_sensitivity: ");
//    Serial.println(delta_sensitivity);
//    Serial.print("[DBG] delta_intensity: ");
//    Serial.println(delta_intensity);
//    Serial.print("[DBG] range_sensitivity: ");
//    Serial.println(range_sensitivity);
//    Serial.print("[DBG] Pulse: ");
//    Serial.println(pulseList[i]);
//    Serial.print("Assigned Intensity: ");
//    Serial.println(intensityList[i]);
//    Serial.print("High: ");
//    Serial.print(MIN_DISTANCE);
//    Serial.print(" MED: ");
//    Serial.print(MED_DISTANCE);
//    Serial.print(" LOW: ");
//    Serial.print(MAX_DISTANCE);
//    Serial.println("");



    // record if a change occurred - this will signal whether the corresponding slave must update its intensity


//        Serial.print(pulseList[i]);
//        Serial.print(" ");
  }
//    Serial.println("");
//    // Print the intensity
//    int i;
//    for(i = 0; i<3; i++){
//        Serial.print(intensityList[i]);
//        Serial.print(" ");
//    }
//    Serial.println("");
//    Serial.println("********");
}

/***********************************/
/* *** Communications Handling *** */
/***********************************/

/*
 * The topology of the system is such that there is one master and two slaves.
 *  The master is the glasses and the slaves are the wristbands.
 *
 * A pipe is an address to which you can write. A simple analogy to understand how the NRF24 handles channels and
 * pipes is a mailbox in a multi-story building. All the mail comes through the same slot in the
 * front entrance (channel). A clerk picks up the mail and places it in the correct mailbox (pipe). Each node has
 * its own pipe and they all listen to the same channel.
 *
 * The pipe to which the master writes must match the pipe to which the slave listens to.
 * The master iterates through a list of pipes, writes to each one the intensity of the corresponding motor, waits
 * for the slave to acknowledge receipt and continues to the next pipe.
 *
 * Communication should only occur when there has been a change in vibration intensity.
 */
void Blindsight_Library::update_nodes(){
  unsigned long t;
  //Serial.println("Update Nodes Function");
  // iterate through each node
  //Serial.println("[DBG] ---------- Node Update ----------");
  for (byte node = 0; node < NUMBER_OF_SENSORS; node++) {
    // verify there was change in intensity to justify communication
    if (intensity_bool_list[node]){ //(true)
      if(!node){
        digitalWrite(3, HIGH);
      }
      t = millis();
      radio.openWritingPipe(nodeAddresses[node]);         // open the corresponding pipe
      //Serial.print("OpenWritingPipe: ");
      //Serial.println(millis() - t);
      bool tx_sent;                                       // boolean to indicate if radio.write() tx was successful
      t = millis();
      tx_sent = radio.write(&intensityList[node], sizeof(intensityList[node]));
      //Serial.print("RadioWrite: ");
      //Serial.println(millis() - t);
      // if tx success - receive and read slave node ack reply
      if (tx_sent) {
        //Serial.println("[DBG] tx_sent");
        t = millis();
        bool ackavailable = radio.isAckPayloadAvailable();
        //Serial.print("isACKavailable: ");
        //Serial.println(millis() - t);
        if (ackavailable) {
          //Serial.println("ackpayloadavailable?");
          // read ack payload and copy data to relevant remoteNodeData array
          t = millis();
          radio.read(&remoteNodeData[node], sizeof(remoteNodeData[node]));
          //Serial.print("RadioRead: ");
          //Serial.println(millis() - t);
//            Serial.print("[Scc] Successfully received data from node: ");
//            Serial.println(node);
//            Serial.print("  ---- The node reply received was: ");
//            Serial.println(remoteNodeData[node][1]);
//            Serial.print("tx_sent: ");
//            Serial.println(tx_sent);
//            Serial.print("temp: ");
//            Serial.println(temp);
//            Serial.print("node: ");
//            Serial.println(node);
//            Serial.print("Intensity sent: ");
//            Serial.println(intensityList[node]);
          //Serial.print("tx_sent: ");
          //Serial.println(tx_sent);
          //Serial.print("temp: ");
          //Serial.println(temp);
          //Serial.print("node: ");
          //Serial.println(node);
          //Serial.print("nodeAddresses[node]: ");

        } else {
          //Serial.println("[DBG] No ack packet available");
        }

        Serial.print("Time: ");
        Serial.println(millis() - t);
        digitalWrite(3, LOW);

      } else {
        //Serial.print("[Err] The transmission to the selected node failed:");
        //Serial.println(node);

        //Serial.println(nodeAddresses[node]);

        // Do something...
      }
    }else{
      //Serial.println("No sensor update");
    }
  }

}
/* Print Function
 * This function prints the timing data for all timed functions. This is for debugging purposes.
 */
void Blindsight_Library::print_timing(){
  //Serial.print("********* Function Timing *********");
  for(int i = 5; i < 5; i++){
    Serial.println(*timed_function_names[i]);
    Serial.print(": ");
    Serial.print(timed_function_times[i]);
  }

}


/* Blocks until vibrating modules produce an acknowledgement packet. This means that the modules are online and not
 * in sleep mode.
 *
 */
void Blindsight_Library::check_modules_online() {
  Serial.println("[DBG] Check_Module_online Function");
  bool module_online;
  Serial.println("[Err] Blocked Until Both Modules Reply");
  for (byte node = 0; node < NUMBER_OF_SENSORS; node++) {
    //Serial.println("[DBG] For Loop");
    module_online = false;
    radio.openWritingPipe(nodeAddresses[node]);         // open the corresponding pipe
    while (!module_online) {
      //Serial.println("[DBG] While loop");
      bool tx_sent;                                       // boolean to indicate if radio.write() tx was successful
      tx_sent = radio.write(&intensityList[node], sizeof(intensityList[node]));

      if (tx_sent) {
        //Serial.println("[DBG] tx_sent");
        if (radio.isAckPayloadAvailable()) {
          //Serial.println("[DBG] ackpayloadavailable?");
          // read ack payload and copy data to relevant remoteNodeData array
          radio.read(&remoteNodeData[node], sizeof(remoteNodeData[node]));
          module_online = true;
          Serial.println("[Scc] Module Replied");
        }
      }else{
        //Serial.println("tx_not_sent");
      }
    }
  }
  Serial.println("[Scc] Both Modules Replied, System Allowed to Run");
}

// Sets up low power mode
void low_power_mode(){
  //Serial.println("[DBG] LPM Function");
  // command nodes to LPM
  for (byte node = 0; node < NUMBER_OF_SENSORS; node++) {
    // verify there was change in intensity to justify communication
    radio.openWritingPipe(nodeAddresses[node]);         // open the corresponding pipe
    bool tx_sent;                                       // boolean to indicate if radio.write() tx was successful
    int LPM = 9999;
    tx_sent = radio.write(&LPM, sizeof(LPM));

    // if tx success - receive and read slave node ack reply
    if (tx_sent) {
      //Serial.println("[DBG] tx_sent");
      if (radio.isAckPayloadAvailable()) {
        //Serial.println("ackpayloadavailable?");
        // read ack payload and copy data to relevant remoteNodeData array
        radio.read(&remoteNodeData[node], sizeof(remoteNodeData[node]));

//          Serial.print("[+] Successfully received data from node: ");
//          Serial.println(node);
//          Serial.print("  ---- The node reply received was: ");
//          Serial.println(remoteNodeData[node][1]);

      } else { //(No reply else)
        //Serial.println("[DBG] No ack packet available");
      }
    } else { //(Tx_sent else)
//        Serial.print("[Err] The transmission to the selected node failed:");
//        Serial.println(node);
    }
  }
  //delay(1000);
  // Wait until PAUSE button is released before entering LPM to avoid problems with interrupts
  while(digitalRead(PAUSE_PIN) == LOW){
    ;;
  }
  //Serial.println("Powering Radio Down");
  // Local radio to LPM
  radio.powerDown();                                    // power down the radio
  //attachInterrupt(1, PAUSE_ISR, FALLING); // attach hardware interrupt to pause pin on falling edge
  // MCU to LPM
  //Serial.println("Attaching Interrupt");
  attachInterrupt(1, PAUSE_ISR, LOW);                   // configure external interrupt to wake device
  //Serial.println("Going to Sleep");
  //sei();
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);  // Enter power down state with ADC and BOD module disabled.
  // Wake up when wake up pin is low.
  Serial.println("Woke up");
  detachInterrupt(1);                                   // detach interrupt
  //Serial.println("Detached Interrupt");
  radio.powerUp();                                      // power up the radio
  //Serial.println("Radio Powered Up");
}


/* PAUSE_ISR Handles the HWI triggered by the PAUSE_BUTTON
 *  Changes the state of the device between PAUSED/RUNNING
 *
 */
void PAUSE_ISR(){
  /* Debouncing is needed in order to keep the interrupt from triggering multiple times due to jitter from the button.
   * Since in practice the button is only pressed once in a while it is safe to assume that once pressed there is no need to immediately check it again.
   * Instead of debouncing the button with delays, just detach the interrupt so that the jitter won't trigger it again. The device is going to go to sleep
   * as soon as the ISR exits anyways.
   */
  detachInterrupt(1);

  // Invert the state of the device so that the while loop quits and the device goes to ultra low power mode.
  if(blindsight_running){
    blindsight_running = 0;
  }else{
    blindsight_running = 1;
  }
  //blindsight_running != blindsight_running;

}
