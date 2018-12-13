//
// Created by Alan Fernandez on 12/11/2018.
/*
 *
 *
 * Sources:
 * Bound Intensity to 0-255: https://stats.stackexchange.com/questions/281162/scale-a-number-between-a-range
 */
#include "Blindsight_Ultrasonic.h"
#include <Arduino.h>

/* Read Sensor Function
 * This function polls each sensor and retrieves distance data.
 */
void Blindsight_Ultrasonic::read_sensor(){
    // Iterate through each sensor in the list
    for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
        // Moving average with a window of two. Notice the use of the custom BS_getDistance() function.
        pulseList[i] = (pulseList[i] + (((pulseIn(sensorList[i], HIGH)+38.447)/5.4307) / 1000.0))/2;

        // The PulseIn function returns 0 if no pulse is received.
        if (pulseList[i] == 0){
            Serial.print("ERROR WITH SENSOR: ");
            Serial.println(i);
        }
    }
}

/* Set Motor Intensity Function
 * This function sets the PWM for the motors.
 * The distance for each sensor is scaled between 0-255 using the following formula:
 * scaled_intensity = (intensity - min_distance)/(max_distance - min_distance) * (upper_bound - lower_bound) + lower_bound
 *
 * Parameters
 *  sensorX_d: The distance measurement for the particular sensor
 */
void Blindsight_Ultrasonic::set_motor_intensity(){
    int m_int[3] = {10,4,2};
    float motor_intensity;
    for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
        motor_intensity = 255.0 - ((float)pulseList[i]-MIN_DIST_MEASUREMENT)/(MAX_DIST_MEASUREMENT-MIN_DIST_MEASUREMENT)*255.0; // Scale the distance to 0-255

        m_int[i] = motor_intensity;

        // Set the intensity of the motor, check first for high intensity given high priority
        if(motor_intensity > MIN_DISTANCE){
            analogWrite(motor_list[i], MAX_INTENSITY);
            intensityList[i] = MAX_INTENSITY;

        }else if(motor_intensity > MED_DISTANCE){
            analogWrite(motor_list[i], MID_INTENSITY);
            intensityList[i] = MID_INTENSITY;

        }else if(motor_intensity > MAX_DISTANCE){
            analogWrite(motor_list[i], LOW_INTENSITY);
            intensityList[i] = LOW_INTENSITY;

        }else{
            analogWrite(motor_list[i], NO_INTENSITY);
            intensityList[i] = NO_INTENSITY;
        }


        Serial.print(pulseList[i]);
        Serial.print(" ");
    }
    Serial.println("");
    // Print the intensity
    int i;
    for(i = 0; i<3; i++){
        Serial.print(intensityList[i]);
        Serial.print(" ");
    }
    Serial.println("");
    Serial.println("********");
}

/* Print Function
 * This function prints the distance data for all sensors. This is for debugging purposes.
 */
void Blindsight_Ultrasonic::printall(){
    for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(pulseList[i]);
        Serial.print(" ");
    }
    Serial.println("");
}

/*  Button Press Handler
 *  This function handles the button presses. There are two soft buttons: +intensity, -intensity. These adjust the intensity of vibration.
 *  Parameters:
 *    buttonPressed The index of button pressed
 */
void Blindsight_Ultrasonic::buttonPress(int buttonPressed){
    if (buttonPressed == 0){
        // Increase the intensity
        if (intensity_multiplier < 0.9){
            intensity_multiplier += 0.1;
        }else{
            intensity_multiplier = 1.0;
        }
    }else if (buttonPressed == 1){
        // Decrease the intesity
        if (intensity_multiplier > 0.1){
            intensity_multiplier -= 0.1;
        }else{
            intensity_multiplier = 0.0;
        }
    }
}

/* Temperature Check Function
 * This function verifies the current temperature and determines whether a recalibration is necessary.
 * According to MaxBotix, a recalibration of the ultrasonic sensors is warranted when there is a 5C change.
 * Parameters:
 *  previous_temperature: The temperature at which the last calibration was made
 *
 * Return:
 *  t_check: A flag to trigger the re-calibration.
 */
int Blindsight_Ultrasonic::checkTemperature(float calibration_temperature){
    int t_check = false;

    // Read the temperature sensor
    int actual_temp = 0;

    // Compare the last calibration temperature to current temperature
    if (abs(actual_temp - calibration_temperature) >= 5){
        calibration_temperature = actual_temp;
        t_check = true;
    }

    return t_check;
}

/* BS_getDistance Function
 *
 * This function interprets the output of the Ultrasonic sensor and returns a
 * distance measurement.
 *
 * Parameters:
 * sensor_id The id/pin of the sensor to query.
 *
 * Return:
 * measured_distance Calculated distance in meters.
 */
float Blindsight_Ultrasonic::getDistance(int sensor_id){
    /* The pulseIn function waits for the pin to transition to 1 and then back to 0. The third parameter is the timeout in microseconds.
      Each sensor should provide a reading within 50ms, 10ms extra for redundancy. If a pulse is not received within that timeframe then
      there must be a problem with the sensors.
  */
    long ultrasonic_pulse = pulseIn(sensor_id, 1);
    return ultrasonic_pulse;
}


/* sensorSetup Function
 *
 * This function configured the Ultrasonic sensors and returns a boolean flag
 * depending on whether the setup suceeds or not.
 *
 * Parameters:
 * number_of_sensors Number of sensors to setup
 *
 * Returns:
 * setup_status Flag indicating the success of the setup operation
 */
int Blindsight_Ultrasonic::sensorSetup(int pin_list[]){

    // Configure all sensor pins
    for(int i = 0; i < sizeof(pin_list)/sizeof(pin_list[0]); i++){
        pinMode(pin_list[i], INPUT);
    }

    int setup_status = 1;

    return setup_status;
}


/* Start Ranging Function
 * This function initiates the ranging sequence by pulsing the leading sensor with a 25uS pulse. This triggers the leading sensor, which then
 * propagates the signal down the chain of sensors. Each sensor will range for 50ms and trigger the fol0ing sensor.
 * This function needs to be called every time ranging needs to take place.
 */
void Blindsight_Ultrasonic::start_ranging(){
    digitalWrite(TRIGGER_PIN,1);
    delayMicroseconds(25);
    digitalWrite(TRIGGER_PIN,0);
}


/* Sensor Recalibration Function
 * This function cycles the sensor to recalibrate them.
 * The power up sequence takes 450ms according to the datasheet of the MX1020 sensors.
 * There is no delay added to wait for the 150ms calibration and first read sequence because the pulseIn
 * function in read_sensors() will wait until a pulse is received before continuing.
 */
void Blindsight_Ultrasonic::recalibrate_sensors(){
    digitalWrite(SENSOR_POWER_PIN, 0); // Turn off the sensors
    delay(50); // Small delay to allow sensors to power down

    digitalWrite(SENSOR_POWER_PIN, 1); // Turn on the sensors
    delay(250*100); // Delay 250ms for the power-up delay

    start_ranging(); // Trigger the first reading
    read_sensor();
}