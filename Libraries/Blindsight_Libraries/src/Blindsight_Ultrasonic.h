/*


Authors: Alan Fernandez
Date: 10/29/2018
Worcester Polytechnic Institute (WPI)
Blindsight MQP '19
*/

#ifndef BLINDSIGHT_LIBRARIES_BLINDSIGHT_ULTRASONIC_H
#define BLINDSIGHT_LIBRARIES_BLINDSIGHT_ULTRASONIC_H

/*
 * DEFINITIONS
 */
#define TEMP_CHECK_INTERVAL 5       // How often to check ambient temperature. t in minutes.
#define MOTOR_CENTER        6       // Motor 2 output
#define MOTOR_LEFT          9       // Motor 1 output
#define MOTOR_RIGHT         5       // Motor 3 output
#define TRIGGER_PIN         13      // Trigger pin to initiate ultrasonic scan
#define PAUSE_PIN           10      // Pin connected to soft power button
#define SENSOR_POWER_PIN    2       // Pin connected to the sensor MOSFET
#define SENSOR_PIN_ONE      30      // Pin connected to sensor one
#define SENSOR_PIN_TWO      31      // Pin connected to sensor two
#define SENSOR_PIN_THREE    32      // Pin connected to sensor three

#define sensitivity_increase 12    // How much sensitivity changes with button press
#define sensitivity_decrease 12

#define MAX_DIST_MEASUREMENT 18    // This is the minimum distance reported by the sensor
#define MIN_DIST_MEASUREMENT 300   // This is the maximum obstacle distance

#define NUMBER_OF_SENSORS   3
#define NUMBER_OF_MOTORS    3


/*
 * GLOBAL VARIABLES
 */

extern float intensity_multiplier;    // controls the intensity of vibrations
extern int blindsightActivated;     // controls whether sensing is taking place

extern long calibration_temp;       // last calibration temperature
extern long pulseList[];            // contains the collected pulses
extern int motor_list[NUMBER_OF_MOTORS];  // Contains motor pins
extern int sensorList[NUMBER_OF_SENSORS]; // Contains sensor pins

extern unsigned long button_last_read; // for button debouncing
extern unsigned long last_temp_check;  // how long ago the last temperature calibration took place


class Blindsight_Ultrasonic{
public:
    /* Read Sensor Function
     * This function polls each sensor and retrieves distance data.
     */
    void read_sensor();

    /* Set Motor Intensity Function
     * This function sets the PWM for the motors.
     * The distance for each sensor is scaled between 0-255 using the following formula:
     * scaled_intensity = (intensity - min_distance)/(max_distance - min_distance) * (upper_bound - lower_bound) + lower_bound
     *
     * Parameters
     *  sensorX_d: The distance measurement for the particular sensor
     */
    void set_motor_intensity(long sensor_pulses[]);

    /* Print Function
     * This function prints the distance data for all sensors. This is for debugging purposes.
     */
    void printall();

    /*  Button Press Handler
     *  This function handles the button presses. There are two soft buttons: +intensity, -intensity. These adjust the intensity of vibration.
     *  Parameters:
     *    buttonPressed The index of button pressed
     */
    void buttonPress(int buttonPressed);

    /* Temperature Check Function
     * This function verifies the current temperature and determines whether a recalibration is necessary.
     * According to MaxBotix, a recalibration of the ultrasonic sensors is warranted when there is a 5C change.
     * Parameters:
     *  previous_temperature: The temperature at which the last calibration was made
     *
     * Return:
     *  t_check: A flag to trigger the re-calibration.
     */
    int checkTemperature(float calibration_temperature);

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
    float getDistance(int sensor_id);


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
    int sensorSetup(int pin_list[]);


    /* Sensor Recalibration Function
     * This function cycles the sensor to recalibrate them.
     * The power up sequence takes 450ms according to the datasheet of the MX1020 sensors.
     * There is no delay added to wait for the 150ms calibration and first read sequence because the pulseIn
     * function in read_sensors() will wait until a pulse is received before continuing.
     */
    void recalibrate_sensors();


    /* Start Ranging Function
     * This function initiates the ranging sequence by pulsing the leading sensor with a 25uS pulse. This triggers the leading sensor, which then
     * propagates the signal down the chain of sensors. Each sensor will range for 50ms and trigger the following sensor.
     * This function needs to be called every time ranging needs to take place.
     */
    void start_ranging();

};
#endif //BLINDSIGHT_LIBRARIES_BLINDSIGHT_ULTRASONIC_H
