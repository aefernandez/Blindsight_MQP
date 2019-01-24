/*
 * This library contains the functions for Blindsight.
 *
 * Authors:
 *  Alan Fernandez, Aatreya Chakravarti
 *
 * Worcester Polytechnic Institute (WPI)
 * WPI Blindsight MQP '19
 * 1/16/19
 */

#ifndef BLINDSIGHT_LIBRARY_LIBRARY_H
#define BLINDSIGHT_LIBRARY_LIBRARY_H

#include "RF24.h"

/******************************************/
/** Sensitivity and Intensity Parameters **/
/******************************************/

// NOTE THAT THESE ARE GOING TO HAVE TO CHANGE TO BE VARIABLES IF THEY'RE GOING TO BE ADJUSTABLE

#define MAX_DIST_MEASUREMENT 6.0   // This is the maximum distance reported by the sensor
#define MIN_DIST_MEASUREMENT 0.5   // This is the minimum distance reported by the sensor (actually 0.1)

/* Describes the intensity buckets for the motors */
#define MAX_INTENSITY       255     // Intensity setting for the vibrating motors
#define MID_INTENSITY       195     //
#define LOW_INTENSITY       105
#define NO_INTENSITY        0

/* Describes the distance steps in equivalent motor intensity */
#define MIN_DISTANCE       250     // Distance converted to motor intensity
#define MED_DISTANCE       237     //
#define MAX_DISTANCE       216
#define OUT_OF_RANGE       210

#define NUMBER_OF_SENSORS   2


extern int TEMP_SENSOR_PIN;
extern int TRIGGER_PIN;
extern int PAUSE_PIN;

extern bool blindsight_running;

extern RF24 radio;
extern const byte nodeAddresses[][5];
extern int remoteNodeData[][2];

extern int delta_sensitivity;
extern int delta_intensity;
extern const char *timed_function_names[];
extern long timed_function_times[];

extern int sensorList[NUMBER_OF_SENSORS]; // Contains sensor pins
extern int intensityList[NUMBER_OF_SENSORS];
extern int intensity_bool_list[NUMBER_OF_SENSORS];

extern unsigned long button_last_read; // for button debouncing
extern unsigned long last_temp_check;  // how long ago the last temperature calibration took place

extern float calibration_temperature;       // last calibration temperature
extern float pulseList[];            // contains the collected pulses

class Blindsight_Library {
 public:
  /* Read Sensor Function
  * This function polls each sensor and retrieves distance data.
  */
  void read_sensor();

  /* Print Function
   * This function prints the distance data for all sensors. This is for debugging purposes.
   */
  void printall();

  /* Start Ranging Function
   * This function initiates the ranging sequence by pulsing the leading sensor with a 25uS pulse. This triggers the leading sensor, which then
   * propagates the signal down the chain of sensors. Each sensor will range for 50ms and trigger the fol0ing sensor.
   * This function needs to be called every time ranging needs to take place.
   */
  void start_ranging();

  /* Temperature Check Function
   * This function verifies the current temperature and determines whether a recalibration is necessary.
   * According to MaxBotix, a recalibration of the ultrasonic sensors is warranted when there is a 5C change.
   * Parameters:
   *  previous_temperature: The temperature at which the last calibration was made
   *
   * Return:
   *  t_check: A flag to trigger the re-calibration.
   */
  bool checkTemperature();

  /* Sensor Recalibration Function
   * This function cycles the sensor to recalibrate them.
   * The power up sequence takes 450ms according to the datasheet of the MX1020 sensors.
   * There is no delay added to wait for the 150ms calibration and first read sequence because the pulseIn
   * function in read_sensors() will wait until a pulse is received before continuing.
   */
  void recalibrate_sensors();

  /* Calculate Motor Intensity Function
   * This function sets the PWM for the motors.
   * The distance for each sensor is scaled between 0-255 using the following formula:
   * scaled_intensity = (intensity - min_distance)/(max_distance - min_distance) * (upper_bound - lower_bound) + lower_bound
   *
   * Parameters
   *  sensorX_d: The distance measurement for the particular sensor
   */
  void calculate_motor_intensity();

  /* Print Function
   * This function prints the timing data for all timed functions. This is for debugging purposes.
   */
  void print_timing();

  /*  Button Press Handler
   *  This function handles the button presses. There are two soft buttons: +intensity, -intensity. These adjust the intensity of vibration.
   *  Parameters:
   *    buttonPressed The index of button pressed
   */
  void buttonPress(int buttonPressed);

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
  void update_nodes();

  /* Blocks until vibrating modules produce an acknowledgement packet. This means that the modules are online and not
   * in sleep mode.
   *
   */
  void check_modules_online();

  void low_power_mode();

  void print_sensor_data();


};

void PAUSE_ISR();
#endif //BLINDSIGHT_LIBRARY_LIBRARY_H