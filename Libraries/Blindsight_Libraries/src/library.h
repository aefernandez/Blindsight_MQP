/*
 * This library contains the functions for Blindsight.
 *
 * Authors:
 *  Alan Fernandez, Aatreya Chakravarti
 *
 * Worcester Polytechnic Institute (WPI)
 * WPI Blindsight MQP '19
 * 1/16/19
 * Sources:
 * ISR Inside a Class:  https://forum.arduino.cc/index.php?topic=394475.0
 */

#ifndef BLINDSIGHT_LIBRARY_LIBRARY_H
#define BLINDSIGHT_LIBRARY_LIBRARY_H

#include "RF24.h"

/******************************************/
/** Sensitivity and Intensity Parameters **/
/******************************************/

#define MAX_DIST_MEASUREMENT 6.0   // This is the maximum distance reported by the sensor
#define MIN_DIST_MEASUREMENT 0.5   // This is the minimum distance reported by the sensor (actually 0.1)

/* Describes the intensity buckets for the motors */
#define PWM_MINIMUM         105
#define NO_INTENSITY        0

#define NUMBER_OF_SENSORS   2

/* Pins */
extern int TEMP_SENSOR_PIN;
extern int TRIGGER_PIN_1;
extern int TRIGGER_PIN_2;
extern int PAUSE_PIN;

/* Device Status */
extern bool blindsight_running;         // boolean indicating if device is PAUSED

/* Communications */
extern RF24 radio;                      // RF24 object
extern const byte nodeAddresses[][5];   // addresses for each node
extern int remoteNodeData[][2];         // data received from each node

/* User Input */
extern int delta_sensitivity;           // contains the sensitivity potentiometer value
extern int delta_intensity;             // contains the intensity potentiometer value
extern unsigned long button_last_read;  // for button debouncing
extern unsigned long last_temp_check;   // how long ago the last temperature calibration took place


/* Debugging */
extern const char *timed_function_names[];  // contains the name of each function
extern long timed_function_times[];         // contains the execution time of each function

/* Ranging */
extern float pulseList[];                           // contains the collected pulses
extern int sensorList[NUMBER_OF_SENSORS];           // contains sensor pins
extern int intensityList[NUMBER_OF_SENSORS];        // contains the PWM value for the corresponding motor
extern int intensity_bool_list[NUMBER_OF_SENSORS];  // contains a flag indicating if PWM value changed

/* Temperature Calibration */
extern float calibration_temperature; // last calibration temperature


/* Blindsight Class */
class Blindsight_Library {
 public:
  /* Read Sensor Function
  * This function polls each sensor and retrieves distance data.
  */
  void read_sensor();

  /* Start Ranging Function
   * This function initiates the ranging sequence by pulsing the leading sensor with a 25uS pulse. This triggers the leading sensor, which then
   * propagates the signal down the chain of sensors. Each sensor will range for 50ms and trigger the fol0ing sensor.
   * This function needs to be called every time ranging needs to take place.
   */
  void start_ranging(int sensor);

  /* Temperature Check Function
   * This function verifies the current temperature and determines whether a recalibration is necessary.
   * According to MaxBotix, a recalibration of the ultrasonic sensors is warranted when there is a 5C change.
   * Parameters:
   *  previous_temperature: The temperature at which the last calibration was made
   *
   * Return:
   *  t_check: A flag to trigger the re-calibration.
   */
  bool check_temperature();

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
   */
  void check_modules_online();

  /*********************/
  /* *** Debugging *** */
  /*********************/

  /* Prints obtained pulse data. */
  void print_sensor_data();

  /* Print Function
  * This function prints the timing data for all timed functions. This is for debugging purposes.
  */
  void print_timing();

};

/* The following functions are placed outside the class in order for the code to operate as intended.
 * If the ISR was inside the class then the interrupt handler would not know to which instance of the class
 * to return to upon exiting the ISR. Read more... https://forum.arduino.cc/index.php?topic=394475.0
 */

/* PAUSE_ISR()
 * Handles the interrupts triggered by the PAUSE button.
 */
void PAUSE_ISR();

/* Low Power Mode Function
 * Configures low power mode. Commands the slave nodes to sleep and places the master to sleep as well.
 */
void low_power_mode();

#endif //BLINDSIGHT_LIBRARY_LIBRARY_H