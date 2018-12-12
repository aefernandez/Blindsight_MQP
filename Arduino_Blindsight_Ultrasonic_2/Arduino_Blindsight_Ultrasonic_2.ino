/*
Test code for the ultrasonic version of Blindsight.
This code controls the Blindsight Assistive Device. It pulses a number of ultrasonic sensors and relays the distance information to vibrating
motors. The vibrating motors are controlled independently and received the intensity information through a communication module. 

*/

/*
 * DEFINITIONS 
 */
#define TEMP_CHECK_INTERVAL 5 // How often to check ambient temperature. t in minutes. 
#define MOTOR_CENTER 6 // Motor 2 output
#define MOTOR_LEFT 9   // Motor 1 output
#define MOTOR_RIGHT 5  // Motor 3 output
#define TRIGGER_PIN 13 // Trigger pin to initiate ultrasonic scan
#define PAUSE_PIN 10   // Pin connected to soft power button
#define SENSOR_POWER_PIN 2 // Pin connected to the sensor MOSFET

/* 
 *  CONSTANTS
 */
const int motor_list[] = {MOTOR_RIGHT, MOTOR_CENTER, MOTOR_LEFT};
const int sensorList[] = {3, 5, 6};   // LISTS WITH THE SENSOR PINS AND SENSOR OUTPUT
const int sensitivity_increase = 12;  // 
const int sensitivity_decrease = 12;

const int MAX_DIST_MEASUREMENT = 18;  // This is the minimum distance reported by the sensor
const int MIN_DIST_MEASUREMENT = 300; // This is the maximum obstacle distance
/*
 * GLOBAL VARIABLES
 */
bool blindsightActivated = true;  
unsigned long button_last_read = millis(); // for button debouncing
long pulseList[] = {0, 0, 0};
int intensity_multiplier = 0.5; // controls the intensity of vibrations
unsigned long last_temp_check = millis();
long calibration_temp = 0;

/* Setup Function
 * Initial setup of the device
 */
void setup () {
  // Begin serial communication
  Serial.begin(9600);
  
  // Configure all sensor pins
  for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
    pinMode(sensorList[i], INPUT);
  }

  // Configure all buttons
  pinMode(TRIGGER_PIN,OUTPUT);
}

/* Read Sensor Function
 * This function polls each sensor and retrieves distance data.
 */
void read_sensor(){
  /* The pulseIn function waits for the pin to transition to HIGH and then back to LOW. The third parameter is the timeout in microseconds. 
      Each sensor should provide a reading within 50ms, 10ms extra for redundancy. If a pulse is not received within that timeframe then 
      there must be a problem with the sensors. 
  */
  //unsigned long sTimeout = 55000;
  //Serial.print("SensorList Size ");
  //Serial.println(sizeof(sensorList)/sizeof(sensorList[0]));
  
  for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
    pulseList[i] = (pulseList[i] + pulseIn(sensorList[i], HIGH))/2;
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
void set_motor_intensity(long sensor_pulses[]){
  for (int i = 0; i < sizeof(sensor_pulses)/sizeof(sensor_pulses[0]); i++){
    int motor_intensity = (sensor_pulses[i]-MIN_DIST_MEASUREMENT)/(MAX_DIST_MEASUREMENT-MIN_DIST_MEASUREMENT)*255; // Scale the distance to 0-255
    analogWrite(motor_list[i], motor_intensity);
  }
} 
 
/* Start Ranging Function
 * This function initiates the ranging sequence by pulsing the leading sensor with a 25uS pulse. This triggers the leading sensor, which then 
 * propagates the signal down the chain of sensors. Each sensor will range for 50ms and trigger the following sensor. 
 * This function needs to be called every time ranging needs to take place. 
 */
void start_sensor(){
  digitalWrite(TRIGGER_PIN,HIGH);
  delayMicroseconds(25);
  digitalWrite(TRIGGER_PIN,LOW);
}

/* Print Function
 * This function prints the distance data for all sensors. This is for debugging purposes.
 */
void printall(){         
  for(int i = 0; i < sizeof(sensorList)/sizeof(sensorList[0]); i++){
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" ");
      Serial.print(((pulseList[i]+38.447)/5.4307) / 1000);
      Serial.print(" ");
  }
  Serial.println("");
}

/*  Button Press Handler
 *  This function handles the button presses. There are two soft buttons: +intensity, -intensity. These adjust the intensity of vibration.
 *  Parameters: 
 *    buttonPressed The index of button pressed
 */
void buttonPress(int buttonPressed){
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
bool checkTemperature(float calibration_temperature){
  bool t_check = false;
  
  // Read the temperature sensor 
  int actual_temp = 0;
  
  // Compare the last calibration temperature to current temperature
  if (abs(actual_temp - calibration_temperature) >= 5){
    calibration_temperature = actual_temp;
    t_check = true;
  }
  
  return t_check;
} 

/* Sensor Recalibration Function
 * This function cycles the sensor to recalibrate them. 
 * The power up sequence takes 450ms according to the datasheet of the MX1020 sensors. 
 * There is no delay added to wait for the 150ms calibration and first read sequence because the pulseIn
 * function in read_sensors() will wait until a pulse is received before continuing. 
 */
void recalibrate_sensors(){
  digitalWrite(SENSOR_POWER_PIN, LOW); // Turn off the sensors
  delay(50); // Small delay to allow sensors to power down

  digitalWrite(SENSOR_POWER_PIN, HIGH); // Turn on the sensors
  delay(250*100); // Delay 250ms for the power-up delay

  start_sensor(); // Trigger the first reading
  read_sensor(); // Read the sensors
}

/* Main Function
 * Main loop of the program.
 */
void loop () {
  // If the device is not paused then range. 
  if (blindsightActivated){
    start_sensor();
    read_sensor();
    printall();
    // set_motor_intensity(pulseList);
    // delay((sizeof(sensorList)/sizeof(sensorList[0]))*50); // This delay time changes by 50 for every sensor in the chain. *****PULSEIN WAITS... SO WHY HAVE THIS?****
  }
  
  // Handle button presses, debouncing included
  if (button_last_read < millis()-50 && analogRead(sensitivity_increase) != 0){
    button_last_read = millis(); // debounce the button
    buttonPress(0);
  }
  if (button_last_read < millis()-50 && analogRead(sensitivity_decrease) != 0){
    button_last_read = millis(); // debounce the button
    buttonPress(1);
  }

  // Verify if enough time has elapsed to check ambient temperature
//  if (millis() - last_temp_check > TEMP_CHECK_INTERVAL * 60000){
//    if (checkTemperature(calibration_temp)){
//      recalibrate_sensors();
//    }
//    last_temp_check = millis();
//  }
  
}
