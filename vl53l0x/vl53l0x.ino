#include "Adafruit_VL53L0X.h"

// GLOBALS  
const int XS_SENS1 = 30;
const int XS_SENS2 = 31;
const int XS_SENS3 = 32;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println("Adafruit VL53L0X");
  
  // Start up sequence
  pinMode(XS_SENS1,OUTPUT);
  pinMode(XS_SENS2,OUTPUT);
  pinMode(XS_SENS3,OUTPUT); 
  delay(10);
  digitalWrite(XS_SENS1, LOW);
  digitalWrite(XS_SENS2, LOW);
  digitalWrite(XS_SENS3, LOW);
  delay(10);

  digitalWrite(XS_SENS1, HIGH);
  if (!lox1.begin(0x30)) {
    Serial.println(F("Failed to boot VL53L0X Sens 1"));
    while(1);
  }
  digitalWrite(XS_SENS2, HIGH);
  if (!lox2.begin(0x31)) {
    Serial.println(F("Failed to boot VL53L0X Sens 2"));
    while(1);
  }
  digitalWrite(XS_SENS3, HIGH);
  if (!lox3.begin(0x35)) {
    Serial.println(F("Failed to boot VL53L0X Sens 3"));
    while(1);
  }
    
  // power 
  Serial.println(F("VL53L0X API Ranging\n\n")); 
}


void loop() {
  
  // SENSOR 1 DATA GRABBING
  VL53L0X_RangingMeasurementData_t measure_sens1;
  lox1.rangingTest(&measure_sens1, false); // pass in 'true' to get debug data printout!
  if (measure_sens1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Sens1(mm): "); Serial.print(measure_sens1.RangeMilliMeter); Serial.print(" | ");
  } else {
    Serial.print("Sens1(mm): "); Serial.print("**"); Serial.print(" | ");
  }
  
  // SENSOR 2 DATA GRABBING
  VL53L0X_RangingMeasurementData_t measure_sens2;
  lox2.rangingTest(&measure_sens2, false);
  if (measure_sens2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Sens2(mm): "); Serial.print(measure_sens2.RangeMilliMeter); Serial.print(" | ");
  } else {
    Serial.print("Sens1(mm): "); Serial.print("**"); Serial.print(" | ");
  }

  // SENSOR 3 DATA GRABBING
  VL53L0X_RangingMeasurementData_t measure_sens3;
  lox3.rangingTest(&measure_sens3, false);
  if (measure_sens3.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Sens3(mm): "); Serial.print(measure_sens3.RangeMilliMeter); Serial.println(" ");
  } else {
    Serial.print("Sens1(mm): "); Serial.print("**"); Serial.println(" ");
  }
}
