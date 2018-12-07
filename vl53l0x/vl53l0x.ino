#include "Adafruit_VL53L0X.h"

// GLOBALS  
const int XS_SENS1 = 30;
const int XS_SENS2 = 31;
const int XS_SENS3 = 32;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

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
  
  // power 
  Serial.println(F("VL53L0X API Ranging\n\n")); 
}


void loop() {
  
  // SENSOR 1 INITIALIZATION AND DATA GRABBING
  digitalWrite(XS_SENS1, HIGH);
  if (!lox.begin(0x30)) {
    Serial.println(F("Failed to boot VL53L0X Sens 1"));
    while(1);
  }
  VL53L0X_RangingMeasurementData_t measure_sens1;
  lox.rangingTest(&measure_sens1, false); // pass in 'true' to get debug data printout!
  if (measure_sens1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Sens1(mm): "); Serial.print(measure_sens1.RangeMilliMeter); Serial.print(" | ");
  } else {
    Serial.print("Sens1(mm): "); Serial.print("**"); Serial.print(" | ");
  }
  digitalWrite(XS_SENS1, LOW);
  
  // SENSOR 2 INITIALIZATION AND DATA GRABBING
  digitalWrite(XS_SENS2, HIGH);
  if (!lox.begin(0x31)) {
    Serial.println(F("Failed to boot VL53L0X Sens 2"));
    while(1);
  }
  VL53L0X_RangingMeasurementData_t measure_sens2;
  lox.rangingTest(&measure_sens2, false);
  if (measure_sens2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Sens2(mm): "); Serial.print(measure_sens2.RangeMilliMeter); Serial.print(" | ");
  } else {
    Serial.print("Sens1(mm): "); Serial.print("**"); Serial.print(" | ");
  }
  digitalWrite(XS_SENS2, LOW);

  // SENSOR 3 INITIALIZATION AND DATA GRABBING
  digitalWrite(XS_SENS3, HIGH);
  if (!lox.begin(0x32)) {
    Serial.println(F("Failed to boot VL53L0X Sens 3"));
    while(1);
  }
  VL53L0X_RangingMeasurementData_t measure_sens3;
  lox.rangingTest(&measure_sens3, false);
  if (measure_sens3.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Sens3(mm): "); Serial.print(measure_sens3.RangeMilliMeter); Serial.println(" ");
  } else {
    Serial.print("Sens1(mm): "); Serial.print("**"); Serial.println(" ");
  }
  digitalWrite(XS_SENS3, LOW);
}
