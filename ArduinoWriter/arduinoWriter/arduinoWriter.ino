#include <ArduinoJson.h>
#include "DFRobot_ORP_PRO.h"
#include "DFRobot_PH.h"

// Water Pressure Sensor Wiki
// https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257

// Water Turbidity Sensor Wiki
// https://wiki.dfrobot.com/Turbidity_sensor_SKU__SEN0189

// Water Flow Rate Sensor Docs
// https://www.hobbytronics.co.uk/datasheets/sensors/YF-S201.pdf

// Water pH Sensor Docs
// https://wiki.dfrobot.com/Gravity__Analog_pH_Sensor_Meter_Kit_V2_SKU_SEN0161-V2

// Water ORP Sensor Docs
// https://wiki.dfrobot.com/Gravity_Analog_ORP_Sensor_PRO_SKU_SEN0464



int count = 0;

int PRES_PIN_1 = 6;
int PRES_PIN_2 = 7;
int TURB_PIN = 9;
int ORP_PIN =  11;
int PH_PIN = 13;
float PRES_CAL_1 = 5.00;
float PRES_CAL_2 = 5.00;
float PRES_1 = 0;
float PRES_2 = 0;
float TURB = 0;
float ORP = 0;
float PH = 0;
float WATER_TEMP = 25;

DynamicJsonDocument JSON_DOC(1024);

DFRobot_ORP_PRO ORPCalc(0);

DFRobot_PH PhCalc;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  PhCalc.begin();
  while(!Serial) continue;
  Serial.println("Initialising");
}

float getWaterPressure(int port, double offset) {
  float v = analogRead(port) * 5.00 / 1024;     //Sensor output voltage
  float p = (v - offset) * 250;             //Calculate water pressure
  return p;
}

float getWaterPressureCal(int port) {
  float lowest = 10000;
  for (int i=0; i<10; i++) {
    lowest = min(analogRead(port), lowest);
    delay(200);
  }
  return lowest * 5.00 / 1024;
}

float getTurbidityVoltage(int port) {
  return analogRead(port) * (5.0 / 1024.0);
}

float getOrp(int port) {
  float voltage = ((unsigned long)analogRead(ORP_PIN) * 5000 + 1024 / 2) / 1024;
  float orp = ORPCalc.getORP(voltage);
  return orp;
}

float getPh(int port) {
  float voltage = analogRead(port)/1024.0*5000;
  float phValue = PhCalc.readPH(voltage, WATER_TEMP);
  return phValue;
}

void calibrateSensors() {
  PRES_CAL_1 = getWaterPressureCal(PRES_PIN_1);
  PRES_CAL_2 = getWaterPressureCal(PRES_PIN_2);
}

void transmitMetrics() {
    serializeJson(JSON_DOC, Serial);
}

void updateMetrics() {
  JSON_DOC["Count"] = ++count;
  JSON_DOC["PRES_CAL_1"] = PRES_CAL_1;
  JSON_DOC["PRES_1"] = PRES_1;
  JSON_DOC["PRES_CAL_2"] = PRES_CAL_2;
  JSON_DOC["PRES_2"] = PRES_2;
  JSON_DOC["TURB"] = TURB;
  JSON_DOC["ORP"] = ORP;
  JSON_DOC["PH"] = PH;
}

void readSensors() {
  PRES_1 = getWaterPressure(PRES_PIN_1, PRES_CAL_1);
  PRES_2 = getWaterPressure(PRES_PIN_2, PRES_CAL_2);
  TURB = getTurbidityVoltage(TURB_PIN);
  ORP = getOrp(ORP_PIN);
  PH = getPh(PH_PIN);
}

void loop() {

  calibrateSensors();

  while (true) {
    readSensors();
    updateMetrics();
    transmitMetrics();
    Serial.println();
    delay(5000);
  }
}
