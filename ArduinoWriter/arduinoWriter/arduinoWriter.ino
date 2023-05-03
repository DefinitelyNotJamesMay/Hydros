#include <ArduinoJson.h>
#include "DFRobot_ORP_PRO.h"
#include "DFRobot_PH.h"

// Water Pressure Sensor Wiki
// https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257

// Water Turbidity Sensor Wiki
// https://wiki.dfrobot.com/Turbidity_sensor_SKU__SEN0189

// Water Flow Rate Sensor Docs
// https://www.hobbytronics.co.uk/datasheets/sensors/YF-S201.pdf
// https://www.hobbytronics.co.uk/yf-s201-water-flow-meter?gclid=Cj0KCQjw3a2iBhCFARIsAD4jQB3m8NgQ9UUeDcLTjy7j_B-LwBIhLd6MbNCakVnXvN3H1UzxBZN9XnYaAnpWEALw_wcB

// Water pH Sensor Docs
// https://wiki.dfrobot.com/Gravity__Analog_pH_Sensor_Meter_Kit_V2_SKU_SEN0161-V2

// Water ORP Sensor Docs
// https://wiki.dfrobot.com/Gravity_Analog_ORP_Sensor_PRO_SKU_SEN0464



int count = 0;

int PRES_PIN_1 = A6;
int PRES_PIN_2 = A7;
int TURB_PIN = A9;
int ORP_PIN =  A12;
int PH_PIN = A15;
int FLOW_PIN_1 = 9;
int FLOW_PIN_2 = 3;
int VALVE_BUTTON_PIN_1 = 2;
int VALVE_BUTTON_PIN_2 = 3;
int VALVE_RELAY_PIN_1 = 4;
int VALVE_RELAY_PIN_2 = 5;
float PRES_CAL_1 = 0.483398;
float PRES_CAL_2 = 0.478516;
float TURB_CAL = 0.07;
float PRES_1 = 0;
float PRES_2 = 0;
float TURB = 0;
float ORP = 0;
float PH = 0;
float WATER_TEMP = 25;
float FLOW_1 = 0;
float FLOW_2 = 0;
volatile int FLOW_COUNT = 10;
int LAST_FLOW_READ = millis();
DynamicJsonDocument JSON_DOC(1024);
DynamicJsonDocument INPUT_JSON(1024);
DFRobot_ORP_PRO ORPCalc(-14);
DFRobot_PH PhCalc;
bool TRANSMIT_STATISTICS = true;
bool FILTER = false;
bool FILTER_BUTTON = false;
bool DISPENSE = false;
bool DISPENSE_BUTTON = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(VALVE_BUTTON_PIN_1, INPUT);
  pinMode(VALVE_BUTTON_PIN_2, INPUT);
  pinMode(VALVE_RELAY_PIN_1, OUTPUT);
  pinMode(VALVE_RELAY_PIN_2, OUTPUT);
  pinMode(FLOW_PIN_1, INPUT);
  digitalWrite(VALVE_RELAY_PIN_1, HIGH);
  digitalWrite(VALVE_RELAY_PIN_2, HIGH);
  Serial.begin(9600);
  Serial.setTimeout(100);
  PhCalc.begin();
  while(!Serial) continue;
  Serial.println("Initialising");
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN_1), countFlow, RISING);
}

int countFlow() {
  FLOW_COUNT++;
}

float getWaterPressure(int port, double offset) {
  float v = analogRead(port) * 5.00 / 1024;     //Sensor output voltage
  float p = (v - offset) * 250 / 100;             //Calculate water pressure
  return p;
}

float getWaterPressureCal(int port) {
  float lowest = 10000.00;
  for (int i=0; i<10; i++) {
    lowest = min(analogRead(port), lowest);
    delay(100);
  }
  return lowest * 5.00 / 1024;
}

float getTurbidity(int port) {
  double voltage = (analogRead(port)* 5.0 / 1024.0);
  double turbidity = (-1120.4 * voltage * voltage) + (5742.3 * voltage) - 4352.9;
  return turbidity;
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

float getFlowRate() {
  float duration = (millis() - LAST_FLOW_READ) / 1000;
  LAST_FLOW_READ = millis();
  float flowRate = FLOW_COUNT / 7.5 / duration;
  FLOW_COUNT = 0;
  return flowRate;
}

void calibrateSensors() {
  // PRES_CAL_1 = getWaterPressureCal(PRES_PIN_1);
  // PRES_CAL_2 = getWaterPressureCal(PRES_PIN_2);
  PRES_CAL_1 = 0.48;
  PRES_CAL_2 = 0.48;
}

void transmitMetrics() {
  if (!TRANSMIT_STATISTICS) return;
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
  JSON_DOC["FLOW_1"] = FLOW_1;
}

void readSensors() {
  PRES_1 = getWaterPressure(PRES_PIN_1, PRES_CAL_1);
  PRES_2 = getWaterPressure(PRES_PIN_2, PRES_CAL_2);
  TURB = getTurbidity(TURB_PIN);
  ORP = getOrp(ORP_PIN);
  PH = getPh(PH_PIN);
  FLOW_1 = getFlowRate();
}

void getInput() {

  FILTER_BUTTON = digitalRead(VALVE_BUTTON_PIN_1) == HIGH;
  DISPENSE_BUTTON = digitalRead(VALVE_BUTTON_PIN_2) == HIGH;


  const auto deser_err = deserializeJson(INPUT_JSON, Serial);
  if (!deser_err) {
    TRANSMIT_STATISTICS = INPUT_JSON["STATS"];
    FILTER = INPUT_JSON["FILTER"];
    DISPENSE = INPUT_JSON["DISPENSE"];
  }
}

void filter() {
  if (FILTER || FILTER_BUTTON) {
    digitalWrite(VALVE_RELAY_PIN_1, LOW);
  } else {
    digitalWrite(VALVE_RELAY_PIN_1, HIGH);
  }
}

void dispense() {
  if (DISPENSE || DISPENSE_BUTTON) {
    digitalWrite(VALVE_RELAY_PIN_2, LOW);
  } else {
    digitalWrite(VALVE_RELAY_PIN_2, HIGH);
  }
}

void printDebug() {
  Serial.println("Dispense: " + DISPENSE);
  Serial.println("Dispense Button: " + DISPENSE_BUTTON);
  Serial.println("Filter: " + FILTER);
  Serial.println("Filter Button: " + FILTER_BUTTON);
}

void printWaterPressure(float press) {
  int stars = press;
  Serial.print(stars);
  for (int i=0; i<stars; i++) {
    Serial.print("*");

  }
}

void loop() {

  calibrateSensors();

  while (true) {
    getInput();
    readSensors();
    updateMetrics();
    // printWaterPressure(PRES_1);
    transmitMetrics();
    filter();
    dispense();
    delay(500);
    Serial.println();
  }
}
