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

int PRES_PIN_1 = A1;
int PRES_PIN_2 = A2;
int TURB_PIN = A3;
int ORP_PIN =  A4;
int PH_PIN = A4;
int FLOW_PIN_1 = 2;
int FLOW_PIN_2 = 3;
int VALVE_BUTTON_PIN_1 = 6;
int VALVE_BUTTON_PIN_2 = 3;
int VALVE_RELAY_PIN_1 = 4;
int VALVE_RELAY_PIN_2 = 5;
int PUMP_RELAY_PIN_1 = 7; // Controls Pump 1
int PUMP_RELAY_PIN_2 = 8; // Controls Pump 2
float PRES_CAL_1 = 0.483398;
float PRES_CAL_2 = 0.478516;
float TURB_CAL = 0.07;
float PRES_1 = 0;
double PRES_1_SUM = 0;
double PRES_2_SUM = 0;
int PRES_SAMPLES;
float PRES_2 = 0;
float TURB = 0;
float ORP = 0;
float PH = 0;
float WATER_TEMP = 25;
float FLOW_1 = 0;
float FLOW_2 = 0;
volatile int FLOW_COUNT_1 = 0;
volatile int FLOW_COUNT_2 = 0;
DynamicJsonDocument JSON_DOC(1024);
DynamicJsonDocument INPUT_JSON(1024);
DFRobot_ORP_PRO ORPCalc(-14);
DFRobot_PH PhCalc;
bool TRANSMIT_STATISTICS = true;
bool FILTER = false;
bool FILTER_BUTTON = false;
bool DISPENSE = false;
bool DISPENSE_BUTTON = false;
unsigned long LAST_FLOW_READ_1 = millis();
unsigned long LAST_FLOW_READ_2 = millis();
int LOOP_DURATION = 500;
int LOOPS = 50;
String PUMP_1 = String("NORMAL");
String PUMP_2 = "NORMAL";
String VALVE_1 = "NORMAL";
String VALVE_2 = "NORMAL";


void setup() {
  // put your setup code here, to run once:
  pinMode(VALVE_BUTTON_PIN_1, INPUT);
  pinMode(VALVE_BUTTON_PIN_2, INPUT);
  pinMode(VALVE_RELAY_PIN_1, OUTPUT);
  pinMode(VALVE_RELAY_PIN_2, OUTPUT);
  pinMode(PUMP_RELAY_PIN_1, OUTPUT);
  pinMode(PUMP_RELAY_PIN_2, OUTPUT);
  pinMode(FLOW_PIN_1, INPUT_PULLUP);
  digitalWrite(VALVE_RELAY_PIN_1, HIGH);
  digitalWrite(VALVE_RELAY_PIN_2, HIGH);
  digitalWrite(PUMP_RELAY_PIN_1, HIGH);
  digitalWrite(PUMP_RELAY_PIN_2, HIGH);
  Serial.begin(9600);
  Serial.setTimeout(100);
  PhCalc.begin();
  while(!Serial) continue;
  Serial.println("Initialising");
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN_1), countFlow1, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN_2), countFlow2, RISING);
  sei();
}

int countFlow1() {
  FLOW_COUNT_1++;
}

int countFlow2() {
  FLOW_COUNT_2++;
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

// float getFlowRate(volatile int * countPtr, unsigned long duration) {
//   LAST_FLOW_READ_1 = cur;
//   int count = &countPtr;
//   float rate = (count / 7.5  * 1000 / duration);
//   count = 0;
//   return rate;
// }

float getFlowRate1() {
  unsigned long cur = millis();
  int duration = cur - LAST_FLOW_READ_1;
  LAST_FLOW_READ_1 = cur;
  float rate = (FLOW_COUNT_1 / 7.5  * 1000 / duration);
  FLOW_COUNT_1 = 0;
  return rate;
}

float getFlowRate2() {
  unsigned long cur = millis();
  int duration = cur - LAST_FLOW_READ_2;
  LAST_FLOW_READ_2 = cur;
  float rate = (FLOW_COUNT_2 / 7.5  * 1000 / duration);
  FLOW_COUNT_2 = 0;
  return rate;
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
  double pres_1_avg = PRES_1_SUM / LOOPS;
  double pres_2_avg = PRES_2_SUM / LOOPS;
  PRES_1_SUM = 0;
  PRES_2_SUM = 0;
  JSON_DOC["Count"] = ++count;
  JSON_DOC["PRES_1"] = pres_1_avg;
  JSON_DOC["PRES_2"] = pres_2_avg;
  // JSON_DOC["TURB"] = TURB;
  // JSON_DOC["ORP"] = ORP;
  // JSON_DOC["PH"] = PH;
  JSON_DOC["FLOW_1"] = FLOW_1;
}

void readSensors() {
  PRES_1_SUM += getWaterPressure(PRES_PIN_1, PRES_CAL_1);
  PRES_2_SUM += getWaterPressure(PRES_PIN_2, PRES_CAL_2);
  PRES_1 = getWaterPressure(PRES_PIN_1, PRES_CAL_1);
  PRES_2 = getWaterPressure(PRES_PIN_2, PRES_CAL_2);
  TURB = getTurbidity(TURB_PIN);
  ORP = getOrp(ORP_PIN);
  PH = getPh(PH_PIN);
  FLOW_1 = getFlowRate1();
}

void getInput() {

  FILTER_BUTTON = digitalRead(VALVE_BUTTON_PIN_1) == HIGH;
  DISPENSE_BUTTON = digitalRead(VALVE_BUTTON_PIN_2) == HIGH;


  const auto deser_err = deserializeJson(INPUT_JSON, Serial);
  if (!deser_err) {
    if (INPUT_JSON.containsKey("STATS")) TRANSMIT_STATISTICS = INPUT_JSON["STATS"];
    
    if (INPUT_JSON.containsKey("FILTER")) FILTER = INPUT_JSON["FILTER"];

    if (INPUT_JSON.containsKey("DISPENSE")) DISPENSE = INPUT_JSON["DISPENSE"];

    if (INPUT_JSON.containsKey("PUMP_1")) PUMP_1 = INPUT_JSON["PUMP_1"].as<String>();

    if (INPUT_JSON.containsKey("PUMP_2")) PUMP_2 = INPUT_JSON["PUMP_2"].as<String>();

    if (INPUT_JSON.containsKey("VALVE_1")) VALVE_1 = INPUT_JSON["VALVE_1"].as<String>();

    if (INPUT_JSON.containsKey("VALVE_2")) VALVE_2 = INPUT_JSON["VALVE_2"].as<String>();
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

void overrideProgram() {
  if (PUMP_1 == "OVERRIDE_ON") {
    digitalWrite(PUMP_RELAY_PIN_1, LOW);
  } else if (PUMP_1 == "OVERRIDE_OFF") {
    digitalWrite(PUMP_RELAY_PIN_1, HIGH);
  }
  if (PUMP_2 == "OVERRIDE_ON") {
    digitalWrite(PUMP_RELAY_PIN_2, LOW);
  } else if (PUMP_2 == "OVERRIDE_OFF") {
    digitalWrite(PUMP_RELAY_PIN_2, HIGH);
  }
  if (VALVE_1 == "OVERRIDE_OPEN") {
    digitalWrite(VALVE_RELAY_PIN_1, LOW);
  } else if (VALVE_1 == "OVERRIDE_CLOSED") {
    digitalWrite(VALVE_RELAY_PIN_1, HIGH);
  }
  if (VALVE_2 == "OVERRIDE_OPEN") {
    digitalWrite(VALVE_RELAY_PIN_2, LOW);
  } else if (VALVE_2 == "OVERRIDE_CLOSED") {
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

  // Look into serial plotter! https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-plotter
  
  while (true) {
    
    for (int i=0; i<LOOPS; i++) {
      readSensors();
      delay(10);
    }
    getInput();
    readSensors();
    updateMetrics();
    // printWaterPressure(PRES_1);
    transmitMetrics();
    filter();
    dispense();
    overrideProgram();
    delay(LOOP_DURATION);
    Serial.println();
  }
}
