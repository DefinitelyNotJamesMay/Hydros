// This sketch allows 2 active low relays to be controlled using serial commands, and writes ON/OFF status to an LCD.
// It also reads a flow rate sensor and writes the value to the LCD.
// Alex Yiannakou 12/05/2023

#include <LiquidCrystal.h>

int flowSensorPin = 2;
int relay1Pin = 3; // Controls Pump 1
int relay2Pin = 4; // Controls Pump 2
int relay3Pin = 5; // Controls Valve 1
int relay4Pin = 6; // Controls Valve 2
volatile int flowPulseCount = 0;

int rs = 8;  // LCD pin 4
int en = 9;  // LCD pin 6
int d4 = 10; // LCD pin 11
int d5 = 11; // LCD pin 12
int d6 = 12; // LCD pin 13
int d7 = 13; // LCD pin 14

LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Initialise the LCD

void setup() {

  // Set up relay, flow sensor, and interrupt pins
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowPulse, RISING); // Attach inttupt to flow sensor

  // Initialise relays to OFF
  digitalWrite(relay1Pin, HIGH);
  digitalWrite(relay2Pin, HIGH);
  digitalWrite(relay3Pin, HIGH);
  digitalWrite(relay4Pin, HIGH);

  Serial.begin(9600); // Open serial port
  
  lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows
  lcd.setCursor(0, 0); // Set cursor to first position
  lcd.print("P1: OFF P2: OFF");
}

void loop() {
  if (Serial.available() > 0) {   // Checks if any data is waiting in the serial port input buffer
    String inputString = Serial.readStringUntil('\n');   // If data is waiting, read the input string

    // Turn relays ON/OFF and print status to LCD based on serial commands
    if (inputString == "p1on") {
      digitalWrite(relay1Pin, LOW);
      Serial.println("Pump 1: ON");
      lcd.setCursor(4, 0);
      lcd.print("ON ");
    }
    else if (inputString == "p1off") {
      digitalWrite(relay1Pin, HIGH);
      Serial.println("Pump 1: OFF");
      lcd.setCursor(4, 0);
      lcd.print("OFF");
    }
    else if (inputString == "p2on") {
      digitalWrite(relay2Pin, LOW);
      Serial.println("Pump 2: ON");
      lcd.setCursor(12, 0);
      lcd.print("ON ");
    }
    else if (inputString == "p2off") {
      digitalWrite(relay2Pin, HIGH);
      Serial.println("Pump 2: OFF");
      lcd.setCursor(12, 0);
      lcd.print("OFF");
    }
    else if (inputString == "v1on") {
      digitalWrite(relay3Pin, LOW);
      Serial.println("Valve 1: ON");
    }
    else if (inputString == "v1off") {
      digitalWrite(relay3Pin, HIGH);
      Serial.println("Valve 1: OFF");
    }
    else if (inputString == "v2on") {
      digitalWrite(relay4Pin, LOW);
      Serial.println("Valve 2: ON");
    }
    else if (inputString == "v2off") {
      digitalWrite(relay4Pin, HIGH);
      Serial.println("Valve 2: OFF");
    }
    else if (inputString == "alloff") {
      digitalWrite(relay1Pin, HIGH);
      digitalWrite(relay2Pin, HIGH);
      digitalWrite(relay3Pin, HIGH);
      digitalWrite(relay4Pin, HIGH);
      Serial.println("All Components Off");
    }
  }
  
  // Read flow sensor and update LCD
  float flowRate = getFlowRate();
  lcd.setCursor(0, 1);
  lcd.print("Flow: ");
  lcd.print(flowRate);
  lcd.print(" L/min ");
}

void flowPulse() {
  flowPulseCount++;
}

float getFlowRate() {
  static unsigned long lastTime = 0;
  static float lastFlowRate = 0;
  unsigned long now = millis();
  float flowRate = 0;
  
  if (now - lastTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));
    flowRate = (flowPulseCount * 2.25) / (now - lastTime) * 1000;
    lastFlowRate = flowRate;
    lastTime = now;
    flowPulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(flowSensorPin), flowPulse, RISING);
  } else {
    flowRate = lastFlowRate;
  }
  
  return flowRate;
}
