#include <ArduinoJson.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) continue;
  Serial.println("Finding world");
}

int count = 0;

void loop() {
  // put your main code here, to run repeatedly:

  DynamicJsonDocument doc(1024);
  int count = 0;

  doc["Hello"] = "World";

  while (true) {
    Serial.println("Hello World");
    doc["Count"] = count++;
    serializeJson(doc, Serial);
    delay(5000);
  }
}
