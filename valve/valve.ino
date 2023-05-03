const int button1 = 14; // declare pin 11 as a constant variable
const int relay1 = 16;   // declare pin 2 as a constant variable
const int button2 = 15;
const int relay2 = 17;

void setup() {
  pinMode(button1, INPUT);  // set pin 11 as input
  pinMode(relay1, OUTPUT);  // set pin 2 as output
  pinMode(button2, INPUT);  // set pin 12 as input
  pinMode(relay2, OUTPUT);  // set pin 3 as output
  Serial.begin(9600);     // initialize serial communication
}

void loop() {
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  // if (digitalRead(button1) == HIGH) {  // check if pin 11 is high (inverted due to pull-up)
  //   digitalWrite(relay1, LOW);         // set pin 2 to low
  // } else {
  //   digitalWrite(relay1, HIGH);        // set pin 2 to high
  // }

  // if (digitalRead(button2) == HIGH) {  // check if pin 11 is high (inverted due to pull-up)
  //   digitalWrite(relay2, LOW);         // set pin 2 to low
  // } else {
  //   digitalWrite(relay2, HIGH);        // set pin 2 to high
  // }

  int button1Status = digitalRead(button1);  // read the status of button1
  int relay1Status = digitalRead(relay1);    // read the status of pin2
  Serial.print("Button 1: ");                // print the label for pin11
  Serial.print(button1Status);               // print the value of pin11
  Serial.print("\t Relay 1: ");              // print the label for pin2
  Serial.print(relay1Status);                // print the value of pin2 and move to the next line

  int button2Status = digitalRead(button2);  // read the status of pin11
  int relay2Status = digitalRead(relay2);    // read the status of pin2
  Serial.print("\t Button 2: ");             // print the label for pin11
  Serial.print(button2Status);               // print the value of pin11
  Serial.print("\t Relay 2: ");              // print the label for pin2
  Serial.println(relay2Status);              // print the value of pin2 and move to the next line

  delay(100);
}