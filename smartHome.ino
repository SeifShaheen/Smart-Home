#include <ArduinoBLE.h>
#include <Servo.h>
unsigned long globalTime = millis();
//=====================Bluetooth=================================
const char* password = "1234";  // Predefined password
String receivedData = "";

bool isAuthenticated = false;

int AC = 8;      // Pin for LED 1
int lights = 9;  // Pin for LED 2

// BLE UUIDs
const char* serviceUuid = "12345678-1234-1234-1234-123456789abc";
const char* characteristicUuid = "87654321-4321-4321-4321-cba987654321";

BLEService smartHomeService(serviceUuid);
BLEStringCharacteristic smartHomeCharacteristic(characteristicUuid, BLERead | BLEWrite, 32);
//=====================Ultrasonic=================================
int trigPin = 6;
int echoPin = 5;
int flag = 0;
long time;
float distance;
//=====================Water Level=================================
int readpin = A4;
int readValue;
int full = 11;
int empty = 10;
//=====================Flame Sensor=================================
// const int flameSensorPin = 7;
//=====================Gas Sensor=================================
const int alarm = 4;
int led = 12;
int smokeLevel = A0;
int threVal = 300;
//=====================Parking Sensor=================================
int parkingTrigPin = 3;
int ParkingEchoPin = 2;
Servo parkingServo;
int close = 50;
int open = 160;
//=====================DOOR Sensor=================================
int doorClose = 15;
int doorOpen = 120;
Servo doorServo;
void setup() {
  //=====================Bluetooth=================================
  pinMode(AC, OUTPUT);
  pinMode(lights, OUTPUT);

  digitalWrite(AC, LOW);
  digitalWrite(lights, LOW);

  Serial.begin(9600);

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1)
      ;
  }

  BLE.setLocalName("SmartHome");
  BLE.setAdvertisedService(smartHomeService);
  smartHomeService.addCharacteristic(smartHomeCharacteristic);
  BLE.addService(smartHomeService);
  BLE.advertise();
  Serial.println("BLE is ready and advertising!");
  //=====================Ultrasonic=================================
  pinMode(trigPin, OUTPUT);  // SETTING OUTPUT PIN
  pinMode(echoPin, INPUT);   // SETTING INPUT PIN
  //=====================Water Level=================================
  pinMode(readpin, INPUT);
  pinMode(full, OUTPUT);
  pinMode(empty, OUTPUT);
  //=====================Flame Sensor=================================
  // pinMode(flameSensorPin, INPUT);
  // pinMode(alarm, OUTPUT);
  //=====================Gas Sensor=================================
  pinMode(smokeLevel, INPUT);
  pinMode(led, OUTPUT);
  //=====================Parking Sensor=================================
  parkingServo.attach(13);
  pinMode(parkingTrigPin, OUTPUT);  // SETTING OUTPUT PIN
  pinMode(ParkingEchoPin, INPUT);
  //=====================DOOR Sensor=================================
  doorServo.attach(7);
  doorServo.write(doorClose);
}

void loop() {
  //=====================Bluetooth=================================
  BLEDevice central = BLE.central();

  if (central) {
    Serial.println("Connected to a central device!");

    while (central.connected()) {
      if (smartHomeCharacteristic.written()) {
        String receivedValue = smartHomeCharacteristic.value();

        if (!isAuthenticated) {
          if (receivedValue == password) {
            smartHomeCharacteristic.writeValue("true");
            isAuthenticated = true;
            Serial.println("Authentication successful!");
          } else {
            smartHomeCharacteristic.writeValue("false");
            Serial.println("Authentication failed!");
          }
        } else {
          // Control LEDs based on received commands
          if (receivedValue == "LED1_ON\n") {
            digitalWrite(AC, HIGH);
          } else if (receivedValue == "LED1_OFF\n") {
            digitalWrite(AC, LOW);
          } else if (receivedValue == "LED2_ON\n") {
            digitalWrite(lights, HIGH);
          } else if (receivedValue == "LED2_OFF\n") {
            digitalWrite(lights, LOW);
          } else if (receivedValue == "OPEN_DOOR\n") {
            doorServo.write(doorOpen);
          } else if (receivedValue == "CLOSE_DOOR\n") {
            doorServo.write(doorClose);
          }
        }
      }

      //=====================Ultrasonic=================================
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);

      // transmitting sound for 10 microseconds
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // calculating distance
      time = pulseIn(echoPin, HIGH);
      Serial.print("time: ");
      Serial.println(time);
      distance = time * 0.0343 / 2;
      // Printing out the final output => distance
      Serial.print("Distance:");
      Serial.println(distance);
      if (distance < 6 && !flag && millis() - globalTime > 5000) {
        globalTime = millis();
        digitalWrite(lights, HIGH);
        flag = 1;
      } else if (distance < 6 && flag && millis() - globalTime > 5000) {
        globalTime = millis();
        digitalWrite(lights, LOW);
        flag = 0;
      }
      //=====================Water Level=================================

      readValue = analogRead(readpin);  // put your main code here, to run repeatedly:
      if (readValue >= 630) {
        digitalWrite(full, HIGH);
        digitalWrite(empty, LOW);
      } else if (readValue <= 350) {
        digitalWrite(empty, HIGH);
        digitalWrite(full, LOW);
      }
      Serial.println(readValue);
      //=====================Flame Sensor=================================
      // int flameDetected = digitalRead(flameSensorPin);

      // if (flameDetected == LOW) {
      //   Serial.println("Flame detected!");
      //   tone(alarm, 2000, 200);
      //   delay(500);
      // } else {
      //   Serial.println("No flame detected.");
      //   noTone(alarm);
      // }

      //=====================Gas Sensor=================================
      int sensorVal = analogRead(smokeLevel);
      if (sensorVal > threVal) {
        sensorVal = analogRead(smokeLevel);
        tone(alarm, 2000, 200);
        digitalWrite(led, HIGH);
        delay(100);
        digitalWrite(led, LOW);
        delay(100);
        Serial.println(sensorVal);

      } else {
        noTone(alarm);
        digitalWrite(led, LOW);
      }
      Serial.println(sensorVal);
      //=====================Parking Sensor=================================
      digitalWrite(parkingTrigPin, LOW);
      delayMicroseconds(2);

      // transmitting sound for 10 microseconds
      digitalWrite(parkingTrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(parkingTrigPin, LOW);

      // calculating distance
      time = pulseIn(ParkingEchoPin, HIGH);
      Serial.print("time: ");
      Serial.println(time);
      distance = time * 0.0343 / 2;
      // Printing out the final output => distance
      Serial.print("Distance:");
      Serial.println(distance);
      if (distance < 20) {
        globalTime = millis();
        parkingServo.write(open);
      }
      if (millis() - globalTime > 5000) {
        parkingServo.write(close);
      }
    }
    //=====================Ultrasonic=================================
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // transmitting sound for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // calculating distance
    time = pulseIn(echoPin, HIGH);
    Serial.print("time: ");
    Serial.println(time);
    distance = time * 0.0343 / 2;
    // Printing out the final output => distance
    Serial.print("Distance:");
    Serial.println(distance);
    if (distance < 6 && !flag && millis() - globalTime > 5000) {
      globalTime = millis();
      digitalWrite(lights, HIGH);
      flag = 1;
    } else if (distance < 6 && flag && millis() - globalTime > 5000) {
      globalTime = millis();
      digitalWrite(lights, LOW);
      flag = 0;
    }
    //=====================Water Level=================================

    readValue = analogRead(readpin);  // put your main code here, to run repeatedly:
    if (readValue >= 630) {
      digitalWrite(full, HIGH);
      digitalWrite(empty, LOW);
    } else if (readValue <= 350) {
      digitalWrite(empty, HIGH);
      digitalWrite(full, LOW);
    }
    Serial.println(readValue);
    //=====================Flame Sensor=================================
    // int flameDetected = digitalRead(flameSensorPin);

    // if (flameDetected == LOW) {
    //   Serial.println("Flame detected!");
    //   tone(alarm, 2000, 200);
    // } else {
    //   Serial.println("No flame detected.");
    //   noTone(alarm);
    // }

    // delay(500);
    //=====================Gas Sensor=================================
    int sensorVal = analogRead(smokeLevel);
    if (sensorVal > threVal) {
      sensorVal = analogRead(smokeLevel);
      tone(alarm, 2000, 200);
      digitalWrite(led, HIGH);
      delay(100);
      digitalWrite(led, LOW);
      delay(100);
      Serial.println(sensorVal);
    } else if (sensorVal <= threVal) {
      noTone(alarm);
      digitalWrite(led, LOW);
    }
    Serial.println(sensorVal);
    //=====================Parking Sensor=================================
    digitalWrite(parkingTrigPin, LOW);
    delayMicroseconds(2);

    // transmitting sound for 10 microseconds
    digitalWrite(parkingTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(parkingTrigPin, LOW);

    // calculating distance
    time = pulseIn(ParkingEchoPin, HIGH);
    Serial.print("time: ");
    Serial.println(time);
    distance = time * 0.0343 / 2;
    // Printing out the final output => distance
    Serial.print("Distance:");
    Serial.println(distance);
    if (distance < 20) {
      globalTime = millis();
      parkingServo.write(open);
    }
    if (millis() - globalTime > 5000) {
      parkingServo.write(close);
    }
  }
  Serial.println("Disconnected from central device.");
  isAuthenticated = false;  // Reset authentication for the next session

  //=====================Ultrasonic=================================
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // transmitting sound for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // calculating distance
  time = pulseIn(echoPin, HIGH);
  Serial.print("time: ");
  Serial.println(time);
  distance = time * 0.0343 / 2;
  // Printing out the final output => distance
  Serial.print("Distance:");
  Serial.println(distance);
  if (distance < 6 && !flag && millis() - globalTime > 5000) {
    globalTime = millis();
    digitalWrite(lights, HIGH);
    flag = 1;
  } else if (distance < 6 && flag && millis() - globalTime > 5000) {
    globalTime = millis();
    digitalWrite(lights, LOW);
    flag = 0;
  }
  //=====================Water Level=================================

  readValue = analogRead(readpin);  // put your main code here, to run repeatedly:
  if (readValue >= 630) {
    digitalWrite(full, HIGH);
    digitalWrite(empty, LOW);
  } else if (readValue <= 350) {
    digitalWrite(empty, HIGH);
    digitalWrite(full, LOW);
  }
  Serial.println(readValue);
  //=====================Flame Sensor=================================
  // int flameDetected = digitalRead(flameSensorPin);

  // if (flameDetected == LOW) {
  //   Serial.println("Flame detected!");
  //   tone(alarm, 2000, 200);
  //   delay(500);
  // } else {
  //   Serial.println("No flame detected.");
  //   noTone(alarm);
  // }

  // delay(500);
  //=====================Gas Sensor=================================
  int sensorVal = analogRead(smokeLevel);
  while (sensorVal > threVal) {
    sensorVal = analogRead(smokeLevel);
    tone(alarm, 2000, 200);
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
    Serial.println(sensorVal);
  }
  if (sensorVal <= threVal) {
    noTone(alarm);
    digitalWrite(led, LOW);
  }
  Serial.println(sensorVal);
  //=====================Parking Sensor=================================
  digitalWrite(parkingTrigPin, LOW);
  delayMicroseconds(2);

  // transmitting sound for 10 microseconds
  digitalWrite(parkingTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(parkingTrigPin, LOW);

  // calculating distance
  time = pulseIn(ParkingEchoPin, HIGH);
  Serial.print("time: ");
  Serial.println(time);
  distance = time * 0.0343 / 2;
  // Printing out the final output => distance
  Serial.print("Distance:");
  Serial.println(distance);
  if (distance < 20) {
    globalTime = millis();
    parkingServo.write(open);
  }
  if (millis() - globalTime > 5000) {
    parkingServo.write(close);
  }
}
