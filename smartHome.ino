#include <ArduinoBLE.h>
#include <Servo.h>

unsigned long globalTimeGarage = 0;
unsigned long globalTimeDoor = 0;

//=====================Bluetooth=================================
const char* password = "1234";
bool isAuthenticated = false;
String receivedData = "";
int AC = 8;
int lights = 9;
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
int doorClose = 20;
int doorOpen = 120;
Servo doorServo;

void setup() {
  // Bluetooth
  pinMode(AC, OUTPUT);
  pinMode(lights, OUTPUT);
  digitalWrite(AC, LOW);
  digitalWrite(lights, LOW);

  Serial.begin(9600);

  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("SmartHome");
  BLE.setAdvertisedService(smartHomeService);
  smartHomeService.addCharacteristic(smartHomeCharacteristic);
  BLE.addService(smartHomeService);
  BLE.advertise();
  Serial.println("BLE is ready and advertising!");

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Water level
  pinMode(readpin, INPUT);
  pinMode(full, OUTPUT);
  pinMode(empty, OUTPUT);

  // Gas Sensor
  pinMode(smokeLevel, INPUT);
  pinMode(led, OUTPUT);
  pinMode(alarm, OUTPUT);

  // Parking Sensor
  parkingServo.attach(13);
  pinMode(parkingTrigPin, OUTPUT);
  pinMode(ParkingEchoPin, INPUT);

  // Door
  doorServo.attach(7);
  doorServo.write(doorClose);
}

void handleBluetooth() {
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
          if (receivedValue == "LED1_ON\n") digitalWrite(AC, HIGH);
          else if (receivedValue == "LED1_OFF\n") digitalWrite(AC, LOW);
          else if (receivedValue == "LED2_ON\n") digitalWrite(lights, HIGH);
          else if (receivedValue == "LED2_OFF\n") digitalWrite(lights, LOW);
          else if (receivedValue == "OPEN_DOOR\n") doorServo.write(doorOpen);
          else if (receivedValue == "CLOSE_DOOR\n") doorServo.write(doorClose);
        }
      }

      runSensors(); // Keep sensors running during Bluetooth connection
    }

    Serial.println("Disconnected from central device.");
    isAuthenticated = false;
  }
}

void runSensors() {
  // -------- Ultrasonic (for lights) --------
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  time = pulseIn(echoPin, HIGH);
  distance = time * 0.0343 / 2;
  Serial.print("Ultrasonic Distance: "); Serial.println(distance);

  if (distance < 6 && millis() - globalTimeDoor > 5000) {
    globalTimeDoor = millis();
    flag = !flag;
    digitalWrite(lights, flag ? HIGH : LOW);
  }

  // -------- Water Level --------
  readValue = analogRead(readpin);
  if (readValue >= 630) {
    digitalWrite(full, HIGH);
    digitalWrite(empty, LOW);
  } else if (readValue <= 350) {
    digitalWrite(empty, HIGH);
    digitalWrite(full, LOW);
  }
  Serial.print("Water Level: "); Serial.println(readValue);

  // -------- Gas Sensor --------
  int sensorVal = analogRead(smokeLevel);
  Serial.print("Gas Sensor: "); Serial.println(sensorVal);
  if (sensorVal > threVal) {
    tone(alarm, 2000, 200);
    digitalWrite(led, HIGH); delay(100);
    digitalWrite(led, LOW); delay(100);
  } else {
    noTone(alarm);
    digitalWrite(led, LOW);
  }

  // -------- Parking Sensor --------
  digitalWrite(parkingTrigPin, LOW); delayMicroseconds(2);
  digitalWrite(parkingTrigPin, HIGH); delayMicroseconds(10);
  digitalWrite(parkingTrigPin, LOW);

  time = pulseIn(ParkingEchoPin, HIGH);
  distance = time * 0.0343 / 2;
  Serial.print("Parking Distance: "); Serial.println(distance);

  if (distance < 20) {
    globalTimeGarage = millis();
    parkingServo.write(open);
  }
  if (millis() - globalTimeGarage > 5000) {
    parkingServo.write(close);
  }
}

void loop() {
  handleBluetooth();  // Bluetooth connection + commands
  runSensors();       // Keep sensors running if not connected
}
