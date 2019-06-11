#define TINY_GSM_MODEM_SIM808
#define SerialMon Serial
#define LED_PIN 8

#include <HCSR04.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Secrets.h>

SoftwareSerial SerialAT(2, 3);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
HCSR04 distanceSensor(9, 10, 60);
MPU6050 mpu6050(Wire);
Secrets secret;

const char* apn  = secret.APN;
const char* broker = secret.BROKER_HOST;
const int port = secret.BROKER_PORT;
const char* clientID = "Developed Device";
const char* username = secret.BROKER_USERNAME;
const char* password = secret.BROKER_PASSWORD;
const char* topicUltraGyro = "GarbageBin/1/UltraGyro";
const char* topicGPS = "GarbageBin/1/GPS";
long lastReconnectAttempt = 0;
long lastSend = 0;

const float garbageBinHeight = 60.0;
float level = 0.0;
float prevLevel = 0.0;
float latitude = 0.0;
float longitude = 0.0;
float prevLatitude = 0.0;
float prevLongitude = 0.0;
float angle = 0.0;
int overflowAngle = 45;

bool sameCoordFlag = false;
bool lidFlags[4] = {false, false, false, false};

void setup() {
  SerialMon.begin(9600);
  delay(10);
  SerialAT.begin(9600);
  delay(3000);
  Wire.begin();

  SerialMon.println("Initializing");
  modem.restart();

  bool networkFlag = false;
  while (networkFlag == false) {
    SerialMon.print("Network Connection: ");
    if (!modem.waitForNetwork()) {
      SerialMon.println("FAIL");
    } else {
      SerialMon.println("PASS");
      networkFlag = true;
    }
  }

  bool apnFlag = false;
  while (apnFlag == false)
  {
    SerialMon.print("Internet Connection: ");
    if (!modem.gprsConnect(apn, "", "")) {
      SerialMon.println("FAIL");
    } else {
      SerialMon.println("PASS");
      apnFlag = true;
    }
  }

  modem.enableGPS();
  mqtt.setServer(broker, port);
  mqtt.setCallback(mqttCallback);
  pinMode(LED_PIN, OUTPUT);

  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
}

void loop() {
  mpu6050.update();
  if (!mqtt.connected()) {
    SerialMon.println("Broker Connection: FAIL");
    digitalWrite(LED_PIN, LOW);
    unsigned long t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        digitalWrite(LED_PIN, HIGH);
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  else {
    unsigned long z = millis();
    if (z - lastSend > 3000L) {
      lastSend = z;

      if (modem.getGPS(&latitude, &longitude)) {
        if (abs(prevLatitude - latitude) > 0.000003 && abs(prevLongitude - longitude) > 0.000003) {
          Serial.println("Latitude,Longitude: " + String(latitude) + ',' + String(longitude));
          mqtt.publish(topicGPS, (String(latitude) + ',' + String(longitude)).c_str());
          prevLatitude = latitude;
          prevLongitude = longitude;
          sameCoordFlag = false;
        }
      } else {
        if (!sameCoordFlag) {
          Serial.println("Latitude,Longitude: 14.641897,121.002980");
          mqtt.publish(topicGPS, "14.641897,121.002980");
          sameCoordFlag = true;
        }
      }

      angle = abs(mpu6050.getAngleY());
      SerialMon.println("Angle: " + String(angle));
      if (angle <= 2) {
        level = abs(((garbageBinHeight - distanceSensor.ping_median(11)) / garbageBinHeight) * 100);
        if (!lidFlags[0]) {
          lidFlags[0] = true;
          lidFlags[1] = false;
          lidFlags[2] = false;
          lidFlags[3] = false;
          SerialMon.println("Level,Lid: " + String(level) + ',' + "Closed");
          mqtt.publish(topicUltraGyro, (String(level) + ',' + "Closed").c_str());
        } else if (abs(prevLevel - level) > 3) {
          SerialMon.println("Level,Lid: " + String(level) + ',' + "Closed");
          mqtt.publish(topicUltraGyro, (String(level) + ',' + "Closed").c_str());
          prevLevel = level;
        }
      } else if (angle >= 2) {
        level = abs(((garbageBinHeight - distanceSensor.ping_median(11)) / garbageBinHeight) * 100);
        if (angle <= overflowAngle && level >= 90) {
          if (!lidFlags[2]) {
            lidFlags[0] = false;
            lidFlags[1] = false;
            lidFlags[2] = true;
            lidFlags[3] = false;
            SerialMon.println("Level,Lid: 100.1,Overflow");
            mqtt.publish(topicUltraGyro, "100.1,Overflow");
          }
        } else if (angle <= 45) {
          if (!lidFlags[3]) {
            lidFlags[0] = false;
            lidFlags[1] = false;
            lidFlags[2] = false;
            lidFlags[3] = true;
            Serial.println("Level,Lid: " + String(level) + ',' + "Partial");
            mqtt.publish(topicUltraGyro, (String(level) + ',' + "Partial").c_str());
          }
          if (abs(prevLevel - level) > 3) {
            Serial.println("Level,Lid: " + String(level) + ',' + "Partial");
            mqtt.publish(topicUltraGyro, (String(level) + ',' + "Partial").c_str());
            prevLevel = level;
          }
        } else {
          if (!lidFlags[1]) {
            lidFlags[0] = false;
            lidFlags[1] = true;
            lidFlags[2] = false;
            lidFlags[3] = false;
            SerialMon.println("Level,Lid: 0,Open");
            mqtt.publish(topicUltraGyro, "0,Open");
          }
        }
      }
    }
  }
  mqtt.loop();
}

boolean mqttConnect() {
  SerialMon.print("Broker Connection: ");
  boolean status = mqtt.connect(clientID, username, password, NULL, 2, NULL, NULL);
  if (status == false) {
    SerialMon.println("FAIL");
    return false;
  }
  SerialMon.println("PASS");
  mqtt.subscribe("GarbageBin/1/OverflowAngle");
  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  if (String(topic) == "GarbageBin/1/OverflowAngle") {
    Serial.println(payload[0] + payload[1]);
    byte temp = payload[0] + payload[1];
    if (temp == 99) {
      overflowAngle = 30;
    } else if (temp == 105) {
      overflowAngle = 45;
    } else {
      overflowAngle = 60;
    }
  }
}

