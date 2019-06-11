//Definition - MPU6050
#define INTERRUPT_PIN 2
//Definition - SIM808
#define TINY_GSM_MODEM_SIM808
//Definition - Misc
#define LED_PIN 8

//Library - MPU6050
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
//Library - SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
//Library - HC-SR04
#include <HCSR04.h>
//Library - Secrets
#include <Secrets.h>

//Constructor - MPU6050
MPU6050 mpu;
//Constructor - SIM808
SoftwareSerial SerialAT(10, 11);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
//Constructor - HC-SR04
HCSR04 ultrasonic(9, 7, 33);
//Struct - Secrets
Secrets secret;

//Variables - MPU6050
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
uint8_t angle = 255;
//Variables - SIM808
const char* apn  = secret.APN;
const char* broker = secret.BROKER_HOST;
const int port = secret.BROKER_PORT;
const char* clientID = "Developed Device";
const char* username = secret.BROKER_USERNAME;
const char* password = secret.BROKER_PASSWORD;
const char* topicUltraGyro = "GarbageBin/1/UltraGyro";
const char* topicGPS = "GarbageBin/1/GPS";
unsigned long lastReconnectAttempt = 0;
unsigned long lastSend = 0;
//Variables - HC-SR04
const float garbageBinHeight = 32.0;
uint8_t level = 0;
uint8_t prevLevel = 0;
uint8_t closedPrevLevel = 0;
//Variables - Misc
float latitude = 0.0;
float longitude = 0.0;
float prevLatitude = 0.0;
float prevLongitude = 0.0;
bool sameCoordFlag = false;
bool lidFlags[4] = {false, false, false, false};
uint8_t overflowAngle = 45;

void setup() {
  initializeInternal();
  initializeMPU6050();
  initializeSIM808();
  initializeMisc();
}
void loop() {
  updateAngle();
  if (!mqtt.connected()) {
    mqttReconnect();
    return;
  } else {
    unsigned long z = millis();
    if (z - lastSend > 3000L) {
      lastSend = z;
      gpsHandler();
      readingHandler();
    }
  }
  mqtt.loop();
}

//Functions - Internal
void initializeInternal() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
  while (!Serial);
  //Serial.println("Booting...");
}

//Functions - MPU6050
void dmpDataReady() {
  mpuInterrupt = true;
}
void initializeMPU6050() {
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setYGyroOffset(2.13);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}
void updateAngle() {
  if (dmpReady) {
    while (!mpuInterrupt /*&& fifoCount < packetSize*/) {
      if (mpuInterrupt /*&& fifoCount < packetSize*/) {
        fifoCount = mpu.getFIFOCount();
      }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      angle = 255;

    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      angle = abs(ypr[1] * 180 / M_PI) > 255 ? 255 : abs(ypr[1] * 180 / M_PI);
    }
  }
}
//Functions - SIM808
void initializeSIM808() {
  SerialAT.begin(9600);
  delay(10000);
  modem.restart();

  bool networkFlag = false;
  while (networkFlag == false) {
    if (modem.waitForNetwork()) {
      //Serial.println("Network: Connected");
      networkFlag = true;
    }
  }
  bool apnFlag = false;
  while (apnFlag == false) {
    if (modem.gprsConnect(apn, "", "")) {
      //Serial.println("Internet: Connected");
      apnFlag = true;
    }
  }

  modem.enableGPS();
  mqtt.setServer(broker, port);
  mqtt.setCallback(mqttCallback);
}
boolean mqttConnect() {
  //Serial.print("Broker Connection: ");
  boolean status = mqtt.connect(clientID, username, password, NULL, 2, NULL, NULL);
  if (status == false) {
    //Serial.println("Disconnected");
    return false;
  }
  //Serial.println("Connected");
  mqtt.subscribe("GarbageBin/1/OverflowAngle");
  return mqtt.connected();
}
void mqttReconnect() {
  //Serial.println("Broker Connection: Disconnected");
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
}
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  if (String(topic) == "GarbageBin/1/OverflowAngle") {
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
//Functions - HC-SR04
uint8_t getLevel() {
  level = ultrasonic.ping_median(9);
  if (prevLevel == 100 && level == 33) {
    return 255;
  } else {
    return abs(((garbageBinHeight - level) / garbageBinHeight) * 100);
  }
}
//Functions - Misc
void initializeMisc() {
  pinMode(LED_PIN, OUTPUT);
}
void gpsHandler() {
  if (modem.getGPS(&latitude, &longitude)) {
    if (abs(prevLatitude - latitude) > 0.000003 && abs(prevLongitude - longitude) > 0.000003) {
      //Serial.println("Latitude,Longitude: " + String(latitude) + ',' + String(longitude));
      mqtt.publish(topicGPS, (String(latitude) + ',' + String(longitude)).c_str());
      prevLatitude = latitude;
      prevLongitude = longitude;
      sameCoordFlag = false;
    }
  } else {
    if (!sameCoordFlag) {
      //Serial.println("Latitude,Longitude: 14.590565,120.977569");
      mqtt.publish(topicGPS, "14.590565,120.977569");
      sameCoordFlag = true;
    }
  }
}
void readingHandler() {
  //Serial.println("Angle: " + String(angle));
  if (angle != 255) {
    if (angle <= 3) {
      level = getLevel();
      if (level != 255 && level != 0) {
        if (!lidFlags[0]) {
          lidFlagSetter(0);
          //Serial.println("Level,Lid: " + String(level) + ',' + "Closed");
          mqtt.publish(topicUltraGyro, (String(level) + ',' + "Closed").c_str());
          prevLevel = level;
        } else if (abs(prevLevel - level) > 3) {
          //Serial.println("Level,Lid: " + String(level) + ',' + "Closed");
          mqtt.publish(topicUltraGyro, (String(level) + ',' + "Closed").c_str());
          prevLevel = level;
        }
      }
    } else if (angle >= 3) {
      level = getLevel();
      if (angle <= overflowAngle && level >= 90) {
        if (!lidFlags[2]) {
          lidFlagSetter(2);
          //Serial.println("Level,Lid: 100.1,Overflow");
          mqtt.publish(topicUltraGyro, "100.1,Overflow");
          prevLevel = 100;
        }
      } else if (angle <= 45) {
        if (level != 255) {
          if (!lidFlags[3]) {
            lidFlagSetter(3);
            //Serial.println("Level,Lid: " + String(level) + ',' + "Partial");
            mqtt.publish(topicUltraGyro, (String(level) + ',' + "Partial").c_str());
            prevLevel = level;
          } else if (abs(prevLevel - level) > 3) {
            //Serial.println("Level,Lid: " + String(level) + ',' + "Partial");
            mqtt.publish(topicUltraGyro, (String(level) + ',' + "Partial").c_str());
            prevLevel = level;
          }
        }
      }
      else {
        if (!lidFlags[1]) {
          lidFlagSetter(1);
          //Serial.println("Level,Lid: 0,Open");
          mqtt.publish(topicUltraGyro, "0,Open");
          prevLevel = 0;
        }
      }
    }
  }
}
void lidFlagSetter(int f) {
  for (int i = 0; i < 4; i++) {
    if (i == f) {
      lidFlags[i] = true;
    } else {
      lidFlags[i] = false;
    }
  }
}

