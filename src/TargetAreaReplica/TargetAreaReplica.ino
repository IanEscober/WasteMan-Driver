#define TINY_GSM_MODEM_SIM900
#define SerialMon Serial

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Secrets.h>

SoftwareSerial SerialAT(10, 11);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
Secrets secret;

const char* apn  = secret.APN;
const char* broker = secret.BROKER_HOST;
const int port = secret.BROKER_PORT;
const char* clientID = "Target Area Replica";
const char* username = secret.BROKER_USERNAME;
const char* password = secret.BROKER_PASSWORD;
long lastReconnectAttempt = 0;

const int selectPins[3] = {2, 3, 4};
const int zOutput = 5;
const int zInput = A0;

bool isGPSSent = false;
int prevValues[7] = {0, 0, 0, 0, 0, 0, 0};
bool pin3Flag = false;
bool pin0Flag = false;

long lastSend = 0;

const int ledPin = 9;

void setup() {
  SerialMon.begin(9600);
  delay(10);
  SerialAT.begin(9600);
  delay(3000);

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

  mqtt.setServer(broker, port);

  for (int i = 0; i < 3; i++) {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], HIGH);
  }

  pinMode(zInput, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("Broker Connection: FAIL");
    digitalWrite(ledPin, LOW);
    unsigned long t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        digitalWrite(ledPin, HIGH);
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  else {
    if (!isGPSSent) {
      sendGPS();
    }

    unsigned long z = millis();
    if (z - lastSend > 3000L) {
      lastSend = z;

      for (byte pin = 0; pin < 7; pin++) {
        selectMuxPin(pin);
        int inputValue = analogRead(A0);

        //Special Case For Switch 2
        if (pin == 0) {
          if (inputValue > 1000 && !pin0Flag) {
            pin0Flag = true;
            sendUltrasonic(pin, inputValue);
          }
          else if (inputValue < 1000 && pin0Flag) {
            pin0Flag = false;
            sendUltrasonic(pin, inputValue);
          }
        }
        //Special Case For Switch 5
        else if (pin == 3) {
          if (inputValue > 20 && !pin3Flag) {
            pin3Flag = true;
            sendUltrasonic(pin, inputValue);
          }
          else if (inputValue < 20 && pin3Flag) {
            pin3Flag = false;
            sendUltrasonic(pin, inputValue);
          }
        }
        //General Case
        else if (abs(prevValues[pin] - inputValue) > 500) {
          sendUltrasonic(pin, inputValue);
          prevValues[pin] = inputValue;
        }
        Serial.print(String(inputValue) + "\t");
      }
      Serial.println();
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
  return mqtt.connected();
}

void selectMuxPin(byte pin) {
  for (int i = 0; i < 3; i++) {
    if (pin & (1 << i))
      digitalWrite(selectPins[i], HIGH);
    else
      digitalWrite(selectPins[i], LOW);
  }
}

void sendGPS() {
  mqtt.publish("GarbageBin/2/GPS", "14.642872,121.000337");
  mqtt.publish("GarbageBin/3/GPS", "14.641456,120.999197");
  mqtt.publish("GarbageBin/4/GPS", "14.640927,120.995453");
  mqtt.publish("GarbageBin/5/GPS", "14.640015,121.003259");
  mqtt.publish("GarbageBin/6/GPS", "14.638686,120.998219");
  mqtt.publish("GarbageBin/7/GPS", "14.638925,121.005412");
  mqtt.publish("GarbageBin/8/GPS", "14.638003,120.998315");
  isGPSSent = true;
}

void sendUltrasonic(byte pin, int inputValue) {
  switch (pin) {
    case 0:
      {
        if (inputValue > 1000) {
          mqtt.publish("GarbageBin/2/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/2/UltraGyro", "0,Closed");
        }
        break;
      }

    case 1:
      {
        if (inputValue > 500) {
          mqtt.publish("GarbageBin/3/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/3/UltraGyro", "0,Closed");
        }
        break;
      }

    case 2:
      {
        if (inputValue > 500) {
          mqtt.publish("GarbageBin/4/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/4/UltraGyro", "0,Closed");
        }
        break;
      }

    case 3:
      {
        if (inputValue > 20) {
          mqtt.publish("GarbageBin/5/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/5/UltraGyro", "0,Closed");
        }
        break;
      }

    case 4:
      {
        if (inputValue > 500) {
          mqtt.publish("GarbageBin/6/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/6/UltraGyro", "0,Closed");
        }
        break;
      }

    case 5:
      {
        if (inputValue > 500) {
          mqtt.publish("GarbageBin/7/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/7/UltraGyro", "0,Closed");
        }
        break;
      }

    case 6:
      {
        if (inputValue > 500) {
          mqtt.publish("GarbageBin/8/UltraGyro", "80,Closed");
        } else {
          mqtt.publish("GarbageBin/8/UltraGyro", "0,Closed");
        }
        break;
      }
  }
}

