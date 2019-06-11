# WasteMan-Driver
WasteMan-Driver enables the IoT-based devices to monitor its sensors and send data to the broker which is then received by the backend ([Wasteman](https://github.com/ianescober/WasteMan)). The Message Queuing Telemetry Transport (MQTT) Protocol serves as the medium of transmission between the broker and devices.

## Devices
### Block Diagram
![WasteMan-Driver Block Diagram](https://github.com/IanEscober/WasteMan-Driver/blob/master/docs/Block-Diagram.png)
### Prototypes
#### Garbage Bin
![WasteMan-Driver Garbage Bin](https://github.com/IanEscober/WasteMan-Driver/blob/master/docs/Garbage-Bin.png)
#### Target Area Replica
![WasteMan-Driver Target Area Replica](https://github.com/IanEscober/WasteMan-Driver/blob/master/docs/Target-Area-Replica.png)

## Technologies/Libraries/Frameworks
### HC-SR04 - Ultrasonic Sensor
- [Hand-made](https://github.com/IanEscober/WasteMan-Driver/blob/master/lib/HCSR04.cpp) ([Reference](https://bitbucket.org/teckel12/arduino-new-ping/downloads/)) - Monitoring of the waste level
### MPU6050 - Gyroscope and Accelerometer
- [MPU6050 by Electronic Cats](https://github.com/ElectronicCats/mpu6050) - Detection of the lid angle
### SIM 900A and 808 - GSM and GPS Module
- [TinyGSM](https://github.com/vshymanskyy/TinyGSM) - Sending/Receving of data to the broker

## Requirements
- [Arduino Sketch](https://www.arduino.cc/en/Main/Software) - For the arduino hardware libraries

## Installation/Operation
- [WasteMan-Driver Wiki](https://github.com/IanEscober/WasteMan-Driver/wiki)

## Contribution
Yeet a Pull Request

## License
[MIT](https://github.com/IanEscober/WasteMan-Driver/blob/master/License)
