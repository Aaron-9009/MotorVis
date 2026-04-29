# MotoVis Jacket Capstone Project
**Undergraduate Capstone Final Project**
*<br> University of Southern California | 2026*

### Team Members

>**Aaron Gonzalez**
 *<br> B.S. Electrical and Computer Engineering*

>**Will Bradley**
 *<br> B.S. Electrical and Computer Engineering*

>**Pela Karamalonis**
 *<br> B.S. Electrical and Computer Engineering*

**Note** <br>
In collaboration with: <br>
>**Lily Nguyen-Wilson**
 <br>  *Otis College of Art and Design | B.S. Electrical and Computer Engineering*

as designer and producer of physical MotorVis Jacket. 




## Overview
Running on the Adafruit ESP32 Feather V2, these files contain the firmware used to control the MotoVis Jacket electronics for the current working prototype. The ESP32 acts as the central microcontroller for the jacket, collecting data from the connected sensors, packaging that data into structured binary packets, and making those packets available to the MotoVis Android companion app over Bluetooth Low Energy (BLE).

The firmware establishes a custom BLE General Attribute Profile (GATT) service, where the ESP32 operates as the BLE server and the Android phone operates as the BLE client. Within this custom service, each major sensor system is exposed through its own characteristic, allowing the companion app to independently read or subscribe to updates from the GNSS module, accelerometer, heart sensor, and battery monitor.

The GNSS module provides live GPS location data, which is packaged as latitude and longitude values for the Android app to decode and display. The ADXL345 accelerometer provides calibrated motion data and crash-evidence alert flags that the app uses as the starting point for crash validation. The PulseSensor Amped provides heart-rate-related data for rider monitoring, while the battery monitoring firmware reports the jacket’s battery voltage so the app can display power status.

The firmware is organized into separate components so each subsystem can be developed, tested, and maintained independently. Sensor managers handle the low-level hardware communication, while the main BLE firmware file is responsible for building the GATT service, publishing sensor packets, handling client connections, and managing jacket controls such as buttons, LEDs, pairing behavior, and temporary sleep behavior.


## Hardware
- Adafruit EPS32 Feather V2
- ADXL345 Accelerometer
- PulseSensor Amped
- SparkFun GNSS Max-M10S
- GNSS Active Antena

