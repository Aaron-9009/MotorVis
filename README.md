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


### In collaboration with:

>**Lily Nguyen-Wilson**
<br>*Otis College of Art and Design | B.F.A. Product Design*
<br>Designer and producer of MotorVis Jacket




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

## Firmware
| Directory | File | Description |
| ----------- | ----------- | ----------- |
| main | 


## Software
## domain.manager

### **CoreManagers.kt** 
>**Description:** <br>
Contains the domain-layer managers that shit between the raw hardware data and the ViewModel. It is responisbile for telemetry processing, vital handling,crash validation, emergency SMS dispatching, and BLE connection coordination. 

**Types**
| Type | Visibility | Purpose |
| ----------- | ----------- | ----------- |
| TelemetryManager | public | Converts the raw *JacketTelemetrySnapshot* updates from the *SensorDataRepository* into a UI-Ready *TelemeteryState*. It also keeps the latest accelerometer, GPS, and battery values while calculating the speed. | 
| TelemetryAccumulator | private nested data class | For interal use in `TelemetryManager.scan()` to remember the last valid GPS snapshot and the current TelemetryState. | 
| TelemetryGpsSpeedUpdate | interal data class | Returns moel for speed calculations. Contains the calculated speed and the updated previous validated GPS refrence. | 
| VitalsManager | public | Gets the users vitals from the telemetery packets and formats them as a `Flow<VitalsState>`. |
| AlertManager | public | Builds the emergency dispatch payloads, message, and sends the SMS through `PhoneSmsDispatcher`. It then records the sms status to the repo logs. |
| CrashValidationManager | public | Is looking through the telemetery data for crash flags, validates teh crash flags and if validated starts a 15-second countdown. There is a cancellation feature which is not activated valled the `AlertManager`. | 
| BluetoothManager | public | Coordinates app-level Bluetooth features such as: scan, connect, disconnect, telemetry collection, RSSI updates, and connecion state handeling. |

**Functions**

| Function | Return | Description |
| ----------- | ----------- | ----------- |
| `calculatedTelemetryGpsSpeedUpdate()`| *TelemetryGpsSpeedUpdate* | Calculates displayed speedo nly when a new valid GPS packet is recieved. It avoids fake speed changes from stale GPS data. Invalid GPS data resets the speed to 0 mph and clears the previous valid GPS baseline. |
|`SmsDispatchResult.`<br>`toEmergencyDeliveryStatus(contactNumber)` | String | Converts the reulst of an SMS send attempt to a readable delivery status. |
| `buildEmergencySmsMessage()` | String | Builds the S body send during an emergency. | 

**TelemetryManager Members**
| Member | Type | Description |
| ----------- | ----------- | ----------- |
| *telemetryState*| Flow<TelemetryState> | Public stream feeding the ViewModel. Produces a clean TelemetryState with current speed, location, acceleration, and battery reading. |

**VitalsManager Members**
| Member | Type | Description |
| ----------- | ----------- | ----------- |
| *vitalsState*| Flow<VitalsState> | Public stream containing physiological data extracted from telemetry. |

**AlertManager Members**
| Member | Type | Description |
| ----------- | ----------- | ----------- |
| `dispatchEmergency()` | EmergencyDispatch | Creates the emergency payload after a validated crash countdown expires. It selects the last known data, builds SMS message, sends it to the saved number, records the dispatch, and logs the SMS delivery status. |

### **CrashValidationRules.kt**
>**Description:** <br>
Contains the custom crash-validation rules used after the ESP32 reports accelerometer crash evidence. It decides whether the app should start the emegrency countdownor ignore the event as a likely flase positive. 

**Types**
| Type | Visibility | Purpose |
| ----------- | ----------- | ----------- |
| CrashValidationAssessment | internal data class | Result obeject returned by crash validation. Tells the app whether to start the countdown, what to log, what validation summary to attach to the dispatch payload, the speed during the validation, and which signals supported the final decision. |
| CrashValidationRules | internal object | Evaluates the accelerometer evidence, GPS riding context, speed drops, and testing override logic to decide whthere a potential crahs should trigger a countdown. |

**Fields**
| Field | Type | Description |
| ----------- | ----------- | ----------- |
| shouldStartCountdown | boolean | `true`: when the crash candidate passes calidation |
| logMessgae | String | Full message is recorded into the alert log with an explination why the candidate was validated or tossed. |
| validationSummary | String | Technical summary of evidence, accel. values, speed, speed drops, and location. Used in the emergency dispatch payloads. | 
| currentGpsSpeedMph | Float | Current speed used for validation decision. | 
| corroboratingSignals | List<String> | List of supporting crash signals: ESP flags, acceleration spikes, delta spikes, rapid speed drops. |

**Constants**
| Constant | Value | Description | 
| ----------- | ----------- | ----------- |
| minRidingSpeedMph | **8f** | Minimum speed used to determine wheteher the rider was in a riding context. |
| accelerationSpikeG | **2.8f** | Total acceleration threshold that counts as a corroborating impact signal | 
| dynamicAccelerationSpikeG | **1.5f** | Dynamic acceleration threshold that counts as a counted spike. |
| sampleDeltaSpikeG | **1.5f** | Sample-to-sample acceleration delta threshold. | 
| criticalAccelerationG | **4.0f** | Higher total acceleration threshold used for immediate valdiation. | 
| criticalDynamicAccelerationG | **3.0f** | Higher dynamic acceleration threshold used for immediate validation. |
| criticalSampleDeltaG | **3.0f** | Higher sample delta threshold used for immediate validation. |
| speedDropThresholdMph | **7f** | Speed drop threshold that counts for validation. | 

**Functions**
| Function | Return | Description |
| ----------- | ----------- | ----------- |
| `assess(snapshot, currentGpsSpeedMph,` <br> `previousGpsSpeedMph, forceAccelerometerAlertDispatchForTesting)` | CrashValidationAssessment | Main crash rule evaluator. Builds evidence and returns if found valid, otherwise returns tossed assessment. | 
| `buildValidationSummary()` | String | Private helper that formats the technical validation summary 

