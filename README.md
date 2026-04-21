# MotorVis Files
Adafruit ESP32 Feather V2 Microcontroller (MC) files for MotorVis product

## IDE Setup
I used the following when coding the files used for the MC:
- Visual Studio Code
- ESP-IDF Extension

NOTE: It was lowk a pain to get the IDE set up and working properly with the microcontoller...

## Status on Current Firmware
# Android App
1. Gave up on the smooth mechanics for the Jacket render, will try and get it working after I get the logic for the important stuff established
2. 

# MC Firmware
1. Getting the BLE service to connect to the app.
2. Getting a unique identification so that the application can recognize the MC as the smart jacket. 

## BLE Structure / Ops.
I am still working on getting the data to send smoothly from the MC to the App but I was able to integrate the GNSS data into the GATT Server that I had made. I wrote my portion in C (mainly cause I had previously worked with C in making a GATT Server lol) but I got the 
C++ firmware for the GPS working. Here is an outline of how it is currently working but I will be updating it as I implement the rest of the sensors and data handling. 

# GATT Outline:
1. BLE Stack Init:
- Initalizes the MC in BLE-only mode and enables the Blueroid stack for GATT ops. It then registers registers GAP event handlers and GATT event handlers. I also named the MC "MotorVis Jacket" as a temp name but we can easily just change this when we finalize the name we want or we can even estbalish some sort of custom naming like:
- "[name] Jacket"
2. Advertising Setup: 
- The MC is advertised as a BLE perph publicly as with the establisehd name but includes:
1. 128-bit service UUID
2. Non-connectable Advertising parameters w/ 20-40ms intervals. 

3. GATT Service Struct
- Single Primary Service 
- GPS Charactersitic:
    - 128 UUID
    - Read / Notify Properties
   w/ descriptor 
    - CCCD
    - 16-bit UUID
    - Read / Write 

4. GPS Characteristics
- 13-byte payload:
    - timestamp
    - lat
    - long
    - fix_valid: Validation flag
- Gets initated with invalid 0 data
- Is based on a notification subscription when new GPS fixes are avaliable. 

5. CCCD Handling: 
- Tracks client notif. subs. 
- 
