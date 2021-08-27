## Dynamixel XL-320 Arduino Library

This Arduino library can control Dynamixel XL-320 servos using
a Half-Duplex SoftwareSerial interface. No other hardware or 
circuitry is needed besides an Arduino board and one or more XL-320 servos.

## Installation
To use this libary install the Dynamixel-XL320.zip archive from github releases to the 
Arduino library folder.
```
Arduino -> Sketch -> Include Library -> Add Zip Library
```

## Configuration
Before using the XL-320 with this library, you may need to configure the XL-320.
The default baud rate for the XL-320 is 1,000,000 which is not supported with SoftwareSerial. You will need to change the baud rate to 9,600, 57,600 or 115,200.

You can use the XL320-Setup sketch included in the examples to configure the buad rate (9,600 / 57,600 / 115,200), set the servo ID number (0 - 252), and set the desired control mode (Wheel or Joint). Wheel mode allows the servo to turn 360 degrees like a standard DC motor. Joint mode allows you to set the desired rotation angle -150 to +150 degrees. The XL320-Setup sketch uses the hardware serial port on the arduino to do the initial servo configuration. After the servo is configured you can use the DynamixelXL320 class to control it using half-duplex SoftwareSerial.

### Using XL320-Setup Sketch
```
- Set the 'new_baud' variable to desired baud rate of 9600, 57600 or 115200.

- Set the 'new_id' variable to desired device ID (0 to 252).
  The factory default is 1.

- Set the 'new_mode' variable to desired control mode (Wheel or Joint).
  The factory default is Joint.

- Set the 'factory_reset' variable to true to do a factory reset of
  all other device settings.

- Connect one of the Arduino ground pins to XL-320 ground wire.
- Connect the Vin pin on the Arduino to the XL-320 power wire.
- The XL-320 will work powered with 5V from Arduino USB port.
- Connect pins RX-0 and TX-1 (Arduino Uno) to joined XL-320 data wire.

- Upload the XL320-Setup sketch to the Arduino.
- If using an Arduino Uno board, you will not be able to have the RX and TX pins
  connected while uploading the sketch since there is only one hardware Serial port
  shared with the USB interface.

- After a few seconds the XL-320 servo should reboot and
  the LED on the XL-320 should turn green to indicate success.
```

## Testing an XL-320 Servo
You can use the XL320-Test sketch included in the examples to test an XL-320 servo. The sketch will ping all devices on the bus and print the device ID, Model, and Firmware version to the hardware serial port which you can view using the Arduino serial monitor. After pinging for devices, the sketch will cycle through several LED color changes on all the connected servos.

### Using The XL320-Test Sketch
```
- Connect one of the Arduino ground pins to XL-320 ground wire.
- Connect the Vin pin on the Arduino to the XL-320 power wire.
- The XL-320 will work powered with 5V from Arduino USB port but
  will report a voltage out of range alert and will not have 
  full speed/power.
- Better option: Power the Arduino from DC power jack (7-8V).
- Best option: Power the XL-320 directly from external power 
  source (7-8V) and connect common grounds.
- Connect pins 2 and 3 (Arduino Uno) to joined XL-320 data wire.
- Upload the XL320-Setup sketch to the Arduino.
```

Example Code
```c
#include "Dynamixel-XL320.h"

//Using pins 2 and 3 (Arduino Uno) to connect to XL-320 data wire.
DynamixelXL320 xl320(2, 3);

//Display any error code or alert condition reported by the XL-320
void errorCallback(uint8_t id, uint8_t error, bool alert)
{
    char msg[30];
    if (error != 0) {
        sprintf(msg, "Error %d for Id %d", error, id);
        Serial.println(msg);
    }
    if (alert) {
        uint8_t alertCode = xl320.getAlertCode(id);
        if (alertCode > 0) {
            sprintf(msg, "Alert %d for Id %d", alertCode, id);
            Serial.println(msg);
        }
    }
    xl320.clearError(id);
}

//Callback to display ID, Model, and Firmware Version for all XL-320 devices on the bus
void pingCallback(uint8_t id, uint16_t model, uint8_t fwVersion)
{
    char msg[60];
    sprintf(msg, "Device id: %d, model: %d, firmware: %d", id, model, fwVersion);
    Serial.println(msg);
}

void setup()
{
    Serial.begin(9600);

    //set error/alert callback handler
    xl320.setErrorCallback(errorCallback);

    //set to 9600, 57600, or 115200
    //use XL320-Setup.ino sketch to change
    //from factory default of 1,000,000
    xl320.begin(DynamixelXL320::Baud::BAUD_57600);

    delay(100);

    // discover all devices on the bus
    if (xl320.ping(pingCallback)) {
        Serial.println("Ping Complete");
    } else {
        Serial.println("Ping Failed");
    }
}

void loop()
{
    //cycle through LED colors on all connected XL-320 servos
    xl320.setLED(DynamixelXL320::BROADCAST, DynamixelXL320::Led::LED_BLUE);
    delay(1000);
    xl320.setLED(DynamixelXL320::BROADCAST, DynamixelXL320::Led::LED_GREEN);
    delay(1000);
    xl320.setLED(DynamixelXL320::BROADCAST, DynamixelXL320::Led::LED_RED);
    delay(1000);
    xl320.setLED(DynamixelXL320::BROADCAST, DynamixelXL320::Led::LED_WHITE);
    delay(1000);
    xl320.setLED(DynamixelXL320::BROADCAST, DynamixelXL320::Led::LED_OFF);
    delay(5000);
}
```
