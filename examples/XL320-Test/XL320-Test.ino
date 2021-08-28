#include "Dynamixel-XL320.h"

/*
This library uses SoftwareSerial to communicate with Dynamixel XL-320 servos.
No hardware other than an Arduino board and one or more XL-320 servos is required.
Connect one of the Arduino ground pins to the XL-320 ground wire.
Connect the Vin pin on the Arduino to the XL-320 power wire.
The XL-320 will work powered with 5V from Arduino USB port but
  will report a voltage out of range alert and will not have full speed/power.
Better option: Power the Arduino from DC power jack (7-8V).
Best option: Power the XL-320 directly from external power source (7-8V) and connect common grounds.
*/

#define MAX_DEVICES 16

typedef struct {
    uint8_t id;
    uint16_t model;
    uint8_t fwVersion;
} DeviceInfo;

//Make sure the pins you use support change interrupts for SoftwareSerial
//Refer to: https://www.arduino.cc/en/Reference/SoftwareSerial
//Using pins 2 and 3 (Arduino Uno) to connect to XL-320 data wire.
DynamixelXL320 xl320(2, 3);

//Number of devices
int deviceCount = 0;

//Info for each device
DeviceInfo deviceInfo[MAX_DEVICES];

//Print info for each device
void printDevices() {
    char msg[60];
    for (int i = 0; i < deviceCount; i++) {
        DeviceInfo& d = deviceInfo[i];
        sprintf(msg, "Device id: %d, model: %d, firmware: %d", d.id, d.model, d.fwVersion);
        Serial.println(msg);
    }
}

//Store info for each device found on the bus
void pingCallback(uint8_t id, uint16_t model, uint8_t fwVersion) {
    if (deviceCount < MAX_DEVICES) {
        deviceInfo[deviceCount++] = { id, model, fwVersion };
    }
}

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

void setup()
{
    Serial.begin(9600);
    while (!Serial) { };

    //set error/alert callback handler
    xl320.setErrorCallback(errorCallback);

    //set to 9600, 57600, or 115200
    //use XL320-Setup.ino sketch to change
    //from factory default of 1,000,000
    xl320.begin(DynamixelXL320::Baud::BAUD_57600);

    //ping/discover all connected devices
    if (xl320.ping(pingCallback)) {
        printDevices();
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
