#include <Arduino.h>

/*
Use this Arduio sketch to configure a Dynamixel XL-320 servo.
The default XL-320 baud rate of 1,000,000 is too fast for 
  SoftwareSerial so this setup sketch uses the hardware Serial port.

Set the 'new_baud' variable to desired baud rate of
  9600, 57600 or 115200.
Set the 'new_id' variable to desired device id (0 to 252).
  Factory default is 1.
Set the 'new_mode' variable to desired control mode (Wheel or Joint).
  Factory default is Joint.
Set the 'factory_reset' variable to true to do a factory reset 
  of all other device settings.

Connect one of Arduino ground pins to the XL-320 ground wire.
Connect the Arduino Vin pin to the XL-320 power wire.
The XL-320 will work powered with 5V from Arduino USB port.

Connect pins RX-0 and TX-1 (Arduino Uno) to joined XL-320 data wire.
Upload the XL320-Setup sketch to the Arduino
If using an Arduino Uno board, you will not be able to have the RX and TX pins
  connected while uploading the sketch since there is only one hardware Serial port
  shared with the USB interface.

After a few seconds the XL-320 servo should reboot and
  the LED on the XL-320 should turn green to indicate success.
*/

uint16_t update_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);

const uint8_t INST_WRITE = 0x03;
const uint8_t INST_RESET = 0x06;
const uint8_t INST_REBOOT = 0x08;
const uint8_t BROADCAST_ID = 0xFE;
const uint8_t ADDR_ID = 0x03;
const uint8_t ADDR_BAUD_RATE = 0x04;
const uint8_t ADDR_CTRL_MODE = 0x0B;
const uint8_t ADDR_LED = 0x19;
const uint8_t LED_GREEN = 0x02;

enum Baud : uint8_t
{
    BAUD_9600 = 0,
    BAUD_57600 = 1,
    BAUD_115200 = 2,
    //BAUD_1Mbps = 3
};

enum Mode : uint8_t
{
    WHEEL = 1,
    JOINT = 2
};

//Set to desired id
//Should be between 0 and 252
//Factory default is 1
uint8_t new_id = 1;

//Set to desired baud rate
//Should be 9600 or 57600 or 115200 for SoftwareSerial
//Factory default is 1000000
//Baud new_baud = Baud::BAUD_9600;
Baud new_baud = Baud::BAUD_57600;
//Baud new_baud = Baud::BAUD_115200;

//Set to desired control mode
//Factory default is JOINT
//Mode new_mode = Mode::WHEEL;
Mode new_mode = Mode::JOINT;

//True to do a factory reset of control table
//False will only reboot without factory reset
bool factory_reset = true;

void setup()
{
    if (new_id > 252) {
        return; //new id should be between 0 and 252
    }
    if (new_baud > Baud::BAUD_115200) {
        return; //new baud should be 9600, 57600 or 115200 for SoftwareSerial
    }

    //set new baud rate
    uint8_t baudrate = 0xFF;
    int32_t baudrates[] = {9600, 57600, 115200, 1000000};
    for (int i = 0; i <= 3; i++)
    {   //cycle through all baud rates
        Serial.begin(baudrates[i]);
        uint8_t buffer[] = {0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID, 0x06, 0x00, INST_WRITE, ADDR_BAUD_RATE, 0x00, new_baud, 0x00, 0x00};
        uint16_t crc = update_crc(0, buffer, 11);
        buffer[11] = (crc & 0x00FF);
        buffer[12] = (crc >> 8) & 0x00FF;
        Serial.write((uint8_t)0);
        Serial.write(buffer, 13);
        delay(100);
        Serial.end();
    }

    Serial.begin(baudrates[new_baud]);

    {   //set new id
        uint8_t buffer[] = {0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID, 0x06, 0x00, INST_WRITE, ADDR_ID, 0x00, new_id, 0x00, 0x00};
        uint16_t crc = update_crc(0, buffer, 11);
        buffer[11] = (crc & 0x00FF);
        buffer[12] = (crc >> 8) & 0x00FF;
        Serial.write((uint8_t)0);
        Serial.write(buffer, 13);
        delay(100);
        Serial.end();
    }

    Serial.begin(baudrates[new_baud]);

    if (factory_reset)
    {
        //do a factory reset - all values except ID and Baud rate
        uint8_t buffer[] = {0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID, 0x04, 0x00, INST_RESET, 0x02, 0x00, 0x00};
        uint16_t crc = update_crc(0, buffer, 9);
        buffer[9] = (crc & 0x00FF);
        buffer[10] = (crc >> 8) & 0x00FF;
        Serial.write((uint8_t)0);
        Serial.write(buffer, 11);
    }
    else
    {
        //reboot device
        uint8_t buffer[] = {0xFF, 0xFF, 0xFD, 0x00, BROADCAST_ID, 0x03, 0x00, INST_REBOOT, 0x00, 0x00};
        uint16_t crc = update_crc(0, buffer, 8);
        buffer[8] = (crc & 0x00FF);
        buffer[9] = (crc >> 8) & 0x00FF;
        Serial.write((uint8_t)0);
        Serial.write(buffer, 10);
    }

    delay(100);
    Serial.end();

    //device will reboot
    delay(1000);

    Serial.begin(baudrates[new_baud]);

    if (!factory_reset || new_mode != Mode::JOINT)
    {
        //set control mode
        uint8_t buffer[] = {0xFF, 0xFF, 0xFD, 0x00, new_id, 0x06, 0x00, INST_WRITE, ADDR_CTRL_MODE, 0x00, new_mode, 0x00, 0x00};
        uint16_t crc = update_crc(0, buffer, 11);
        buffer[11] = (crc & 0x00FF);
        buffer[12] = (crc >> 8) & 0x00FF;
        Serial.write((uint8_t)0);
        Serial.write(buffer, 13);
    }

    //set led to green to indicate success
    uint8_t buffer[] = {0xFF, 0xFF, 0xFD, 0x00, new_id, 0x06, 0x00, INST_WRITE, ADDR_LED, 0x00, LED_GREEN, 0x00, 0x00};
    uint16_t crc = update_crc(0, buffer, 11);
    buffer[11] = (crc & 0x00FF);
    buffer[12] = (crc >> 8) & 0x00FF;
    Serial.write((uint8_t)0);
    Serial.write(buffer, 13);
}

void loop()
{
}

//update_crc function from robotis documentation
uint16_t update_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
