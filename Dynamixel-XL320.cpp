#include <Dynamixel-XL320.h>


DynamixelXL320::DynamixelXL320(uint8_t rxPin, uint8_t txPin)
    : _serial(rxPin, txPin), _mode(MODE_RECEIVE), _errorCallback(NULL)
{
    memset(_errors, 0, sizeof(_errors));
}

void DynamixelXL320::begin(Baud baud)
{
    _serial.setTimeout(500);
    _serial.begin(baud == BAUD_115200 ? 115200 : baud == BAUD_57600 ? 57600 : 9600);
}

void DynamixelXL320::setErrorCallback(ErrorCallback errorCallback) {
    _errorCallback = errorCallback;
}

uint8_t DynamixelXL320::getError(uint8_t id) {
    return (id < 253 ? _errors[id] & 0x7F : 0x00);
}

void DynamixelXL320::clearError(uint8_t id) {
    if (id < 253) _errors[id] = 0;
}

bool DynamixelXL320::getAlert(uint8_t id) {
    return (id < 253 ? (_errors[id] & 0x80) > 0 : false);
}

uint8_t DynamixelXL320::getAlertCode(uint8_t id) {
    uint16_t value = 0;
    return (readValue(id, ADDR_ERROR_STATUS, VALUE_8_BIT, &value, false) ? (uint8_t)value : 0x00);
}

bool DynamixelXL320::ping(PingCallback callback, uint8_t id)
{
    if ((id > 252 && id != BROADCAST) || (!callback)) return false;
    uint8_t buffer[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x03, 0x00, INST_PING, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return sendAndRead(id, buffer, 10, 14, true, [](uint8_t* packet, uint16_t length, void* cbState) {
        ((PingCallback)cbState)(packet[4], packet[9] + (packet[10] << 8), packet[11]);
    }, (void*)callback);
}

bool DynamixelXL320::setLED(uint8_t id, Led value)
{
    return (value >= Led::LED_OFF && value <= Led::LED_WHITE) ?
        writeValue(id, ADDR_LED, VALUE_8_BIT, (uint16_t)value) : false;
}

DynamixelXL320::Led DynamixelXL320::getLED(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_LED, VALUE_8_BIT, &value) &&
        value >= Led::LED_OFF && value <= Led::LED_WHITE
        ? (Led)value : Led::LED_UNKNOWN);
}

bool DynamixelXL320::setControlMode(uint8_t id, Control value)
{
    return (value >= Control::MODE_WHEEL && value <= Control::MODE_JOINT) ?
        writeValue(id, ADDR_CONTROL_MODE, VALUE_8_BIT, (uint16_t)value) : false;
}

DynamixelXL320::Control DynamixelXL320::getControlMode(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_CONTROL_MODE, VALUE_8_BIT, &value) &&
        value >= Control::MODE_WHEEL && value <= Control::MODE_JOINT
        ? (Control)value : Control::MODE_UNKNOWN);
}

bool DynamixelXL320::setTorqueEnable(uint8_t id, Torque value)
{
    return (value >= Torque::TORQUE_DISABLED && value <= Torque::TORQUE_ENABLED) ?
        writeValue(id, ADDR_TORQUE_ENABLE, VALUE_8_BIT, (uint16_t)value) : false;
}

DynamixelXL320::Torque DynamixelXL320::getTorqueEnable(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_TORQUE_ENABLE, VALUE_8_BIT, &value) &&
        value >= Torque::TORQUE_DISABLED && value <= Torque::TORQUE_ENABLED
        ? (Torque)value : Torque::TORQUE_UNKNOWN);
}

bool DynamixelXL320::setMaxTorque(uint8_t id, uint16_t value)
{
    return (value >= 0 && value <= 2047) ?
        writeValue(id, ADDR_MAX_TORQUE, VALUE_16_BIT, value) : false;
}

uint16_t DynamixelXL320::getMaxTorque(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_MAX_TORQUE, VALUE_16_BIT, &value) ? value : 0xFFFF);
}  

bool DynamixelXL320::setGoalPosition(uint8_t id, uint16_t value)
{
    return (value >= 0 && value <= 1023) ?
        writeValue(id, ADDR_GOAL_POSITION, VALUE_16_BIT, value) : false;
}

uint16_t DynamixelXL320::getGoalPosition(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_GOAL_POSITION, VALUE_16_BIT, &value) ? value : 0xFFFF);
}

uint16_t DynamixelXL320::getCurrentPosition(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_CURRENT_POSITION, VALUE_16_BIT, &value) ? value : 0xFFFF);
}

bool DynamixelXL320::setGoalSpeed(uint8_t id, uint16_t value)
{
    return (value >= 0 && value <= 2047) ?
        writeValue(id, ADDR_GOAL_SPEED, VALUE_16_BIT, value) : false;
}

uint16_t DynamixelXL320::getGoalSpeed(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_GOAL_SPEED, VALUE_16_BIT, &value) ? value : 0xFFFF);
}

uint16_t DynamixelXL320::getCurrentSpeed(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_CURRENT_SPEED, VALUE_16_BIT, &value) ? value : 0xFFFF);
}

bool DynamixelXL320::getIsMoving(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_MOVING, VALUE_8_BIT, &value) ? (bool)value : false);
}

uint16_t DynamixelXL320::getCurrentLoad(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_CURRENT_LOAD, VALUE_16_BIT, &value) ? value : 0xFFFF);
}  

float DynamixelXL320::getCurrentVoltage(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_CURRENT_VOLTAGE, VALUE_8_BIT, &value) ? (float)value / 10.0 : 0.0);
}

uint8_t DynamixelXL320::getCurrentTemperature(uint8_t id)
{
    uint16_t value = 0;
    return (readValue(id, ADDR_CURRENT_TEMPERATURE, VALUE_8_BIT, &value) ? (uint8_t)value : 0x00);
}  


bool DynamixelXL320::sendPacket(uint8_t* buffer, uint16_t size)
{
    if (!buffer || size < 10) return false;
    uint16_t crc = update_crc(0, buffer, size - 2);
    buffer[size-2] = (crc & 0x00FF);
    buffer[size-1] = (crc >> 8) & 0x00FF;
    if (_mode != MODE_SEND) {
        //switch to tx mode
        _serial.stopListening();
        _serial.enableTx();
        _mode = MODE_SEND;
        //prevent framing errors
        _serial.write((uint8_t)0);
    }
    return _serial.write(buffer, size) == size;
}

uint16_t DynamixelXL320::readPacket(uint8_t* buffer, uint16_t size, bool processErrors)
{
    if (!buffer || size < 11) return 0;
    if (_mode != MODE_RECEIVE) {
        //switch to rx mode
        _serial.disableTx();
        _serial.listen();
        _mode = MODE_RECEIVE;
    }
    if (_serial.readBytes(buffer, 1) != 1 || buffer[0] != 0xFF) return 0;
    if (_serial.readBytes(buffer+1, 1) != 1 || buffer[1] != 0xFF) return 0;
    if (_serial.readBytes(buffer+2, 1) != 1 || buffer[2] != 0xFD) return 0;
    if (_serial.readBytes(buffer+3, 4) != 4) return 0;
    uint16_t length = buffer[5] + (buffer[6] << 8);
    if (length < 4 || length + 7 > size) return 0;
    if (_serial.readBytes(buffer+7, length) != length || buffer[7] != 0x55) return 0;
    if (processErrors) {
        uint8_t id = buffer[4]; if (id < 253) _errors[id] = buffer[8];
    }
    //TODO: check CRC
    return length + 7;
}

bool DynamixelXL320::sendAndRead(uint8_t id, uint8_t* buffer, uint16_t send_size, uint16_t read_size, bool processErrors, PacketCallback cb, void* cbState)
{
    if (!sendPacket(buffer, send_size)) return false;
    if (read_size > 0) {
        if (readPacket(buffer, read_size, processErrors) != read_size) return false;
        if (cb) cb(buffer, read_size, cbState);
        if (id != BROADCAST && buffer[4] != id) return false;
        if (id == BROADCAST) {
            while (_serial.available() && readPacket(buffer, read_size, processErrors) == read_size) {
                if (cb) cb(buffer, read_size, cbState);
            };
        }
    }
    if (processErrors) checkErrors();
    return true;
}

bool DynamixelXL320::writeValue(uint8_t id, uint8_t address, uint8_t nbytes, uint16_t value, bool readStatus, PacketCallback cb, void* cbState)
{
    if ((id > 252 && id != BROADCAST) || (nbytes < 1 || nbytes > 2)) return false;
    uint8_t buffer[] = { 0xFF, 0xFF, 0xFD, 0x00, id, (uint8_t)(nbytes > 1 ? 0x07 : 0x06), 0x00, INST_WRITE, address, 0x00, (uint8_t)(value & 0x00FF), (uint8_t)(value >> 8), 0x00, 0x00 };
    return sendAndRead(id, buffer, nbytes > 1 ? 14 : 13, readStatus ? 11 : 0, true, cb, cbState);
}

bool DynamixelXL320::readValue(uint8_t id, uint8_t address, uint8_t nbytes, uint16_t* value, bool processErrors)
{
    if (id > 252 || nbytes < 1 || nbytes > 2) return false;
    uint8_t buffer[] = { 0xFF, 0xFF, 0xFD, 0x00, id, 0x07, 0x00, INST_READ, address, 0x00, nbytes, 0x00, 0x00, 0x00 };
    if (!sendAndRead(id, buffer, sizeof(buffer), nbytes > 1 ? 13 : 12, processErrors)) return false;
    *value = nbytes > 1 ? buffer[9] + (buffer[10] << 8) : buffer[9];
    return true;
}

void DynamixelXL320::checkErrors()
{
    if (!_errorCallback) return;
    for (uint8_t i = 0; i < 253; i++) {
        if (_errors[i] != 0) {
            _errorCallback(i, _errors[i] & 0x7F, (_errors[i] & 0x80) > 0);
        }
    }
}  

//update_crc function from robotis documentation
uint16_t DynamixelXL320::update_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
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
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    
    return crc_accum;
}
