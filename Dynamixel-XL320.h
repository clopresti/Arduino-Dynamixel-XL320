#include <Arduino.h>
#include <SoftwareSerial.h>

#ifndef _DynamixelXL320_
#define _DynamixelXL320_

#define MAX_ID_FOR_ERRORS 16

class DynamixelXL320
{
public:

  static const uint8_t BROADCAST = 0xFE;

  enum Baud : uint8_t {
    BAUD_9600 = 0,
    BAUD_57600 = 1,
    BAUD_115200 = 2,
    //BAUD_1Mbps = 3
  };

  enum Led : int8_t {
    LED_OFF = 0,
    LED_RED = 1,
    LED_GREEN = 2,
    LED_YELLOW = 3,
    LED_BLUE = 4,
    LED_PINK = 5,
    LED_BLUE_GREEN = 6,
    LED_WHITE = 7,
    LED_UNKNOWN = -1
  };

  enum Control : uint8_t {
    MODE_WHEEL = 1,
    MODE_JOINT = 2,
    MODE_UNKNOWN = 0
  };

  enum Torque : int8_t {
    TORQUE_DISABLED = 0,
    TORQUE_ENABLED = 1,
    TORQUE_UNKNOWN = -1
  };

  enum Alert : uint8_t {
    OVERLOAD = 0x01, //Motor is overloaded
    OVER_HEATING = 0x02, //Temperature is out of operational range
    INPUT_VOLTAGE = 0X04, //Voltage is out of operational range
  };

  enum Error : uint8_t {
    RESULT_FAIL = 0x01,
    INSTRUCTION = 0x02,
    CRC_ERROR = 0x03,
    DATA_RANGE = 0x04,
    DATA_LENGTH = 0x05,
    DATA_LIMIT = 0x06,
    ACCESS_ERROR = 0x07
  };

  typedef void (*ErrorCallback)(uint8_t id, uint8_t error, bool alert);
  typedef void (*PingCallback)(uint8_t id, uint16_t model, uint8_t fwVersion);

  //constructor
  DynamixelXL320(uint8_t rxPin, uint8_t txPin);

  void      begin(Baud baud = Baud::BAUD_9600, uint8_t timeout = 100);
  bool      ping(PingCallback callback, uint8_t id = BROADCAST);  

  uint8_t   getError(uint8_t id);
  bool      getAlert(uint8_t id);
  uint8_t   getAlertCode(uint8_t id);
  void      clearError(uint8_t id);
  void      setErrorCallback(ErrorCallback errorCallback);

  Led       getLED(uint8_t id);
  bool      setLED(uint8_t id, Led value);

  Control   getControlMode(uint8_t id);
  bool      setControlMode(uint8_t id, Control value);

  uint16_t  getGoalPosition(uint8_t id);
  bool      setGoalPosition(uint8_t id, uint16_t value);

  uint16_t  getGoalSpeed(uint8_t id);
  bool      setGoalSpeed(uint8_t id, uint16_t value);

  Torque    getTorqueEnable(uint8_t id);
  bool      setTorqueEnable(uint8_t id, Torque value);

  uint16_t  getMaxTorque(uint8_t id);
  bool      setMaxTorque(uint8_t id, uint16_t value);

  bool      getIsMoving(uint8_t id);
  uint16_t  getCurrentPosition(uint8_t id);
  uint16_t  getCurrentSpeed(uint8_t id);
  uint16_t  getCurrentLoad(uint8_t id);
  float     getCurrentVoltage(uint8_t id);
  uint8_t   getCurrentTemperature(uint8_t id);

private:

  typedef void (*PacketCallback)(uint8_t* buffer, uint16_t length, void* state);

  void      checkErrors();
  bool      sendPacket(uint8_t* buffer, uint16_t size);
  bool      readPacket(uint8_t* buffer, uint16_t size, bool processErrors = true);
  bool      sendAndRead(uint8_t id, uint8_t* buffer, uint16_t send_size, uint16_t read_size = 0, bool processErrors = true, PacketCallback cb = NULL, void* cbState = NULL);
  bool      writeValue(uint8_t id, uint8_t address, uint8_t nbytes, uint16_t value, bool readStatus = true, PacketCallback cb = NULL, void* cbState = NULL);
  bool      readValue(uint8_t id, uint8_t address, uint8_t nbytes, uint16_t* value, bool processErrors = true);
  uint16_t  update_crc(uint16_t crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);

  class HalfDuplexSoftwareSerial : public SoftwareSerial
  {
  public:
    HalfDuplexSoftwareSerial(uint8_t rxPin, uint8_t txPin)
      : SoftwareSerial(rxPin, txPin), _txPin(txPin) {
    }
    void enableTx() { pinMode(_txPin, OUTPUT); }
    void disableTx() { pinMode(_txPin, INPUT); }
  private:
    uint8_t _txPin;
  };

  uint8_t _mode;
  ErrorCallback _errorCallback;
  HalfDuplexSoftwareSerial _serial;
  uint8_t _errors[MAX_ID_FOR_ERRORS + 1];
};

#endif
