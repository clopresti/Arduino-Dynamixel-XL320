#include <Arduino.h>
#include <SoftwareSerial.h>


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

  void      begin(Baud baud = Baud::BAUD_9600);
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

  bool sendPacket(uint8_t* buffer, uint16_t size);
  uint16_t readPacket(uint8_t* buffer, uint16_t size, bool processErrors = true);
  bool sendAndRead(uint8_t id, uint8_t* buffer, uint16_t send_size, uint16_t read_size = 0, bool processErrors = true, PacketCallback cb = NULL, void* cbState = NULL);
  bool writeValue(uint8_t id, uint8_t address, uint8_t nbytes, uint16_t value, bool readStatus = true, PacketCallback cb = NULL, void* cbState = NULL);
  bool readValue(uint8_t id, uint8_t address, uint8_t nbytes, uint16_t* value, bool processErrors = true);
  void checkErrors();

  uint16_t update_crc(uint16_t crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);

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

  static const uint8_t MODE_RECEIVE = 0;
  static const uint8_t MODE_SEND = 1;

  static const uint8_t VALUE_8_BIT = 1;
  static const uint8_t VALUE_16_BIT = 2;

  static const uint8_t INST_PING = 0x01;
  static const uint8_t INST_READ = 0x02;
  static const uint8_t INST_WRITE = 0x03;

  static const uint8_t ADDR_CONTROL_MODE = 0x0B; //11
  static const uint8_t ADDR_MAX_TORQUE = 0x0F; //15
  static const uint8_t ADDR_TORQUE_ENABLE = 0x18; //24
  static const uint8_t ADDR_LED = 0x19; //25
  static const uint8_t ADDR_GOAL_POSITION = 0x1E; //30
  static const uint8_t ADDR_GOAL_SPEED = 0x20; //32
  static const uint8_t ADDR_CURRENT_POSITION = 0x25; //37
  static const uint8_t ADDR_CURRENT_SPEED = 0x27; //39
  static const uint8_t ADDR_CURRENT_LOAD = 0x29; //41
  static const uint8_t ADDR_CURRENT_VOLTAGE = 0x2D; //45
  static const uint8_t ADDR_CURRENT_TEMPERATURE = 0x2E; //46
  static const uint8_t ADDR_MOVING = 0x31; //49
  static const uint8_t ADDR_ERROR_STATUS = 0x32; //50

  uint8_t _mode;
  uint8_t _errors[253];
  ErrorCallback _errorCallback;
  HalfDuplexSoftwareSerial _serial;
};
