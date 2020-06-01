#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12REGISTERMAP_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12REGISTERMAP_H_

#include <cstdint>
#include <string>
#include <unordered_map>

namespace dynamixel
{

enum class MemoryRegion : uint8_t
{
  EEPROM,
  RAM,
};

enum class AccessMode : uint8_t
{
  R,
  W,
  RW,
};

enum class AxA12RegisterType : uint8_t
{
  MODEL_NUMBER_L = 0x00,
  MODEL_NUMBER_H = 0x01,
  FW_VERSION = 0x02,
  ID = 0x03,
  BAUD_RATE = 0x04,
  RETURN_DELAY_TIME = 0x05,
  CW_ANGLE_LIMIT_L = 0x06,
  CW_ANGLE_LIMIT_H = 0x07,
  CCW_ANGLE_LIMIT_L = 0x08,
  CCW_ANGLE_LIMIT_H = 0x09,
  HIGHEST_LIMIT_TEMPERATURE = 0x0B,
  LOWEST_LIMIT_VOLTAGE = 0x0C,
  HIGHEST_LIMIT_VOLTAGE = 0x0D,
  MAX_TORQUE_L = 0x0E,
  MAX_TORQUE_H = 0x0F,
  STATUS_RETURN_LEVEL = 0x10,
  ALARM_LED = 0x11,
  ALARM_SHUTDOWN = 0x12,
  TORQUE_ENABLE = 0x18,
  LED = 0x19,
  CW_COMPLIANCE_MARGIN = 0x1A,
  CCW_COMPLIANCE_MARGIN = 0x1B,
  CW_COMPLIANCE_SLOPE = 0x1C,
  CCW_COMPLIANCE_SLOPE = 0x1D,
  GOAL_POSITION_L = 0x1E,
  GOAL_POSITION_H = 0x1F,
  MOVING_SPEED_L = 0x20,
  MOVING_SPEED_H = 0x21,
  TORQUE_LIMIT_L = 0x22,
  TORQUE_LIMIT_H = 0x23,
  PRESENT_POSITION_L = 0x24,
  PRESENT_POSITION_H = 0x25,
  PRESENT_SPEED_L = 0x26,
  PRESENT_SPEED_H = 0x27,
  PRESENT_LOAD_L = 0x28,
  PRESENT_LOAD_H = 0x29,
  PRESENT_VOLTAGE = 0x2A,
  PRESENT_TEMPERATURE = 0x2B,
  INSTRUCTION_REGISTERED = 0x2C,
  IS_MOVING = 0x2E,
  EEPROM_LOCK = 0x2F,
  PUNCH_L = 0x30,
  PUNCH_H = 0x31,
};

class AxA12Register
{
  public:
    AxA12Register(
        AxA12RegisterType type,
        MemoryRegion memory_region,
        AccessMode access_mode);
    AxA12Register(
        AxA12RegisterType type,
        MemoryRegion memory_region,
        AccessMode access_mode,
        uint8_t initial_value);
    AxA12RegisterType GetType() const;
    MemoryRegion GetMemoryRegion() const;
    AccessMode GetAccessMode() const;
    bool HasInitialValue() const;
    uint8_t GetInitialValue() const;

  private:
    AxA12RegisterType type_;
    MemoryRegion memory_region_;
    AccessMode access_mode_;
    bool has_initial_value_;
    uint8_t initial_value_;
};

const std::unordered_map<AxA12RegisterType, AxA12Register> &GetAxA12RegisterMap();

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12REGISTERMAP_H_
