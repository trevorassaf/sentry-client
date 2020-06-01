#include "dynamixel/AxA12RegisterMap.h"

#include <cassert>

namespace dynamixel
{

AxA12Register::AxA12Register(
    AxA12RegisterType type,
    MemoryRegion memory_region,
    AccessMode access_mode)
  : type_{type},
    memory_region_{memory_region},
    access_mode_{access_mode},
    has_initial_value_{false} {}

AxA12Register::AxA12Register(
    AxA12RegisterType type,
    MemoryRegion memory_region,
    AccessMode access_mode,
    uint8_t initial_value)
  : type_{type},
    memory_region_{memory_region},
    access_mode_{access_mode},
    has_initial_value_{true},
    initial_value_{initial_value} {}

AxA12RegisterType AxA12Register::GetType() const
{
  return type_;
}

MemoryRegion AxA12Register::GetMemoryRegion() const
{
  return memory_region_;
}

AccessMode AxA12Register::GetAccessMode() const
{
  return access_mode_;
}

bool AxA12Register::HasInitialValue() const
{
  return has_initial_value_;
}

uint8_t AxA12Register::GetInitialValue() const
{
  return initial_value_;
}

#define AX_A12_REG_WO_INITIAL(type, memory_region, access_mode) \
  {AxA12RegisterType::type, AxA12Register(AxA12RegisterType::type, MemoryRegion::memory_region, AccessMode::access_mode)}

#define AX_A12_REG_WI_INITIAL(type, memory_region, access_mode, initial_value) \
  {AxA12RegisterType::type, AxA12Register(AxA12RegisterType::type, MemoryRegion::memory_region, AccessMode::access_mode, initial_value)}


std::unordered_map<AxA12RegisterType, AxA12Register> AXA12_REGISTER_MAP =
{
  AX_A12_REG_WI_INITIAL(MODEL_NUMBER_L, EEPROM, R, 0x0C),
  AX_A12_REG_WI_INITIAL(MODEL_NUMBER_H, EEPROM, R, 0x00),
  AX_A12_REG_WO_INITIAL(FW_VERSION, EEPROM, R),
  AX_A12_REG_WI_INITIAL(ID, EEPROM, RW, 0x01),
  AX_A12_REG_WI_INITIAL(BAUD_RATE, EEPROM, RW, 0x01),
  AX_A12_REG_WI_INITIAL(RETURN_DELAY_TIME, EEPROM, RW, 0xFA),
  AX_A12_REG_WI_INITIAL(CW_ANGLE_LIMIT_L, EEPROM, RW, 0x00),
  AX_A12_REG_WI_INITIAL(CW_ANGLE_LIMIT_H, EEPROM, RW, 0x00),
  AX_A12_REG_WI_INITIAL(CCW_ANGLE_LIMIT_L, EEPROM, RW, 0xFF),
  AX_A12_REG_WI_INITIAL(CCW_ANGLE_LIMIT_H, EEPROM, RW, 0x03),
  AX_A12_REG_WI_INITIAL(HIGHEST_LIMIT_TEMPERATURE, EEPROM, RW, 0x46),
  AX_A12_REG_WI_INITIAL(LOWEST_LIMIT_VOLTAGE, EEPROM, RW, 0x3C),
  AX_A12_REG_WI_INITIAL(HIGHEST_LIMIT_VOLTAGE, EEPROM, RW, 0xBE),
  AX_A12_REG_WI_INITIAL(MAX_TORQUE_L, EEPROM, RW, 0xFF),
  AX_A12_REG_WI_INITIAL(MAX_TORQUE_H, EEPROM, RW, 0x03),
  AX_A12_REG_WI_INITIAL(STATUS_RETURN_LEVEL, EEPROM, RW, 0x02),
  AX_A12_REG_WI_INITIAL(ALARM_LED, EEPROM, RW, 0x24),
  AX_A12_REG_WI_INITIAL(ALARM_SHUTDOWN, EEPROM, RW, 0x24),
  AX_A12_REG_WI_INITIAL(TORQUE_ENABLE, RAM, RW, 0x00),
  AX_A12_REG_WI_INITIAL(LED, RAM, RW, 0x00),
  AX_A12_REG_WI_INITIAL(CW_COMPLIANCE_MARGIN, RAM, RW, 0x01),
  AX_A12_REG_WI_INITIAL(CCW_COMPLIANCE_MARGIN, RAM, RW, 0x01),
  AX_A12_REG_WI_INITIAL(CW_COMPLIANCE_SLOPE, RAM, RW, 0x20),
  AX_A12_REG_WI_INITIAL(CCW_COMPLIANCE_SLOPE, RAM, RW, 0x20),
  AX_A12_REG_WO_INITIAL(GOAL_POSITION_L, RAM, RW),
  AX_A12_REG_WO_INITIAL(GOAL_POSITION_H, RAM, RW),
  AX_A12_REG_WO_INITIAL(MOVING_SPEED_L, RAM, RW),
  AX_A12_REG_WO_INITIAL(MOVING_SPEED_H, RAM, RW),
  AX_A12_REG_WO_INITIAL(TORQUE_LIMIT_L, RAM, RW),
  AX_A12_REG_WO_INITIAL(TORQUE_LIMIT_H, RAM, RW),
  AX_A12_REG_WO_INITIAL(PRESENT_POSITION_L, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_POSITION_H, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_SPEED_L, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_SPEED_H, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_LOAD_L, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_LOAD_H, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_VOLTAGE, RAM, R),
  AX_A12_REG_WO_INITIAL(PRESENT_TEMPERATURE, RAM, R),
  AX_A12_REG_WI_INITIAL(INSTRUCTION_REGISTERED, RAM, R, 0x00),
  AX_A12_REG_WI_INITIAL(IS_MOVING, RAM, R, 0x00),
  AX_A12_REG_WI_INITIAL(EEPROM_LOCK, RAM, RW, 0x00),
  AX_A12_REG_WI_INITIAL(PUNCH_L, RAM, RW, 0x20),
  AX_A12_REG_WI_INITIAL(PUNCH_H, RAM, RW, 0x00),
};

const std::unordered_map<AxA12RegisterType, AxA12Register> &GetAxA12RegisterMap()
{
  return AXA12_REGISTER_MAP;
}

}  // namespace dynamixel
