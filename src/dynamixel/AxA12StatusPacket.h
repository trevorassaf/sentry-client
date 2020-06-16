#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12STATUSPACKET_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12STATUSPACKET_H_

#include <bitset>
#include <cstdint>
#include <vector>

namespace dynamixel
{

enum class AxA12ErrorType : uint8_t
{
  INPUT_VOLTAGE = 0,
  ANGLE_LIMIT   = 1,
  OVERHEATING   = 2,
  RANGE         = 3,
  CHECKSUM      = 4,
  OVERLOAD      = 5,
  INSTRUCTION   = 6,
};

class AxA12StatusPacketHeader
{
  public:
    AxA12StatusPacketHeader();
    AxA12StatusPacketHeader(
        uint8_t id,
        uint8_t length,
        uint8_t error_bitset);
    uint8_t GetId() const;
    uint8_t GetLength() const;
    bool HasError(AxA12ErrorType type) const;
    bool HasError() const;

  private:
    bool is_valid_;
    uint8_t id_;
    uint8_t length_;
    std::bitset<8> error_bitset_;
};

class AxA12StatusPacket
{
public:
  AxA12StatusPacket();
  AxA12StatusPacket(
      AxA12StatusPacketHeader header,
      std::vector<uint8_t> params,
      uint8_t crc);

  const AxA12StatusPacketHeader &GetHeader() const;
  const std::vector<uint8_t> &GetParameters() const;
  uint8_t GetCrc() const;
  std::string ToString() const;

private:
  bool is_valid_;
  AxA12StatusPacketHeader header_;
  std::vector<uint8_t> params_;
  uint8_t crc_;
};

} // namespace dynamixel

#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12STATUSPACKET_H_
