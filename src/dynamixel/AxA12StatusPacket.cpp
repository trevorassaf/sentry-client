#include "dynamixel/AxA12StatusPacket.h"

#include <cassert>
#include <sstream>

#include "glog/logging.h"

namespace dynamixel
{

AxA12StatusPacketHeader::AxA12StatusPacketHeader() : is_valid_{false} {}

AxA12StatusPacketHeader::AxA12StatusPacketHeader(
    uint8_t id,
    uint8_t length,
    uint8_t error_bitset)
  : is_valid_{true},
    id_{id},
    length_{length},
    error_bitset_{error_bitset} {}

uint8_t AxA12StatusPacketHeader::GetId() const
{
  assert(is_valid_);
  return id_;
}

uint8_t AxA12StatusPacketHeader::GetLength() const
{
  assert(is_valid_);
  return length_;
}

bool AxA12StatusPacketHeader::HasError(AxA12ErrorType type) const
{
  assert(is_valid_);
  return error_bitset_.test(static_cast<size_t>(type));
}

AxA12StatusPacket::AxA12StatusPacket() : is_valid_{false} {}

AxA12StatusPacket::AxA12StatusPacket(
    AxA12StatusPacketHeader header,
    std::vector<uint8_t> params,
    uint8_t crc)
  : is_valid_{true},
    header_{header},
    params_{std::move(params)},
    crc_{crc} {}

const AxA12StatusPacketHeader &AxA12StatusPacket::GetHeader() const
{
  assert(is_valid_);
  return header_;
}

const std::vector<uint8_t> &AxA12StatusPacket::GetParameters() const
{
  assert(is_valid_);
  return params_;
}

uint8_t AxA12StatusPacket::GetCrc() const
{
  assert(is_valid_);
  return crc_;
}

std::string AxA12StatusPacket::ToString() const
{
  std::ostringstream stream;
  stream << "\n- id=" << static_cast<int>(header_.GetId());
  stream << "\n- length=" << static_cast<int>(header_.GetLength());
  stream << "\n- HasError(INPUT_VOLTAGE)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::INPUT_VOLTAGE));
  stream << "\n- HasError(ANGLE_LIMIT)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::ANGLE_LIMIT));
  stream << "\n- HasError(OVERHEATING)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::OVERHEATING));
  stream << "\n- HasError(RANGE)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::RANGE));
  stream << "\n- HasError(CHECKSUM)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::CHECKSUM));
  stream << "\n- HasError(OVERLOAD)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::OVERLOAD));
  stream << "\n- HasError(INSTRUCTION)="
         << static_cast<int>(header_.HasError(AxA12ErrorType::INSTRUCTION));

  for (size_t i = 0; i < params_.size(); ++i)
  {
    stream << "\n- params[" << i << "]=0x" << std::hex
           << static_cast<int>(params_.at(i)) << std::dec;
  }

  stream << "\n- crc=" << static_cast<int>(crc_);
  return stream.str();
}

} // namespace dynamixel
