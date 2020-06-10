#include "dynamixel/AxA12ReaderWriter.h"

#include <cassert>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>

#include "Uart/UartClient.h"

#include "dynamixel/AxA12InstructionType.h"
#include "dynamixel/AxA12Utils.h"

namespace
{

std::string ToString(const std::vector<uint8_t> &buffer)
{
  std::stringstream stream;
  for (uint8_t byte : buffer)
  {
    stream << "0x" << std::hex << static_cast<size_t>(byte) << " ";
  }
  return stream.str();
}

} // namespace

namespace dynamixel
{

AxA12ReaderWriter::AxA12ReaderWriter() : is_initialized_{false} {}

AxA12ReaderWriter::AxA12ReaderWriter(Uart::UartClient *uart)
  : is_initialized_{true},
    uart_{uart} {}

AxA12ReaderWriter::AxA12ReaderWriter(AxA12ReaderWriter &&other)
{
  StealResources(&other);
}

AxA12ReaderWriter &AxA12ReaderWriter::operator=(AxA12ReaderWriter &&other)
{
  if (this != &other)
  {
    StealResources(&other);
  }
  return *this;
}

bool AxA12ReaderWriter::Write(const AxA12Packet &packet)
{
  assert(is_initialized_);

  std::vector<uint8_t> write_buffer;
  write_buffer.reserve(
      CalculateWriteBufferSize(packet.GetParameters().size()));

  const std::vector<uint8_t> &header_bytes = GetHeaderBytes();
  write_buffer.insert(
      std::end(write_buffer),
      std::begin(header_bytes),
      std::end(header_bytes));

  write_buffer.push_back(packet.GetId());
  write_buffer.push_back(
      CalculatePacketLengthParameter(packet.GetParameters().size()));
  write_buffer.push_back(
      static_cast<uint8_t>(packet.GetInstructionType()));

  for (uint8_t p : packet.GetParameters())
  {
    write_buffer.push_back(p);
  }

  write_buffer.push_back(CalculatePacketCrc(packet));

  LOG(ERROR) << "bozkurtus -- UART write buffer: " << ToString(write_buffer);

  if (!uart_->Write(write_buffer.data(), write_buffer.size()))
  {
    LOG(ERROR) << "Failed to write uart data. Buffer: " << ToString(write_buffer);
    return false;
  }

  return true;
}

void AxA12ReaderWriter::StealResources(AxA12ReaderWriter *other)
{
  is_initialized_ = other->is_initialized_;
  other->is_initialized_ = false;
  uart_ = other->uart_;
  other->uart_ = nullptr;
}

}  // namespace dynamixel
