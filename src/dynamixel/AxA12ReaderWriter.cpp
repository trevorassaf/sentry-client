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

constexpr size_t AXA12_STATUS_PACKET_HEADER_LENGTH = 5;

constexpr size_t FIRST_PREAMBLE_HEADER_BYTE_IDX = 0;
constexpr size_t SECOND_PREAMBLE_HEADER_BYTE_IDX = 1;
constexpr size_t ID_HEADER_BYTE_IDX = 2;
constexpr size_t LENGTH_HEADER_BYTE_IDX = 3;
constexpr size_t ERROR_HEADER_BYTE_IDX = 4;

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

bool AxA12ReaderWriter::Write(const AxA12InstructionPacket &packet)
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

  if (!uart_->Write(write_buffer.data(), write_buffer.size()))
  {
    LOG(ERROR) << "Failed to send instruction packet.";
    return false;
  }

  return true;
}

bool AxA12ReaderWriter::Read(AxA12StatusPacket *out_packet)
{
  assert(out_packet);

  AxA12StatusPacketHeader header;
  if (!ReadStatusPacketHeader(&header))
  {
    LOG(ERROR) << "Failed to read status packet header";
    return false;
  }

  if (!ReadStatusPacketBody(std::move(header), out_packet))
  {
    LOG(ERROR) << "Failed to read status packet body";
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

bool AxA12ReaderWriter::ReadStatusPacketHeader(AxA12StatusPacketHeader *out_header)
{
  assert(out_header);

  uint8_t header_buffer[AXA12_STATUS_PACKET_HEADER_LENGTH];
  if (!uart_->Read(header_buffer, sizeof(header_buffer)))
  {
    LOG(ERROR) << "Uart read operation failed during header read";
    return false;
  }

  // Ensure first two header bytes match
  const std::vector<uint8_t> &header_header_bytes = GetHeaderBytes();
  assert(header_header_bytes.size() == 2);
  if (header_header_bytes.at(FIRST_PREAMBLE_HEADER_BYTE_IDX) !=
      header_buffer[FIRST_PREAMBLE_HEADER_BYTE_IDX])
  {
    LOG(ERROR) << "First header byte does not match: expected header="
               << static_cast<int>(header_header_bytes.at(FIRST_PREAMBLE_HEADER_BYTE_IDX))
               << ", empirical header="
               << static_cast<int>(header_buffer[FIRST_PREAMBLE_HEADER_BYTE_IDX]);
    return false;
  }

  if (header_header_bytes.at(SECOND_PREAMBLE_HEADER_BYTE_IDX) !=
      header_buffer[SECOND_PREAMBLE_HEADER_BYTE_IDX])
  {
    LOG(ERROR) << "Second header byte does not match: expected header="
               << static_cast<int>(header_header_bytes.at(SECOND_PREAMBLE_HEADER_BYTE_IDX))
               << ", empirical header="
               << static_cast<int>(header_buffer[SECOND_PREAMBLE_HEADER_BYTE_IDX]);
    return false;
  }

  *out_header = AxA12StatusPacketHeader{
      header_buffer[ID_HEADER_BYTE_IDX],
      header_buffer[LENGTH_HEADER_BYTE_IDX],
      header_buffer[ERROR_HEADER_BYTE_IDX]};

  return true;
}

bool AxA12ReaderWriter::ReadStatusPacketBody(
    AxA12StatusPacketHeader header,
    AxA12StatusPacket *out_packet)
{
  assert(out_packet);

  size_t body_length = header.GetLength() - 1;

  std::vector<uint8_t> body_buffer;
  body_buffer.resize(body_length);

  if (!uart_->Read(body_buffer.data(), body_buffer.size()))
  {
    LOG(ERROR) << "Uart read operation failed during body read";
    return false;
  }

  uint8_t crc = body_buffer.back();
  body_buffer.pop_back();

  *out_packet = AxA12StatusPacket{
      std::move(header),
      std::move(body_buffer),
      crc};

  return true;
}

}  // namespace dynamixel
