#ifndef SENTRY_CLIENT_DYNAMIXEL_AXA12BAUDRATE_H_
#define SENTRY_CLIENT_DYNAMIXEL_AXA12BAUDRATE_H_

#include <cstdint>

namespace dynamixel
{

enum class AxA12BaudRate : uint8_t
{
  BAUD_1000000 = 1,
  BAUD_500000 = 3,
  BAUD_400000 = 4,
  BAUD_250000 = 7,
  BAUD_200000 = 9,
  BAUD_117647 = 16,
  BAUD_57142 = 34,
  BAUD_19230 = 103,
  BAUD_9615 = 207,
};

const char *ToString(AxA12BaudRate baud);

}  // namespace dynamixel
#endif // SENTRY_CLIENT_DYNAMIXEL_AXA12BAUDRATE_H_ 
