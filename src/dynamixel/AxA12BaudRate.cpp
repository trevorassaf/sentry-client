#include "dynamixel/AxA12BaudRate.h"

namespace dynamixel
{

const char *ToString(AxA12BaudRate rate)
{
  switch (rate)
  {
    case AxA12BaudRate::BAUD_1000000:
      return "1000000 bps";
    case AxA12BaudRate::BAUD_500000:
      return "500000 bps";
    case AxA12BaudRate::BAUD_400000:
      return "400000 bps";
    case AxA12BaudRate::BAUD_250000:
      return "250000 bps";
    case AxA12BaudRate::BAUD_200000:
      return "200000 bps";
    case AxA12BaudRate::BAUD_117647:
      return "117647 bps";
    case AxA12BaudRate::BAUD_57142:
      return "57142 bps";
    case AxA12BaudRate::BAUD_19230:
      return "19230 bps";
    case AxA12BaudRate::BAUD_9615:
      return "9615 bps";
  }
}

} // namespace dynamixel
