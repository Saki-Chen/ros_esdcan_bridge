#pragma once
#include <stdint.h>
namespace esdcan
{
    constexpr uint8_t CAN_BYTES_COUNT = 8;
    using CanBytes = uint8_t[CAN_BYTES_COUNT];
    using ConstCanBytes = const uint8_t[CAN_BYTES_COUNT];
}