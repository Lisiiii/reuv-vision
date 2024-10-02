#pragma once

#include <cstdint>

enum class Event : uint8_t { None, Run, Slow, Stop };

struct __attribute__((packed)) Command {
  const uint8_t header = 0xa5;
  float x;
  float y;
  float w;
  uint8_t event;
  const uint8_t end = 0xb6;
};
