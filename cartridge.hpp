#pragma once

#include <cstdint>

namespace Cartridge
{
    void initIO();
    void readROM(uint32_t addr, uint16_t *data, int count);
}