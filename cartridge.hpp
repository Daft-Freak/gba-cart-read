#pragma once

#include <cstdint>

namespace Cartridge
{
    // partial, nulls added to string-ish fields
    struct HeaderInfo
    {
        char title[13];
        char gameCode[5];
        bool checksumValid;
    };

    void initIO();

    void readROM(uint32_t addr, uint16_t *data, int count);
    void readRAMSave(uint16_t addr, uint8_t *data, int count);

    HeaderInfo readHeader();

    uint32_t getROMSize();
}