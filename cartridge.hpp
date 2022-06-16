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

    enum class SaveType
    {
        Unknown,
        EEPROM_512,
        EEPROM_8K,
        RAM, // 32K
        Flash_64K,
        Flash_128K
    };

    void initIO();

    void readROM(uint32_t addr, uint16_t *data, int count);
    void readRAMSave(uint16_t addr, uint8_t *data, int count);

    HeaderInfo readHeader();

    uint32_t getROMSize();

    SaveType getSaveType(uint32_t romSize);
}