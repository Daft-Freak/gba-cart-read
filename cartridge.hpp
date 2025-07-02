#pragma once

#include <cstdint>

namespace Cartridge
{
    // partial, nulls added to string-ish fields
    struct GBAHeaderInfo
    {
        char title[13];
        char gameCode[5];
        bool checksumValid;
    };
    struct DMGHeaderInfo
    {
        char title[17];
        uint8_t cartType;
        uint8_t romSize;
        uint8_t ramSize;
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

    enum class MBCType
    {
        None = 0,
        MBC1,
        MBC1M, // multicart - wired slightly differently
        MBC2,
        MBC3,
        MBC5
    };

    void initIO();

    void readROM(uint32_t addr, volatile uint16_t *data, int count);

    void readRAMSave(uint16_t addr, volatile uint8_t *data, int count);
    void writeRAMSave(uint16_t addr, volatile const uint8_t *data, int count);

    void readFlashSave(uint32_t addr, volatile uint8_t *data, int count);

    void readEEPROMSave(uint16_t addr, volatile uint64_t *data, int count, bool is8k);

    // raw access to GB cart
    void readDMG(uint16_t addr, volatile uint8_t *data, int count);

    GBAHeaderInfo readGBAHeader();
    DMGHeaderInfo readDMGHeader();

    uint32_t getGBAROMSize();

    uint32_t getDMGROMSize(const DMGHeaderInfo &header);
    uint32_t getDMGRAMSize(const DMGHeaderInfo &header);

    SaveType getSaveType(uint32_t romSize);

    MBCType getMBCType(const DMGHeaderInfo &header);
}