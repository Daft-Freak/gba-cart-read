#include <algorithm>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#include "cartridge.hpp"
#include "filesystem.hpp"

static char curGameCode[4]{0};

static Cartridge::SaveType saveType = Cartridge::SaveType::Unknown;

static void readROM(uint32_t offset, uint32_t len, uint8_t *buf)
{
    // might have to split this to not wrap the address
    // (assuming nothing ever tries to read > 64k halfwords)
    auto maxLen = 0x20000 - (offset & 0x1FFFF);

    Cartridge::readROM(offset, reinterpret_cast<uint16_t *>(buf), std::min(len, maxLen) / 2);

    if(maxLen < len)
        Cartridge::readROM(offset + maxLen, reinterpret_cast<uint16_t *>(buf + maxLen), (len - maxLen) / 2);
}

static void readSave(uint32_t offset, uint32_t len, uint8_t *buf)
{
    if(saveType == Cartridge::SaveType::EEPROM_512)
        Cartridge::readEEPROMSave(offset, reinterpret_cast<uint64_t *>(buf), len / 8, false);
    else if(saveType == Cartridge::SaveType::EEPROM_8K)
        Cartridge::readEEPROMSave(offset, reinterpret_cast<uint64_t *>(buf), len / 8, true);
    else if(saveType == Cartridge::SaveType::Flash_128K)
        Cartridge::readFlashSave(offset, buf, len);
    else
        Cartridge::readRAMSave(offset, buf, len); // this is okay for 64k flash
}

int main()
{
    stdio_init_all();

    tusb_init();

    Cartridge::initIO();

    uint32_t lastCheckTime = 0;

    while(true)
    {
        tud_task();

        auto curTime = to_ms_since_boot(get_absolute_time());

        // check more frequently if no cart
        uint32_t interval = curGameCode[0] == 0 ? 100 : 1000;

        // cart detection polling
        if(curTime - lastCheckTime > interval)
        {
            auto header = Cartridge::readHeader();

            if(header.checksumValid)
            {
                // valid header, check if cart is changing
                if(memcmp(curGameCode, header.gameCode, 4) != 0)
                {
                    // update cart size
                    memcpy(curGameCode, header.gameCode, 4);
        
                    auto romSize = Cartridge::getROMSize();
                    uint32_t saveSize = 0;
                    saveType = Cartridge::getSaveType(romSize);

                    switch(saveType)
                    {
                        case Cartridge::SaveType::Unknown:
                            break;
                        case Cartridge::SaveType::EEPROM_512:
                            saveSize = 512; break;
                        case Cartridge::SaveType::EEPROM_8K:
                            saveSize = 8 * 1024; break;
                        case Cartridge::SaveType::RAM:
                            saveSize = 32 * 1024; break;
                        case Cartridge::SaveType::Flash_64K:
                            saveSize = 64 * 1024; break;
                        case Cartridge::SaveType::Flash_128K:
                            saveSize = 128 * 1024; break;
                    }

                    Filesystem::setTargetSize(romSize + saveSize);

                    Filesystem::resetFiles();
                    Filesystem::addFile(0, romSize, header.gameCode, "GBA", readROM);

                    if(saveSize)
                        Filesystem::addFile(romSize, saveSize, header.gameCode, "SAV", readSave);
                }
            }
            else
            {
                memset(curGameCode, 0, 4);
                Filesystem::setTargetSize(0);
            }

            lastCheckTime = curTime;
        }
    }

    return 0;
}
