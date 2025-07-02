#include <algorithm>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#include "cartridge.hpp"
#include "filesystem.hpp"

#ifdef PICO_RP2350
#define GB_DETECT_PIN 28
#define STATUS_LED_PIN 31
#endif

static char curGameCode[4]{0};

static Cartridge::SaveType saveType = Cartridge::SaveType::Unknown;

static void statusSet(bool value)
{
#ifdef STATUS_LED_PIN
    gpio_put(STATUS_LED_PIN, !value);
#endif
}

static void statusToggle()
{
#ifdef STATUS_LED_PIN
    gpio_xor_mask64(1 << STATUS_LED_PIN);
#endif
}

static absolute_time_t ABSOLUTE_TIME_INITIALIZED_VAR(lastAccessBlinkTime, 0);

static void blinkLEDForAccess()
{
    auto time = get_absolute_time();
    if(absolute_time_diff_us(lastAccessBlinkTime, time) > 80000)
    {
        statusToggle();
        lastAccessBlinkTime = time;
    }
}

static void readDMGROM(uint32_t offset, uint32_t len, uint8_t *buf)
{
    // blink while reading
    blinkLEDForAccess();

    return Cartridge::readDMG(offset, buf, len);
}

static void readROM(uint32_t offset, uint32_t len, uint8_t *buf)
{
    // might have to split this to not wrap the address
    // (assuming nothing ever tries to read > 64k halfwords)
    auto maxLen = 0x20000 - (offset & 0x1FFFF);

    // blink while reading
    blinkLEDForAccess();

    Cartridge::readROM(offset, reinterpret_cast<uint16_t *>(buf), std::min(len, maxLen) / 2);

    if(maxLen < len)
        Cartridge::readROM(offset + maxLen, reinterpret_cast<uint16_t *>(buf + maxLen), (len - maxLen) / 2);
}

static void readSave(uint32_t offset, uint32_t len, uint8_t *buf)
{
    // blink while reading
    blinkLEDForAccess();

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
    tusb_init();

    stdio_init_all();

    // wait to be mounted before scanning for cart
    while(!tud_mounted()) tud_task();

    Cartridge::initIO();

#ifdef STATUS_LED_PIN
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, true);
#endif

#ifdef GB_DETECT_PIN
    gpio_init(GB_DETECT_PIN);
    gpio_set_dir(GB_DETECT_PIN, false);
#endif

    statusSet(false);

    uint32_t lastCheckTime = 0;
    uint32_t lastBlinkTime = 0;

    bool gbDetected = false;

    while(true)
    {
        tud_task();

        auto curTime = to_ms_since_boot(get_absolute_time());

        // check more frequently if no cart
        bool validGame = curGameCode[0] != 0;
        uint32_t interval = !validGame ? 100 : 1000;

        // cart detection polling
        if(curTime - lastCheckTime > interval)
        {
#ifdef GB_DETECT_PIN
            gbDetected = !gpio_get(GB_DETECT_PIN);
#endif
            // blink LED if searching
            if(!validGame && curTime - lastBlinkTime >= 500)
            {
                statusToggle();
                lastBlinkTime = curTime;
            }

            uint32_t romSize = 0;

            if(!gbDetected) // !GB == GBA
            {
                auto header = Cartridge::readGBAHeader();


                if(header.checksumValid)
                {
                    // valid header, check if cart is changing
                    if(memcmp(curGameCode, header.gameCode, 4) != 0)
                    {
                        // update cart size
                        memcpy(curGameCode, header.gameCode, 4);
            
                        romSize = Cartridge::getGBAROMSize();
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
                    else
                    {
                        // same game, don't reset
                        lastCheckTime = curTime;
                        continue;
                    }
                }
            }
            else // GB
            {
                auto header = Cartridge::readDMGHeader();

                if(header.checksumValid)
                {
                    romSize = 32 * 1024; // TODO

                    if(!curGameCode[0])
                    {
                        Filesystem::setTargetSize(romSize);

                        Filesystem::resetFiles();
                        // title will get truncated to 8 chars...
                        Filesystem::addFile(0, romSize, header.title, "GB", readDMGROM);
                    }

                    curGameCode[0] = 1;
                }
            }

            // keep lit if valid
            if(curGameCode[0] != 0 && !validGame)
                statusSet(true);
            
            if(!romSize)
            {
                memset(curGameCode, 0, 4);
                Filesystem::setTargetSize(0);
            }

            lastCheckTime = curTime;
        }

        // restore LED after access
        if(curGameCode[0] != 0 && absolute_time_diff_us(lastAccessBlinkTime, get_absolute_time()) > 1000000)
            statusSet(true);
    }

    return 0;
}
