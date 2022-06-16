#include "pico/stdlib.h"
#include "pico/time.h"
#include "tusb.h"

#include "cartridge.hpp"
#include "filesystem.hpp"

static char curGameCode[4]{0};

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
                    Filesystem::setTargetSize(romSize);

                    Filesystem::resetFiles();
                    Filesystem::addFile(0, romSize, header.gameCode, "GBA");
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
