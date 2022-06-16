#include <array>
#include "pico/stdlib.h"
#include "tusb.h"

#include "cartridge.hpp"
#include "filesystem.hpp"

int main()
{
    stdio_init_all();

    tusb_init();

    Cartridge::initIO();

    auto header = Cartridge::readHeader();

    if(header.checksumValid)
    {
        // size check
        // check for incrementing values
        uint32_t size = 1 << 22; // start at 4MB

        for(; size < 1 << 25; size <<= 1)
        {
            uint16_t val;

            // checking first 16 halfwords
            bool has_data = false;
            for(int i = 0; i < 16; i++)
            {
                Cartridge::readROM(size + i * 2, &val, 1);
                if(val != i)
                    has_data = true;
            }

            // found end of ROM, stop
            if(!has_data)
                break;
        }

        Filesystem::setTargetSize(size);
    }

    while(true)
    {
        tud_task();
    }

    return 0;
}
