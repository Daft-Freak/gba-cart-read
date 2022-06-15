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

    uint16_t buf[96];
    Cartridge::readROM(0, buf, std::size(buf));
    
    // checksum check
    auto headerBytes = reinterpret_cast<uint8_t *>(buf);
    int checksum = 0;
    for(int i = 0xA0; i < 0xBD; i++)
        checksum = checksum - headerBytes[i];

    checksum = (checksum - 0x19) & 0xFF;

    if(headerBytes[0xBD] == checksum)
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
