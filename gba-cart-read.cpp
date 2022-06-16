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
        Filesystem::setTargetSize(Cartridge::getROMSize());
    }

    while(true)
    {
        tud_task();
    }

    return 0;
}
