#include "pico/stdlib.h"
#include "tusb.h"

#include "cartridge.hpp"
#include "filesystem.hpp"

int main()
{
    stdio_init_all();

    tusb_init();

    Cartridge::initIO();

    Filesystem::setTargetSize(32 * 1024 * 1024); // TODO

    while(true)
    {
        tud_task();
    }

    return 0;
}
