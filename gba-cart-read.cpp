#include "pico/stdlib.h"
#include "tusb.h"

#include "cartridge.hpp"

int main()
{
    stdio_init_all();

    tusb_init();

    Cartridge::initIO();

    while(true)
    {
        tud_task();
    }

    return 0;
}
