#include <cassert>

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "cartridge.hpp"

// 24 bit addr on pins 0-23
// 16 bit data on pins 0-15

static const int wrPin = 24;
static const int rdPin = 25;
static const int romCSPin = 26;
static const int ramCSPin = 27;

namespace Cartridge
{
    void initIO()
    {
        // init all IOs
        auto mask = (1 << 28) - 1;
        gpio_init_mask(mask);
        gpio_set_dir_out_masked(mask);

        // inactive
        gpio_put_all(1 << wrPin | 1 << rdPin | 1 << romCSPin | 1 << ramCSPin);
    }

    // read func
    void readROM(uint32_t addr, uint16_t *data, int count)
    {
        assert((addr & 1) == 0);
        assert((addr & 0xFFFF) + count <= 0xFFFF);

        auto addrMask = (1 << 24) - 1;
        auto dataMask = (1 << 16) - 1;

        // write address
        gpio_put_masked(addrMask, addr >> 1);

        const int delay = 1;
        sleep_us(delay);
        
        // cs active
        gpio_put(romCSPin, 0);
        sleep_us(delay);

        // set pins to input
        gpio_set_dir_in_masked(dataMask);

        while(count--)
        {
            // rd active
            gpio_put(rdPin, 0);
            sleep_us(delay);

            *data++ = gpio_get_all() & dataMask;

            gpio_put(rdPin, 1);
            sleep_us(delay);
        }

        gpio_put(romCSPin, 1);
        gpio_set_dir_out_masked(dataMask);
        sleep_us(delay);
    }
}