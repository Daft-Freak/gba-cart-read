#include <cassert>

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "cartridge.hpp"
#include "gba-cart.pio.h"

// 24 bit addr on pins 0-23
// 16 bit data on pins 0-15

static const int wrPin = 24;
static const int rdPin = 25;
static const int romCSPin = 26;
static const int ramCSPin = 27;

namespace Cartridge
{
    int pioSM = -1;

    static void gba_multi_program_init(PIO pio, uint sm, uint offset)
    {
        int basePin = 0;
        int addressPins = 24;
        int dataPins = 16;

        uint32_t sideMask = 1 << wrPin | 1 << rdPin | 1 << romCSPin | 1 << ramCSPin;
        uint32_t pinMask = sideMask | ((1 << dataPins) - 1);

        pio_sm_set_pins_with_mask(pio, sm, sideMask, pinMask); // set all control pins inactive
        pio_sm_set_pindirs_with_mask(pio, sm, pinMask, pinMask); // set all pins as output

        for(int i = 0; i < 16; i++)
            pio_gpio_init(pio, basePin + i);

        pio_gpio_init(pio, wrPin);
        pio_gpio_init(pio, rdPin);
        pio_gpio_init(pio, romCSPin);
        pio_gpio_init(pio, ramCSPin);

        pio_sm_config c = gba_cart_program_get_default_config(offset);

        sm_config_set_in_shift(&c, true, true, 16);
        sm_config_set_out_shift(&c, true, false, 32);

        sm_config_set_in_pins(&c, basePin);
        sm_config_set_out_pins(&c, basePin, dataPins);
        sm_config_set_sideset_pins(&c, basePin + addressPins);

        // default wait states for cart ROM are 4/2, most games use 3/1
        // ... so we can get a few MHz
        sm_config_set_clkdiv_int_frac(&c, 15, 0);

        pio_sm_init(pio, sm, offset, &c);
        pio_sm_set_enabled(pio, sm, true);
    }

    void initIO()
    {
        // init PIO
        uint offset = pio_add_program(pio0, &gba_cart_program);
        pioSM = pio_claim_unused_sm(pio0, true);
        gba_multi_program_init(pio0, pioSM, offset);

        // init high address bits (PIO doesn't control these)
        auto mask = 0xFF << 16;
        gpio_init_mask(mask);
        gpio_set_dir_out_masked(mask);
    }

    // read func
    void readROM(uint32_t addr, uint16_t *data, int count)
    {
        assert((addr & 1) == 0);
        assert((addr & 0xFFFF) + count <= 0xFFFF);

        // write high bits of address
        gpio_put_masked(0xFF << 16, addr >> 1);

        // count and low bits of address
        pio_sm_put_blocking(pio0, pioSM, count - 1);
        pio_sm_put_blocking(pio0, pioSM, (addr >> 1) & 0xFFFF);

        while(count--)
            *data++ = pio_sm_get_blocking(pio0, pioSM) >> 16;
    }
}