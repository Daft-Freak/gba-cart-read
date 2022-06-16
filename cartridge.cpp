#include <array>
#include <cassert>
#include <cstring>

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
        assert((addr & 0xFFFF) + count <= 0x10000);

        // write high bits of address
        gpio_put_masked(0xFF << 16, addr >> 1);

        // count and low bits of address
        pio_sm_put_blocking(pio0, pioSM, count - 1);
        pio_sm_put_blocking(pio0, pioSM, (addr >> 1) & 0xFFFF);

        while(count--)
            *data++ = pio_sm_get_blocking(pio0, pioSM) >> 16;
    }

    void readRAMSave(uint16_t addr, uint8_t *data, int count)
    {
        // should also be good for 64k flash
        assert(addr + count <= 0x10000);

        // disable PIO, and manually set pins
        pio_sm_set_enabled(pio0, pioSM, false);

        auto addressMask = (1 << 16) - 1;

        pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << ramCSPin); // cs active
        sleep_us(1);
        
        // the high 8 bits from the ROM addr are the data pins here
        gpio_set_dir_in_masked(0xFF << 16);

        while(count--)
        {
            pio_sm_set_pins_with_mask(pio0, pioSM, addr, addressMask); // write address
            pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << rdPin); // rd
            sleep_us(1);

            *data++ = gpio_get_all() >> 16;

            pio_sm_set_pins_with_mask(pio0, pioSM, 1 << rdPin, 1 << rdPin);
            sleep_us(1);
            addr++;
        }

        pio_sm_set_pins_with_mask(pio0, pioSM, 1 << ramCSPin, 1 << ramCSPin); // cs inactive

        gpio_set_dir_out_masked(0xFF << 16);

        pio_sm_set_enabled(pio0, pioSM, true);
    }

    HeaderInfo readHeader()
    {
        HeaderInfo header = {};

        uint16_t buf[96];
        Cartridge::readROM(0, buf, std::size(buf));
        
        // checksum check
        auto headerBytes = reinterpret_cast<uint8_t *>(buf);
        int checksum = 0;
        for(int i = 0xA0; i < 0xBD; i++)
            checksum = checksum - headerBytes[i];

        checksum = (checksum - 0x19) & 0xFF;

        if(headerBytes[0xBD] != checksum)
            return header;

        header.checksumValid = true;

        memcpy(header.title, headerBytes + 0xA0, 12);
        memcpy(header.gameCode, headerBytes + 0xAC, 4);

        return header;
    }

    uint32_t getROMSize()
    {
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

        return size;
    }

    SaveType getSaveType(uint32_t romSize)
    {
        // search for save type marker
        // EEPROM, SRAM, FLASH, FLASH512, FLASH1M
        uint16_t eepromStart = 0x4545 /*EE*/, sramStart = 0x5253 /*SR*/, flashStart = 0x4C46 /*FL*/;

        uint16_t buf[6]{};

        // skip the header, should be word aligned
        for(uint32_t offset = 0xC0; offset < romSize; offset += 4)
        {
            Cartridge::readROM(offset, buf, 1);

            // first two bytes match, check the rest
            if(buf[0] == eepromStart || buf[0] == sramStart || buf[0] == flashStart)
            {
                Cartridge::readROM(offset, buf, 5);

                if(memcmp(buf, "EEPROM_V", 8) == 0)
                    return SaveType::Unknown; // TODO (could be 512bytes or 8K)

                if(memcmp(buf, "SRAM_V", 6) == 0)
                    return SaveType::RAM;

                if(memcmp(buf, "FLASH_V", 7) == 0 || memcmp(buf, "FLASH512_V", 10) == 0)
                    return SaveType::Flash_64K;

                if(memcmp(buf, "FLASH1M_V", 9) == 0)
                    return SaveType::Flash_128K;
            }
        }

        return SaveType::Unknown;
    }
}