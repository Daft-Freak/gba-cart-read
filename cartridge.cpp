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

    static int curFlashBank = -1;

    static void initPIO(PIO pio, uint sm, uint offset)
    {
        int basePin = 0;
        int addressPins = 24;
        int dataPins = 16;

        uint32_t sideMask = 1 << wrPin | 1 << rdPin | 1 << romCSPin ;
        uint32_t pinMask = sideMask | ((1 << dataPins) - 1);

        pio_sm_set_pins_with_mask(pio, sm, sideMask, pinMask); // set all control pins inactive
        pio_sm_set_pindirs_with_mask(pio, sm, pinMask, pinMask); // set all pins as output

        for(int i = 0; i < 16; i++)
            pio_gpio_init(pio, basePin + i);

        pio_gpio_init(pio, wrPin);
        pio_gpio_init(pio, rdPin);
        pio_gpio_init(pio, romCSPin);

        pio_sm_config c = gba_rom_read_program_get_default_config(offset);

        sm_config_set_in_shift(&c, false, true, 16);
        sm_config_set_out_shift(&c, false, true, 32);

        sm_config_set_in_pins(&c, basePin);
        sm_config_set_out_pins(&c, basePin, dataPins);
        sm_config_set_sideset_pins(&c, basePin + addressPins);

        // default wait states for cart ROM are 4/2, most games use 3/1
        // ... so we can get a few MHz
        sm_config_set_clkdiv_int_frac(&c, 35, 0);

        pio_sm_init(pio, sm, offset, &c);
    }

    void initIO()
    {
        // init PIO
        uint offset = pio_add_program(pio0, &gba_rom_read_program);
        pioSM = pio_claim_unused_sm(pio0, true);
        initPIO(pio0, pioSM, offset);

        // init high address bits (PIO doesn't control these)
        auto mask = 0xFF << 16;
        gpio_init_mask(mask);
        gpio_set_dir_out_masked(mask);

        // and RAM CS
        gpio_init(ramCSPin);
        gpio_set_dir(ramCSPin, true);
        gpio_put(ramCSPin, true);
    }

    // read func
    void readROM(uint32_t addr, uint16_t *data, int count)
    {
        assert((addr & 1) == 0);
        assert(((addr >> 1) & 0xFFFF) + count <= 0x10000);

        // write high bits of address
        gpio_put_masked(0xFF << 16, addr >> 1);

        // count and low bits of address
        pio_sm_put_blocking(pio0, pioSM, (count - 1) << 16 | ((addr >> 1) & 0xFFFF));
        pio_sm_put_blocking(pio0, pioSM, 0x0000FFFF); // masks to set input/output

        pio0->fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + pioSM); // clear stall flag
        pio_sm_set_enabled(pio0, pioSM, true);

        while(count--)
            *data++ = pio_sm_get_blocking(pio0, pioSM);

        // wait for stall
        while(!(pio0->fdebug & (1 << (PIO_FDEBUG_TXSTALL_LSB + pioSM))));

        pio_sm_set_enabled(pio0, pioSM, false);
    }

    void readRAMSave(uint16_t addr, uint8_t *data, int count)
    {
        // should also be good for 64k flash
        assert(addr + count <= 0x10000);

        // manually set pins
        auto addressMask = (1 << 16) - 1;

        gpio_put(ramCSPin, false); // cs active
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

        gpio_put(ramCSPin, true); // cs inactive

        gpio_set_dir_out_masked(0xFF << 16);
    }

    void writeRAMSave(uint16_t addr, const uint8_t *data, int count)
    {
        // also used for flash commands
        assert(addr + count <= 0x8000);

        // manually set pins
        auto addressMask = (1 << 16) - 1;

        gpio_put(ramCSPin, false); // cs active
        sleep_us(1);
        
        while(count--)
        {
            pio_sm_set_pins_with_mask(pio0, pioSM, addr, addressMask); // write address
            gpio_put_masked(0xFF << 16, *data++ << 16); // write data

            pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << wrPin); // wr
            sleep_us(1);
            pio_sm_set_pins_with_mask(pio0, pioSM, 1 << wrPin, 1 << wrPin);

            addr++;
        }

        gpio_put(ramCSPin, true); // cs inactive
    }

    // only needed for 128k saves
    void readFlashSave(uint32_t addr, uint8_t *data, int count)
    {
        // no crossing banks
        assert((addr & 0xFFFF) + count <= 0x10000);

        int bank = addr >> 16;

        if(bank != curFlashBank)
        {
            // switch
            uint8_t v = 0xAA;
            writeRAMSave(0x5555, &v, 1);
            v = 0x55;
            writeRAMSave(0x2AAA, &v, 1);
            v = 0xB0;
            writeRAMSave(0x5555, &v, 1);

            v = bank;
            writeRAMSave(0, &v, 1);

            curFlashBank = bank;
        }

        readRAMSave(addr & 0xFFFF, data, count);
    }

    void readEEPROMSave(uint16_t addr, uint64_t *data, int count, bool is8k)
    {
        assert((addr & 7) == 0);
        assert((addr / 8) + count <= (is8k ? 0x400 : 0x40));

        // manually set pins
        // (this could probably use PIO more)
        auto addressMask = (1 << 16) - 1;

        // write address (0x1FFFF00)
        gpio_put_masked(0xFF << 16, 0xFF << 16);
        pio_sm_set_pins_with_mask(pio0, pioSM, 0xFF80, addressMask);

        pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << romCSPin); // cs active
        
        sleep_us(1);

        // borrow the lowest bit of data
        gpio_init(0);
        gpio_set_dir(0, true);

        // writing a stream of bits to the ROM area
        auto writeBit = [](int b)
        {
            gpio_put(0, b);
            pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << wrPin); // WR active
            sleep_us(1);
            pio_sm_set_pins_with_mask(pio0, pioSM, 1 << wrPin, 1 << wrPin);
            sleep_us(1);
        };

        auto readBit = []()
        {
            pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << rdPin); // RD active
            sleep_us(1);
            int ret = gpio_get(0);
            pio_sm_set_pins_with_mask(pio0, pioSM, 1 << rdPin, 1 << rdPin);
            sleep_us(1);

            return ret;
        };

        while(count--)
        {
            writeBit(1);
            writeBit(1);

            int addrBits = is8k ? 14 : 6;
            uint16_t shiftedAddr = addr / 8;
            if(!is8k)
                shiftedAddr <<= 8;

            // write address, MSB first
            while(addrBits--)
            {
                writeBit((shiftedAddr >> 13) & 1);
                shiftedAddr <<= 1;
            }

            writeBit(0);

            // toggle cs
            pio_sm_set_pins_with_mask(pio0, pioSM, 1 << romCSPin, 1 << romCSPin);
            sleep_us(1);
            pio_sm_set_pins_with_mask(pio0, pioSM, 0, 1 << romCSPin);

            gpio_set_dir(0, false); // input
            
            // ignore first 4 bits
            readBit();
            readBit();
            readBit();
            readBit();

            // read 64 bits
            uint64_t val = 0;

            for(int i = 0; i < 64; i++)
                val = val << 1 | readBit();

            *data++ = __builtin_bswap64(val);

            gpio_set_dir(0, true); // output

            addr += 8;
        }

        pio_sm_set_pins_with_mask(pio0, pioSM, 1 << romCSPin, 1 << romCSPin); // cs inactive

        pio_gpio_init(pio0, 0);
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
                {
                    // if it's a 512 byte EEPROM and we address it as an 8K one, the 6 address bits it cares about will be 0
                    // ... so it should always return the same value. (I think, don't have one to test)

                    uint64_t val0, val;
                    readEEPROMSave(0, &val0, 1, true);

                    for(int eepromOff = 8; eepromOff < 512; eepromOff += 8)
                    {
                        readEEPROMSave(eepromOff, &val, 1, true);

                        if(val != val0)
                            return SaveType::EEPROM_8K;
                    }

                    return SaveType::EEPROM_512;
                }

                if(memcmp(buf, "SRAM_V", 6) == 0)
                    return SaveType::RAM;

                if(memcmp(buf, "FLASH_V", 7) == 0 || memcmp(buf, "FLASH512_V", 10) == 0)
                    return SaveType::Flash_64K;

                if(memcmp(buf, "FLASH1M_V", 9) == 0)
                {
                    curFlashBank = -1;
                    return SaveType::Flash_128K;
                }
            }
        }

        return SaveType::Unknown;
    }
}