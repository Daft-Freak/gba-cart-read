#include <array>
#include <cassert>
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "tusb.h"

#include "cartridge.hpp"
#include "gba-cart.pio.h"

// 24 bit addr on pins 0-23
// 16 bit data on pins 0-15

static const int wrPin = 24;
static const int rdPin = 25;
static const int romCSPin = 26;
static const int ramCSPin = 27;

#ifdef PICO_RP2350
#define DIR_OVERRIDE_PIN 29
#endif

namespace Cartridge
{
    static int pioSM = -1;
    static unsigned int romProgramOffset, eepromProgramOffset;

    static int dmaChannel;

    static int curFlashBank = -1;

    /*
    N4 S2 (default)
    CS -_______-_ ...
    RD ---__-__-- ...

    N3 S1
    CS -_____-_ ...
    RD ---_-_-- ...
    */
    static constexpr int romClkDiv = 4; // 125 / 4 / 2 = 15.625Mhz, program has delays for N3/S1
    static constexpr int eepromClkDiv = 32; // uses N8/S8, so go 8x slower

    // high address bits for GBA ROM or 8-bit data bus 
    static void highAddrData8Out()
    {
#ifdef DIR_OVERRIDE_PIN
        gpio_put(DIR_OVERRIDE_PIN, true);
#endif
        gpio_set_dir_out_masked(0xFF << 16);
    }

    static void highAddrData8In()
    {
        gpio_set_dir_in_masked(0xFF << 16);
#ifdef DIR_OVERRIDE_PIN
        gpio_put(DIR_OVERRIDE_PIN, false);
        sleep_ms(2); // ??
#endif
    }

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

        sm_config_set_clkdiv_int_frac(&c, romClkDiv, 0);

        pio_sm_init(pio, sm, offset, &c);
    }

    void initIO()
    {
        // init PIO
        romProgramOffset = pio_add_program(pio0, &gba_rom_read_program);
        eepromProgramOffset = pio_add_program(pio0, &gba_eeprom_read_program);
        pioSM = pio_claim_unused_sm(pio0, true);
        initPIO(pio0, pioSM, romProgramOffset);

#ifdef DIR_OVERRIDE_PIN
        // buffer direction control
        gpio_init(DIR_OVERRIDE_PIN);
        gpio_set_dir(DIR_OVERRIDE_PIN, true);
#endif

        // init high address bits (PIO doesn't control these)
        auto mask = 0xFF << 16;
        gpio_init_mask(mask);
        highAddrData8Out();

        // and RAM CS
        gpio_init(ramCSPin);
        gpio_set_dir(ramCSPin, true);
        gpio_put(ramCSPin, true);

        // DMA
        dmaChannel = dma_claim_unused_channel(true);

        auto config = dma_channel_get_default_config(dmaChannel);
        channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
        channel_config_set_dreq(&config, pio_get_dreq(pio0, pioSM, false));
        channel_config_set_read_increment(&config, false);
        channel_config_set_write_increment(&config, true);

        dma_channel_set_config(dmaChannel, &config, false);
        dma_channel_set_read_addr(dmaChannel, &pio0->rxf[pioSM], false);
    }

    // read func
    void readROM(uint32_t addr, volatile uint16_t *data, int count)
    {
        assert((addr & 1) == 0);
        assert(((addr >> 1) & 0xFFFF) + count <= 0x10000);

        // write high bits of address
        gpio_put_masked(0xFF << 16, addr >> 1);

        // switch program
        pio_sm_set_clkdiv_int_frac(pio0, pioSM, romClkDiv, 0);
        auto start = romProgramOffset + gba_rom_read_wrap_target;
        pio_sm_set_wrap(pio0, pioSM, start, romProgramOffset + gba_rom_read_wrap);
        pio_sm_exec(pio0, pioSM, pio_encode_jmp(start));

        // count and low bits of address
        pio_sm_put_blocking(pio0, pioSM, (count - 1) | ((addr >> 1) & 0xFFFF) << 16);
        pio_sm_put_blocking(pio0, pioSM, 0x0000FFFF); // masks to set input/output

        // setup DMA for read
        dma_hw->ch[dmaChannel].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_BSWAP_BITS; // no swapping
        dma_channel_set_trans_count(dmaChannel, count, false);
        dma_channel_set_write_addr(dmaChannel, data, true);

        pio0->fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + pioSM); // clear stall flag
        pio_sm_set_enabled(pio0, pioSM, true);

        while(dma_channel_is_busy(dmaChannel));// wait for DMA

        // wait for stall
        while(!(pio0->fdebug & (1 << (PIO_FDEBUG_TXSTALL_LSB + pioSM))));

        pio_sm_set_enabled(pio0, pioSM, false);
    }

    void readRAMSave(uint16_t addr, volatile uint8_t *data, int count)
    {
        // should also be good for 64k flash
        assert(addr + count <= 0x10000);

        // the high 8 bits from the ROM addr are the data pins here
        highAddrData8In();

        // manually set pins
        gpio_put(ramCSPin, false); // cs active
        sleep_us(1);

        while(count--)
        {
            pio_sm_put_blocking(pio0, pioSM, addr << 16);
            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_pins, 16) | pio_encode_sideset_opt(3, 0b101)); // out address, rd active
            
            sleep_us(1);

            *data++ = gpio_get_all() >> 16;

            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_null, 16) | pio_encode_sideset_opt(3, 0b111)); // discard remaining bits, rd inactive
            sleep_us(1);
            addr++;
        }

        gpio_put(ramCSPin, true); // cs inactive

        highAddrData8Out();
    }

    void writeRAMSave(uint16_t addr, volatile const uint8_t *data, int count)
    {
        // also used for flash commands
        assert(addr + count <= 0x8000);

        // manually set pins
        gpio_put(ramCSPin, false); // cs active
        sleep_us(1);
        
        while(count--)
        {
            gpio_put_masked(0xFF << 16, *data++ << 16); // write data

            pio_sm_put_blocking(pio0, pioSM, addr << 16);
            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_pins, 16) | pio_encode_sideset_opt(3, 0b110)); // out address, wr active

            sleep_us(1);
            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_null, 16) | pio_encode_sideset_opt(3, 0b111)); // discard remaining bits, wr inactive

            addr++;
        }

        gpio_put(ramCSPin, true); // cs inactive
    }

    // only needed for 128k saves
    void readFlashSave(uint32_t addr, volatile uint8_t *data, int count)
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

    void readEEPROMSave(uint16_t addr, volatile uint64_t *data, int count, bool is8k)
    {
        assert((addr & 7) == 0);
        assert((addr / 8) + count <= (is8k ? 0x400 : 0x40));

        uint32_t romAddr = 0x1FFFF00;

        // write high bits of address (0x1FFFF00)
        gpio_put_masked(0xFF << 16, romAddr >> 1);

        // switch program
        pio_sm_set_clkdiv_int_frac(pio0, pioSM, eepromClkDiv, 0);
        auto start = eepromProgramOffset + gba_eeprom_read_wrap_target;
        pio_sm_set_wrap(pio0, pioSM, start, eepromProgramOffset + gba_eeprom_read_wrap);
        pio_sm_exec(pio0, pioSM, pio_encode_jmp(start));

        // counts and low bits of (rom) address
        const int addrBits = is8k ? 14 : 6;
        const int dataBits = 64;
        uint32_t addrData = (addrBits + 2) << 8 /*extra 3 bits*/ | (dataBits - 1) | ((romAddr >> 1) & 0xFFFF) << 16;

        pio_sm_put_blocking(pio0, pioSM, addrData);
        // the EEPROM address
        pio_sm_put_blocking(pio0, pioSM, 0xC0000000 /*11 prefix*/ | (addr / 8) << (is8k ? 16 : 24)); // either 9 or 17 bits total...
        // masks to set input/output
        pio_sm_put_blocking(pio0, pioSM, 0x0000FFFF);

        // setup DMA for read
        dma_hw->ch[dmaChannel].al1_ctrl |= DMA_CH0_CTRL_TRIG_BSWAP_BITS; // swap bytes
        dma_channel_set_trans_count(dmaChannel, count * 4 /*64 bit*/, false);
        dma_channel_set_write_addr(dmaChannel, data, true);

        pio0->fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + pioSM); // clear stall flag
        pio_sm_set_enabled(pio0, pioSM, true);

        // keep pushing reads
        for(int i = 1; i < count; i++)
        {
            addr += 8;
            pio_sm_put_blocking(pio0, pioSM, addrData);
            pio_sm_put_blocking(pio0, pioSM, 0xC0000000 | (addr / 8) << (is8k ? 16 : 24));
            pio_sm_put_blocking(pio0, pioSM, 0x0000FFFF);
        }

        while(dma_channel_is_busy(dmaChannel)); // wait for DMA

        // wait for stall
        while(!(pio0->fdebug & (1 << (PIO_FDEBUG_TXSTALL_LSB + pioSM))));

        pio_sm_set_enabled(pio0, pioSM, false);
    }

    void readDMG(uint16_t addr, volatile uint8_t *data, int count)
    {
        // this is basically the GBA RAM code using the other CS
        assert(addr + count <= 0x10000);

        // data pins
        highAddrData8In();

        while(count--)
        {
            pio_sm_put_blocking(pio0, pioSM, addr << 16);
            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_pins, 16) | pio_encode_sideset_opt(3, 0b001)); // out address, rd+cs active
            
            sleep_us(1);

            *data++ = gpio_get_all() >> 16;

            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_null, 16) | pio_encode_sideset_opt(3, 0b011)); // discard remaining bits, rd inactive
            sleep_us(1);
            addr++;
        }

        pio_sm_exec(pio0, pioSM, pio_encode_nop() | pio_encode_sideset_opt(3, 0b111)); // cs inactive

        highAddrData8Out();
    }

    void writeDMG(uint16_t addr, volatile const uint8_t *data, int count)
    {
        while(count--)
        {
            gpio_put_masked(0xFF << 16, *data++ << 16); // write data

            pio_sm_put_blocking(pio0, pioSM, addr << 16);
            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_pins, 16) | pio_encode_sideset_opt(3, 0b010)); // out address, wr+cs active

            sleep_us(1);
            pio_sm_exec(pio0, pioSM, pio_encode_out(pio_null, 16) | pio_encode_sideset_opt(3, 0b011)); // discard remaining bits, wr inactive

            addr++;
        }

        pio_sm_exec(pio0, pioSM, pio_encode_nop() | pio_encode_sideset_opt(3, 0b111)); // cs inactive
    }

    void readMBC1ROM(uint32_t addr, volatile uint8_t *data, int count)
    {
        if(addr < 0x4000)
        {
            assert(addr + count < 0x4000);
            readDMG(addr, data, count);
        }
        else
        {
            int bank = addr / 0x4000;
            // switch bank
            uint8_t v = bank & 0x1F;
            writeDMG(0x2000, &v, 1);
            // TODO: high 2 bits to 4000 if > 512k
            readDMG(0x4000 + (addr & 0x3FFF), data, count);
        }
    }

    GBAHeaderInfo readGBAHeader()
    {
        GBAHeaderInfo header = {};

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

    DMGHeaderInfo readDMGHeader()
    {
        DMGHeaderInfo header = {};

        uint8_t dmgHeader[0x50];
        Cartridge::readDMG(0x100, dmgHeader, 0x50);

        // logo check (only the first 16 bytes)
        static const uint8_t logoData[]{0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D};

        if(memcmp(dmgHeader + 4, logoData, sizeof(logoData)) != 0)
            return header;
        
        // checksum check
        int checksum = 0;
        for(int i = 0x34; i < 0x4D; i++)
            checksum = checksum - dmgHeader[i] - 1;

        checksum = checksum & 0xFF;

        if(dmgHeader[0x4D] != checksum)
            return header;

        header.checksumValid = true;

        header.cartType = dmgHeader[0x47];
        header.romSize = dmgHeader[0x48];
        header.ramSize = dmgHeader[0x49];

        memcpy(header.title, dmgHeader + 0x34, 16);

        // clear cgb flag
        if(header.title[15] & 0x80)
            header.title[15] = 0;

        return header;
    }

    uint32_t getGBAROMSize()
    {
        // check for incrementing values (floating bus)
        // ... also compare single halfword reads vs one continuous read
        uint32_t size = 1 << 22; // start at 4MB

        for(; size < 1 << 25; size <<= 1)
        {
            // read as a single block
            uint16_t blockVal[16];
            Cartridge::readROM(size, blockVal, 16);

            // checking first 16 halfwords
            bool has_data = false;
            for(int i = 0; i < 16; i++)
            {
                // read individually
                uint16_t val;
                Cartridge::readROM(size + i * 2, &val, 1);

                if(val != i)
                    has_data = true;

                // data doesn't match, stop
                if(val != blockVal[i])
                {
                    has_data = false;
                    break;
                }
            }

            // found end of ROM, stop
            if(!has_data)
                break;
        }

        return size;
    }

    uint32_t getDMGROMSize(const DMGHeaderInfo &header)
    {
        if(header.romSize <= 8)
            return (32 * 1024) << header.romSize;

        return 0;
    }

    uint32_t getDMGRAMSize(const DMGHeaderInfo &header)
    {
        // TODO: MBC2 has internal RAM

        switch(header.ramSize)
        {
            case 0:
            default:
                return 0;
            case 2:
                return 8 * 1024;
            case 3:
                return 32 * 1024;
            case 4:
                return 128 * 1024;
            case 5:
                return 64 * 1024;
        }
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
            // this can take too long and break usb
            // FIXME: bit of a hack
            if((offset & 0xFFFFF) == 0)
                tud_task();

            Cartridge::readROM(offset, buf, 1);
            __compiler_memory_barrier(); // GCC 11+ really wants to optimise most of this function out...

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
                        __compiler_memory_barrier();

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

    MBCType getMBCType(const DMGHeaderInfo &header)
    {
        switch(header.cartType)
        {
            case 0:
                return MBCType::None;
            case 1:
            case 2: // + RAM
            case 3: // + RAM + Battery
                return MBCType::MBC1;

            case 5:
            case 6: // + Battery
                return MBCType::MBC2;

            case 0x0F: // + Timer + Battery
            case 0x10: // + Timer + RAM + Battery
            case 0x11:
            case 0x12: // + RAM
            case 0x13: // + RAM + Battery
                return MBCType::MBC3;

            case 0x19:
            case 0x1A: // + RAM
            case 0x1B: // + RAM + Battery
            case 0x1C: // + Rumble
            case 0x1D: // + Rumble + RAM
            case 0x1E: // + Rumble + RAM + Battery 
                return  MBCType::MBC5;

            default:
                return MBCType::None;
        }
    }
}