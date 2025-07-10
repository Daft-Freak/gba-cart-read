# GBA Cartridge Reader

Hardware and software for an RP2xxx-based Game Boy and Game Boy Advance cartridge reader that presents the contents as a USB storage device.

## Board
- `board` contains the "v1" PGA2040-based GBA-only board
- `board-v4` contains the new RP2350-based design that also supports Game Boy carts and can _theoretically_ be expanded. (to be added when I make the fixed version)
- (v2/3 designs were never made. For completeness v2 added the Game Boy support, and v3 was a custom RP2040 design)

## Case

- "v1": https://www.printables.com/model/972096-gba-cartridge-reader-case

## Software
For the old RP2040-based boards, just build like any other pico-sdk based project. For the RP2350-based board configure with `cmake -DPICO_BOARD=pico2 -DPICO_RP2350A=0 [...]`.

### Tested GBA carts
 - 8K EEPROM
 - 32k SRAM
 - 64k flash
 - 128k flash

### Tested GB/GBC carts (v4)
 - No MBC
 - MBC1 (<= 512k ROM / 8k RAM)
 - MBC2
 - MBC3
 - MBC5