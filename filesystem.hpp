#pragma once

#include <cstdint>

namespace Filesystem
{
    void setTargetSize(uint32_t size);
    uint32_t getNumSectors();

    // TODO: multiple sectors
    void readSector(uint32_t sector, uint8_t *buf);
}