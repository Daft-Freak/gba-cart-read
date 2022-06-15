#pragma once

#include <cstdint>

namespace Filesystem
{
    // TODO: multiple sectors
    void readSector(uint32_t sector, uint8_t *buf);
}