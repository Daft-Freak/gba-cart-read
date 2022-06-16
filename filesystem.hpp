#pragma once

#include <cstdint>

namespace Filesystem
{
    void setTargetSize(uint32_t size);
    uint32_t getNumSectors();

    void addFile(uint32_t offset, uint32_t size, const char *shortName, const char *shortExt);
    void resetFiles();

    // TODO: multiple sectors
    void readSector(uint32_t sector, uint8_t *buf);
}