#pragma once

#include <cstdint>

namespace Filesystem
{
    using ReadFunc = void(*)(uint32_t offset, uint32_t len, uint8_t *buf);

    void setTargetSize(uint32_t size);
    uint32_t getNumSectors();

    void addFile(uint32_t offset, uint32_t size, const char *shortName, const char *shortExt, ReadFunc readFn);
    void resetFiles();

    // TODO: multiple sectors
    void readSector(uint32_t sector, uint8_t *buf);
}