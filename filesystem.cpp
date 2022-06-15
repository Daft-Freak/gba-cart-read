#include <algorithm>
#include <cstring>

#include "filesystem.hpp"

#include "cartridge.hpp"

struct [[gnu::packed]] VBR
{
    // BPB
    uint8_t jumpInstruction[3];
    char oemName[8];
    uint16_t bytesPerSector;
    uint8_t sectorsPerCluster;
    uint16_t numReservedSectors;
    uint8_t numFATs;
    uint16_t maxRootEntries;
    uint16_t totalSectors;
    uint8_t mediaDescriptor;
    uint16_t sectorsPerFAT;
    uint16_t sectorsPerTrack;
    uint16_t numHeads;
    uint32_t numHiddenSectors;
    uint32_t totalSectors32;

    // EBPB
    uint8_t driveNumber;
    uint8_t reserved;
    uint8_t extendedBootSig; // = 0x29
    uint32_t volumeId;
    char volumeLabel[11];
    char fsType[8];
};

static_assert(sizeof(VBR) == 62);

struct DirEntry
{
    char shortName[8];
    char shortExt[3];
    uint8_t attributes;
    uint8_t reserved;
    uint8_t createTimeFine;
    uint16_t createTime;
    uint16_t createDate;
    uint16_t accessDate;
    uint16_t eaIndex; // high bytes of cluster for FAT32
    uint16_t modifiedTime;
    uint16_t modifiedDate;
    uint16_t startCluster;
    uint32_t fileSize;
};

static_assert(sizeof(DirEntry) == 32);

static constexpr unsigned int nextPowerOf2(unsigned int i)
{
    i--;
    i |= i >> 1;
    i |= i >> 2;
    i |= i >> 4;
    i |= i >> 8;
    i |= i >> 16;
    i++;
    return i;
}

namespace Filesystem
{
    const int numReservedSectors = 1;
    const int maxRootEntries = 16;
    const int sectorSize = 512;

    static uint32_t targetSize = 1;
    static int sectorsPerCluster = 1, sectorsPerFAT = 1;
    uint32_t numSectors = 1;

    // this is how much data we want
    void setTargetSize(uint32_t size)
    {
        targetSize = size;

        const int maxClusters = 4085; // FAT12

        // worst-case FAT size
        const int largestFAT = 12; // 4096 clusters

        const int paddedSectors = (targetSize + (sectorSize - 1)) / sectorSize + numReservedSectors + (maxRootEntries * 32) / sectorSize + largestFAT;

        // calc cluster size
        sectorsPerCluster = nextPowerOf2((paddedSectors + maxClusters - 1) / maxClusters);

        const int numClusters = paddedSectors / sectorsPerCluster;

        // + 1 to round, + 2 for reserved
        sectorsPerFAT = (((numClusters + 3) / 2 * 3) + (sectorSize - 1)) / sectorSize;

        numSectors = paddedSectors - (largestFAT - sectorsPerFAT);
    }

    uint32_t getNumSectors()
    {
        return numSectors;
    }

    void readSector(uint32_t sector, uint8_t *buf)
    {
        const char *label = "DAFTVOLUME "; // 11 chars

        // offsets
        const uint32_t rootDirStart = numReservedSectors + sectorsPerFAT /* * numFATs*/;
        const uint32_t dataRegionStart = rootDirStart + maxRootEntries * 32 / sectorSize;

        if(sector == 0) // VBR
        {
            memset(buf, 0, sectorSize);
            auto vbr = reinterpret_cast<VBR *>(buf);

            vbr->jumpInstruction[0] = 0xEB;
            vbr->jumpInstruction[1] = 0x3C;
            vbr->jumpInstruction[2] = 0x90;

            memcpy(vbr->oemName, "DAFTFAT ", 8);

            vbr->bytesPerSector = sectorSize;
            vbr->sectorsPerCluster = sectorsPerCluster;
            vbr->numReservedSectors = numReservedSectors;
            vbr->numFATs = 1;
            vbr->maxRootEntries = maxRootEntries;

            if(numSectors > 0xFFFF)
            {
                vbr->totalSectors = 0;
                vbr->totalSectors32 = numSectors;
            }
            else
                vbr->totalSectors = uint16_t(numSectors);

            vbr->mediaDescriptor = 0xF8; // fixed disk
            vbr->sectorsPerFAT = sectorsPerFAT;

            vbr->driveNumber = 0x80; // first fixed disk
            vbr->extendedBootSig = 0x29;
            vbr->volumeId = 0x12345678;
            memcpy(vbr->volumeLabel, label, 11);
            memcpy(vbr->fsType, "FAT12   ", 8);
        }
        else if(sector >= numReservedSectors && sector < rootDirStart) // FAT
        {
            auto fatSector = sector - numReservedSectors;

            // reserved clusters
            memset(buf, 0, sectorSize);
            if(fatSector == 0)
            {
                buf[0] = 0xF8;
                buf[1] = 0xFF;
                buf[2] = 0xFF;
            }

            // chains for files
            uint32_t fileStart = 2;
            uint32_t fileLen = targetSize / (sectorSize * sectorsPerCluster);

            int startCluster = std::max(fileStart, fatSector * sectorSize / 3 * 2);
            int endCluster = std::min(fileStart + fileLen, ((fatSector + 1) * sectorSize + 2) / 3 * 2); // rounded up

            for(int cluster = startCluster; cluster < endCluster; cluster++)
            {
                int pairOff = cluster / 2 * 3 - fatSector * sectorSize;

                // next or EOF
                int val = cluster == fileStart + fileLen - 1 ? 0xFFF : cluster + 1;

                if(cluster & 1)
                {
                    // high bits
                    if(pairOff != -2) // low bits are in previous sector
                        buf[pairOff + 1] = (buf[pairOff + 1] & 0xF) | (val & 0xF) << 4;

                    if(pairOff != sectorSize - 2) // high bits are in next sector
                        buf[pairOff + 2] = val >> 4;
                }
                else
                {
                    // low bits
                    if(pairOff != -1) // low bits are in previous sector
                        buf[pairOff + 0] = val & 0xFF;

                    if(pairOff != sectorSize - 1) // high bits are in next sector
                        buf[pairOff + 1] = (buf[pairOff + 1] & 0xF0) | val >> 8;
                }
            }
        }
        else if(sector >= rootDirStart && sector < dataRegionStart) // root dir
        {
            memset(buf, 0, sectorSize);
            auto entries = reinterpret_cast<DirEntry *>(buf);

            memcpy(entries, label, 11); // name+ext of first entry
            entries[0].attributes = 0x8; // label

            // A file
            memcpy(entries[1].shortName, "ROM     ", 8);
            memcpy(entries[1].shortExt, "GBA", 3);
            entries[1].startCluster = 2;
            entries[1].fileSize = targetSize;
        }
        else if(sector >= dataRegionStart) // data region
        {
            int off = (sector - dataRegionStart) * sectorSize;
            Cartridge::readROM(off, (uint16_t *)buf, sectorSize / 2);
        }
    }
}