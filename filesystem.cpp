#include <algorithm>
#include <cassert>
#include <cstring>

#include "filesystem.hpp"

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

    static uint32_t targetSize = 0;
    static int sectorsPerCluster = 1, sectorsPerFAT = 1;
    static uint32_t numSectors = 0;

    static DirEntry rootEntries[maxRootEntries]{};
    static ReadFunc fileReadFuncs[maxRootEntries]{};
    static int curDirEntry = 1; // 0 is the label

    // this is how much data we want
    void setTargetSize(uint32_t size)
    {
        targetSize = size;

        if(!targetSize)
        {
            numSectors = 0;
            return;
        }

        const int maxClusters = 4085; // FAT12

        // worst-case FAT size
        const int largestFAT = 12; // 4096 clusters

        int targetSectors = (targetSize + (sectorSize - 1)) / sectorSize;
        int paddedSectors =  targetSectors + numReservedSectors + (maxRootEntries * 32) / sectorSize + largestFAT;

        // calc cluster size
        sectorsPerCluster = nextPowerOf2((paddedSectors + maxClusters - 1) / maxClusters);

        // round to cluster size
        paddedSectors += sectorsPerCluster - (targetSectors % sectorsPerCluster);

        const int numClusters = paddedSectors / sectorsPerCluster;

        // + 1 to round, + 2 for reserved
        sectorsPerFAT = (((numClusters + 3) / 2 * 3) + (sectorSize - 1)) / sectorSize;

        numSectors = paddedSectors - (largestFAT - sectorsPerFAT);
    }

    uint32_t getNumSectors()
    {
        return numSectors;
    }

    void addFile(uint32_t offset, uint32_t size, const char *shortName, const char *shortExt, ReadFunc readFn)
    {
        if(curDirEntry >= maxRootEntries)
            return;
        
        auto &entry = rootEntries[curDirEntry];
        fileReadFuncs[curDirEntry] = readFn;

        auto clusterSize = sectorsPerCluster * sectorSize;

        // space pad the name
        int len = strlen(shortName);
        memcpy(entry.shortName, shortName, std::max(8, len));
        for(int i = len; i < 8; i++)
            entry.shortName[i] = ' ';

        memcpy(entry.shortExt, shortExt, 3);

        entry.startCluster = offset / clusterSize + 2; // must be a multiple of cluster size
        entry.fileSize = size;

        curDirEntry++;
    }

    void resetFiles()
    {
        curDirEntry = 1;
        memset(rootEntries + 1, 0, sizeof(DirEntry) * (maxRootEntries - 1));
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

            auto startCluster = fatSector * sectorSize / 3 * 2;
            auto endCluster = ((fatSector + 1) * sectorSize + 2) / 3 * 2; // rounded up

            // reserved clusters
            memset(buf, 0, sectorSize);
            if(fatSector == 0)
            {
                buf[0] = 0xF8;
                buf[1] = 0xFF;
                buf[2] = 0xFF;
                startCluster = 2;
            }

            // chains for files
            uint32_t cluster = startCluster;

            auto clusterSize = sectorsPerCluster * sectorSize;

            while(cluster < endCluster)
            {
                uint32_t fileStart = 0;
                uint32_t fileLen = 0;

                // find file
                // assuming the files are in order...
                for(auto &entry : rootEntries)
                {
                    fileLen = (entry.fileSize + clusterSize - 1) / clusterSize;
                    if(cluster < entry.startCluster + fileLen)
                    {
                        fileStart = entry.startCluster;
                        break;
                    }
                }

                // didin't find a file
                if(!fileStart)
                    break;

                // clamping
                cluster = std::max(fileStart, cluster);
                auto fileEndCluster = std::min(fileStart + fileLen, endCluster);

                for(; cluster < fileEndCluster; cluster++)
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
        }
        else if(sector >= rootDirStart && sector < dataRegionStart) // root dir
        {
            // we're assuming that there is exactly one sector of dir entries
            assert(maxRootEntries * sizeof(DirEntry) == sectorSize);

            // make sure the label is there
            memcpy(rootEntries, label, 11); // name+ext of first entry
            rootEntries[0].attributes = 0x8; // label

            memcpy(buf, rootEntries, sectorSize);
        }
        else if(sector >= dataRegionStart) // data region
        {
            int off = (sector - dataRegionStart) * sectorSize;

            // find file
            int i = 0;
            for(auto &entry : rootEntries)
            {
                auto startByte = (entry.startCluster - 2) * sectorsPerCluster * sectorSize;
                if(off >= startByte && off - startByte < entry.fileSize)
                {
                    fileReadFuncs[i](off - startByte, sectorSize, buf);
                    break;
                }
                i++;
            }
        }
    }
}