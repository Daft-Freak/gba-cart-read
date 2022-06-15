#include <cstring>

#include "tusb.h"

#include "usb.hpp"

#include "config.h"
#include "filesystem.hpp"

namespace USB
{
    // CDC helper
    void writeCDC(const char *message)
    {
        if(!tud_cdc_connected())
            return;

        auto len = strlen(message);

        uint32_t done = tud_cdc_write(message, len);

        while(done < len)
        {
            tud_task();
            if(!tud_ready())
                break;

            done += tud_cdc_write((const char *)message + done, len - done);
        }
    }
}

// USB MSC glue
static bool storageEjected = false;

void tud_mount_cb()
{
    storageEjected = false;
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;

    const char vid[] = USB_VENDOR_STR;
    const char pid[] = USB_PRODUCT_STR " Storage";
    const char rev[] = "1.0";

    memcpy(vendor_id  , vid, strlen(vid));
    memcpy(product_id , pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    if(storageEjected)
    {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
        return false;
    }

    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void) lun;

    *block_size = 512;
    *block_count = Filesystem::getNumSectors();
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void) lun;
    (void) power_condition;

    if(load_eject)
    {
        if (start)
        {
        }
        else
            storageEjected = true;
    }

    return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void) lun;

    auto byteBuf = reinterpret_cast<uint8_t *>(buffer);

    auto bytesLeft = bufsize;

    while(bytesLeft > 0)
    {
        Filesystem::readSector(lba, byteBuf);
        bytesLeft -= 512;
        byteBuf += 512;
        lba++;
    }

    return bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void) lun;

    return -1;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
    void const* response = NULL;
    uint16_t resplen = 0;

    switch (scsi_cmd[0])
    {
        case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
            // Host is about to read/write etc ... better not to disconnect disk
            resplen = 0;
        break;

        default:
            // Set Sense = Invalid Command Operation
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

            // negative means error -> tinyusb could stall and/or response with failed status
            resplen = -1;
        break;
    }

    return resplen;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    return false;
}