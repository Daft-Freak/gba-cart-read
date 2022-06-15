#include <cstdarg>
#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "tusb.h"

#include "config.h"

// 24 bit addr on pins 0-23
// 16 bit data on pins 0-15

static const int wrPin = 24;
static const int rdPin = 25;
static const int romCSPin = 26;
static const int ramCSPin = 27;

void readROM(uint32_t addr, uint16_t *data, int count);

// "borrowed" debug code
void usbDebug(const char *message)
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

int debugf(const char * psFormatString, ...)
{
    va_list args;
    va_start(args, psFormatString);

    // get length
    va_list tmp_args;
    va_copy(tmp_args, args);
    int len = vsnprintf(nullptr, 0, psFormatString, tmp_args) + 1;
    va_end(tmp_args);

    auto buf = new char[len];
    int ret = vsnprintf(buf, len, psFormatString, args);
    usbDebug(buf);
    va_end(args);

    delete[] buf;
    return ret;
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
    *block_count = (32 * 1024 * 1024) / 512; // TODO
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

    auto addr = lba * 512 + offset;
    readROM(addr, (uint16_t *)buffer, bufsize / 2);

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

// read func
void readROM(uint32_t addr, uint16_t *data, int count)
{
    assert((addr & 1) == 0);
    assert((addr & 0xFFFF) + count <= 0xFFFF);

    auto addrMask = (1 << 24) - 1;
    auto dataMask = (1 << 16) - 1;

    // write address
    gpio_put_masked(addrMask, addr >> 1);

    const int delay = 1;
    sleep_us(delay);
    
    // cs active
    gpio_put(romCSPin, 0);
    sleep_us(delay);

    // set pins to input
    gpio_set_dir_in_masked(dataMask);

    while(count--)
    {
        // rd active
        gpio_put(rdPin, 0);
        sleep_us(delay);

        *data++ = gpio_get_all() & dataMask;

        gpio_put(rdPin, 1);
        sleep_us(delay);
    }

    gpio_put(romCSPin, 1);
    gpio_set_dir_out_masked(dataMask);
    sleep_us(delay);
}

int main()
{
    stdio_init_all();

    tusb_init();

    // init all IOs
    auto mask = (1 << 28) - 1;
    gpio_init_mask(mask);
    gpio_set_dir_out_masked(mask);

    // inactive
    gpio_put_all(1 << wrPin | 1 << rdPin | 1 << romCSPin | 1 << ramCSPin);

    while(true)
    {
        /*uint16_t buf[16];
        readROM(0, buf, 16);

        for(auto &hw : buf)
            debugf("%04X ", hw);

        debugf("\n");*/

        tud_task();
    }

    return 0;
}
