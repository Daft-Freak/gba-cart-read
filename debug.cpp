#include <cstdarg>
#include <cstdio>

#include "debug.hpp"

#include "usb.hpp"

// "borrowed" debug code
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
    USB::writeCDC(buf);
    va_end(args);

    delete[] buf;
    return ret;
}
