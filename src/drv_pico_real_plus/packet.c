// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Pico Real Plus Driver */

#include <stdio.h>
#include <string.h>
#include "picorp.h"

#ifdef _MSC_VER
#define inline __inline
#endif

inline static uint8_t read_uint8(const unsigned char** buffer)
{
    uint8_t ret = **buffer;
    *buffer += 1;
    return ret;
}

inline static int16_t read_int16(const uint8_t** buffer)
{
    int16_t ret = **buffer | (*(*buffer + 1) << 8);
    *buffer += 2;
    return ret;
}

inline static int32_t read_int24(const uint8_t** buffer)
{
    int32_t ret = **buffer | (*(*buffer + 1) << 8) | (*(*buffer + 2) << 16);
    if (ret & (0x80 << 16)){
        ret |= 0xff << 24;
    }
    *buffer += 3;
    return ret;
}

void picorp_decode_packet(picorp_raw_data_t* data, const unsigned char* buffer)
{
    ++buffer; // 0
    for (int i = 0; i<3; ++i){ // 1 ~ 9
        data->acceleration[i] = read_int24(&buffer);
    }

    for (int i = 0; i<3; ++i){ // 10 ~ 18
        data->gyroscope[i] = read_int24(&buffer);
    }

    data->temperature = read_int16(&buffer); // 19 ~ 20

    buffer += 26; // 21 ~ 46

    for (int i = 0; i<3; ++i){ // 47 ~ 55
        data->magnetometer[i] = read_int24(&buffer);
    }

    data->buttons = read_uint8(&buffer); // 56

    data->battery = read_uint8(&buffer); // 57
}
