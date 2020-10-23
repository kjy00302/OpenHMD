// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Pico Real Plus Driver */

#ifndef PICORP_H
#define PICORP_H

#include "../openhmdi.h"

typedef enum {
    PICORP_BUTTON_HOME = 1,
    PICORP_BUTTON_BACK = 2,
    PICORP_BUTTON_SELECT = 4,
    PICORP_BUTTON_VOLUP = 8,
    PICORP_BUTTON_VOLDOWN = 16,
    PICORP_BUTTON_PROX = 32,
} picorp_button;

typedef struct {
    int32_t acceleration[3];
    int32_t gyroscope[3];
    int32_t magnetometer[3];
    int16_t temperature;
    uint8_t buttons;
    uint8_t battery;
} picorp_raw_data_t;

void picorp_decode_packet(picorp_raw_data_t* data, const unsigned char* buffer);

#endif
