// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Pico Real Plus Driver */


#include <string.h>
#include <hidapi.h>
#include "../openhmdi.h"

#include "picorp.h"

typedef struct {
    ohmd_device base;
    hid_device* handle;
    fusion sensor_fusion;
    picorp_raw_data_t raw_data;
    uint32_t tick;
    vec3f accel,gyro,mag;
} picorp_priv;

#define TICK_LEN (1.0f / 500000.0f) // 500Hz ticks

static void handle_hmd_data_packet(picorp_priv* priv, unsigned char* buf)
{
    uint32_t last_sample_tick = priv->tick;
    uint32_t tick_delta = 500;
    if (last_sample_tick > 0)
        tick_delta = priv->tick - last_sample_tick;

    float dt = tick_delta * TICK_LEN;

    picorp_decode_packet(&priv->raw_data, buf);

    priv->accel.x = priv->raw_data.acceleration[0] * 0.0001f;
    priv->accel.y = priv->raw_data.acceleration[1] * 0.0001f;
    priv->accel.z = priv->raw_data.acceleration[2] * 0.0001f;

    priv->gyro.x = priv->raw_data.gyroscope[0] * 0.0001f * 3.1415925f / 180.0f;
    priv->gyro.y = priv->raw_data.gyroscope[1] * 0.0001f * 3.1415925f / 180.0f;
    priv->gyro.z = priv->raw_data.gyroscope[2] * 0.0001f * 3.1415925f / 180.0f;


    priv->mag.x = priv->raw_data.magnetometer[0] * 0.01f;
    priv->mag.y = priv->raw_data.magnetometer[1] * 0.01f;
    priv->mag.z = priv->raw_data.magnetometer[2] * 0.01f;

    ofusion_update(&priv->sensor_fusion, dt, &priv->gyro, &priv->accel, &priv->mag);
}

static void update_device(ohmd_device* device)
{
    picorp_priv* priv = (picorp_priv*)device;
    unsigned char buffer[64];

    int size = hid_read(priv->handle, buffer, 64);
    if (size < 0){
        LOGE("error reading from device");
        return;
    }


    if (buffer[0] == 0x01){
        handle_hmd_data_packet(priv, buffer);
    }
    else {
        LOGE("unknown message header: %u", buffer[0]);
    }
}

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
    picorp_priv* priv = (picorp_priv*)device;

    switch (type) {
        case OHMD_ROTATION_QUAT: {
            *(quatf*)out = priv->sensor_fusion.orient;
            break;
        }

        case OHMD_POSITION_VECTOR:
            out[0] = out[1] = out[2] = 0;
            break;

        case OHMD_DISTORTION_K:
            // TODO this should be set to the equivalent of no distortion
            memset(out, 0, sizeof(float) * 6);
            break;

        case OHMD_CONTROLS_STATE:
            out[0] = (priv->raw_data.buttons & PICORP_BUTTON_HOME) != 0;
            out[1] = (priv->raw_data.buttons & PICORP_BUTTON_BACK) != 0;
            out[2] = (priv->raw_data.buttons & PICORP_BUTTON_SELECT) != 0;
            out[3] = (priv->raw_data.buttons & PICORP_BUTTON_VOLUP) != 0;
            out[4] = (priv->raw_data.buttons & PICORP_BUTTON_VOLDOWN) != 0;
            break;

        default:
            ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
            return OHMD_S_INVALID_PARAMETER;
            break;
    }

    return OHMD_S_OK;
}

static void close_device(ohmd_device* device)
{
    LOGD("closing picorp device");
    picorp_priv* priv = (picorp_priv*)device;
    hid_close(priv->handle);
    free(device);
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
    picorp_priv* priv = ohmd_alloc(driver->ctx, sizeof(picorp_priv));
    if (!priv)
        return NULL;

    priv->base.ctx = driver->ctx;

    priv->handle = hid_open_path(desc->path);

    if (!priv->handle) {
        free(priv);
        return NULL;
    }

    ohmd_set_default_device_properties(&priv->base.properties);

    priv->base.properties.hsize = 0.119232f;
    priv->base.properties.vsize = 0.067068f;
    priv->base.properties.hres = 3840;
    priv->base.properties.vres = 2160;
    priv->base.properties.lens_sep = 0.062f;
    priv->base.properties.lens_vpos = 0.040326f;
    priv->base.properties.fov = DEG_TO_RAD(101.0f);
    priv->base.properties.ratio = (3840.0f / 2160.0f) / 2.0f;

    priv->base.properties.control_count = 5;
    priv->base.properties.controls_hints[0] = OHMD_HOME;
    priv->base.properties.controls_hints[1] = OHMD_BUTTON_B;
    priv->base.properties.controls_hints[2] = OHMD_BUTTON_A;
    priv->base.properties.controls_hints[3] = OHMD_VOLUME_PLUS;
    priv->base.properties.controls_hints[4] = OHMD_VOLUME_MINUS;
    priv->base.properties.controls_types[0] = OHMD_DIGITAL;
    priv->base.properties.controls_types[1] = OHMD_DIGITAL;
    priv->base.properties.controls_types[2] = OHMD_DIGITAL;
    priv->base.properties.controls_types[3] = OHMD_DIGITAL;
    priv->base.properties.controls_types[4] = OHMD_DIGITAL;

    ohmd_calc_default_proj_matrices(&priv->base.properties);

    //FIXME: find correct distortion
    /*
    double a = 0.0001474;
    double b = 0.7307;
    double c = 0.3883;

    ohmd_set_universal_distortion_k(&(priv->base.properties), a, b, c, 1-(a+b+c));
    ohmd_set_universal_aberration_k(&(priv->base.properties), 0.994f, 1.0f, 1.014f);
    */

    priv->base.update = update_device;
    priv->base.close = close_device;
    priv->base.getf = getf;

    ofusion_init(&priv->sensor_fusion);

    return (ohmd_device*)priv;
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
    struct hid_device_info* devs = hid_enumerate(0x2d40, 0x0012);
    struct hid_device_info* cur_dev = devs;

    // HMD
    while (cur_dev) {
        ohmd_device_desc* desc;
        desc = &list->devices[list->num_devices++];

        strcpy(desc->driver, "OpenHMD Pico Real Plus Driver");
        strcpy(desc->vendor, "Pico Interactive");
        strcpy(desc->product, "Pico Real Plus");

        desc->device_class = OHMD_DEVICE_CLASS_HMD;
        desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

        strcpy(desc->path, cur_dev->path);
        desc->driver_ptr = driver;

        cur_dev = cur_dev->next;
    }

    hid_free_enumeration(devs);
}

static void destroy_driver(ohmd_driver* drv)
{
    LOGD("shutting down picorp driver");
    free(drv);
}

ohmd_driver* ohmd_create_picorp_drv(ohmd_context *ctx)
{
    ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));
    if(!drv)
        return NULL;

    drv->get_device_list = get_device_list;
    drv->open_device = open_device;
    drv->destroy = destroy_driver;

    return drv;
}
