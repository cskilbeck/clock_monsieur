//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

struct fcontrol_data_t
{
    uint16_t pad0[7];

    uint16_t lattmg0 : 1;
    uint16_t idmena : 1;
    uint16_t idmrpt : 1;
    uint16_t idmcur : 2;
    uint16_t oldena : 1;
    uint16_t psmode : 3;
    uint16_t : 7;

    uint8_t dsprpt : 1;
    uint8_t tmgrst : 1;
    uint8_t espwm : 1;
    uint8_t lodvlt : 2;
    uint8_t lsdvlt : 2;
    uint8_t lattmg1 : 1;

    uint8_t global_bc : 7;
    uint8_t blank : 1;

    uint8_t dc[14];

    void set_dc(int channel, uint8_t value);
};

static_assert(sizeof(fcontrol_data_t) == 256 / 8);

struct display_data_t
{
    fcontrol_data_t fcontrol;
    uint16_t grayscale_buffer[256];
};

//////////////////////////////////////////////////////////////////////

// start display_task on core 1, ambient light sensor task on core 0
void display_init();

// wait for display refresh, returns current backbuffer (256 pixels)
display_data_t *display_update();

// notify display task that we're done writing into backbuffer
void display_flip();
