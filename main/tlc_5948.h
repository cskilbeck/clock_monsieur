//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#define TLC5948_HOST SPI2_HOST
#define TLC5948_I2S_NUM I2S_NUM_0    // For GSCLK

//////////////////////////////////////////////////////////////////////

// Format of the control data

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
};

static_assert(sizeof(fcontrol_data_t) == 256 / 8);

struct tlc5948_control_t
{
    uint16_t brightness[16];
    fcontrol_data_t fcntrl;

    void reset();
    void set_dc(int channel, uint8_t value);
    void set_brightness(int channel, uint16_t value);
};

//////////////////////////////////////////////////////////////////////

extern tlc5948_control_t tlc5948_control;

void display_init();

// wait for 'vertical blank' type thing
void display_waitvb();

// call this when you have set the values in tlc5948_control.brightness
void display_set_grayscale();

// call this when you have set the other stuff in tlc5948_control
void display_set_fcntrl();
