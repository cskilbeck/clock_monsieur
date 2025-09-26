//////////////////////////////////////////////////////////////////////

#pragma once

//////////////////////////////////////////////////////////////////////

#define TLC5948_HOST SPI2_HOST       // Or SPI3_HOST
#define TLC5948_I2S_NUM I2S_NUM_0    // For GSCLK, use I2S_NUM_0 or I2S_NUM_1

//////////////////////////////////////////////////////////////////////

struct fcontrol_data_t
{
    union
    {
        struct
        {
            uint32_t blank : 1;
            uint32_t dsprpt : 1;
            uint32_t tmgrst : 1;
            uint32_t espwm : 1;
            uint32_t lodvlt : 2;
            uint32_t lsdvlt : 2;
            uint32_t lattmg : 2;
            uint32_t idmena : 1;
            uint32_t idmrpt : 1;
            uint32_t idmcur : 2;
            uint32_t oldena : 1;
            uint32_t psmode : 3;
        };
        uint32_t fcntrl_data;
    };
};

struct tlc5948_control_t
{
    uint16_t brightness[16];

    fcontrol_data_t fcntrl;

    uint8_t global_bc;

    uint8_t dc[16];
};

//////////////////////////////////////////////////////////////////////

extern tlc5948_control_t tlc5948_control;

esp_err_t tlc5948_init(void);
void tlc5948_set_grayscale();
void tlc5948_set_fcntrl();
