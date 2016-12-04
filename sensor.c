/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Sensor abstraction layer.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "sccb.h"
#include "wiring.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/io_mux_reg.h"
#include "sensor.h"
#include "ov9650.h"
#include "ov2640.h"
#include "ov7725.h"
#include "rom/lldesc.h"
#include "esp_intr.h"
#include "camera.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"
#include "framebuffer.h"

#define REG_PID        0x0A
#define REG_VER        0x0B

#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

#define MAX_XFER_SIZE (0xFFFC)

static const char* TAG = "camera";

sensor_t sensor;

static camera_config_t s_config;
static lldesc_t s_dma_desc[2];
static uint32_t* s_dma_buf[2];
static uint8_t* s_fb;
static sensor_t s_sensor;
static bool s_initialized = false;
static int s_fb_w;
static int s_fb_h;
static size_t s_fb_size;

static volatile int isr_count = 0;
static volatile int line_count = 0;
static volatile int cur_buffer = 0;
static int buf_line_width;
static int buf_height;
static volatile bool i2s_running = 0;
static SemaphoreHandle_t data_ready;
static SemaphoreHandle_t frame_ready;


const int resolution[][2] = {
    {40,    30 },    /* 40x30 */
    {64,    32 },    /* 64x32 */
    {64,    64 },    /* 64x64 */
    {88,    72 },    /* QQCIF */
    {160,   120},    /* QQVGA */
    {128,   160},    /* QQVGA2*/
    {176,   144},    /* QCIF  */
    {240,   160},    /* HQVGA */
    {320,   240},    /* QVGA  */
    {352,   288},    /* CIF   */
    {640,   480},    /* VGA   */
    {800,   600},    /* SVGA  */
    {1280,  1024},   /* SXGA  */
    {1600,  1200},   /* UXGA  */
};

static void i2s_init();
static void i2s_run(size_t line_width, int height);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init(int line_width);
static void line_filter_task(void *pvParameters);

static int extclk_config(int frequency, int pin)
{
    //Enable LEDC 
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;                 //Timer configuration
    timer_conf.bit_num = 3;                         //Timer duty depth (=11 bit?)
    timer_conf.freq_hz = frequency;                 //Timer frequency
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;   //Timer speed mode
    timer_conf.timer_num = 0;                       //Timer number (0-3)
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return -1;
    }

    //Channel configuration
    ledc_channel_config_t ch_conf;
    ch_conf.channel = 0;                        //Channel 
    ch_conf.timer_sel = 0;                      //Timer source (0-3) (See timer configuration)
    ch_conf.intr_type = LEDC_INTR_DISABLE;      //Deactivate interrupt on channel
    ch_conf.duty = 4;                           //Duty cycle (0 to (2*bit_num)-1)
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;  //Channel speed mode
    ch_conf.gpio_num = pin;                     //Pin number
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
        return -1;
    }

    return 0;
}

#define ETS_I2S0_INUM 13 //https://github.com/espressif/esp-idf/blob/master/components/esp32/include/soc/soc.h#L259-L294

static void i2s_init()
{
    xt_set_interrupt_handler(ETS_I2S0_INUM, &i2s_isr, NULL);
    intr_matrix_set(0, ETS_I2S0_INTR_SOURCE, ETS_I2S0_INUM);

    gpio_num_t pins[] = {
            s_config.pin_d7,
            s_config.pin_d6,
            s_config.pin_d5,
            s_config.pin_d4,
            s_config.pin_d3,
            s_config.pin_d2,
            s_config.pin_d1,
            s_config.pin_d0,
            s_config.pin_vsync,
            s_config.pin_href,
            s_config.pin_pclk
    };

    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for (int i = 0; i < sizeof(pins)/sizeof(gpio_num_t); ++i) {
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    gpio_matrix_in(s_config.pin_d0,    I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(s_config.pin_d1,    I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(s_config.pin_d2,    I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(s_config.pin_d3,    I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(s_config.pin_d4,    I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(s_config.pin_d5,    I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(s_config.pin_d6,    I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(s_config.pin_d7,    I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(s_config.pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(s_config.pin_href,  I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(s_config.pin_pclk,  I2S0I_WS_IN_IDX, false);

    periph_module_enable(PERIPH_I2S0_MODULE);

    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_IN_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_AHBM_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 1, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_LC_CONF_REG(0), 0x1, 0, I2S_AHBM_FIFO_RST_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_FIFO_RESET_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_FIFO_RESET_S);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_SLAVE_MOD_S);
    SET_PERI_REG_BITS(I2S_CONF2_REG(0), 0x1, 1, I2S_LCD_EN_S);
    SET_PERI_REG_BITS(I2S_CONF2_REG(0), 0x1, 1, I2S_CAMERA_EN_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A, 1, I2S_CLKM_DIV_A_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B, 0, I2S_CLKM_DIV_B_S);
    SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM, 2, I2S_CLKM_DIV_NUM_S);

    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), 0x1, 1, I2S_DSCR_EN_S);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_RIGHT_FIRST_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_MSB_RIGHT_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_MSB_SHIFT_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_MONO_S);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_SHORT_SYNC_S);
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), I2S_RX_FIFO_MOD, 1, I2S_RX_FIFO_MOD_S);
    SET_PERI_REG_BITS(I2S_FIFO_CONF_REG(0), 0x1, 1, I2S_RX_FIFO_MOD_FORCE_EN_S);
    SET_PERI_REG_BITS(I2S_CONF_CHAN_REG(0), I2S_RX_CHAN_MOD, 1, I2S_RX_CHAN_MOD_S);
    SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BITS_MOD, 16, I2S_RX_BITS_MOD_S);

}

static void i2s_fill_buf(int index) {
    ESP_INTR_DISABLE(ETS_I2S0_INUM);

    SET_PERI_REG_BITS(I2S_RXEOF_NUM_REG(0), I2S_RX_EOF_NUM, (buf_line_width - 2) * 2, I2S_RX_EOF_NUM_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), I2S_INLINK_ADDR, ((uint32_t) &s_dma_desc), I2S_INLINK_ADDR_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), 0x1, 1, I2S_INLINK_START_S);

    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);

    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), (REG_READ(I2S_CONF_REG(0)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    while (GET_PERI_REG_BITS2(I2S_STATE_REG(0), 0x1, I2S_TX_FIFO_RESET_BACK_S));

    SET_PERI_REG_BITS(I2S_INT_ENA_REG(0), 0x1, 1, I2S_IN_DONE_INT_ENA_S);
    ESP_INTR_ENABLE(ETS_I2S0_INUM);
    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 1, I2S_RX_START_S);
}

static void i2s_stop() {
    ESP_INTR_DISABLE(ETS_I2S0_INUM);

    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), (REG_READ(I2S_CONF_REG(0)) & 0xfffffff0) | 0xf);
    (void) REG_READ(I2S_CONF_REG(0));
    REG_WRITE(I2S_CONF_REG(0), REG_READ(I2S_CONF_REG(0)) & 0xfffffff0);

    SET_PERI_REG_BITS(I2S_CONF_REG(0), 0x1, 0, I2S_RX_START_S);
    i2s_running = false;
}

static void i2s_run(size_t line_width, int height)
{
    buf_line_width = line_width;
    buf_height = height;

    // wait for vsync
    ESP_LOGD(TAG, "Waiting for VSYNC");
    while(gpio_get_level(s_config.pin_vsync) != 0);
    while(gpio_get_level(s_config.pin_vsync) == 0);
    ESP_LOGD(TAG, "Got VSYNC");
    // wait a bit
    delay(2);

    // start RX
    cur_buffer = 0;
    line_count = 0;
    isr_count = 0;
    i2s_running = true;
    i2s_fill_buf(cur_buffer);
}

void sensor_init0()
{
    // Init FB mutex
    mutex_init(&JPEG_FB()->lock);

    // Save fb_enabled flag state
    int fb_enabled = JPEG_FB()->enabled;

    // Clear framebuffers
    memset(MAIN_FB(), 0, sizeof(*MAIN_FB()));
    memset(JPEG_FB(), 0, sizeof(*JPEG_FB()));

    // Set default quality
    JPEG_FB()->quality = 35;

    // Set fb_enabled
    JPEG_FB()->enabled = fb_enabled;
}

int sensor_init()
{
    /* Do a power cycle */
    DCMI_PWDN_HIGH();
    systick_sleep(10);

    DCMI_PWDN_LOW();
    systick_sleep(10);

    /* Initialize the SCCB interface */
    SCCB_Init();
    systick_sleep(10);

    // Configure the sensor external clock (XCLK) to XCLK_FREQ.
    //
    // Max pixclk is 2.5 * HCLK:
    //  STM32F427@180MHz PCLK = 71.9999MHz
    //  STM32F769@216MHz PCLK = 86.4000MHz
    //
    // OV2640:
    //  The sensor's internal PLL (when CLKRC=0x80) doubles the XCLK_FREQ
    //  (XCLK=XCLK_FREQ*2), and the unscaled PIXCLK output is XCLK_FREQ*4
    //
    // OV7725 PCLK when prescalar is enabled (CLKRC[6]=0):
    //  Internal clock = Input clock × PLL multiplier / [(CLKRC[5:0] + 1) × 2]
    //
    // OV7725 PCLK when prescalar is disabled (CLKRC[6]=1):
    //  Internal clock = Input clock × PLL multiplier
    //

    // Configure external clock timer.
    if (extclk_config(OMV_XCLK_FREQUENCY) != 0) {
        // Timer problem
        return -1;
    }

    /* Reset the sesnor state */
    memset(&sensor, 0, sizeof(sensor_t));

    /* Some sensors have different reset polarities, and we can't know which sensor
       is connected before initializing SCCB and probing the sensor, which in turn
       requires pulling the sensor out of the reset state. So we try to probe the
       sensor with both polarities to determine line state. */
    sensor.reset_pol = ACTIVE_HIGH;

    /* Reset the sensor */
    DCMI_RESET_HIGH();
    systick_sleep(10);

    DCMI_RESET_LOW();
    systick_sleep(10);

    /* Probe the sensor */
    sensor.slv_addr = SCCB_Probe();
    if (sensor.slv_addr == 0) {
        /* Sensor has been held in reset,
           so the reset line is active low */
        sensor.reset_pol = ACTIVE_LOW;

        /* Pull the sensor out of the reset state */
        DCMI_RESET_HIGH();
        systick_sleep(10);

        /* Probe again to set the slave addr */
        sensor.slv_addr = SCCB_Probe();
        if (sensor.slv_addr == 0)  {
            // Probe failed
            return -2;
        }
    }

    /* Read the sensor information */
    sensor.id.PID  = SCCB_Read(sensor.slv_addr, REG_PID);
    sensor.id.VER  = SCCB_Read(sensor.slv_addr, REG_VER);
    sensor.id.MIDL = SCCB_Read(sensor.slv_addr, REG_MIDL);
    sensor.id.MIDH = SCCB_Read(sensor.slv_addr, REG_MIDH);

    /* Call the sensor-specific init function */
    switch (sensor.id.PID) {
        case OV9650_PID:
            ov9650_init(&sensor);
            break;
        case OV2640_PID:
            ov2640_init(&sensor);
            break;
        case OV7725_PID:
            ov7725_init(&sensor);
            break;
        default:
            /* Sensor not supported */
            return -3;
    }

    /* Configure the DCMI DMA Stream */
    if (dma_config() != 0) {
        // DMA problem
        return -4;
    }

    /* Configure the DCMI interface. This should be called
       after ovxxx_init to set VSYNC/HSYNC/PCLK polarities */
    if (dcmi_config(DCMI_JPEG_DISABLE) != 0){
        // DCMI config failed
        return -5;
    }

    /* All good! */
    return 0;
}

int sensor_reset()
{
    // Reset the sesnor state
    sensor.sde = 0xFF;
    sensor.pixformat=0xFF;
    sensor.framesize=0xFF;
    sensor.framerate=0xFF;
    sensor.gainceiling=0xFF;

    // Reset image filter
    sensor_set_line_filter(NULL, NULL);

    // Call sensor-specific reset function
    sensor.reset(&sensor);

    // Just in case there's a running DMA request.
    HAL_DMA_Abort(&DMAHandle);
    return 0;
}

int sensor_get_id()
{
    return sensor.id.PID;
}

int sensor_read_reg(uint8_t reg)
{
    return SCCB_Read(sensor.slv_addr, reg);
}

int sensor_write_reg(uint8_t reg, uint8_t val)
{
    return SCCB_Write(sensor.slv_addr, reg, val);
}

int sensor_set_pixformat(pixformat_t pixformat)
{
    uint32_t jpeg_mode = DCMI_JPEG_DISABLE;

   if (sensor.pixformat == pixformat) {
        // No change
        return 0;
    }

    if (sensor.set_pixformat == NULL
        || sensor.set_pixformat(&sensor, pixformat) != 0) {
        // Operation not supported
        return -1;
    }

    // Set pixel format
    sensor.pixformat = pixformat;

    // Set JPEG mode
    if (pixformat == PIXFORMAT_JPEG) {
        jpeg_mode = DCMI_JPEG_ENABLE;
    }

    // Skip the first frame.
    fb->bpp = 0;

    return dcmi_config(jpeg_mode);
}

int sensor_set_framesize(framesize_t framesize)
{
    if (sensor.framesize == framesize) {
        // No change
        return 0;
    }

    // Call the sensor specific function
    if (sensor.set_framesize == NULL
        || sensor.set_framesize(&sensor, framesize) != 0) {
        // Operation not supported
        return -1;
    }

    // Set framebuffer size
    sensor.framesize = framesize;

    // Skip the first frame.
    fb->bpp = 0;

    if (framesize > OMV_MAX_RAW_FRAME) {
        // Crop higher resolutions to QVGA
        sensor_set_windowing(190, 120, 320, 240);
    } else {
        fb->w = resolution[framesize][0];
        fb->h = resolution[framesize][1];
        HAL_DCMI_DisableCROP(&DCMIHandle);
    }

    return 0;
}

int sensor_set_framerate(framerate_t framerate)
{
    if (sensor.framerate == framerate) {
       /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (sensor.set_framerate == NULL
        || sensor.set_framerate(&sensor, framerate) != 0) {
        /* operation not supported */
        return -1;
    }

    /* set the frame rate */
    sensor.framerate = framerate;

    return 0;
}

int sensor_set_windowing(int x, int y, int w, int h)
{
    fb->w = w;
    fb->h = h;
    HAL_DCMI_ConfigCROP(&DCMIHandle, x*2, y, w*2-1, h-1);
    HAL_DCMI_EnableCROP(&DCMIHandle);
    return 0;
}

int sensor_set_contrast(int level)
{
    if (sensor.set_contrast != NULL) {
        return sensor.set_contrast(&sensor, level);
    }
    return -1;
}

int sensor_set_brightness(int level)
{
    if (sensor.set_brightness != NULL) {
        return sensor.set_brightness(&sensor, level);
    }
    return -1;
}

int sensor_set_saturation(int level)
{
    if (sensor.set_saturation != NULL) {
        return sensor.set_saturation(&sensor, level);
    }
    return -1;
}

int sensor_set_gainceiling(gainceiling_t gainceiling)
{
    if (sensor.gainceiling == gainceiling) {
        /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (sensor.set_gainceiling == NULL
        || sensor.set_gainceiling(&sensor, gainceiling) != 0) {
        /* operation not supported */
        return -1;
    }

    sensor.gainceiling = gainceiling;
    return 0;
}

int sensor_set_quality(int qs)
{
    /* call the sensor specific function */
    if (sensor.set_quality == NULL
        || sensor.set_quality(&sensor, qs) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_colorbar(int enable)
{
    /* call the sensor specific function */
    if (sensor.set_colorbar == NULL
        || sensor.set_colorbar(&sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_whitebal(int enable)
{
    /* call the sensor specific function */
    if (sensor.set_whitebal == NULL
        || sensor.set_whitebal(&sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_gain_ctrl(int enable)
{
    /* call the sensor specific function */
    if (sensor.set_gain_ctrl == NULL
        || sensor.set_gain_ctrl(&sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_exposure_ctrl(int enable)
{
    /* call the sensor specific function */
    if (sensor.set_exposure_ctrl == NULL
        || sensor.set_exposure_ctrl(&sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_hmirror(int enable)
{
    /* call the sensor specific function */
    if (sensor.set_hmirror == NULL
        || sensor.set_hmirror(&sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_vflip(int enable)
{
    /* call the sensor specific function */
    if (sensor.set_vflip == NULL
        || sensor.set_vflip(&sensor, enable) != 0) {
        /* operation not supported */
        return -1;
    }
    return 0;
}

int sensor_set_special_effect(sde_t sde)
{
    if (sensor.sde == sde) {
        /* no change */
        return 0;
    }

    /* call the sensor specific function */
    if (sensor.set_special_effect == NULL
        || sensor.set_special_effect(&sensor, sde) != 0) {
        /* operation not supported */
        return -1;
    }

    sensor.sde = sde;
    return 0;
}

int sensor_set_line_filter(line_filter_t line_filter_func, void *line_filter_args)
{
    // Set line pre-processing function and args
    sensor.line_filter_func = line_filter_func;
    sensor.line_filter_args = line_filter_args;
    return 0;
}

// This function is called back after each line transfer is complete,
// with a pointer to the line buffer that was used. At this point the
// DMA transfers the next line to the other half of the line buffer.
// Note:  For JPEG this function is called once (and ignored) at the end of the transfer.
void DCMI_DMAConvCpltUser(uint32_t addr)
{
    uint8_t *src = (uint8_t*) addr;
    uint8_t *dst = fb->pixels;

    if (sensor.line_filter_func && sensor.line_filter_args) {
        int bpp = ((sensor.pixformat == PIXFORMAT_GRAYSCALE) ? 1:2);
        dst += line++ * fb->w * bpp;
        // If there's an image filter installed call it.
        // Note: BPP is the target BPP, not the line bpp (the line is always 2 bytes per pixel) if the target BPP is 1
        // it means the image currently being read is going to be Grayscale, and the function needs to output w * 1BPP.
        sensor.line_filter_func(src, fb->w * 2 , dst, fb->w * bpp, sensor.line_filter_args);
    } else {
        // Else just process the line normally.
        if (sensor.pixformat == PIXFORMAT_GRAYSCALE) {
            dst += line++ * fb->w;
            // If GRAYSCALE extract Y channel from YUV
            for (int i=0; i<fb->w; i++) {
                dst[i] = src[i<<1];
            }
        } else if (sensor.pixformat == PIXFORMAT_RGB565) {
            dst += line++ * fb->w * 2;
            for (int i=0; i<fb->w * 2; i++) {
                dst[i] = src[i];
            }
        }
    }
}

int sensor_snapshot(image_t *image, line_filter_t line_filter_func, void *line_filter_args)
{
    static int overflow_count = 0;
    uint32_t addr, length, tick_start;

    // Set line filter
    sensor_set_line_filter(line_filter_func, line_filter_args);

    // Setup the size and address of the transfer
    if (sensor.pixformat == PIXFORMAT_JPEG) {
        // Sensor has hardware JPEG set max frame size.
        length = MAX_XFER_SIZE;
        addr = (uint32_t) (fb->pixels);
    } else {
        // No hardware JPEG, set w*h*2 bytes per pixel.
        length =(fb->w * fb->h * 2)/4;
        addr = (uint32_t) &_line_buf;
    }

    // Clear line counter
    line = 0;

    // Snapshot start tick
    tick_start = HAL_GetTick();

    // Enable DMA IRQ
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    if (sensor.pixformat == PIXFORMAT_JPEG) {
        // Start a regular transfer
        HAL_DCMI_Start_DMA(&DCMIHandle,
                DCMI_MODE_SNAPSHOT, addr, length);
    } else {
        // Start a multibuffer transfer (line by line)
        HAL_DCMI_Start_DMA_MB(&DCMIHandle,
                DCMI_MODE_SNAPSHOT, addr, length, fb->h);
    }

    // Wait for frame
    while ((DCMI->CR & DCMI_CR_CAPTURE) != 0) {
        if ((HAL_GetTick() - tick_start) >= 3000) {
            // Sensor timeout, most likely a HW issue.
            // Abort the DMA request.
            HAL_DMA_Abort(&DMAHandle);
            return -1;
        }
    }

    // Abort DMA transfer.
    // Note: In JPEG mode the DMA will still be waiting for data since
    // the max frame size is set, so we need to abort the DMA transfer.
    HAL_DMA_Abort(&DMAHandle);

    // Disable DMA IRQ
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);

    // Fix the BPP
    switch (sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
            fb->bpp = 1;
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_RGB565:
            fb->bpp = 2;
            break;
        case PIXFORMAT_JPEG:
            // Read the number of data items transferred
            fb->bpp = (MAX_XFER_SIZE - DMAHandle.Instance->NDTR)*4;
            break;
    }

    // Set the user image.
    if (image != NULL) {
        image->w = fb->w;
        image->h = fb->h;
        image->bpp = fb->bpp;
        image->pixels = fb->pixels;
    }

    return 0;
}
