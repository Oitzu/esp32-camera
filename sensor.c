/*
 * Portions of this file is part of the OpenMV project. (see sensor_* functions in the end of file)
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Sensor abstraction layer.
 */
 
 /*
 * 
 * DMA, I2S and transfer handling is from the esp32-demo-cam project https://github.com/igrr/esp32-cam-demo
 *
 * Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#if CONFIG_OV9650_SUPPORT
    #include "ov9650.h"
#endif
#if CONFIG_OV2640_SUPPORT
    #include "ov2640.h"
#endif
#if CONFIG_OV7725_SUPPORT
    #include "ov7725.h"
#endif

#include "rom/lldesc.h"
#include "esp_intr.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"

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
static bool s_initialized = false;
static int s_fb_w;
static int s_fb_h;
static size_t s_fb_size;
static int bpp;                     //bytes per pixel
static bool jpeg_mode;

static volatile int isr_count = 0;
static volatile int line_count = 0;
static volatile int cur_buffer = 0;
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
static esp_err_t dma_desc_init(size_t buf_size);
static esp_err_t dma_buf_realloc(size_t buf_size);
static void fb_filler_task(void *pvParameters);

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

static esp_err_t dma_desc_init(size_t buf_size)
{
    /* I2S peripheral captures 16 bit of data every clock cycle,
    even though we are only using 8 bits.
    On top of that we need two bytes per pixel.
    */
    for (int i = 0; i < 2; ++i) {
        ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        s_dma_buf[i] = (uint32_t*) malloc(buf_size);
        if (s_dma_buf[i] == NULL) {
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGV(TAG, "dma_buf[%d]=%p\n", i, s_dma_buf[i]);

        s_dma_desc[i].length = buf_size;     // size of a single DMA buf
        s_dma_desc[i].size = buf_size;       // total size of the chain
        s_dma_desc[i].owner = 1;
        s_dma_desc[i].sosf = 1;
        s_dma_desc[i].buf = (uint8_t*) s_dma_buf[i];
        s_dma_desc[i].offset = i;
        s_dma_desc[i].empty = 0;
        s_dma_desc[i].eof = 1;
        s_dma_desc[i].qe.stqe_next = NULL;
    }
    return ESP_OK;
}

static esp_err_t dma_buf_realloc(size_t buf_size)
{
    for (int i = 0; i < 2; ++i) {
        ESP_LOGD(TAG, "Reallocating DMA buffer #%d, size=%d", i, buf_size);
        realloc(s_dma_buf[i], buf_size);
        if (s_dma_buf[i] == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
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

    SET_PERI_REG_BITS(I2S_RXEOF_NUM_REG(0), I2S_RX_EOF_NUM, (s_fb_w - 2) * 2, I2S_RX_EOF_NUM_S);
    SET_PERI_REG_BITS(I2S_IN_LINK_REG(0), I2S_INLINK_ADDR, ((uint32_t) &s_dma_desc[index]), I2S_INLINK_ADDR_S);
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

esp_err_t init(const camera_config_t* config)
{
    memcpy(&s_config, config, sizeof(s_config)); //Copy config
    
    /* Do a power cycle */
    gpio_set_level(s_config.pin_pwdn, 1);
    systick_sleep(10);

    gpio_set_level(s_config.pin_pwdn, 0);
    systick_sleep(10);

    /* Initialize the SCCB interface */
    SCCB_Init(s_config.pin_sscb_sda, s_config.pin_sscb_scl);
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
    ESP_LOGD(TAG, "Enabling XCLK output");
    // Configure external clock timer.
    if (extclk_config(s_config.xclk_freq_hz, s_config.pin_xclk) != 0) {
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

    ESP_LOGD(TAG, "Resetting camera");
    /* Reset the sensor */
    gpio_set_level(s_config.pin_reset, 1);
    systick_sleep(10);

    gpio_set_level(s_config.pin_reset, 0);
    systick_sleep(10);

    ESP_LOGD(TAG, "Searching for camera address");
    /* Probe the sensor */
    sensor.slv_addr = SCCB_Probe();
    if (sensor.slv_addr == 0) {
        /* Sensor has been held in reset,
           so the reset line is active low */
        sensor.reset_pol = ACTIVE_LOW;

        /* Pull the sensor out of the reset state */
        gpio_set_level(s_config.pin_reset, 1);
        systick_sleep(10);

        /* Probe again to set the slave addr */
        sensor.slv_addr = SCCB_Probe();
        if (sensor.slv_addr == 0)  {
            // Probe failed
            return ESP_ERR_CAMERA_NOT_DETECTED;
        }
    }
    ESP_LOGD(TAG, "Detected camera at address=0x%02x", sensor.slv_addr);

    /* Read the sensor information */
    sensor.id.PID  = SCCB_Read(sensor.slv_addr, REG_PID);
    sensor.id.VER  = SCCB_Read(sensor.slv_addr, REG_VER);
    sensor.id.MIDL = SCCB_Read(sensor.slv_addr, REG_MIDL);
    sensor.id.MIDH = SCCB_Read(sensor.slv_addr, REG_MIDH);

    /* Call the sensor-specific init function */
    switch (sensor.id.PID) {
#if CONFIG_OV9650_SUPPORT
        case OV9650_PID:
            ov9650_init(&sensor);
            break;
#endif
#if CONFIG_OV2640_SUPPORT
        case OV2640_PID:
            ov2640_init(&sensor);
            break;
#endif
#if CONFIG_OV7725_SUPPORT
        case OV7725_PID:
            ov7725_init(&sensor);
            break;
#endif
        default:
            /* Sensor not supported */
            ESP_LOGD(TAG, "Detected camera not supported.");
            return ESP_ERR_CAMERA_NOT_SUPPORTED;
    }
    
    /* Set bytes per pixel */
    bpp = 2;
    
    /* Allocating buffers */
    s_fb_w = resolution[sensor.framesize][0] * bpp;
    s_fb_h = resolution[sensor.framesize][1];
    
    s_fb_size = s_fb_w * s_fb_h;
    s_fb = (uint8_t*) malloc(s_fb_size);
    dma_desc_init(s_fb_w * 2);              //We need the dma buffer twice the size because i2s captures 16bit per cycle.
    
    ESP_LOGD(TAG, "Initializing I2S and DMA");
    i2s_init();
    esp_err_t err = dma_desc_init(s_fb_w);
    if (err != ESP_OK) {
        free(s_fb);
        return err;
    }
    
    /* Creating data and frame semaphore */
    data_ready = xSemaphoreCreateBinary();
    frame_ready = xSemaphoreCreateBinary();

    /* Creating task that pushes lines into fb */
    xTaskCreatePinnedToCore(&fb_filler_task, "fb_filler", 2048, NULL, 10, NULL, 0);
    
    ESP_LOGD(TAG, "Init done");
    /* All good! */
    return ESP_OK;
}

static void fb_filler_task(void *pvParameters) {
    static int prev_buf = -1;
    while (true) {
        xSemaphoreTake(data_ready, portMAX_DELAY);
        int buf_idx = !cur_buffer;
        if (prev_buf != -1 && prev_buf == buf_idx) {
            ets_printf("! %d\n", line_count);               //We probably processing the same frame a second time.
        }
        uint8_t* pfb = s_fb + line_count * s_fb_w;  //Get pointer of target buffer for current line
        const uint32_t* buf = s_dma_buf[buf_idx];           //Get pointer of the source buffer
        for (int i = 0; i < s_fb_w; ++i) {
            uint32_t v = *buf;                              //Extract 4 bytes from the source buffer
            uint8_t comp = (v & 0xff0000) >> 16;            //For luminance only we only want the 3. byte
            *pfb = comp;                                    //Write byte to target buffer
            ++buf;                                          //Move pointer of source buffer 4 bytes forward
            ++pfb;                                          //Move pointer of target buffer 1 byte forward
        }
        ++line_count;
        prev_buf = buf_idx;
        if (!i2s_running) {                                 //i2s no longer running the frame is probably complete
            prev_buf = -1;
            xSemaphoreGive(frame_ready);
        }
    }
}

static void IRAM_ATTR i2s_isr(void* arg) {
    REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG(0)) & 0xffffffc0) | 0x3f);
    cur_buffer = !cur_buffer;
    if (isr_count == s_fb_h - 2) {
        i2s_stop();
    }
    else {
        i2s_fill_buf(cur_buffer);
        ++isr_count;
    }
    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(data_ready, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

int sensor_reset()
{
    // Reset the sesnor state
    sensor.sde = 0xFF;
    sensor.pixformat=0xFF;
    sensor.framesize=0xFF;
    sensor.framerate=0xFF;
    sensor.gainceiling=0xFF;

    // Call sensor-specific reset function
    sensor.reset(&sensor);

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
    jpeg_mode = false;

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
        jpeg_mode = true;
    }

    return ESP_OK;
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
    
    //realloc memory
    s_fb_w = resolution[sensor.framesize][0] * 2;
    s_fb_h = resolution[sensor.framesize][1];
    s_fb_size = s_fb_w * s_fb_h;
    realloc(s_fb, s_fb_size);
    dma_buf_realloc(s_fb_w * 4);

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