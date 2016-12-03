/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Sensor abstraction layer.
 *
 */
#include <stdlib.h>
#include <string.h>
#include "mp.h"
#include "irq.h"
#include "sccb.h"
#include "ov9650.h"
#include "ov2640.h"
#include "ov7725.h"
#include "sensor.h"
#include "systick.h"
#include "framebuffer.h"
#include "omv_boardconfig.h"

#define REG_PID        0x0A
#define REG_VER        0x0B

#define REG_MIDH       0x1C
#define REG_MIDL       0x1D

#define MAX_XFER_SIZE (0xFFFC)

sensor_t sensor;
TIM_HandleTypeDef  TIMHandle;
DMA_HandleTypeDef  DMAHandle;
DCMI_HandleTypeDef DCMIHandle;

static int line = 0;
extern uint8_t _line_buf;

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

static int extclk_config(int frequency, int pin)
{
    //Enable LEDC 
    periph_module_enable(PERIPH_LEDC_MODULE);

    //Timer configuration
    ledc_timer_config_t timer_conf;
    //Timer duty depth
    timer_conf.bit_num = 3; //=11 bit?
    //Timer frequency
    timer_conf.freq_hz = frequency;
    //Timer speed mode
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    //Timer number (0-3)
    timer_conf.timer_num = 0;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return -1;
    }

    //Channel configuration
    ledc_channel_config_t ch_conf;
    //Channel 
    ch_conf.channel = 0;
    //Timer source (0-3) (See timer configuration)
    ch_conf.timer_sel = 0;
    //Deactivate interrupt on channel
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    //Duty cycle (0 to (2*bit_num)-1)
    ch_conf.duty = 4;
    //Channel speed mode
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    //Pin number
    ch_conf.gpio_num = pin;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
        return -1;
    }

    return 0;
}

static int interface_config(uint32_t jpeg_mode)
{
    // DCMI configuration
    DCMIHandle.Instance         = DCMI;
    // VSYNC clock polarity
    DCMIHandle.Init.VSPolarity  = SENSOR_HW_FLAGS_GET(&sensor, SENSOR_HW_FLAGS_VSYNC) ?
                                    DCMI_VSPOLARITY_HIGH : DCMI_VSPOLARITY_LOW;
    // HSYNC clock polarity
    DCMIHandle.Init.HSPolarity  = SENSOR_HW_FLAGS_GET(&sensor, SENSOR_HW_FLAGS_HSYNC) ?
                                    DCMI_HSPOLARITY_HIGH : DCMI_HSPOLARITY_LOW;
    // PXCLK clock polarity
    DCMIHandle.Init.PCKPolarity = SENSOR_HW_FLAGS_GET(&sensor, SENSOR_HW_FLAGS_PIXCK) ?
                                    DCMI_PCKPOLARITY_RISING : DCMI_PCKPOLARITY_FALLING;

    DCMIHandle.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;    // Enable Hardware synchronization
    DCMIHandle.Init.CaptureRate = DCMI_CR_ALL_FRAME;        // Capture rate all frames
    DCMIHandle.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B; // Capture 8 bits on every pixel clock
    DCMIHandle.Init.JPEGMode = jpeg_mode;                   // Set JPEG Mode
    #if defined(STM32F765xx) || defined(STM32F769xx)
    DCMIHandle.Init.ByteSelectMode  = DCMI_BSM_ALL;         // Capture all received bytes
    DCMIHandle.Init.ByteSelectStart = DCMI_OEBS_ODD;        // Ignored
    DCMIHandle.Init.LineSelectMode  = DCMI_LSM_ALL;         // Capture all received lines
    DCMIHandle.Init.LineSelectStart = DCMI_OELS_ODD;        // Ignored
    #endif

    // Associate the DMA handle to the DCMI handle
    __HAL_LINKDMA(&DCMIHandle, DMA_Handle, DMAHandle);

   // Initialize the DCMI
    HAL_DCMI_DeInit(&DCMIHandle);
    if (HAL_DCMI_Init(&DCMIHandle) != HAL_OK) {
        // Initialization Error
        return -1;
    }

    // Configure and enable DCMI IRQ Channel
    HAL_NVIC_SetPriority(DCMI_IRQn, IRQ_PRI_DCMI, IRQ_SUBPRI_DCMI);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
    return 0;
}

static int dma_config()
{
    // DMA Stream configuration
    DMAHandle.Instance              = DMA2_Stream1;             /* Select the DMA instance          */
    DMAHandle.Init.Channel          = DMA_CHANNEL_1;            /* DMA Channel                      */
    DMAHandle.Init.Direction        = DMA_PERIPH_TO_MEMORY;     /* Peripheral to memory transfer    */
    DMAHandle.Init.MemInc           = DMA_MINC_ENABLE;          /* Memory increment mode Enable     */
    DMAHandle.Init.PeriphInc        = DMA_PINC_DISABLE;         /* Peripheral increment mode Enable */
    DMAHandle.Init.PeriphDataAlignment  = DMA_PDATAALIGN_WORD;  /* Peripheral data alignment : Word */
    DMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;      /* Memory data alignment : Word     */
    DMAHandle.Init.Mode             = DMA_NORMAL;               /* Normal DMA mode                  */
    DMAHandle.Init.Priority         = DMA_PRIORITY_HIGH;        /* Priority level : high            */
    DMAHandle.Init.FIFOMode         = DMA_FIFOMODE_ENABLE;      /* FIFO mode enabled                */
    DMAHandle.Init.FIFOThreshold    = DMA_FIFO_THRESHOLD_FULL;  /* FIFO threshold full              */
    DMAHandle.Init.MemBurst         = DMA_MBURST_INC4;          /* Memory burst                     */
    DMAHandle.Init.PeriphBurst      = DMA_PBURST_SINGLE;        /* Peripheral burst                 */

    // Configure and disable DMA IRQ Channel
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, IRQ_PRI_DMA21, IRQ_SUBPRI_DMA21);
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);

    // Initialize the DMA stream
    HAL_DMA_DeInit(&DMAHandle);
    if (HAL_DMA_Init(&DMAHandle) != HAL_OK) {
        // Initialization Error
        return -1;
    }

    return 0;
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
