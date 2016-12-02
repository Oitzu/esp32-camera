/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Framebuffer pointer.
 *
 */
#ifndef __FRAMEBUFFER_H__
#define __FRAMEBUFFER_H__
#include "mutex.h"
extern char _fb_base;
static struct framebuffer {
    int w,h;
    int bpp;
    uint8_t pixels[];
// Note all instances of fb point to the same memory address.
}*__attribute__ ((__unused__)) fb = (struct framebuffer *) &_fb_base;

extern char _jpeg_buf;
static struct jpegbuffer {
    int w,h;
    int size;
    int enabled;
    int quality;
    mutex_t lock;
    uint8_t pixels[];
// Note all instances of jepg_fb point to the same memory address.
}*__attribute__ ((__unused__)) jpeg_fb = (struct jpegbuffer *) &_jpeg_buf;

// Use these macros to get a pointer to main or JPEG framebuffer.
#define MAIN_FB()       (fb)
#define JPEG_FB()       (jpeg_fb)
// Returns MAIN FB size
#define MAIN_FB_SIZE()  (MAIN_FB()->w*MAIN_FB()->h*MAIN_FB()->bpp)

// The JPEG offset allows JPEG compression of the framebuffer without overwriting the pixels.
// The offset size may need to be adjusted depending on the quality, otherwise JPEG data may
// overwrite image pixels before they are compressed.
#define FB_JPEG_OFFS_SIZE  (1024)

// Use this macro to get a pointer to the free SRAM area located after the framebuffer.
// If JPEG is enabled, this macro returns pixels + the JPEG image size (usually stored in bpp).
// TODO: FIX this, is main FB still used for JPEG ?
#define FB_PIXELS() ((MAIN_FB()->bpp > 2)? (MAIN_FB()->pixels+MAIN_FB()->bpp) : (MAIN_FB()->pixels+MAIN_FB_SIZE()))
#endif /* __FRAMEBUFFER_H__ */
