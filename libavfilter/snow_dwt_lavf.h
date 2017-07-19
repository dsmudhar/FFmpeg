/*
 * Copyright (C) 2004-2010 Michael Niedermayer <michaelni@gmx.at>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef AVCODEC_SNOW_DWT_H
#define AVCODEC_SNOW_DWT_H

#include <stddef.h>
#include <stdint.h>

typedef int DWTELEM;
typedef short IDWTELEM;

#define MAX_DECOMPOSITIONS 8

typedef struct DWTCompose {
    IDWTELEM *b0;
    IDWTELEM *b1;
    IDWTELEM *b2;
    IDWTELEM *b3;
    int y;
} DWTCompose;

/** Used to minimize the amount of memory used in order to
 *  optimize cache performance. **/
typedef struct slice_buffer_s {
    IDWTELEM **line;   ///< For use by idwt and predict_slices.
    IDWTELEM **data_stack;   ///< Used for internal purposes.
    int data_stack_top;
    int line_count;
    int line_width;
    int data_count;
    IDWTELEM *base_buffer;  ///< Buffer that this structure is caching.
} slice_buffer;

#define DWT_97 0

#define liftS lift
#define W_AM 3
#define W_AO 0
#define W_AS 1

#undef liftS
#define W_BM 1
#define W_BO 8
#define W_BS 4

#define W_CM 1
#define W_CO 0
#define W_CS 0

#define W_DM 3
#define W_DO 4
#define W_DS 3

void ff_snow_inner_add_yblock(const uint8_t *obmc, const int obmc_stride,
                              uint8_t **block, int b_w, int b_h, int src_x,
                              int src_y, int src_stride, slice_buffer *sb,
                              int add, uint8_t *dst8);

int ff_w97_32_c(struct MpegEncContext *v, uint8_t *pix1, uint8_t *pix2, ptrdiff_t line_size, int h);

void ff_spatial_idwt(IDWTELEM *buffer, IDWTELEM *temp, int width, int height,
                     int stride, int type, int decomposition_count);

#endif /* AVCODEC_DWT_H */
