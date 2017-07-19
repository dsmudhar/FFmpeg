/*
 * Copyright (C) 2004-2010 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (C) 2008 David Conrad
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

#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavcodec/me_cmp.h"
#include "snow_dwt_lavf.h"

static void snow_horizontal_compose97i(IDWTELEM *b, IDWTELEM *temp, int width)
{
    const int w2 = (width + 1) >> 1;
    int x;

    temp[0] = b[0] - ((3 * b[w2] + 2) >> 2);
    for (x = 1; x < (width >> 1); x++) {
        temp[2 * x]     = b[x] - ((3 * (b[x + w2 - 1] + b[x + w2]) + 4) >> 3);
        temp[2 * x - 1] = b[x + w2 - 1] - temp[2 * x - 2] - temp[2 * x];
    }
    if (width & 1) {
        temp[2 * x]     = b[x] - ((3 * b[x + w2 - 1] + 2) >> 2);
        temp[2 * x - 1] = b[x + w2 - 1] - temp[2 * x - 2] - temp[2 * x];
    } else
        temp[2 * x - 1] = b[x + w2 - 1] - 2 * temp[2 * x - 2];

    b[0] = temp[0] + ((2 * temp[0] + temp[1] + 4) >> 3);
    for (x = 2; x < width - 1; x += 2) {
        b[x]     = temp[x] + ((4 * temp[x] + temp[x - 1] + temp[x + 1] + 8) >> 4);
        b[x - 1] = temp[x - 1] + ((3 * (b[x - 2] + b[x])) >> 1);
    }
    if (width & 1) {
        b[x]     = temp[x] + ((2 * temp[x] + temp[x - 1] + 4) >> 3);
        b[x - 1] = temp[x - 1] + ((3 * (b[x - 2] + b[x])) >> 1);
    } else
        b[x - 1] = temp[x - 1] + 3 * b[x - 2];
}

static void vertical_compose97iH0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                  int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] += (W_AM * (b0[i] + b2[i]) + W_AO) >> W_AS;
}

static void vertical_compose97iH1(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                  int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] -= (W_CM * (b0[i] + b2[i]) + W_CO) >> W_CS;
}

static void vertical_compose97iL0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                  int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] += (W_BM * (b0[i] + b2[i]) + 4 * b1[i] + W_BO) >> W_BS;
}

static void vertical_compose97iL1(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                  int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] -= (W_DM * (b0[i] + b2[i]) + W_DO) >> W_DS;
}

static void spatial_compose97i_init(DWTCompose *cs, IDWTELEM *buffer, int height,
                                    int stride)
{
    cs->b0 = buffer + avpriv_mirror(-3 - 1, height - 1) * stride;
    cs->b1 = buffer + avpriv_mirror(-3,     height - 1) * stride;
    cs->b2 = buffer + avpriv_mirror(-3 + 1, height - 1) * stride;
    cs->b3 = buffer + avpriv_mirror(-3 + 2, height - 1) * stride;
    cs->y  = -3;
}

static void spatial_compose97i_dy(DWTCompose *cs, IDWTELEM *buffer,
                                  IDWTELEM *temp, int width, int height,
                                  int stride)
{
    int y        = cs->y;
    IDWTELEM *b0 = cs->b0;
    IDWTELEM *b1 = cs->b1;
    IDWTELEM *b2 = cs->b2;
    IDWTELEM *b3 = cs->b3;
    IDWTELEM *b4 = buffer + avpriv_mirror(y + 3, height - 1) * stride;
    IDWTELEM *b5 = buffer + avpriv_mirror(y + 4, height - 1) * stride;

    if (y + 3 < (unsigned)height)
        vertical_compose97iL1(b3, b4, b5, width);
    if (y + 2 < (unsigned)height)
        vertical_compose97iH1(b2, b3, b4, width);
    if (y + 1 < (unsigned)height)
        vertical_compose97iL0(b1, b2, b3, width);
    if (y + 0 < (unsigned)height)
        vertical_compose97iH0(b0, b1, b2, width);

    if (y - 1 < (unsigned)height)
        snow_horizontal_compose97i(b0, temp, width);
    if (y + 0 < (unsigned)height)
        snow_horizontal_compose97i(b1, temp, width);

    cs->b0  = b2;
    cs->b1  = b3;
    cs->b2  = b4;
    cs->b3  = b5;
    cs->y  += 2;
}

static void spatial_idwt_init(DWTCompose *cs, IDWTELEM *buffer, int width,
                                 int height, int stride, int type,
                                 int decomposition_count)
{
    int level;
    for (level = decomposition_count - 1; level >= 0; level--) {
        switch (type) {
        case DWT_97:
            spatial_compose97i_init(cs + level, buffer, height >> level,
                                    stride << level);
            break;
        }
    }
}

static void spatial_idwt_slice(DWTCompose *cs, IDWTELEM *buffer,
                                  IDWTELEM *temp, int width, int height,
                                  int stride, int type,
                                  int decomposition_count, int y)
{
    const int support = type == 1 ? 3 : 5;
    int level;
    if (type == 2)
        return;

    for (level = decomposition_count - 1; level >= 0; level--)
        while (cs[level].y <= FFMIN((y >> level) + support, height >> level)) {
            switch (type) {
            case DWT_97:
                spatial_compose97i_dy(cs + level, buffer, temp, width >> level,
                                      height >> level, stride << level);
                break;
            }
        }
}

void snow_spatial_idwt(IDWTELEM *buffer, IDWTELEM *temp, int width, int height,
                     int stride, int type, int decomposition_count)
{
    DWTCompose cs[MAX_DECOMPOSITIONS];
    int y;
    spatial_idwt_init(cs, buffer, width, height, stride, type,
                         decomposition_count);
    for (y = 0; y < height; y += 4)
        spatial_idwt_slice(cs, buffer, temp, width, height, stride, type,
                              decomposition_count, y);
}

static inline int w_c(struct MpegEncContext *v, uint8_t *pix1, uint8_t *pix2, ptrdiff_t line_size,
                      int w, int h, int type)
{
    int s, i, j;
    const int dec_count = w == 8 ? 3 : 4;
    int tmp[32 * 32], tmp2[32];
    int level, ori;
    static const int scale[2][2][4][4] = {
        {
            { // 9/7 8x8 dec=3
                { 268, 239, 239, 213 },
                { 0,   224, 224, 152 },
                { 0,   135, 135, 110 },
            },
            { // 9/7 16x16 or 32x32 dec=4
                { 344, 310, 310, 280 },
                { 0,   320, 320, 228 },
                { 0,   175, 175, 136 },
                { 0,   129, 129, 102 },
            }
        },
        {
            { // 5/3 8x8 dec=3
                { 275, 245, 245, 218 },
                { 0,   230, 230, 156 },
                { 0,   138, 138, 113 },
            },
            { // 5/3 16x16 or 32x32 dec=4
                { 352, 317, 317, 286 },
                { 0,   328, 328, 233 },
                { 0,   180, 180, 140 },
                { 0,   132, 132, 105 },
            }
        }
    };

    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j += 4) {
            tmp[32 * i + j + 0] = (pix1[j + 0] - pix2[j + 0]) << 4;
            tmp[32 * i + j + 1] = (pix1[j + 1] - pix2[j + 1]) << 4;
            tmp[32 * i + j + 2] = (pix1[j + 2] - pix2[j + 2]) << 4;
            tmp[32 * i + j + 3] = (pix1[j + 3] - pix2[j + 3]) << 4;
        }
        pix1 += line_size;
        pix2 += line_size;
    }

    //ff_spatial_dwt(tmp, tmp2, w, h, 32, type, dec_count);

    s = 0;
    av_assert1(w == h);
    for (level = 0; level < dec_count; level++)
        for (ori = level ? 1 : 0; ori < 4; ori++) {
            int size   = w >> (dec_count - level);
            int sx     = (ori & 1) ? size : 0;
            int stride = 32 << (dec_count - level);
            int sy     = (ori & 2) ? stride >> 1 : 0;

            for (i = 0; i < size; i++)
                for (j = 0; j < size; j++) {
                    int v = tmp[sx + sy + i * stride + j] *
                            scale[type][dec_count - 3][level][ori];
                    s += FFABS(v);
                }
        }
    av_assert1(s >= 0);
    return s >> 9;
}

int ff_w97_32_c(struct MpegEncContext *v, uint8_t *pix1, uint8_t *pix2, ptrdiff_t line_size, int h)
{
    return w_c(v, pix1, pix2, line_size, 32, h, 0);
}
