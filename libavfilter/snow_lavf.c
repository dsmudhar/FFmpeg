/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
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

#include "libavutil/intmath.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"

#include "libavcodec/avcodec.h"
#include "libmotion/me_cmp.h"
#include "libavcodec/internal.h"
#include "libavcodec/rangecoder.h"
#include "libavcodec/mathops.h"
#include "libavcodec/h263.h"

#include "snow_lavf.h"

static const uint8_t obmc32[1024]={
  0,  0,  0,  0,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,  8,  4,  4,  4,  4,  4,  4,  4,  4,  0,  0,  0,  0,
  0,  4,  4,  4,  8,  8,  8, 12, 12, 16, 16, 16, 20, 20, 20, 24, 24, 20, 20, 20, 16, 16, 16, 12, 12,  8,  8,  8,  4,  4,  4,  0,
  0,  4,  8,  8, 12, 12, 16, 20, 20, 24, 28, 28, 32, 32, 36, 40, 40, 36, 32, 32, 28, 28, 24, 20, 20, 16, 12, 12,  8,  8,  4,  0,
  0,  4,  8, 12, 16, 20, 24, 28, 28, 32, 36, 40, 44, 48, 52, 56, 56, 52, 48, 44, 40, 36, 32, 28, 28, 24, 20, 16, 12,  8,  4,  0,
  4,  8, 12, 16, 20, 24, 28, 32, 40, 44, 48, 52, 56, 60, 64, 68, 68, 64, 60, 56, 52, 48, 44, 40, 32, 28, 24, 20, 16, 12,  8,  4,
  4,  8, 12, 20, 24, 32, 36, 40, 48, 52, 56, 64, 68, 76, 80, 84, 84, 80, 76, 68, 64, 56, 52, 48, 40, 36, 32, 24, 20, 12,  8,  4,
  4,  8, 16, 24, 28, 36, 44, 48, 56, 60, 68, 76, 80, 88, 96,100,100, 96, 88, 80, 76, 68, 60, 56, 48, 44, 36, 28, 24, 16,  8,  4,
  4, 12, 20, 28, 32, 40, 48, 56, 64, 72, 80, 88, 92,100,108,116,116,108,100, 92, 88, 80, 72, 64, 56, 48, 40, 32, 28, 20, 12,  4,
  4, 12, 20, 28, 40, 48, 56, 64, 72, 80, 88, 96,108,116,124,132,132,124,116,108, 96, 88, 80, 72, 64, 56, 48, 40, 28, 20, 12,  4,
  4, 16, 24, 32, 44, 52, 60, 72, 80, 92,100,108,120,128,136,148,148,136,128,120,108,100, 92, 80, 72, 60, 52, 44, 32, 24, 16,  4,
  4, 16, 28, 36, 48, 56, 68, 80, 88,100,112,120,132,140,152,164,164,152,140,132,120,112,100, 88, 80, 68, 56, 48, 36, 28, 16,  4,
  4, 16, 28, 40, 52, 64, 76, 88, 96,108,120,132,144,156,168,180,180,168,156,144,132,120,108, 96, 88, 76, 64, 52, 40, 28, 16,  4,
  8, 20, 32, 44, 56, 68, 80, 92,108,120,132,144,156,168,180,192,192,180,168,156,144,132,120,108, 92, 80, 68, 56, 44, 32, 20,  8,
  8, 20, 32, 48, 60, 76, 88,100,116,128,140,156,168,184,196,208,208,196,184,168,156,140,128,116,100, 88, 76, 60, 48, 32, 20,  8,
  8, 20, 36, 52, 64, 80, 96,108,124,136,152,168,180,196,212,224,224,212,196,180,168,152,136,124,108, 96, 80, 64, 52, 36, 20,  8,
  8, 24, 40, 56, 68, 84,100,116,132,148,164,180,192,208,224,240,240,224,208,192,180,164,148,132,116,100, 84, 68, 56, 40, 24,  8,
  8, 24, 40, 56, 68, 84,100,116,132,148,164,180,192,208,224,240,240,224,208,192,180,164,148,132,116,100, 84, 68, 56, 40, 24,  8,
  8, 20, 36, 52, 64, 80, 96,108,124,136,152,168,180,196,212,224,224,212,196,180,168,152,136,124,108, 96, 80, 64, 52, 36, 20,  8,
  8, 20, 32, 48, 60, 76, 88,100,116,128,140,156,168,184,196,208,208,196,184,168,156,140,128,116,100, 88, 76, 60, 48, 32, 20,  8,
  8, 20, 32, 44, 56, 68, 80, 92,108,120,132,144,156,168,180,192,192,180,168,156,144,132,120,108, 92, 80, 68, 56, 44, 32, 20,  8,
  4, 16, 28, 40, 52, 64, 76, 88, 96,108,120,132,144,156,168,180,180,168,156,144,132,120,108, 96, 88, 76, 64, 52, 40, 28, 16,  4,
  4, 16, 28, 36, 48, 56, 68, 80, 88,100,112,120,132,140,152,164,164,152,140,132,120,112,100, 88, 80, 68, 56, 48, 36, 28, 16,  4,
  4, 16, 24, 32, 44, 52, 60, 72, 80, 92,100,108,120,128,136,148,148,136,128,120,108,100, 92, 80, 72, 60, 52, 44, 32, 24, 16,  4,
  4, 12, 20, 28, 40, 48, 56, 64, 72, 80, 88, 96,108,116,124,132,132,124,116,108, 96, 88, 80, 72, 64, 56, 48, 40, 28, 20, 12,  4,
  4, 12, 20, 28, 32, 40, 48, 56, 64, 72, 80, 88, 92,100,108,116,116,108,100, 92, 88, 80, 72, 64, 56, 48, 40, 32, 28, 20, 12,  4,
  4,  8, 16, 24, 28, 36, 44, 48, 56, 60, 68, 76, 80, 88, 96,100,100, 96, 88, 80, 76, 68, 60, 56, 48, 44, 36, 28, 24, 16,  8,  4,
  4,  8, 12, 20, 24, 32, 36, 40, 48, 52, 56, 64, 68, 76, 80, 84, 84, 80, 76, 68, 64, 56, 52, 48, 40, 36, 32, 24, 20, 12,  8,  4,
  4,  8, 12, 16, 20, 24, 28, 32, 40, 44, 48, 52, 56, 60, 64, 68, 68, 64, 60, 56, 52, 48, 44, 40, 32, 28, 24, 20, 16, 12,  8,  4,
  0,  4,  8, 12, 16, 20, 24, 28, 28, 32, 36, 40, 44, 48, 52, 56, 56, 52, 48, 44, 40, 36, 32, 28, 28, 24, 20, 16, 12,  8,  4,  0,
  0,  4,  8,  8, 12, 12, 16, 20, 20, 24, 28, 28, 32, 32, 36, 40, 40, 36, 32, 32, 28, 28, 24, 20, 20, 16, 12, 12,  8,  8,  4,  0,
  0,  4,  4,  4,  8,  8,  8, 12, 12, 16, 16, 16, 20, 20, 20, 24, 24, 20, 20, 20, 16, 16, 16, 12, 12,  8,  8,  8,  4,  4,  4,  0,
  0,  0,  0,  0,  4,  4,  4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,  8,  8,  8,  4,  4,  4,  4,  4,  4,  4,  4,  0,  0,  0,  0,
 //error:0.000020
};
static const uint8_t obmc16[256]={
  0,  4,  4,  8,  8, 12, 12, 16, 16, 12, 12,  8,  8,  4,  4,  0,
  4,  8, 16, 20, 28, 32, 40, 44, 44, 40, 32, 28, 20, 16,  8,  4,
  4, 16, 24, 36, 44, 56, 64, 76, 76, 64, 56, 44, 36, 24, 16,  4,
  8, 20, 36, 48, 64, 76, 92,104,104, 92, 76, 64, 48, 36, 20,  8,
  8, 28, 44, 64, 80,100,116,136,136,116,100, 80, 64, 44, 28,  8,
 12, 32, 56, 76,100,120,144,164,164,144,120,100, 76, 56, 32, 12,
 12, 40, 64, 92,116,144,168,196,196,168,144,116, 92, 64, 40, 12,
 16, 44, 76,104,136,164,196,224,224,196,164,136,104, 76, 44, 16,
 16, 44, 76,104,136,164,196,224,224,196,164,136,104, 76, 44, 16,
 12, 40, 64, 92,116,144,168,196,196,168,144,116, 92, 64, 40, 12,
 12, 32, 56, 76,100,120,144,164,164,144,120,100, 76, 56, 32, 12,
  8, 28, 44, 64, 80,100,116,136,136,116,100, 80, 64, 44, 28,  8,
  8, 20, 36, 48, 64, 76, 92,104,104, 92, 76, 64, 48, 36, 20,  8,
  4, 16, 24, 36, 44, 56, 64, 76, 76, 64, 56, 44, 36, 24, 16,  4,
  4,  8, 16, 20, 28, 32, 40, 44, 44, 40, 32, 28, 20, 16,  8,  4,
  0,  4,  4,  8,  8, 12, 12, 16, 16, 12, 12,  8,  8,  4,  4,  0,
//error:0.000015
};

//linear *64
static const uint8_t obmc8[64]={
  4, 12, 20, 28, 28, 20, 12,  4,
 12, 36, 60, 84, 84, 60, 36, 12,
 20, 60,100,140,140,100, 60, 20,
 28, 84,140,196,196,140, 84, 28,
 28, 84,140,196,196,140, 84, 28,
 20, 60,100,140,140,100, 60, 20,
 12, 36, 60, 84, 84, 60, 36, 12,
  4, 12, 20, 28, 28, 20, 12,  4,
//error:0.000000
};

//linear *64
static const uint8_t obmc4[16]={
 16, 48, 48, 16,
 48,144,144, 48,
 48,144,144, 48,
 16, 48, 48, 16,
//error:0.000000
};

const uint8_t * const lavfsnow_obmc_tab[4]= {
    obmc32, obmc16, obmc8, obmc4
};

int lavfsnow_alloc_blocks(LavfSnowContext *s){
    int w= AV_CEIL_RSHIFT(s->avctx->width,  LOG2_MB_SIZE);
    int h= AV_CEIL_RSHIFT(s->avctx->height, LOG2_MB_SIZE);

    s->b_width = w;
    s->b_height= h;

    av_free(s->block);
    s->block= av_mallocz_array(w * h,  sizeof(LavfBlockNode) << (s->block_max_depth*2));
    if (!s->block)
        return AVERROR(ENOMEM);

    return 0;
}

static void mc_block(LavfPlane *p, uint8_t *dst, const uint8_t *src, int stride, int b_w, int b_h, int dx, int dy){
    static const uint8_t weight[64]={
    8,7,6,5,4,3,2,1,
    7,7,0,0,0,0,0,1,
    6,0,6,0,0,0,2,0,
    5,0,0,5,0,3,0,0,
    4,0,0,0,4,0,0,0,
    3,0,0,5,0,3,0,0,
    2,0,6,0,0,0,2,0,
    1,7,0,0,0,0,0,1,
    };

    static const uint8_t brane[256]={
    0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x11,0x12,0x12,0x12,0x12,0x12,0x12,0x12,
    0x04,0x05,0xcc,0xcc,0xcc,0xcc,0xcc,0x41,0x15,0x16,0xcc,0xcc,0xcc,0xcc,0xcc,0x52,
    0x04,0xcc,0x05,0xcc,0xcc,0xcc,0x41,0xcc,0x15,0xcc,0x16,0xcc,0xcc,0xcc,0x52,0xcc,
    0x04,0xcc,0xcc,0x05,0xcc,0x41,0xcc,0xcc,0x15,0xcc,0xcc,0x16,0xcc,0x52,0xcc,0xcc,
    0x04,0xcc,0xcc,0xcc,0x41,0xcc,0xcc,0xcc,0x15,0xcc,0xcc,0xcc,0x16,0xcc,0xcc,0xcc,
    0x04,0xcc,0xcc,0x41,0xcc,0x05,0xcc,0xcc,0x15,0xcc,0xcc,0x52,0xcc,0x16,0xcc,0xcc,
    0x04,0xcc,0x41,0xcc,0xcc,0xcc,0x05,0xcc,0x15,0xcc,0x52,0xcc,0xcc,0xcc,0x16,0xcc,
    0x04,0x41,0xcc,0xcc,0xcc,0xcc,0xcc,0x05,0x15,0x52,0xcc,0xcc,0xcc,0xcc,0xcc,0x16,
    0x44,0x45,0x45,0x45,0x45,0x45,0x45,0x45,0x55,0x56,0x56,0x56,0x56,0x56,0x56,0x56,
    0x48,0x49,0xcc,0xcc,0xcc,0xcc,0xcc,0x85,0x59,0x5A,0xcc,0xcc,0xcc,0xcc,0xcc,0x96,
    0x48,0xcc,0x49,0xcc,0xcc,0xcc,0x85,0xcc,0x59,0xcc,0x5A,0xcc,0xcc,0xcc,0x96,0xcc,
    0x48,0xcc,0xcc,0x49,0xcc,0x85,0xcc,0xcc,0x59,0xcc,0xcc,0x5A,0xcc,0x96,0xcc,0xcc,
    0x48,0xcc,0xcc,0xcc,0x49,0xcc,0xcc,0xcc,0x59,0xcc,0xcc,0xcc,0x96,0xcc,0xcc,0xcc,
    0x48,0xcc,0xcc,0x85,0xcc,0x49,0xcc,0xcc,0x59,0xcc,0xcc,0x96,0xcc,0x5A,0xcc,0xcc,
    0x48,0xcc,0x85,0xcc,0xcc,0xcc,0x49,0xcc,0x59,0xcc,0x96,0xcc,0xcc,0xcc,0x5A,0xcc,
    0x48,0x85,0xcc,0xcc,0xcc,0xcc,0xcc,0x49,0x59,0x96,0xcc,0xcc,0xcc,0xcc,0xcc,0x5A,
    };

    static const uint8_t needs[16]={
    0,1,0,0,
    2,4,2,0,
    0,1,0,0,
    15
    };

    int x, y, b, r, l;
    int16_t tmpIt   [64*(32+HTAPS_MAX)];
    uint8_t tmp2t[3][64*(32+HTAPS_MAX)];
    int16_t *tmpI= tmpIt;
    uint8_t *tmp2= tmp2t[0];
    const uint8_t *hpel[11];
    av_assert2(dx<16 && dy<16);
    r= brane[dx + 16*dy]&15;
    l= brane[dx + 16*dy]>>4;

    b= needs[l] | needs[r];
    if(p && !(1))
        b= 15;

    if(b&5){
        for(y=0; y < b_h+HTAPS_MAX-1; y++){
            for(x=0; x < b_w; x++){
                int a_1=src[x + HTAPS_MAX/2-4];
                int a0= src[x + HTAPS_MAX/2-3];
                int a1= src[x + HTAPS_MAX/2-2];
                int a2= src[x + HTAPS_MAX/2-1];
                int a3= src[x + HTAPS_MAX/2+0];
                int a4= src[x + HTAPS_MAX/2+1];
                int a5= src[x + HTAPS_MAX/2+2];
                int a6= src[x + HTAPS_MAX/2+3];
                int am=0;
                if(!p || p->fast_mc){
                    am= 20*(a2+a3) - 5*(a1+a4) + (a0+a5);
                    tmpI[x]= am;
                    am= (am+16)>>5;
                }else{
                    am= p->hcoeff[0]*(a2+a3) + p->hcoeff[1]*(a1+a4) + p->hcoeff[2]*(a0+a5) + p->hcoeff[3]*(a_1+a6);
                    tmpI[x]= am;
                    am= (am+32)>>6;
                }

                if(am&(~255)) am= ~(am>>31);
                tmp2[x]= am;
            }
            tmpI+= 64;
            tmp2+= 64;
            src += stride;
        }
        src -= stride*y;
    }
    src += HTAPS_MAX/2 - 1;
    tmp2= tmp2t[1];

    if(b&2){
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w+1; x++){
                int a_1=src[x + (HTAPS_MAX/2-4)*stride];
                int a0= src[x + (HTAPS_MAX/2-3)*stride];
                int a1= src[x + (HTAPS_MAX/2-2)*stride];
                int a2= src[x + (HTAPS_MAX/2-1)*stride];
                int a3= src[x + (HTAPS_MAX/2+0)*stride];
                int a4= src[x + (HTAPS_MAX/2+1)*stride];
                int a5= src[x + (HTAPS_MAX/2+2)*stride];
                int a6= src[x + (HTAPS_MAX/2+3)*stride];
                int am=0;
                if(!p || p->fast_mc)
                    am= (20*(a2+a3) - 5*(a1+a4) + (a0+a5) + 16)>>5;
                else
                    am= (p->hcoeff[0]*(a2+a3) + p->hcoeff[1]*(a1+a4) + p->hcoeff[2]*(a0+a5) + p->hcoeff[3]*(a_1+a6) + 32)>>6;

                if(am&(~255)) am= ~(am>>31);
                tmp2[x]= am;
            }
            src += stride;
            tmp2+= 64;
        }
        src -= stride*y;
    }
    src += stride*(HTAPS_MAX/2 - 1);
    tmp2= tmp2t[2];
    tmpI= tmpIt;
    if(b&4){
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w; x++){
                int a_1=tmpI[x + (HTAPS_MAX/2-4)*64];
                int a0= tmpI[x + (HTAPS_MAX/2-3)*64];
                int a1= tmpI[x + (HTAPS_MAX/2-2)*64];
                int a2= tmpI[x + (HTAPS_MAX/2-1)*64];
                int a3= tmpI[x + (HTAPS_MAX/2+0)*64];
                int a4= tmpI[x + (HTAPS_MAX/2+1)*64];
                int a5= tmpI[x + (HTAPS_MAX/2+2)*64];
                int a6= tmpI[x + (HTAPS_MAX/2+3)*64];
                int am=0;
                if(!p || p->fast_mc)
                    am= (20*(a2+a3) - 5*(a1+a4) + (a0+a5) + 512)>>10;
                else
                    am= (p->hcoeff[0]*(a2+a3) + p->hcoeff[1]*(a1+a4) + p->hcoeff[2]*(a0+a5) + p->hcoeff[3]*(a_1+a6) + 2048)>>12;
                if(am&(~255)) am= ~(am>>31);
                tmp2[x]= am;
            }
            tmpI+= 64;
            tmp2+= 64;
        }
    }

    hpel[ 0]= src;
    hpel[ 1]= tmp2t[0] + 64*(HTAPS_MAX/2-1);
    hpel[ 2]= src + 1;

    hpel[ 4]= tmp2t[1];
    hpel[ 5]= tmp2t[2];
    hpel[ 6]= tmp2t[1] + 1;

    hpel[ 8]= src + stride;
    hpel[ 9]= hpel[1] + 64;
    hpel[10]= hpel[8] + 1;

#define MC_STRIDE(x) (needs[x] ? 64 : stride)

    if(b==15){
        int dxy = dx / 8 + dy / 8 * 4;
        const uint8_t *src1 = hpel[dxy    ];
        const uint8_t *src2 = hpel[dxy + 1];
        const uint8_t *src3 = hpel[dxy + 4];
        const uint8_t *src4 = hpel[dxy + 5];
        int stride1 = MC_STRIDE(dxy);
        int stride2 = MC_STRIDE(dxy + 1);
        int stride3 = MC_STRIDE(dxy + 4);
        int stride4 = MC_STRIDE(dxy + 5);
        dx&=7;
        dy&=7;
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w; x++){
                dst[x]= ((8-dx)*(8-dy)*src1[x] + dx*(8-dy)*src2[x]+
                         (8-dx)*   dy *src3[x] + dx*   dy *src4[x]+32)>>6;
            }
            src1+=stride1;
            src2+=stride2;
            src3+=stride3;
            src4+=stride4;
            dst +=stride;
        }
    }else{
        const uint8_t *src1= hpel[l];
        const uint8_t *src2= hpel[r];
        int stride1 = MC_STRIDE(l);
        int stride2 = MC_STRIDE(r);
        int a= weight[((dx&7) + (8*(dy&7)))];
        int b= 8-a;
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w; x++){
                dst[x]= (a*src1[x] + b*src2[x] + 4)>>3;
            }
            src1+=stride1;
            src2+=stride2;
            dst +=stride;
        }
    }
}

void lavfsnow_pred_block(LavfSnowContext *s, uint8_t *dst, uint8_t *tmp, ptrdiff_t stride, int sx, int sy, int b_w, int b_h, const LavfBlockNode *block, int plane_index, int w, int h){
    if(block->type & BLOCK_INTRA){
        int x, y;
        const unsigned color  = block->color[plane_index];
        const unsigned color4 = color*0x01010101;
        if(b_w==32){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
                *(uint32_t*)&dst[4 + y*stride]= color4;
                *(uint32_t*)&dst[8 + y*stride]= color4;
                *(uint32_t*)&dst[12+ y*stride]= color4;
                *(uint32_t*)&dst[16+ y*stride]= color4;
                *(uint32_t*)&dst[20+ y*stride]= color4;
                *(uint32_t*)&dst[24+ y*stride]= color4;
                *(uint32_t*)&dst[28+ y*stride]= color4;
            }
        }else if(b_w==16){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
                *(uint32_t*)&dst[4 + y*stride]= color4;
                *(uint32_t*)&dst[8 + y*stride]= color4;
                *(uint32_t*)&dst[12+ y*stride]= color4;
            }
        }else if(b_w==8){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
                *(uint32_t*)&dst[4 + y*stride]= color4;
            }
        }else if(b_w==4){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
            }
        }else{
            for(y=0; y < b_h; y++){
                for(x=0; x < b_w; x++){
                    dst[x + y*stride]= color;
                }
            }
        }
    }else{
        uint8_t *src= s->last_avframe[block->ref]->data[plane_index];
        const int scale= plane_index ?  (2*s->mv_scale)>>s->chroma_h_shift : 2*s->mv_scale;
        int mx= block->mx*scale;
        int my= block->my*scale;
        const int dx= mx&15;
        const int dy= my&15;
        const int tab_index= 3 - (b_w>>2) + (b_w>>4);
        sx += (mx>>4) - (HTAPS_MAX/2-1);
        sy += (my>>4) - (HTAPS_MAX/2-1);
        src += sx + sy*stride;
        if(   (unsigned)sx >= FFMAX(w - b_w - (HTAPS_MAX-2), 0)
           || (unsigned)sy >= FFMAX(h - b_h - (HTAPS_MAX-2), 0)){
            s->vdsp.emulated_edge_mc(tmp + MB_SIZE, src,
                                     stride, stride,
                                     b_w+HTAPS_MAX-1, b_h+HTAPS_MAX-1,
                                     sx, sy, w, h);
            src= tmp + MB_SIZE;
        }

        av_assert2(s->chroma_h_shift == s->chroma_v_shift); // only one mv_scale

        av_assert2((tab_index>=0 && tab_index<4) || b_w==32);
        if(    (dx&3) || (dy&3)
            || !(b_w == b_h || 2*b_w == b_h || b_w == 2*b_h)
            || (b_w&(b_w-1))
            || b_w == 1
            || b_h == 1
            || !s->plane[plane_index].fast_mc )
            mc_block(&s->plane[plane_index], dst, src, stride, b_w, b_h, dx, dy);
        else if(b_w==32){
            int y;
            for(y=0; y<b_h; y+=16){
                s->h264qpel.put_h264_qpel_pixels_tab[0][dy+(dx>>2)](dst + y*stride, src + 3 + (y+3)*stride,stride);
                s->h264qpel.put_h264_qpel_pixels_tab[0][dy+(dx>>2)](dst + 16 + y*stride, src + 19 + (y+3)*stride,stride);
            }
        }else if(b_w==b_h)
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index  ][dy+(dx>>2)](dst,src + 3 + 3*stride,stride);
        else if(b_w==2*b_h){
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index+1][dy+(dx>>2)](dst    ,src + 3       + 3*stride,stride);
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index+1][dy+(dx>>2)](dst+b_h,src + 3 + b_h + 3*stride,stride);
        }else{
            av_assert2(2*b_w==b_h);
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index  ][dy+(dx>>2)](dst           ,src + 3 + 3*stride           ,stride);
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index  ][dy+(dx>>2)](dst+b_w*stride,src + 3 + 3*stride+b_w*stride,stride);
        }
    }
}

#define mca(dx,dy,b_w)\
static void mc_block_hpel ## dx ## dy ## b_w(uint8_t *dst, const uint8_t *src, ptrdiff_t stride, int h){\
    av_assert2(h==b_w);\
    mc_block(NULL, dst, src-(HTAPS_MAX/2-1)-(HTAPS_MAX/2-1)*stride, stride, b_w, b_w, dx, dy);\
}

mca( 0, 0,16)
mca( 8, 0,16)
mca( 0, 8,16)
mca( 8, 8,16)
mca( 0, 0,8)
mca( 8, 0,8)
mca( 0, 8,8)
mca( 8, 8,8)

av_cold int lavfsnow_common_init(AVCodecContext *avctx){
    LavfSnowContext *s = avctx->priv_data;
    int width, height;
    int i, j;

    s->avctx= avctx;
    s->max_ref_frames=1; //just make sure it's not an invalid value in case of no initial keyframe
    //s->spatial_decomposition_count = 1;

    ff_me_cmp_init(&s->mecc, avctx);
    ff_hpeldsp_init(&s->hdsp, avctx->flags);
    ff_videodsp_init(&s->vdsp, 8);
    //ff_dwt_init(&s->dwt);
    ff_h264qpel_init(&s->h264qpel, 8);

#define mcf(dx,dy)\
    s->qdsp.put_qpel_pixels_tab       [0][dy+dx/4]=\
    s->qdsp.put_no_rnd_qpel_pixels_tab[0][dy+dx/4]=\
        s->h264qpel.put_h264_qpel_pixels_tab[0][dy+dx/4];\
    s->qdsp.put_qpel_pixels_tab       [1][dy+dx/4]=\
    s->qdsp.put_no_rnd_qpel_pixels_tab[1][dy+dx/4]=\
        s->h264qpel.put_h264_qpel_pixels_tab[1][dy+dx/4];

    mcf( 0, 0)
    mcf( 4, 0)
    mcf( 8, 0)
    mcf(12, 0)
    mcf( 0, 4)
    mcf( 4, 4)
    mcf( 8, 4)
    mcf(12, 4)
    mcf( 0, 8)
    mcf( 4, 8)
    mcf( 8, 8)
    mcf(12, 8)
    mcf( 0,12)
    mcf( 4,12)
    mcf( 8,12)
    mcf(12,12)

#define mcfh(dx,dy)\
    s->hdsp.put_pixels_tab       [0][dy/4+dx/8]=\
    s->hdsp.put_no_rnd_pixels_tab[0][dy/4+dx/8]=\
        mc_block_hpel ## dx ## dy ## 16;\
    s->hdsp.put_pixels_tab       [1][dy/4+dx/8]=\
    s->hdsp.put_no_rnd_pixels_tab[1][dy/4+dx/8]=\
        mc_block_hpel ## dx ## dy ## 8;

    mcfh(0, 0)
    mcfh(8, 0)
    mcfh(0, 8)
    mcfh(8, 8)

//    dec += FFMAX(s->chroma_h_shift, s->chroma_v_shift);

    width= s->avctx->width;
    height= s->avctx->height;

    //FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->spatial_idwt_buffer, width, height * sizeof(short), fail);
    //FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->spatial_dwt_buffer,  width, height * sizeof(int),  fail); //FIXME this does not belong here
    //FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->temp_dwt_buffer,     width, sizeof(int),  fail);
    //FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->temp_idwt_buffer,    width, sizeof(short), fail);
    //FF_ALLOC_ARRAY_OR_GOTO(avctx,  s->run_buffer,          ((width + 1) >> 1), ((height + 1) >> 1) * sizeof(*s->run_buffer), fail);

    for(i=0; i<MAX_REF_FRAMES; i++) {
        for(j=0; j<MAX_REF_FRAMES; j++)
            lavfsnow_scale_mv_ref[i][j] = 256*(i+1)/(j+1);
        s->last_avframe[i] = av_frame_alloc();
        if (!s->last_avframe[i])
            goto fail;
    }

    s->mconly_avframe = av_frame_alloc();
    s->current_avframe = av_frame_alloc();
    if (!s->mconly_avframe || !s->current_avframe)
        goto fail;

    return 0;
fail:
    return AVERROR(ENOMEM);
}

int lavfsnow_common_init_after_header(LavfSnowContext *s) {
    int plane_index;
    int ret, emu_buf_size;
    AVCodecContext *avctx = s->avctx;

    if(!s->scratchbuf) {
        if ((ret = ff_get_buffer(s->avctx, s->mconly_avframe, AV_GET_BUFFER_FLAG_REF)) < 0)
            return ret;
        FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->scratchbuf, FFMAX(s->mconly_avframe->linesize[0], 2*avctx->width+256), 7*MB_SIZE, fail);
        emu_buf_size = FFMAX(s->mconly_avframe->linesize[0], 2*avctx->width+256) * (2 * MB_SIZE + HTAPS_MAX - 1);
        FF_ALLOC_OR_GOTO(avctx, s->emu_edge_buffer, emu_buf_size, fail);
    }

    if(s->mconly_avframe->format != avctx->pix_fmt) {
        av_log(avctx, AV_LOG_ERROR, "pixel format changed\n");
        return AVERROR_INVALIDDATA;
    }

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        int w= s->avctx->width;
        int h= s->avctx->height;

        if(plane_index){
            w = AV_CEIL_RSHIFT(w, s->chroma_h_shift);
            h = AV_CEIL_RSHIFT(h, s->chroma_v_shift);
        }
        s->plane[plane_index].width = w;
        s->plane[plane_index].height= h;

        /*for(level=s->spatial_decomposition_count-1; level>=0; level--){
            for(orientation=level ? 1 : 0; orientation<4; orientation++){
                LavfSubBand *b= &s->plane[plane_index].band[level][orientation];

                b->buf= s->spatial_dwt_buffer;
                b->level= level;
                b->stride= s->plane[plane_index].width << (s->spatial_decomposition_count - level);
                b->width = (w + !(orientation&1))>>1;
                b->height= (h + !(orientation>1))>>1;

                b->stride_line = 1 << (s->spatial_decomposition_count - level);
                b->buf_x_offset = 0;
                b->buf_y_offset = 0;

                if(orientation&1){
                    b->buf += (w+1)>>1;
                    b->buf_x_offset = (w+1)>>1;
                }
                if(orientation>1){
                    b->buf += b->stride>>1;
                    b->buf_y_offset = b->stride_line >> 1;
                }
                b->ibuf= s->spatial_idwt_buffer + (b->buf - s->spatial_dwt_buffer);

                if(level)
                    b->parent= &s->plane[plane_index].band[level-1][orientation];
                //FIXME avoid this realloc
                av_freep(&b->x_coeff);
                b->x_coeff=av_mallocz_array(((b->width+1) * b->height+1), sizeof(lavf_x_and_coeff));
                if (!b->x_coeff)
                    goto fail;
            }
            w= (w+1)>>1;
            h= (h+1)>>1;
        }*/
    }

    return 0;
fail:
    return AVERROR(ENOMEM);
}

int lavfsnow_get_mvs(AVCodecContext *avctx, int16_t (*mvs)[2], int8_t *refs, int w, int h)
{
    LavfSnowContext *s = avctx->priv_data;
    const int b_width  = s->b_width ;// << s->block_max_depth;
    const int b_height = s->b_height;// << s->block_max_depth;
    int x, y;

    if (w != b_width || h != b_height) {
        av_log(avctx, AV_LOG_ERROR, "mvs array dimensions mismatch %dx%d != %dx%d\n",
               w, h, b_width, b_height);
        return AVERROR(EINVAL);
    }

    for (y=0; y<h; y++) {
        for (x=0; x<w; x++) {
            LavfBlockNode *bn= &s->block[x + y*b_width];
            if (s->block_max_depth)
                bn = &s->block[2*x + 2*y*(b_width<<s->block_max_depth)];
            if (bn->type) {
                refs[x + y*w] = -1;
            } else {
                refs[x + y*w] = bn->ref;
                mvs[x + y*w][0] = bn->mx;
                mvs[x + y*w][1] = bn->my;
            }
        }
    }

    return 0;
}
