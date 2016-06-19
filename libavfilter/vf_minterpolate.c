/**
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

#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/motion_vector.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

#define NB_FRAMES 4
#define ALPHA_MAX 1024
//#define LOG2_MB_SIZE 4
#define NB_PIXEL_MVS 20

static const uint8_t obmc_raised_cos32[1024] = {
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  1,  1,  1,  2,  2,  3,  4,  4,  4,  5,  5,  5,  5,  5,  5,  5,  5,  4,  4,  4,  3,  2,  2,  1,  1,  1,  0,  0,  0,
  0,  0,  1,  2,  3,  4,  5,  7,  8, 10, 11, 12, 13, 14, 15, 15, 15, 15, 14, 13, 12, 11, 10,  8,  7,  5,  4,  3,  2,  1,  0,  0,
  0,  1,  2,  3,  5,  8, 10, 13, 16, 19, 21, 24, 26, 27, 28, 29, 29, 28, 27, 26, 24, 21, 19, 16, 13, 10,  8,  5,  3,  2,  1,  0,
  0,  1,  3,  5,  9, 12, 17, 21, 26, 30, 34, 38, 41, 44, 46, 47, 47, 46, 44, 41, 38, 34, 30, 26, 21, 17, 12,  9,  5,  3,  1,  0,
  0,  1,  4,  8, 12, 18, 24, 30, 37, 43, 50, 55, 60, 63, 66, 67, 67, 66, 63, 60, 55, 50, 43, 37, 30, 24, 18, 12,  8,  4,  1,  0,
  0,  2,  5, 10, 17, 24, 32, 41, 50, 58, 67, 74, 80, 85, 89, 90, 90, 89, 85, 80, 74, 67, 58, 50, 41, 32, 24, 17, 10,  5,  2,  0,
  0,  2,  7, 13, 21, 30, 41, 52, 63, 74, 85, 94,102,108,113,115,115,113,108,102, 94, 85, 74, 63, 52, 41, 30, 21, 13,  7,  2,  0,
  0,  3,  8, 16, 26, 37, 50, 63, 77, 90,103,114,124,132,137,140,140,137,132,124,114,103, 90, 77, 63, 50, 37, 26, 16,  8,  3,  0,
  0,  4, 10, 19, 30, 43, 58, 74, 90,106,121,134,146,155,161,164,164,161,155,146,134,121,106, 90, 74, 58, 43, 30, 19, 10,  4,  0,
  0,  4, 11, 21, 34, 50, 67, 85,103,121,138,153,166,177,184,187,187,184,177,166,153,138,121,103, 85, 67, 50, 34, 21, 11,  4,  0,
  1,  4, 12, 24, 38, 55, 74, 94,114,134,153,170,185,196,204,208,208,204,196,185,170,153,134,114, 94, 74, 55, 38, 24, 12,  4,  1,
  1,  5, 13, 26, 41, 60, 80,102,124,146,166,185,200,213,221,226,226,221,213,200,185,166,146,124,102, 80, 60, 41, 26, 13,  5,  1,
  1,  5, 14, 27, 44, 63, 85,108,132,155,177,196,213,226,235,239,239,235,226,213,196,177,155,132,108, 85, 63, 44, 27, 14,  5,  1,
  1,  5, 15, 28, 46, 66, 89,113,137,161,184,204,221,235,244,249,249,244,235,221,204,184,161,137,113, 89, 66, 46, 28, 15,  5,  1,
  1,  5, 15, 29, 47, 67, 90,115,140,164,187,208,226,239,249,254,254,249,239,226,208,187,164,140,115, 90, 67, 47, 29, 15,  5,  1,
  1,  5, 15, 29, 47, 67, 90,115,140,164,187,208,226,239,249,254,254,249,239,226,208,187,164,140,115, 90, 67, 47, 29, 15,  5,  1,
  1,  5, 15, 28, 46, 66, 89,113,137,161,184,204,221,235,244,249,249,244,235,221,204,184,161,137,113, 89, 66, 46, 28, 15,  5,  1,
  1,  5, 14, 27, 44, 63, 85,108,132,155,177,196,213,226,235,239,239,235,226,213,196,177,155,132,108, 85, 63, 44, 27, 14,  5,  1,
  1,  5, 13, 26, 41, 60, 80,102,124,146,166,185,200,213,221,226,226,221,213,200,185,166,146,124,102, 80, 60, 41, 26, 13,  5,  1,
  1,  4, 12, 24, 38, 55, 74, 94,114,134,153,170,185,196,204,208,208,204,196,185,170,153,134,114, 94, 74, 55, 38, 24, 12,  4,  1,
  0,  4, 11, 21, 34, 50, 67, 85,103,121,138,153,166,177,184,187,187,184,177,166,153,138,121,103, 85, 67, 50, 34, 21, 11,  4,  0,
  0,  4, 10, 19, 30, 43, 58, 74, 90,106,121,134,146,155,161,164,164,161,155,146,134,121,106, 90, 74, 58, 43, 30, 19, 10,  4,  0,
  0,  3,  8, 16, 26, 37, 50, 63, 77, 90,103,114,124,132,137,140,140,137,132,124,114,103, 90, 77, 63, 50, 37, 26, 16,  8,  3,  0,
  0,  2,  7, 13, 21, 30, 41, 52, 63, 74, 85, 94,102,108,113,115,115,113,108,102, 94, 85, 74, 63, 52, 41, 30, 21, 13,  7,  2,  0,
  0,  2,  5, 10, 17, 24, 32, 41, 50, 58, 67, 74, 80, 85, 89, 90, 90, 89, 85, 80, 74, 67, 58, 50, 41, 32, 24, 17, 10,  5,  2,  0,
  0,  1,  4,  8, 12, 18, 24, 30, 37, 43, 50, 55, 60, 63, 66, 67, 67, 66, 63, 60, 55, 50, 43, 37, 30, 24, 18, 12,  8,  4,  1,  0,
  0,  1,  3,  5,  9, 12, 17, 21, 26, 30, 34, 38, 41, 44, 46, 47, 47, 46, 44, 41, 38, 34, 30, 26, 21, 17, 12,  9,  5,  3,  1,  0,
  0,  1,  2,  3,  5,  8, 10, 13, 16, 19, 21, 24, 26, 27, 28, 29, 29, 28, 27, 26, 24, 21, 19, 16, 13, 10,  8,  5,  3,  2,  1,  0,
  0,  0,  1,  2,  3,  4,  5,  7,  8, 10, 11, 12, 13, 14, 15, 15, 15, 15, 14, 13, 12, 11, 10,  8,  7,  5,  4,  3,  2,  1,  0,  0,
  0,  0,  0,  1,  1,  1,  2,  2,  3,  4,  4,  4,  5,  5,  5,  5,  5,  5,  5,  5,  4,  4,  4,  3,  2,  2,  1,  1,  1,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
};

static const uint8_t obmc_linear32[1024] = {
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
};

static const uint8_t obmc_linear16[256] = {
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
};

static const uint8_t obmc_linear8[64] = {
  4, 12, 20, 28, 28, 20, 12,  4,
 12, 36, 60, 84, 84, 60, 36, 12,
 20, 60,100,140,140,100, 60, 20,
 28, 84,140,196,196,140, 84, 28,
 28, 84,140,196,196,140, 84, 28,
 20, 60,100,140,140,100, 60, 20,
 12, 36, 60, 84, 84, 60, 36, 12,
  4, 12, 20, 28, 28, 20, 12,  4,
};

static const uint8_t obmc_linear4[16]={
 16, 48, 48, 16,
 48,144,144, 48,
 48,144,144, 48,
 16, 48, 48, 16,
};

#if 0
static const uint8_t * const obmc_tab[4]= {
    obmc_raised_cos32, obmc_linear16, obmc_linear8, obmc_linear4
};
#else
static const uint8_t * const obmc_tab[4]= {
    obmc_linear32, obmc_linear16, obmc_linear8, obmc_linear4
};
#endif

enum MIMode {
    MI_MODE_DUP         = 0,
    MI_MODE_BLEND       = 1,
    MI_MODE_OBMC        = 2,
};

typedef struct Pixel {
    int16_t mv[NB_PIXEL_MVS][2];
    uint32_t weight[NB_PIXEL_MVS];
    int8_t ref[NB_PIXEL_MVS];
    int nb;
} Pixel;

typedef struct MIFrame {
    AVFrame *f;
    int16_t (*mv[2])[2];
    int8_t *ref[2];
} MIFrame;

typedef struct MIContext {
    const AVClass *class;
    enum MIMode mi_mode;
    AVRational frame_rate;
    int search_param;
    int mb_size;

    MIFrame mi_frame[NB_FRAMES];

    int64_t out_pts;

    int chroma_height;
    int chroma_width;
    int chroma_h_shift;
    int chroma_v_shift;
    int nb_planes;

    int b_width, b_height;
    int log2_mv_precission;
    int log2_mb_size;

    Pixel *pixel;

} MIContext;

#define OFFSET(x) offsetof(MIContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption minterpolate_options[] = {
    { "mi_mode", "specify the interpolation mode", OFFSET(mi_mode), AV_OPT_TYPE_INT, {.i64 = MI_MODE_DUP}, MI_MODE_DUP, MI_MODE_OBMC, FLAGS, "mi_mode" },
        CONST("dup",        "", MI_MODE_DUP,   "mi_mode"),
        CONST("blend",      "", MI_MODE_BLEND, "mi_mode"),
        CONST("obmc",       "", MI_MODE_OBMC,  "mi_mode"),
    { "fps", "specify the frame rate", OFFSET(frame_rate), AV_OPT_TYPE_RATIONAL, {.dbl = 25}, 0, INT_MAX, FLAGS },
    { "mb_size", "specify the macroblock size", OFFSET(mb_size), AV_OPT_TYPE_INT, {.i64 = 16}, 2, 16, FLAGS },
    { "search_param", "specify search parameter", OFFSET(search_param), AV_OPT_TYPE_INT, {.i64 = 7}, 4, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(minterpolate);

static int query_formats(AVFilterContext *ctx) 
{
    //FIXME pix_fmts

    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P,
        AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUVJ411P,
        AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA444P,
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static int config_input(AVFilterLink *inlink)
{
    MIContext *mi_ctx = inlink->dst->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    const int height = inlink->h;
    const int width  = inlink->w;

    mi_ctx->chroma_height = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    mi_ctx->chroma_width = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);

    avcodec_get_chroma_sub_sample(inlink->format, &mi_ctx->chroma_h_shift, &mi_ctx->chroma_v_shift); //TODO remove

    mi_ctx->nb_planes = av_pix_fmt_count_planes(inlink->format);

    mi_ctx->log2_mb_size = ff_log2(mi_ctx->mb_size);
    mi_ctx->log2_mv_precission = 2;

    mi_ctx->b_width  = AV_CEIL_RSHIFT(width,  mi_ctx->log2_mb_size);
    mi_ctx->b_height = AV_CEIL_RSHIFT(height, mi_ctx->log2_mb_size);

    if (!(mi_ctx->pixel = av_mallocz_array(width * height, sizeof(*mi_ctx->pixel)))) {
        return AVERROR(ENOMEM);
    }

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    MIContext *mi_ctx = outlink->src->priv;

    outlink->frame_rate = mi_ctx->frame_rate;
    outlink->time_base  = av_inv_q(mi_ctx->frame_rate);

    av_log(0, 0, "FPS %d/%d\n", mi_ctx->frame_rate.num, mi_ctx->frame_rate.den);
    av_log(0, 0, "MB Size: %d, SubPixel Accuracy: %d\n", 1 << mi_ctx->log2_mb_size, 1 << mi_ctx->log2_mv_precission);

    return 0;
}

static int get_roughness(MIContext *mi_ctx, int dir, int mb_x, int mb_y) {
    int x, y;
    int ref0 = mi_ctx->mi_frame[2 - dir].ref[dir][mb_y * mi_ctx->b_width + mb_x];
    int mv_x0 = mi_ctx->mi_frame[2 - dir].mv[dir][mb_y * mi_ctx->b_width + mb_x][0];
    int mv_y0 = mi_ctx->mi_frame[2 - dir].mv[dir][mb_y * mi_ctx->b_width + mb_x][1];
    int roughness = INT_MAX;

    av_assert1(ref0 == 0);

    for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++) {
        for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
            int d, ref, mv_x, mv_y;
            int dir1 = dir;
            if (x == mb_x && y == mb_y) {
                dir1 = 1 - dir;
            }
            ref = mi_ctx->mi_frame[2 - dir].ref[dir1][x + y * mi_ctx->b_width];
            if (ref < 0)
                continue;
            mv_x = mi_ctx->mi_frame[2 - dir].mv[dir1][x + y * mi_ctx->b_width][0];
            mv_y = mi_ctx->mi_frame[2 - dir].mv[dir1][x + y * mi_ctx->b_width][1];
            if (dir != dir1) {
                mv_x = -mv_x;
                mv_y = -mv_y;
            }
            d = FFABS(mv_x0 - mv_x) + FFABS(mv_y0 - mv_y);
            roughness = FFMIN(roughness, d);
        }
    }
    return roughness;
}

static void fill_pixels(MIContext *mi_ctx, int alpha)
{
    int x, y;
    int width = mi_ctx->mi_frame[0].f->width;
    int height = mi_ctx->mi_frame[0].f->height;
    int mb_y, mb_x, dir;

    for (y=0; y < height; y++){
        for (x=0; x < width; x++){
            Pixel *p = &mi_ctx->pixel[x + y * width];
            p->weight[0] = ALPHA_MAX - alpha;
            p->ref[0] = 1;
            p->mv[0][0] = 0;
            p->mv[0][1] = 0;
            p->weight[1] = alpha;
            p->ref[1] = 2;
            p->mv[1][0] = 0;
            p->mv[1][1] = 0;
            p->nb = 2;
//            p->nb = 0; // no blending / filling
        }
    }

    //for (dir = 0; dir < 2; dir++)
    //    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
    //        for (mb_x=0; mb_x < mi_ctx->b_width; mb_x++)
    //            temporal_diff += get_temporal_mv_difference(mi_ctx, dir, mb_x, mb_y, &nomv);

    // no need, each block will have vector, atleast in ESA
    //if (mi_ctx->b_height * mi_ctx->b_width * 5 / 8 > nomv)
    for (dir = 0; dir < 2; dir++) {
        int a = dir ? alpha : (ALPHA_MAX - alpha);

        for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++) {
            for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                int ref = mi_ctx->mi_frame[2 - dir].ref[dir][mb_x + mb_y * mi_ctx->b_width];
                int mv_x = mi_ctx->mi_frame[2 - dir].mv[dir][mb_x + mb_y * mi_ctx->b_width][0];
                int mv_y = mi_ctx->mi_frame[2 - dir].mv[dir][mb_x + mb_y * mi_ctx->b_width][1];
                int start_x, start_y, end_x, end_y;

                if(ref < 0)
                    continue;

                if (get_roughness(mi_ctx, dir, mb_x, mb_y) > 32)
                    continue;

                //av_assert0(ref == 0);

                start_x = (mb_x << mi_ctx->log2_mb_size) - (1 << mi_ctx->log2_mb_size) / 2 + mv_x * a / (ALPHA_MAX << mi_ctx->log2_mv_precission);
                start_y = (mb_y << mi_ctx->log2_mb_size) - (1 << mi_ctx->log2_mb_size) / 2 + mv_y * a / (ALPHA_MAX << mi_ctx->log2_mv_precission);

                start_x = av_clip(start_x, 0, width - 1);
                start_y = av_clip(start_y, 0, height - 1);

                end_x = av_clip(start_x + (2 << mi_ctx->log2_mb_size), 0, width - 1);
                end_y = av_clip(start_y + (2 << mi_ctx->log2_mb_size), 0, height - 1);

                if (dir) {
                    mv_x = -mv_x;
                    mv_y = -mv_y;
                }

                for (y = start_y; y < end_y; y++) {
                    int y_min = -y << mi_ctx->log2_mv_precission;
                    int y_max = (height - y - 1) << mi_ctx->log2_mv_precission;
                    for (x = start_x; x < end_x; x++) {
                        int x_min = -x << mi_ctx->log2_mv_precission;
                        int x_max = (width - x - 1) << mi_ctx->log2_mv_precission;
                        int obmc_weight = obmc_tab[4 - mi_ctx->log2_mb_size][(x - start_x) + ((y - start_y) << (mi_ctx->log2_mb_size + 1))];
                        Pixel *p = &mi_ctx->pixel[x + y * width];

                        if (p->nb + 1 >= NB_PIXEL_MVS) //FIXME discrad the vector of lowest weight
                            continue;

                        p->ref[p->nb] = 1;
                        p->weight[p->nb] = obmc_weight * (ALPHA_MAX - alpha);
                        p->mv[p->nb][0] = av_clip((mv_x * alpha) / ALPHA_MAX, x_min, x_max);
                        p->mv[p->nb][1] = av_clip((mv_y * alpha) / ALPHA_MAX, y_min, y_max);
                        p->nb++;

                        p->ref[p->nb] = 2;
                        p->weight[p->nb] = obmc_weight * alpha;
                        p->mv[p->nb][0] = av_clip(-mv_x * (ALPHA_MAX - alpha) / ALPHA_MAX, x_min, x_max);
                        p->mv[p->nb][1] = av_clip(-mv_y * (ALPHA_MAX - alpha) / ALPHA_MAX, y_min, y_max);
                        p->nb++;
                    }
                }
            }
        }
    }
}

static void interpolate_pixels(MIContext *mi_ctx, AVFrame *out)
{
    int x, y, plane;

    for (plane = 0; plane < mi_ctx->nb_planes; plane++){
        int width = out->width;
        int height = out->height;

        if (plane == 1 || plane == 2) {
            width = mi_ctx->chroma_width;
            height = mi_ctx->chroma_height;
        }

        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++) {
                int i;
                int weight_sum = 0;
                int v = 0;
                Pixel *pixel;
                if (plane == 1 || plane == 2) //FIXME optimize
                    pixel = &mi_ctx->pixel[(x << mi_ctx->chroma_h_shift) + (y << mi_ctx->chroma_v_shift) * out->width];
                else
                    pixel = &mi_ctx->pixel[x + y * out->width];

                for(i = 0; i < pixel->nb; i++) {
                    weight_sum += pixel->weight[i];
                }
                if(!weight_sum)
                    weight_sum = 1; //FIXME this should never occur

                for(i = 0; i < pixel->nb; i++) {
                    int is_chroma = plane == 1 || plane == 2; //FIXME
                    int mv_x = x + (pixel->mv[i][0] >> (mi_ctx->log2_mv_precission + is_chroma));
                    int mv_y = y + (pixel->mv[i][1] >> (mi_ctx->log2_mv_precission + is_chroma));
                    MIFrame *mi_frame = &mi_ctx->mi_frame[pixel->ref[i]];

                    v += pixel->weight[i] * mi_frame->f->data[plane][mv_x + mv_y * mi_frame->f->linesize[plane]];
                }

                out->data[plane][x + y * out->linesize[plane]] = ROUNDED_DIV(v, weight_sum);
            }
        }
    }
}

static void interpolate(AVFilterLink *inlink, AVFrame *out)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi_ctx = ctx->priv;
    AVFrame *frame;
    int plane, alpha;
    int64_t pts;

    pts = av_rescale(out->pts, (int64_t) ALPHA_MAX * outlink->time_base.num * inlink->time_base.den,
                               (int64_t)             outlink->time_base.den * inlink->time_base.num);

    alpha = (pts - mi_ctx->mi_frame[1].f->pts * ALPHA_MAX) / (mi_ctx->mi_frame[2].f->pts - mi_ctx->mi_frame[1].f->pts);
    alpha = av_clip(alpha, 0, ALPHA_MAX);

    switch(mi_ctx->mi_mode) {
    case MI_MODE_DUP:
        pts = av_rescale_q(out->pts, outlink->time_base, inlink->time_base);
        if (FFABS(pts - mi_ctx->mi_frame[1].f->pts) < FFABS(pts - mi_ctx->mi_frame[2].f->pts))
            frame = mi_ctx->mi_frame[1].f;
        else
            frame = mi_ctx->mi_frame[2].f;

        av_frame_copy(out, frame);

        break;
    case MI_MODE_BLEND:
        for (plane=0; plane < mi_ctx->nb_planes; plane++) {
            int x, y;
            int width = out->width;
            int height = out->height;

            if (plane == 1 || plane == 2) {
                width = mi_ctx->chroma_width;
                height = mi_ctx->chroma_height;
            }

            for (y = 0; y < height; y++) {
                for (x = 0; x < width; x++) {
                    out->data[plane][x + y * out->linesize[plane]] =
                                      alpha  * mi_ctx->mi_frame[2].f->data[plane][x + y * mi_ctx->mi_frame[2].f->linesize[plane]] +
                        ((ALPHA_MAX - alpha) * mi_ctx->mi_frame[1].f->data[plane][x + y * mi_ctx->mi_frame[1].f->linesize[plane]] + 512) >> 10;
                }
            }
        }

        break;
    case MI_MODE_OBMC:
        fill_pixels(mi_ctx, alpha);
        interpolate_pixels(mi_ctx, out);

        break;
    }

}

static int inject_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MIContext *mi_ctx = ctx->priv;
    MIFrame mi_frame_tmp, *mi_frame;

    av_frame_free(&mi_ctx->mi_frame[0].f);
    mi_frame_tmp = mi_ctx->mi_frame[0];
    memmove(&mi_ctx->mi_frame[0], &mi_ctx->mi_frame[1], sizeof(mi_ctx->mi_frame[0]) * (NB_FRAMES - 1));
    mi_ctx->mi_frame[NB_FRAMES - 1] = mi_frame_tmp;
    mi_ctx->mi_frame[NB_FRAMES - 1].f = frame;
    mi_frame = &mi_ctx->mi_frame[NB_FRAMES - 1];

    if (mi_ctx->mi_mode == MI_MODE_OBMC) {

        AVFrameSideData *sd = av_frame_get_side_data(mi_frame->f, AV_FRAME_DATA_MOTION_VECTORS);
        int i, mb_y, mb_x, dir;

        for (dir = 0; dir < 2; dir++) {
            if (!mi_frame->mv[dir])
                mi_frame->mv[dir] = av_malloc(mi_ctx->b_width * mi_ctx->b_height * sizeof(*mi_frame->mv[0]));
            if (!mi_frame->ref[dir])
                mi_frame->ref[dir] = av_malloc(mi_ctx->b_width * mi_ctx->b_height * sizeof(*mi_frame->ref[0]));
            if (!mi_frame->mv[dir] || !mi_frame->ref[0])
                return AVERROR(ENOMEM);
        }

        /* this is temp approach to extract mvs. they will be generated by directly calling functions */

        //set to defaults
        for (i = 0; i < mi_ctx->b_height * mi_ctx->b_width; i++) {
            for (dir = 0; dir < 2; dir++) {
                mi_frame->ref[dir][i] = -1; // no motion vectors
                mi_frame->mv[dir][i][0] = 0;
                mi_frame->mv[dir][i][1] = 0;
            }
        }

        if (!sd) {
            printf("no motion vectors, returning\n");
            return 0;
        }
        AVMotionVector *mvs = (AVMotionVector *) sd->data;

        for (i = 0; i < sd->size / sizeof(*mvs); i++) {
            AVMotionVector *mv = &mvs[i];
            int dir = mv->source > 0, j;

            mb_x = (mv->dst_x /*- mv->w / 2*/) / (1 << mi_ctx->log2_mb_size);
            mb_y = (mv->dst_y /*- mv->h / 2*/) / (1 << mi_ctx->log2_mb_size);

            mb_x = av_clip(mb_x, 0, mi_ctx->b_width - 1);
            mb_y = av_clip(mb_y, 0, mi_ctx->b_height - 1);
            j = mb_x + mi_ctx->b_width * mb_y;

            mi_frame->mv[dir][j][0] = (mv->src_x - mv->dst_x) << mi_ctx->log2_mv_precission; //TODO qpel
            mi_frame->mv[dir][j][1] = (mv->src_y - mv->dst_y) << mi_ctx->log2_mv_precission;
            mi_frame->ref[dir][j] = 0;
        }
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi_ctx = ctx->priv;
    int ret;

    av_assert0(frame->pts != AV_NOPTS_VALUE); //FIXME

    if (!mi_ctx->mi_frame[NB_FRAMES - 1].f || frame->pts < mi_ctx->mi_frame[NB_FRAMES - 1].f->pts) {
        av_log(ctx, AV_LOG_VERBOSE, "Initializing out pts from input pts %"PRId64"\n", frame->pts);
        mi_ctx->out_pts = av_rescale_q(frame->pts, inlink->time_base, outlink->time_base);
    }

    if (!mi_ctx->mi_frame[NB_FRAMES - 1].f)
        if (ret = inject_frame(inlink, av_frame_clone(frame)))
            return ret;

    if (ret = inject_frame(inlink, frame))
        return ret;

    if (!mi_ctx->mi_frame[0].f)
        return 0;

    for (;;) {
        AVFrame *out;

        if (av_compare_ts(mi_ctx->mi_frame[NB_FRAMES / 2].f->pts, inlink->time_base, mi_ctx->out_pts, outlink->time_base) < 0)
            break;

        if (!(out = ff_get_video_buffer(ctx->outputs[0], inlink->w, inlink->h)))
            return AVERROR(ENOMEM);

        av_frame_copy_props(out, mi_ctx->mi_frame[NB_FRAMES - 1].f); //FIXME this also copy sidedata
        out->pts = mi_ctx->out_pts++;

        interpolate(inlink, out);

        if ((ret = ff_filter_frame(ctx->outputs[0], out)) < 0)
            return ret;
    }
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    //FIXME
}

static const AVFilterPad minterpolate_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
        .config_props  = config_input,
    },
    { NULL }
};

static const AVFilterPad minterpolate_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
    { NULL }
};

AVFilter ff_vf_minterpolate = {
    .name          = "minterpolate",
    .description   = NULL_IF_CONFIG_SMALL("Frame rate conversion using Motion Interpolation."),
    .priv_size     = sizeof(MIContext),
    .priv_class    = &minterpolate_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = minterpolate_inputs,
    .outputs       = minterpolate_outputs,
};
