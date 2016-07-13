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

//TESTS
#define DEBUG_CLUSTERING 0
#define CACHE_MVS 1
#define EXPORT_MVS 0
//

#define NB_FRAMES 4
#define NB_PIXEL_MVS 20
#define NB_CLUSTER 100

#define ALPHA_MAX 1024
#define CLUSTER_THRESHOLD 8

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

static const uint8_t obmc_linear4[16] = {
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
    MI_MODE_PAPER2      = 3,
};

typedef struct MICluster {
    int64_t sum[2];
    int nb;
} MICluster;

typedef struct MIBlock {
    int cluster;
    int16_t mv[2][2];
    int sb;
    struct MIBlock *mi_block;
} MIBlock;

typedef struct Pixel {
    int16_t mv[NB_PIXEL_MVS][2];
    uint32_t weight[NB_PIXEL_MVS];
    int8_t ref[NB_PIXEL_MVS];
    int nb;
} Pixel;

typedef struct MIFrame {
    AVFrame *avf;
    MIBlock *mi_block;
} MIFrame;

typedef struct MIContext {
    const AVClass *class;
    enum MIMode mi_mode;
    AVRational frame_rate;
    int search_param;
    int mb_size;

    MIFrame mi_frame[NB_FRAMES];
    MICluster mi_cluster[NB_CLUSTER];
    MIFrame out_frame;

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
    { "mi_mode", "specify the interpolation mode", OFFSET(mi_mode), AV_OPT_TYPE_INT, {.i64 = MI_MODE_DUP}, MI_MODE_DUP, 3 /*FIXME*/, FLAGS, "mi_mode" },
        CONST("dup",        "", MI_MODE_DUP,   "mi_mode"),
        CONST("blend",      "", MI_MODE_BLEND, "mi_mode"),
        CONST("obmc",       "", MI_MODE_OBMC,  "mi_mode"),
    { "fps", "specify the frame rate", OFFSET(frame_rate), AV_OPT_TYPE_RATIONAL, {.dbl = 25}, 0, INT_MAX, FLAGS },
    { "mb_size", "specify the macroblock size", OFFSET(mb_size), AV_OPT_TYPE_INT, {.i64 = 16}, 2, 16, FLAGS },
    { "search_param", "specify search parameter", OFFSET(search_param), AV_OPT_TYPE_INT, {.i64 = 7}, 4, INT_MAX, FLAGS },
    { "subpel", "specify motion estimation sub-pixel accuracy (log2 scale)", OFFSET(log2_mv_precission), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS },
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
    int i;

    mi_ctx->chroma_height = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    mi_ctx->chroma_width = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);

    avcodec_get_chroma_sub_sample(inlink->format, &mi_ctx->chroma_h_shift, &mi_ctx->chroma_v_shift); //TODO remove

    mi_ctx->nb_planes = av_pix_fmt_count_planes(inlink->format);

    mi_ctx->log2_mb_size = ff_log2(mi_ctx->mb_size); //FIXME round off first?
    mi_ctx->mb_size = 1 << mi_ctx->log2_mb_size;

    mi_ctx->b_width  = AV_CEIL_RSHIFT(width,  mi_ctx->log2_mb_size);
    mi_ctx->b_height = AV_CEIL_RSHIFT(height, mi_ctx->log2_mb_size);

    for (i = 0; i < NB_FRAMES; i++) {
        MIFrame *mi_frame = &mi_ctx->mi_frame[i];
        mi_frame->mi_block = av_mallocz_array(mi_ctx->b_width * mi_ctx->b_height, sizeof(MIBlock));
        if (!mi_frame->mi_block)
            return AVERROR(ENOMEM);
    }

    if (!(mi_ctx->pixel = av_mallocz_array(width * height, sizeof(Pixel)))) {
        return AVERROR(ENOMEM);
    }

    if (mi_ctx->mi_mode == MI_MODE_PAPER2) {
        if (!(mi_ctx->out_frame.mi_block = av_mallocz_array(mi_ctx->b_width * mi_ctx->b_height, sizeof(MIBlock))))
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
    av_log(0, 0, "MB Size: %d, SubPixel Accuracy: %d\n", mi_ctx->mb_size, 1 << mi_ctx->log2_mv_precission);

    return 0;
}

static int get_roughness(MIContext *mi_ctx, int dir, int mb_x, int mb_y)
{
    int x, y;
    int mv_x0 = mi_ctx->mi_frame[2 - dir].mi_block[mb_y * mi_ctx->b_width + mb_x].mv[dir][0];
    int mv_y0 = mi_ctx->mi_frame[2 - dir].mi_block[mb_y * mi_ctx->b_width + mb_x].mv[dir][1];
    int roughness = INT_MAX;

    for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++) {
        for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
            int d, mv_x, mv_y;
            int dir1 = dir;
            if (x == mb_x && y == mb_y) {
                dir1 = 1 - dir;
            }
            mv_x = mi_ctx->mi_frame[2 - dir].mi_block[x + y * mi_ctx->b_width].mv[dir1][0];
            mv_y = mi_ctx->mi_frame[2 - dir].mi_block[x + y * mi_ctx->b_width].mv[dir1][1];
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

static void fill_pixels_vsb(MIContext *mi_ctx, int mb_x, int mb_y, int log2_mb_size, int mv_x, mv_y, int alpha)
{
    int x, y;
    int width = mi_ctx->mi_frame[0].avf->width;
    int height = mi_ctx->mi_frame[0].avf->height;
    int start_x, start_y, end_x, end_y;
    int a = ALPHA_MAX - alpha;

    //if (get_roughness(mi_ctx, dir, mb_x, mb_y) > 32)
    //    continue;

    start_x = (mb_x << log2_mb_size) - (1 << (log2_mb_size - 1)) + mv_x * a / (ALPHA_MAX << mi_ctx->log2_mv_precission);
    start_y = (mb_y << log2_mb_size) - (1 << (log2_mb_size - 1)) + mv_y * a / (ALPHA_MAX << mi_ctx->log2_mv_precission);

    start_x = av_clip(start_x, 0, width - 1);
    start_y = av_clip(start_y, 0, height - 1);

    end_x = av_clip(start_x + (2 << log2_mb_size), 0, width - 1);
    end_y = av_clip(start_y + (2 << log2_mb_size), 0, height - 1);

    for (y = start_y; y < end_y; y++) {
        int y_min = -y << mi_ctx->log2_mv_precission;
        int y_max = (height - y - 1) << mi_ctx->log2_mv_precission;
        for (x = start_x; x < end_x; x++) {
            int x_min = -x << mi_ctx->log2_mv_precission;
            int x_max = (width - x - 1) << mi_ctx->log2_mv_precission;
            int obmc_weight = obmc_tab[4 - log2_mb_size][(x - start_x) + ((y - start_y) << (log2_mb_size + 1))];
            Pixel *p = &mi_ctx->pixel[x + y * width];

            if (p->nb + 1 >= NB_PIXEL_MVS) //FIXME discard the vector of lowest weight
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

/* adaptive variable-size overlapped block motion compensation */
static void avsobmc(MIContext *mi_ctx, int alpha)
{
    int x, y;
    int width = mi_ctx->mi_frame[0].avf->width;
    int height = mi_ctx->mi_frame[0].avf->height;
    int mb_y, mb_x;

    for (y = 0; y < height; y++){
        for (x = 0; x < width; x++){
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
            // p->nb = 0; // no blending / filling
        }
    }

    // no need, each block will have vector, atleast in ESA
    //if (mi_ctx->b_height * mi_ctx->b_width * 5 / 8 > nomv)

    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++) {
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            MIBlock *mi_block = &mi_ctx->out_frame.mi_block[mb_x + mb_y * mi_ctx->b_width];
            int mv_x, mv_y, n = 1;

            if (mi_block->sb) {
                for (y = 0; y < 2; y++)
                    for (x = 0; x < 2; x++) {
                        MIBlock *sb = &mi_block->mi_block[x + y * 2]; //TODO set correct parent mi_block & n
                        mv_x = sb->mv[0][0];
                        mv_y = sb->mv[0][1];
                        fill_pixels_vsb(mi_ctx, (mb_x << n) + x, (mb_y << n) + y, mi_ctx->log2_mb_size - n, mv_x, mv_y, alpha);
                    }
            } else {
                mv_x = mi_block->mv[0][0];
                mv_y = mi_block->mv[0][1];
                fill_pixels_vsb(mi_ctx, mb_x, mb_y, mi_ctx->log2_mb_size, mv_x, mv_y, alpha);
            }

        }
    }
}

static void fill_pixels(MIContext *mi_ctx, int alpha)
{
    int x, y;
    int width = mi_ctx->mi_frame[0].avf->width;
    int height = mi_ctx->mi_frame[0].avf->height;
    int mb_y, mb_x, dir;

    for (y = 0; y < height; y++){
        for (x = 0; x < width; x++){
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
            // p->nb = 0; // no blending / filling
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
                int mv_x = mi_ctx->mi_frame[2 - dir].mi_block[mb_x + mb_y * mi_ctx->b_width].mv[dir][0];
                int mv_y = mi_ctx->mi_frame[2 - dir].mi_block[mb_x + mb_y * mi_ctx->b_width].mv[dir][1];
                int start_x, start_y, end_x, end_y;

                if (get_roughness(mi_ctx, dir, mb_x, mb_y) > 32)
                    continue;

                start_x = (mb_x << mi_ctx->log2_mb_size) - (mi_ctx->mb_size) / 2 + mv_x * a / (ALPHA_MAX << mi_ctx->log2_mv_precission);
                start_y = (mb_y << mi_ctx->log2_mb_size) - (mi_ctx->mb_size) / 2 + mv_y * a / (ALPHA_MAX << mi_ctx->log2_mv_precission);

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
                    int is_chroma = plane == 1 || plane == 2; //FIXME
                    int mv_x = x + (pixel->mv[i][0] >> (mi_ctx->log2_mv_precission + is_chroma));
                    int mv_y = y + (pixel->mv[i][1] >> (mi_ctx->log2_mv_precission + is_chroma));
                    MIFrame *mi_frame = &mi_ctx->mi_frame[pixel->ref[i]];

                    v += pixel->weight[i] * mi_frame->avf->data[plane][mv_x + mv_y * mi_frame->avf->linesize[plane]];
                    weight_sum += pixel->weight[i];
                }

                if(!weight_sum)
                    weight_sum = 1; //FIXME this should never occur

                out->data[plane][x + y * out->linesize[plane]] = ROUNDED_DIV(v, weight_sum);
            }
        }
    }
}

static int cls;
static int cls_max;
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

    alpha = (pts - mi_ctx->mi_frame[1].avf->pts * ALPHA_MAX) / (mi_ctx->mi_frame[2].avf->pts - mi_ctx->mi_frame[1].avf->pts);
    alpha = av_clip(alpha, 0, ALPHA_MAX);

    switch(mi_ctx->mi_mode) {
        case MI_MODE_DUP:
            pts = av_rescale_q(out->pts, outlink->time_base, inlink->time_base);
            if (FFABS(pts - mi_ctx->mi_frame[1].avf->pts) < FFABS(pts - mi_ctx->mi_frame[2].avf->pts))
                frame = mi_ctx->mi_frame[1].avf;
            else
                frame = mi_ctx->mi_frame[2].avf;

            av_frame_copy(out, frame);

            break;
        case MI_MODE_BLEND:
            for (plane = 0; plane < mi_ctx->nb_planes; plane++) {
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
                                          alpha  * mi_ctx->mi_frame[2].avf->data[plane][x + y * mi_ctx->mi_frame[2].avf->linesize[plane]] +
                            ((ALPHA_MAX - alpha) * mi_ctx->mi_frame[1].avf->data[plane][x + y * mi_ctx->mi_frame[1].avf->linesize[plane]] + 512) >> 10;
                    }
                }
            }

            break;
        case MI_MODE_OBMC:
            fill_pixels(mi_ctx, alpha);
            interpolate_pixels(mi_ctx, out);

            break;
        case MI_MODE_PAPER2:
        {
            int mb_x, mb_y, x, y, dx, dy;
            MIBlock *mi_block;
            MIFrame *out_frame = &mi_ctx->out_frame;

            av_frame_copy(out, mi_ctx->mi_frame[2].avf);

        #if DEBUG_CLUSTERING

            for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
                for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                    int i = 0; int flag = 0;
                    mi_block = &out_frame->mi_block[mb_x + mb_y * mi_ctx->b_width];
                    for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++)
                        for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
                            if ((x - mb_x) && (y - mb_y))
                                continue;

                            if (mi_block->cluster != out_frame->mi_block[x + y * mi_ctx->b_width].cluster) {
                                dx = x - mb_x;
                                dy = y - mb_y;
                                if (!dx && (mi_block->cluster == out_frame->mi_block[x + (mb_y - dy) * mi_ctx->b_width].cluster)) {
                                    if ((mi_block->cluster == out_frame->mi_block[x - 1 + (mb_y - dy) * mi_ctx->b_width].cluster) ||
                                        (mi_block->cluster == out_frame->mi_block[x + 1 + (mb_y - dy) * mi_ctx->b_width].cluster))
                                        flag = 1;
                                } else if (!dy && (mi_block->cluster == out_frame->mi_block[(mb_x - dx) + y * mi_ctx->b_width].cluster)) {
                                    if ((mi_block->cluster == out_frame->mi_block[(mb_x - dx) + (y - 1) * mi_ctx->b_width].cluster) ||
                                        (mi_block->cluster == out_frame->mi_block[(mb_x - dx) + (y - 1) * mi_ctx->b_width].cluster))
                                        flag = 1;
                                }
                                i++;
                            }
                        }

                #if 0
                    if (i == 4) {
                        av_log(0, 0, "isolated block\n");
                        //hack
                        int c = -1;
                        for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++)
                            for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
                                if ((x - mb_x) && (y - mb_y))
                                    continue;
                                if (c < 0)
                                    c = out_frame->mi_block[x + y * mi_ctx->b_width].cluster;
                                else if (c != out_frame->mi_block[x + y * mi_ctx->b_width].cluster) {
                                    c = -1;
                                    break;
                                }
                            }
                        if (c > 0) {
                            av_log(0, 0, "refined\n");
                            mi_block->cluster = c;
                            flag = 1;
                        } else
                            continue;
                    } //else if (i >= 1 && i < 5) {

                    if (flag) {
                    //if (mi_block->cluster == cls) {
                        //av_log(0, 0, "eligible: dx: %d, dy: %d\n", mi_block->mv[0][0], mi_block->mv[0][1]);
                        for (y = mb_y * mi_ctx->mb_size; y < (mb_y + 1) * mi_ctx->mb_size; y++)
                            for (x = mb_x * mi_ctx->mb_size; x < (mb_x + 1) * mi_ctx->mb_size; x++) {
                                out->data[0][x + y * out->linesize[0]] += 20;//FFMIN(255, FFABS(mi_block->mv[0][0]) + FFABS(mi_block->mv[0][1]));
                                //out->data[0][x + y * out->linesize[0]] += 20;
                            }
                    }
                    else {
                        //set MVs to 0
                        mi_block->mv[0][0] = 0;
                        mi_block->mv[0][1] = 0;
                    }
                #else
                    if (mi_block->cluster > 0) {
                    //if (mi_block->cluster == cls) {
                        for (y = mb_y * mi_ctx->mb_size; y < (mb_y + 1) * mi_ctx->mb_size; y++)
                            for (x = mb_x * mi_ctx->mb_size; x < (mb_x + 1) * mi_ctx->mb_size; x++) {
                                if (mi_block->sb) {
                                    out->data[0][x + y * out->linesize[0]] = 255;
                                } else {
                                    int ins = mi_block->cluster % 5;
                                    //out->data[0][x + y * out->linesize[0]] /= (ins < 2 ? 2 : ins);
                                    mi_block->mv[0][0] = 0;
                                    mi_block->mv[0][1] = 0;
                                }
                            }
                        //mi_block->mv[0][0] = 0;
                        //mi_block->mv[0][1] = 0;
                    } else {
                        //set MVs to 0
                        mi_block->mv[0][0] = 0;
                        mi_block->mv[0][1] = 0;
                    }
                #endif
                }

            cls++;
            if (cls >= cls_max)
                cls = 1;
            for (int c = 0; c < cls_max; c++) {
                if (mi_ctx->mi_cluster[cls].nb < 2) {
                    cls++;
                    if (cls >= cls_max) {
                        cls = 0;
                        break;
                    }
                }
            }

        #else
            avsobmc(mi_ctx, alpha);
            interpolate_pixels(mi_ctx, out);
        #endif

        #if EXPORT_MVS
            /* export MVs */
            AVFrameSideData *sd = av_frame_new_side_data(out, AV_FRAME_DATA_MOTION_VECTORS, 4 * mi_ctx->b_width * mi_ctx->b_height * sizeof(AVMotionVector));
            AVMotionVector *mv;
            int count = 0;

            //if (!sd) return AVERROR(ENOMEM);
            mv = (AVMotionVector *) sd->data;
            for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
                for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                    mi_block = &out_frame->mi_block[mb_x + mb_y * mi_ctx->b_width];

                    if (mi_block->sb) {
                        int mb_size = mi_ctx->mb_size / 2;
                        for (y = 0; y < 2; y++)
                            for (x = 0; x < 2; x++) {
                                mv->source = -1;
                                mv->w = mb_size;
                                mv->h = mb_size;
                                mv->src_x = mb_size / 2 * (x ? 3 : 1) + (mb_x << mi_ctx->log2_mb_size) + mi_block->mi_block[x + y * 2].mv[0][0];
                                mv->src_y = mb_size / 2 * (y ? 3 : 1) + (mb_y << mi_ctx->log2_mb_size) + mi_block->mi_block[x + y * 2].mv[0][1];
                                mv->dst_x = mb_size / 2 * (x ? 3 : 1) + (mb_x << mi_ctx->log2_mb_size);
                                mv->dst_y = mb_size / 2 * (y ? 3 : 1) + (mb_y << mi_ctx->log2_mb_size);
                                mv++;
                                count++;
                            }
                    } else {
                        mv->source = -1;
                        mv->w = mi_ctx->mb_size;
                        mv->h = mi_ctx->mb_size;
                        mv->src_x = mi_ctx->mb_size / 2 + (mb_x << mi_ctx->log2_mb_size) + mi_block->mv[0][0];
                        mv->src_y = mi_ctx->mb_size / 2 + (mb_y << mi_ctx->log2_mb_size) + mi_block->mv[0][1];
                        mv->dst_x = mi_ctx->mb_size / 2 + (mb_x << mi_ctx->log2_mb_size);
                        mv->dst_y = mi_ctx->mb_size / 2 + (mb_y << mi_ctx->log2_mb_size);
                        mv++;
                        count++;
                    }
            }

            for (; count < 4 * mi_ctx->b_width * mi_ctx->b_height; count++) {
                mv->source = -1;
                mv->w = mi_ctx->mb_size;
                mv->h = mi_ctx->mb_size;
                mv->src_x = 0;
                mv->src_y = 0;
                mv->dst_x = 0;
                mv->dst_y = 0;
                mv++;
            }
        #endif
        }
            break;
    }

}

#define MIN_VECTOR(dx0, dy0, dx1, dy1)\
do {\
    if (dx1 * dx1 + dy1 * dy1 < dx0 * dx0 + dy0 * dy0) {\
        dx0 = dx1;\
        dy0 = dy1;\
    }\
} while(0)

static uint64_t get_mad(MIContext *mi_ctx, int x, int y, int mv_x, int mv_y)
{
    uint8_t *buf_prev = mi_ctx->mi_frame[1].avf->data[0];
    uint8_t *buf_next = mi_ctx->mi_frame[2].avf->data[0];
    int linesize = mi_ctx->mi_frame[0].avf->linesize[0];
    int64_t mad = 0;
    int i, j;

    buf_prev += (y + mv_y) * linesize;
    buf_next += (y - mv_y) * linesize;

    av_assert0(linesize == mi_ctx->mi_frame[2].avf->linesize[0]); //TEST

    //re-clip again since it's bilateral
    if (x - mv_x + mi_ctx->mb_size >= mi_ctx->mi_frame[1].avf->width || x - mv_x < 0 ||
        y - mv_y + mi_ctx->mb_size >= mi_ctx->mi_frame[1].avf->height || y - mv_y < 0)
        return UINT64_MAX; //TODO do one directional ME.

    for (j = 0; j < mi_ctx->mb_size; j++)
        for (i = 0; i < mi_ctx->mb_size; i++)
            mad += FFABS(buf_prev[x + mv_x + i + j * linesize] - buf_next[x - mv_x + i + j * linesize]);

    return mad;
}

static void search_mv_dia(MIContext *mi_ctx, int x_mb, int y_mb, int *dx, int *dy)
{
    int x, y;
    int x_orig = x_mb, y_orig = y_mb;
    int start_x, start_y, end_x, end_y;
    int step = 2;
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(mi_ctx, x_mb, y_mb, 0, 0)))
        return;

    do {
        start_x = av_clip(x_orig - step, 0, x_orig);
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_x = av_clip(x_orig + step + 1, 0, mi_ctx->mi_frame[0].avf->width - mi_ctx->mb_size);
        end_y = av_clip(y_orig + step + 1, 0, mi_ctx->mi_frame[0].avf->height - mi_ctx->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                // skips already checked current origin
                if (x == x_orig && y == y_orig)
                    continue;

                if (FFABS(x_orig - x) + FFABS(y_orig - y) != step)
                    continue;

                cost = get_mad(mi_ctx, x_mb, y_mb, x - x_mb, y - y_mb);
                if (!cost) {
                    *dx = x - x_mb;
                    *dy = y - y_mb;
                    return;
                } else if (cost < cost_min) {
                    cost_min = cost;
                    x_min_cost = x;
                    y_min_cost = y;
                }
            }
        }

        if (step == 1)
            break;

        if (x_min_cost == x_orig && y_min_cost == y_orig ||
            FFABS(x_min_cost - x_mb) >= mi_ctx->search_param || FFABS(y_min_cost - y_mb) >= mi_ctx->search_param)
            step = 1;
        else {
            //TODO skip repeated
            x_orig = x_min_cost;
            y_orig = y_min_cost;
        }

    } while (step > 0);

    *dx = x_min_cost - x_mb;
    *dy = y_min_cost - y_mb;
}

static void search_mv_esa(MIContext *mi_ctx, int x_mb, int y_mb, int *dx, int *dy)
{
    int x, y;
    int p = mi_ctx->search_param;
    int start_x = av_clip(x_mb - p, 0, x_mb);
    int start_y = av_clip(y_mb - p, 0, y_mb);
    int end_x = av_clip(x_mb + p + 1, 0, mi_ctx->mi_frame[0].avf->width - mi_ctx->mb_size);
    int end_y = av_clip(y_mb + p + 1, 0, mi_ctx->mi_frame[0].avf->height - mi_ctx->mb_size);
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(mi_ctx, x_mb, y_mb, 0, 0)))
        return;

    for (y = start_y; y < end_y; y++)
        for (x = start_x; x < end_x; x++) {
            if ((cost = get_mad(mi_ctx, x_mb, y_mb, x - x_mb, y - y_mb)) < cost_min) {
                cost_min = cost;
                *dx = x - x_mb;
                *dy = y - y_mb;
            } else if (cost == cost_min)
                MIN_VECTOR(*dx, *dy, x - x_mb, y - y_mb);
        }
    //FIXME what if no vector is found? (0, 0) is fine?
}

/* this can be used for extension of overlapped bilateral motion estimation */
static void bilateral_me(MIContext *mi_ctx)
{
    MIFrame *out_frame = &mi_ctx->out_frame;
    MIBlock *mi_block;
    int mb_x, mb_y;
    int mv_x, mv_y;
    const int dir = 0;

    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            mi_block = &out_frame->mi_block[mb_x + mb_y * mi_ctx->b_width];

        #if CACHE_MVS
            int mb_i = mb_x + mb_y * mi_ctx->b_width;
            //MIFrame *mi_frame = &mi_ctx->mi_frame[2];
            //mv_x = (mi_frame->mi_block[mb_i].mv[dir][0] - mi_frame->mi_block[mb_i].mv[1 - dir][0]) / 2;
            //mv_y = (mi_frame->mi_block[mb_i].mv[dir][1] - mi_frame->mi_block[mb_i].mv[1 - dir][1]) / 2;
            mv_x = (mi_ctx->mi_frame[2].mi_block[mb_i].mv[dir][0] - mi_ctx->mi_frame[1].mi_block[mb_i].mv[1 - dir][0]) / 2;
            mv_y = (mi_ctx->mi_frame[2].mi_block[mb_i].mv[dir][1] - mi_ctx->mi_frame[1].mi_block[mb_i].mv[1 - dir][1]) / 2;
        #else
            mv_x = 0; mv_y = 0; //FIXME what if no vector is found? (0, 0) is fine?
            search_mv_esa(mi_ctx, mb_x << mi_ctx->log2_mb_size, mb_y << mi_ctx->log2_mb_size, &mv_x, &mv_y);
            mv_x *= 2;
            mv_y *= 2;
        #endif

            mi_block->mv[dir][0] = mv_x;
            mi_block->mv[dir][1] = mv_y;
            mi_block->cluster = 0;
            mi_block->sb = 0;

            mi_ctx->mi_cluster[0].sum[0] += mv_x;
            mi_ctx->mi_cluster[0].sum[1] += mv_y;

            /* for pre-obmc testing */
        #if !CACHE_MVS
            mi_ctx->mi_frame[2].mi_block[mb_x + mb_y * mi_ctx->b_width].mv[dir][0] = mv_x * 2;
            mi_ctx->mi_frame[2].mi_block[mb_x + mb_y * mi_ctx->b_width].mv[dir][1] = mv_y * 2;
            mi_ctx->mi_frame[1].mi_block[mb_x + mb_y * mi_ctx->b_width].mv[1 - dir][0] = mv_x * -2;
            mi_ctx->mi_frame[1].mi_block[mb_x + mb_y * mi_ctx->b_width].mv[1 - dir][1] = mv_y * -2;
        #endif
    }
    mi_ctx->mi_cluster[0].nb = mi_ctx->b_width * mi_ctx->b_height;
}

static void cluster_mvs(MIContext *mi_ctx)
{
    int changed, c, c_max = 0;
    int mb_x, mb_y, x, y;
    int mv_x, mv_y, avg_x, avg_y, dx, dy;
    MIBlock *mi_block;
    MIFrame *out_frame = &mi_ctx->out_frame;
    MICluster *cluster, *cluster_new;

    do {
        changed = 0;
        for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
            for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                mi_block = &out_frame->mi_block[mb_x + mb_y * mi_ctx->b_width];
                c = mi_block->cluster;
                cluster = &mi_ctx->mi_cluster[c];
                mv_x = mi_block->mv[0][0];
                mv_y = mi_block->mv[0][1];

                if (cluster->nb < 2)
                    continue;

                avg_x = cluster->sum[0] / cluster->nb;
                avg_y = cluster->sum[1] / cluster->nb;
                dx = avg_x - mv_x;
                dy = avg_y - mv_y;
        #if 0
                if (FFABS(avg_x + avg_y - FFABS(mv_x) - FFABS(mv_y)) > (CLUSTER_THRESHOLD << mi_ctx->log2_mv_precission)) {
        #else
                if (FFABS(avg_x - mv_x) > (CLUSTER_THRESHOLD << mi_ctx->log2_mv_precission) ||
                    FFABS(avg_y - mv_y) > (CLUSTER_THRESHOLD << mi_ctx->log2_mv_precission)) {
        #endif

                    int d; //TODO
                    for (d = 1; d < 5; d++)
                    for (y = FFMAX(mb_y - d, 0); y < FFMIN(mb_y + d + 1, mi_ctx->b_height); y++)
                        for (x = FFMAX(mb_x - d, 0); x < FFMIN(mb_x + d + 1, mi_ctx->b_width); x++) {
                            MIBlock *b = &out_frame->mi_block[x + y * mi_ctx->b_width];
                            //if ((x - mb_x) && (y - mb_y))
                            //    continue;
                            if (b->cluster > c) {
                                c = b->cluster;
                                goto break1;
                            }
                        }

                    break1:

                    if (c == mi_block->cluster)
                        c = c_max + 1;

                    if (c >= NB_CLUSTER) {
                        av_log(0, 0, "Out of memory!\n");
                        continue;
                    }

                    cluster_new = &mi_ctx->mi_cluster[c];
                    cluster_new->sum[0] += mv_x;
                    cluster_new->sum[1] += mv_y;
                    cluster->sum[0] -= mv_x;
                    cluster->sum[1] -= mv_y;
                    cluster_new->nb++;
                    cluster->nb--;

                    c_max = FFMAX(c_max, c);
                    cls_max = c_max;
                    mi_block->cluster = c;

                    changed = 1;
                }
            }
    } while (changed);

    /* find boundaries */
    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            mi_block = &out_frame->mi_block[mb_x + mb_y * mi_ctx->b_width];
            for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++)
                for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
                    dx = x - mb_x;
                    dy = y - mb_y;

                    if ((x - mb_x) && (y - mb_y) || !dx && !dy)
                        continue;

                    if (!mb_x || !mb_y || mb_x == mi_ctx->b_width - 1 || mb_y == mi_ctx->b_height - 1)
                        continue; //TODO test if frame boundaries don't introduce artifacts

                    //if (out_frame->mi_block[x + y * mi_ctx->b_width].cluster == 0) {
                    if (mi_block->cluster != out_frame->mi_block[x + y * mi_ctx->b_width].cluster) {
                        if (!dx && mi_block->cluster == out_frame->mi_block[x + (mb_y - dy) * mi_ctx->b_width].cluster ||
                            !dy && mi_block->cluster == out_frame->mi_block[(mb_x - dx) + y * mi_ctx->b_width].cluster)
                            mi_block->sb = 1;
                    }
                }
        }
}

static int var_size_block_me(MIContext *mi_ctx)
{
    int mb_x, mb_y, i;
    MIFrame *out_frame = &mi_ctx->out_frame;
    MIBlock *mi_block;

    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            mi_block = &out_frame->mi_block[mb_x + mb_y * mi_ctx->b_width];
            if (mi_block->sb) {
                mi_block->mi_block = av_mallocz_array(4, sizeof(MIBlock));
                if (!mi_block->mi_block)
                    return AVERROR(ENOMEM);

                for (i = 0; i < 4; i++) {
                    MIBlock *sb = &mi_block->mi_block[i];
                    sb->mv[0][0] = mi_block->mv[0][0];
                    sb->mv[0][1] = mi_block->mv[0][1];
                    sb->sb = 0;
                }

                //TODO if failed sb = 0;
            }
        }

    return 0;
}

/* mvs caching -starts- */
const char *dir = "/Users/dsm/ffmpeg_cache/frame_";
static char *ntos(int64_t num, int size)
{
    int i; int l;
    char *s;

    s = av_malloc(size + 1);
    s[size] = '\0';
    for (i =0; i < size; i++) {
        l = num % 10;
        s[size - 1 - i] = (char) (l + '0');
        num /= 10;
    }
    return s;
}
static int len(int64_t num)
{
    int i = 0;
    while (num) {
        num = num / 10;
        i++;
    }
    return i;
}
static char *get_filename(AVFrame *frame)
{
    int64_t pts = frame->pts;
    char *s = ntos(pts, len(pts));
    char *filename = av_malloc(strlen(dir) + strlen(s) + 1);
    strcpy(filename, dir);
    strcat(filename, s);
    av_freep(&s);
    return filename;
}
static size_t file_size(char *filename)
{
    FILE *file;
    size_t size = 0;

    file = fopen(filename, "r");
    if (file == NULL)
        return 0;
    fseek(file, 0, 2);
    size = ftell(file);
    fclose(file);
    return size;
}
static void copy_mvs(AVFrame *frame)
{
    AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_MOTION_VECTORS);

    if (sd) {
        char *filename = get_filename(frame);

        FILE *file = fopen(filename, "w");
        fwrite(sd->data, sizeof(AVMotionVector), sd->size / sizeof(AVMotionVector), file);
        fclose(file);

        av_freep(&filename);
    }
}
static void read_mvs(MIContext *mi_ctx, MIFrame *mi_frame)
{
    char *filename = get_filename(mi_frame->avf);
    size_t filesize = file_size(filename);

    if (filesize) {
        uint8_t *data = av_malloc(filesize);

        FILE *file = fopen(filename, "r");
        fread(data, sizeof(uint8_t), filesize, file);

        {
            AVMotionVector *mvs = (AVMotionVector *) data;
            int i, dir;

            for (i = 0; i < filesize / sizeof(AVMotionVector); i++) {
                AVMotionVector *mv = &mvs[i];
                int mb_i, mb_y, mb_x;

                mb_x = av_clip((mv->dst_x /*- mv->w / 2*/) >> mi_ctx->log2_mb_size, 0, mi_ctx->b_width - 1);
                mb_y = av_clip((mv->dst_y /*- mv->h / 2*/) >> mi_ctx->log2_mb_size, 0, mi_ctx->b_height - 1);

                dir = mv->source > 0;
                mb_i = mb_x + mi_ctx->b_width * mb_y; //TODO subblocks from export_mvs

                mi_frame->mi_block[mb_i].mv[dir][0] = (mv->src_x - mv->dst_x) << mi_ctx->log2_mv_precission; //TODO qpel
                mi_frame->mi_block[mb_i].mv[dir][1] = (mv->src_y - mv->dst_y) << mi_ctx->log2_mv_precission;

            }
        }
        fclose(file);
        av_freep(&data);
    }
    av_freep(&filename);
}
/* mvs caching -ends- */

static int inject_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MIContext *mi_ctx = ctx->priv;
    MIFrame mi_frame_tmp, *mi_frame;
    int i, dir, ret;

    av_frame_free(&mi_ctx->mi_frame[0].avf);
    mi_frame_tmp = mi_ctx->mi_frame[0];
    memmove(&mi_ctx->mi_frame[0], &mi_ctx->mi_frame[1], sizeof(mi_ctx->mi_frame[0]) * (NB_FRAMES - 1));
    mi_ctx->mi_frame[NB_FRAMES - 1] = mi_frame_tmp;
    mi_ctx->mi_frame[NB_FRAMES - 1].avf = frame;
    mi_frame = &mi_ctx->mi_frame[NB_FRAMES - 1];

    if (mi_ctx->mi_mode == MI_MODE_OBMC) {
        /* keeping this for later use (optimization with exported motion vectors) */
        AVFrameSideData *sd = av_frame_get_side_data(mi_frame->avf, AV_FRAME_DATA_MOTION_VECTORS);

        for (i = 0; i < mi_ctx->b_height * mi_ctx->b_width; i++)
            for (dir = 0; dir < 2; dir++) {
                mi_frame->mi_block[i].mv[dir][0] = 0;
                mi_frame->mi_block[i].mv[dir][1] = 0;
            }

        if (sd) {
            AVMotionVector *mvs = (AVMotionVector *) sd->data;

            for (i = 0; i < sd->size / sizeof(AVMotionVector); i++) {
                AVMotionVector *mv = &mvs[i];
                int mb_i, mb_y, mb_x;

                mb_x = av_clip((mv->dst_x /*- mv->w / 2*/) >> mi_ctx->log2_mb_size, 0, mi_ctx->b_width - 1);
                mb_y = av_clip((mv->dst_y /*- mv->h / 2*/) >> mi_ctx->log2_mb_size, 0, mi_ctx->b_height - 1);

                dir = mv->source > 0;
                mb_i = mb_x + mi_ctx->b_width * mb_y; //TODO subblocks from export_mvs

                mi_frame->mi_block[mb_i].mv[dir][0] = (mv->src_x - mv->dst_x) << mi_ctx->log2_mv_precission; //TODO qpel
                mi_frame->mi_block[mb_i].mv[dir][1] = (mv->src_y - mv->dst_y) << mi_ctx->log2_mv_precission;

            }
        }
    }

#if CACHE_MVS
    /* save mvs to disk, if available */
    copy_mvs(mi_frame->avf);
    read_mvs(mi_ctx, mi_frame);
#endif

    if (mi_ctx->mi_mode == MI_MODE_PAPER2) {

        if (!mi_ctx->mi_frame[0].avf)
            return 0;

        // reset clusters
        for (i = 0; i < NB_CLUSTER; i++) {
            mi_ctx->mi_cluster[i].sum[0] = 0;
            mi_ctx->mi_cluster[i].sum[1] = 0;
            mi_ctx->mi_cluster[i].nb = 0;
        }

        bilateral_me(mi_ctx);
        cluster_mvs(mi_ctx);

        if (ret = var_size_block_me(mi_ctx))
            return ret;
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

    if (!mi_ctx->mi_frame[NB_FRAMES - 1].avf || frame->pts < mi_ctx->mi_frame[NB_FRAMES - 1].avf->pts) {
        av_log(ctx, AV_LOG_VERBOSE, "Initializing out pts from input pts %"PRId64"\n", frame->pts);
        mi_ctx->out_pts = av_rescale_q(frame->pts, inlink->time_base, outlink->time_base);
    }

    if (!mi_ctx->mi_frame[NB_FRAMES - 1].avf)
        if (ret = inject_frame(inlink, av_frame_clone(frame)))
            return ret;

    if (ret = inject_frame(inlink, frame))
        return ret;

    if (!mi_ctx->mi_frame[0].avf)
        return 0;

    for (;;) {
        AVFrame *out;

        if (av_compare_ts(mi_ctx->mi_frame[NB_FRAMES / 2].avf->pts, inlink->time_base, mi_ctx->out_pts, outlink->time_base) < 0)
            break;

        if (!(out = ff_get_video_buffer(ctx->outputs[0], inlink->w, inlink->h)))
            return AVERROR(ENOMEM);

        av_frame_copy_props(out, mi_ctx->mi_frame[NB_FRAMES - 1].avf); //FIXME this also copy sidedata
        out->pts = mi_ctx->out_pts++;

        interpolate(inlink, out);

        if ((ret = ff_filter_frame(ctx->outputs[0], out)) < 0)
            return ret;
    }
    return 0;
}

static av_cold void free_blocks(MIBlock *mi_block, int sb)
{
    if (mi_block->mi_block)
        free_blocks(mi_block->mi_block, 1);
    if (sb)
        av_freep(&mi_block);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MIContext *mi_ctx = ctx->priv;
    int i, m;

    av_freep(&mi_ctx->pixel);
    if (mi_ctx->out_frame.mi_block)
        for (m = 0; m < mi_ctx->b_width * mi_ctx->b_height; m++)
            free_blocks(&mi_ctx->out_frame.mi_block[m], 0);
    av_freep(&mi_ctx->out_frame.mi_block);
    av_frame_free(&mi_ctx->out_frame.avf);

    for (i = 0; i < NB_FRAMES; i++) {
        MIFrame *mi_frame = &mi_ctx->mi_frame[i];
        //for (m = 0; m < mi_ctx->b_width * mi_ctx->b_height; m++)
        //    free_blocks(&mi_frame->mi_block[m], 0);
        av_freep(&mi_frame->mi_block);
        av_frame_free(&mi_frame->avf);
    }
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
