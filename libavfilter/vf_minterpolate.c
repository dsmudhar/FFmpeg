/**
 * Copyright (c) 2014-2015 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2016 Davinder Singh (DSM_) <ds.mudhar<@gmail.com>
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

#include "motion_estimation.h"
#include "libavcodec/mathops.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/motion_vector.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

#define ME_MODE_BIDIRECTIONAL 0
#define ME_MODE_BILATERAL 1

#define MC_MODE_OBMC 0
#define MC_MODE_AOBMC 1

//TESTS
#define DEBUG_CLUSTERING 0
#define CACHE_MVS 0
#define EXPORT_MVS 1

#define NB_FRAMES 4
#define NB_PIXEL_MVS 16
#define NB_CLUSTERS 100

#define ALPHA_MAX 1024
#define CLUSTER_THRESHOLD 4
#define ROUGHNESS 32
#define PX_WEIGHT_MAX 255

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

static const uint8_t * const obmc_tab_linear[4]= {
    obmc_linear32, obmc_linear16, obmc_linear8, obmc_linear4
};

enum MIMode {
    MI_MODE_DUP         = 0,
    MI_MODE_BLEND       = 1,
    MI_MODE_MCI         = 2,
};

typedef struct Cluster {
    int64_t sum[2];
    int nb;
} Cluster;

typedef struct Block {
    int mv;                     ///< motion vector boolean
    int16_t mvs[2][2];          ///< motion vectors[dir][x,y]
    int cid;                    ///< cluster id
    uint64_t sbad;              ///< sum of bilateral abs diffs
    int sb;                     ///< sub-blocks boolean
    struct Block *subs;         ///< sub-blocks
} Block;

typedef struct Pixel {
    int16_t mvs[NB_PIXEL_MVS][2];
    uint32_t weights[NB_PIXEL_MVS];
    int8_t refs[NB_PIXEL_MVS];
    int nb;
} Pixel;

typedef struct Frame {
    AVFrame *avf;
    Block *blocks;
} Frame;

typedef struct MIContext {
    const AVClass *class;
    AVMotionEstContext me_ctx;
    AVRational frame_rate;
    enum MIMode mi_mode;
    enum MEMethod me_method;
    int mc_mode;
    int me_mode;
    int mb_size;
    int search_param;
    int vsbmc;

    Frame frames[NB_FRAMES];
    Cluster clusters[NB_CLUSTERS];
    Block *int_blocks;
    Pixel *pixels;
    int64_t out_pts;
    int b_width, b_height, b_count;
    int log2_mb_size;

    int chroma_height;
    int chroma_width;
    int chroma_h_shift;
    int chroma_v_shift;
    int nb_planes;

    int cls;
    int cls_max;
} MIContext;

#define OFFSET(x) offsetof(MIContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption minterpolate_options[] = {
    { "mi_mode", "specify the interpolation mode", OFFSET(mi_mode), AV_OPT_TYPE_INT, {.i64 = MI_MODE_MCI}, MI_MODE_DUP, MI_MODE_MCI, FLAGS, "mi_mode" },
        CONST("dup",    "duplicate frames",                     MI_MODE_DUP,            "mi_mode"),
        CONST("blend",  "blend frames",                         MI_MODE_BLEND,          "mi_mode"),
        CONST("mci",    "motion compensated interpolation",     MI_MODE_MCI,            "mi_mode"),
    { "mc_mode", "specify the motion compensation mode", OFFSET(mc_mode), AV_OPT_TYPE_INT, {.i64 = MC_MODE_OBMC}, MC_MODE_OBMC, MC_MODE_AOBMC, FLAGS, "mc_mode" },
        CONST("obmc",   "overlapped block motion compensation", MC_MODE_OBMC,           "mc_mode"),
        CONST("aobmc",  "adaptive block motion compensation",   MC_MODE_AOBMC,          "mc_mode"),
    { "me_mode", "specify the motion estimation mode", OFFSET(me_mode), AV_OPT_TYPE_INT, {.i64 = ME_MODE_BILATERAL}, ME_MODE_BIDIRECTIONAL, ME_MODE_BILATERAL, FLAGS, "me_mode" },
        CONST("bidir",  "bidirectional motion estimation",      ME_MODE_BIDIRECTIONAL,  "me_mode"),
        CONST("bilat",  "bilateral motion estimation",          ME_MODE_BILATERAL,      "me_mode"),
    { "me", "specify motion estimation method", OFFSET(me_method), AV_OPT_TYPE_INT, {.i64 = ME_METHOD_UMH}, ME_METHOD_ESA, ME_METHOD_UMH, FLAGS, "me" },
        CONST("esa",    "exhaustive search",                    ME_METHOD_ESA,          "me"),
        CONST("tss",    "three step search",                    ME_METHOD_TSS,          "me"),
        CONST("tdls",   "two dimensional logarithmic search",   ME_METHOD_TDLS,         "me"),
        CONST("ntss",   "new three step search",                ME_METHOD_NTSS,         "me"),
        CONST("fss",    "four step search",                     ME_METHOD_FSS,          "me"),
        CONST("ds",     "diamond search",                       ME_METHOD_DS,           "me"),
        CONST("hexbs",  "hexagon-based search",                 ME_METHOD_HEXBS,        "me"),
        CONST("epzs",   "enhanced predictive zonal search",     ME_METHOD_EPZS,         "me"),
        CONST("umh",    "uneven multi-hexagon search",          ME_METHOD_UMH,          "me"),

    { "fps", "specify the frame rate", OFFSET(frame_rate), AV_OPT_TYPE_RATIONAL, {.dbl = 60}, 0, INT_MAX, FLAGS },
    { "mb_size", "specify the macroblock size", OFFSET(mb_size), AV_OPT_TYPE_INT, {.i64 = 16}, 4, 16, FLAGS },
    { "search_param", "specify search parameter", OFFSET(search_param), AV_OPT_TYPE_INT, {.i64 = 16}, 4, INT_MAX, FLAGS },
    { "vsbmc", "variable-size block motion compensation", OFFSET(vsbmc), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(minterpolate);

static int query_formats(AVFilterContext *ctx) 
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
        AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P,
        AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUVJ411P,
        AV_PIX_FMT_YUVA420P, AV_PIX_FMT_YUVA422P, AV_PIX_FMT_YUVA444P,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static uint64_t get_sbad(AVMotionEstContext *me_ctx, int x, int y, int x_mv, int y_mv)
{
    const int linesize = me_ctx->linesize;
    uint8_t *data_cur = me_ctx->data_cur;
    uint8_t *data_next = me_ctx->data_ref;
    int mv_x = x_mv - x;
    int mv_y = y_mv - y;
    uint64_t sbad = 0;
    int i, j;

    data_cur += (y + mv_y) * linesize;
    data_next += (y - mv_y) * linesize;

    if (x - FFABS(mv_x) < 0 || y - FFABS(mv_y) < 0 ||
        x + FFABS(mv_x) + me_ctx->mb_size > me_ctx->width - 1 ||
        y + FFABS(mv_y) + me_ctx->mb_size > me_ctx->height - 1)
        return UINT64_MAX;

    for (j = 0; j < me_ctx->mb_size; j++)
        for (i = 0; i < me_ctx->mb_size; i++)
            sbad += FFABS(data_cur[x + mv_x + i + j * linesize] - data_next[x - mv_x + i + j * linesize]);

    return sbad;
}

static uint64_t get_sbad_ob(AVMotionEstContext *me_ctx, int x, int y, int x_mv, int y_mv)
{
    const int linesize = me_ctx->linesize;
    uint8_t *data_cur = me_ctx->data_cur;
    uint8_t *data_next = me_ctx->data_ref;
    int mv_x = x_mv - x;
    int mv_y = y_mv - y;
    uint64_t sbad = 0;
    int i, j;

    if (x - FFABS(mv_x) - me_ctx->mb_size / 2 < 0 || y - FFABS(mv_y) - me_ctx->mb_size / 2 < 0 ||
        x + FFABS(mv_x) + me_ctx->mb_size * 3 / 2 > me_ctx->width - 1 ||
        y + FFABS(mv_y) + me_ctx->mb_size * 3 / 2 > me_ctx->height - 1)
        return UINT64_MAX;

    for (j = -me_ctx->mb_size / 2; j < me_ctx->mb_size * 3 / 2; j++)
        for (i = -me_ctx->mb_size / 2; i < me_ctx->mb_size * 3 / 2; i++)
            sbad += FFABS(data_cur[x + mv_x + i + (y + mv_y + j) * linesize] - data_next[x - mv_x + i + (y - mv_y + j) * linesize]);

    return sbad + (mv_x * mv_x + mv_y * mv_y);
}

static int config_input(AVFilterLink *inlink)
{
    MIContext *mi_ctx = inlink->dst->priv;
    AVMotionEstContext *me_ctx = &mi_ctx->me_ctx;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    const int height = inlink->h;
    const int width  = inlink->w;
    int i;

    mi_ctx->chroma_height = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    mi_ctx->chroma_width = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);

    avcodec_get_chroma_sub_sample(inlink->format, &mi_ctx->chroma_h_shift, &mi_ctx->chroma_v_shift); //TODO remove

    mi_ctx->nb_planes = av_pix_fmt_count_planes(inlink->format);

    mi_ctx->log2_mb_size = av_ceil_log2_c(mi_ctx->mb_size);
    mi_ctx->mb_size = 1 << mi_ctx->log2_mb_size;

    mi_ctx->b_width  = AV_CEIL_RSHIFT(width,  mi_ctx->log2_mb_size);
    mi_ctx->b_height = AV_CEIL_RSHIFT(height, mi_ctx->log2_mb_size);
    mi_ctx->b_count = mi_ctx->b_width * mi_ctx->b_height;

    for (i = 0; i < NB_FRAMES; i++) {
        Frame *frame = &mi_ctx->frames[i];
        frame->blocks = av_mallocz_array(mi_ctx->b_count, sizeof(Block));
        if (!frame->blocks)
            return AVERROR(ENOMEM);
    }

    if (mi_ctx->mi_mode == MI_MODE_MCI) {
        if (!(mi_ctx->pixels = av_mallocz_array(width * height, sizeof(Pixel))))
            return AVERROR(ENOMEM);

        if (mi_ctx->me_mode == ME_MODE_BILATERAL)
            if (!(mi_ctx->int_blocks = av_mallocz_array(mi_ctx->b_count, sizeof(Block))))
                return AVERROR(ENOMEM);
    }

    ff_me_init_context(me_ctx, mi_ctx->mb_size, mi_ctx->search_param, width, height);
    if (mi_ctx->me_mode == ME_MODE_BILATERAL)
        me_ctx->get_cost = &get_sbad_ob;

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    MIContext *mi_ctx = outlink->src->priv;

    outlink->frame_rate = mi_ctx->frame_rate;
    outlink->time_base  = av_inv_q(mi_ctx->frame_rate);

    return 0;
}

static void search_mv(MIContext *mi_ctx, Block *blocks, int mb_x, int mb_y, int dir)
{
    AVMotionEstContext *me_ctx = &mi_ctx->me_ctx;
    Block *block = &blocks[mb_x + mb_y * mi_ctx->b_width];

    const int x_mb = mb_x << mi_ctx->log2_mb_size;
    const int y_mb = mb_y << mi_ctx->log2_mb_size;
    int mv[2] = {x_mb, y_mb};
    int i;

    if (mi_ctx->me_method == ME_METHOD_ESA)
        ff_me_search_esa(me_ctx, x_mb, y_mb, mv);
    else if (mi_ctx->me_method == ME_METHOD_TSS)
        ff_me_search_tss(me_ctx, x_mb, y_mb, mv);
    else if (mi_ctx->me_method == ME_METHOD_TDLS)
        ff_me_search_tdls(me_ctx, x_mb, y_mb, mv);
    if (mi_ctx->me_method == ME_METHOD_NTSS)
        ff_me_search_ntss(me_ctx, x_mb, y_mb, mv);
    else if (mi_ctx->me_method == ME_METHOD_FSS)
        ff_me_search_fss(me_ctx, x_mb, y_mb, mv);
    else if (mi_ctx->me_method == ME_METHOD_DS)
        ff_me_search_ds(me_ctx, x_mb, y_mb, mv);
    else if (mi_ctx->me_method == ME_METHOD_HEXBS)
        ff_me_search_hexbs(me_ctx, x_mb, y_mb, mv);
    else if (mi_ctx->me_method == ME_METHOD_EPZS)
        ; //TODO add epzs
    else if (mi_ctx->me_method == ME_METHOD_UMH) {
        int pred[5][2];

        for (i = 0; i < 5; i++) {
            pred[i][0] = x_mb;
            pred[i][1] = y_mb;
        }

        //left mb in current frame
        pred[2][0] += blocks[mb_x - 1 + mb_y * mi_ctx->b_width].mvs[0][0];
        pred[2][1] += blocks[mb_x - 1 + mb_y * mi_ctx->b_width].mvs[0][1];

        //top mb in current frame
        pred[3][0] += blocks[mb_x + (mb_y - 1) * mi_ctx->b_width].mvs[0][0];
        pred[3][1] += blocks[mb_x + (mb_y - 1) * mi_ctx->b_width].mvs[0][1];

        //top-right mb in current frame
        pred[4][0] += blocks[mb_x + 1 + (mb_y - 1) * mi_ctx->b_width].mvs[0][0];
        pred[4][1] += blocks[mb_x + 1 + (mb_y - 1) * mi_ctx->b_width].mvs[0][1];

        //median predictor
        pred[0][0] = mid_pred(pred[2][0], pred[3][0], pred[4][0]);
        pred[0][1] = mid_pred(pred[2][1], pred[3][1], pred[4][1]);

        ff_me_search_umh(me_ctx, pred, x_mb, y_mb, mv);
    }

    block->mvs[dir][0] = mv[0] - x_mb;
    block->mvs[dir][1] = mv[1] - y_mb;
}

static void bilateral_me(MIContext *mi_ctx)
{
    Block *block;
    int mb_x, mb_y;

    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];

            block->cid = 0;
            block->sb = 0;
            block->mv = 0;

            block->mvs[0][0] = 0;
            block->mvs[0][1] = 0;

            if (mb_x == 0 || mb_y == 0 || mb_x == mi_ctx->b_width - 1 || mb_y == mi_ctx->b_height - 1)
                block->mv = 1;

        #if CACHE_MVS
            int mv_x, mv_y;
            int mb_i = mb_x + mb_y * mi_ctx->b_width;
            mv_x = (mi_ctx->frames[2].blocks[mb_i].mvs[0][0] - mi_ctx->frames[1].blocks[mb_i].mvs[1][0]) / 4;
            mv_y = (mi_ctx->frames[2].blocks[mb_i].mvs[0][1] - mi_ctx->frames[1].blocks[mb_i].mvs[1][1]) / 4;
            block->mvs[0][0] = mv_x;
            block->mvs[0][1] = mv_y;
        #endif
        }

#if !CACHE_MVS
    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {

            if (mb_x > 0 && mb_y > 0 && mb_x < mi_ctx->b_width && mb_y < mi_ctx->b_height) {
                search_mv(mi_ctx, mi_ctx->int_blocks, mb_x, mb_y, 0);
            }
        }
#endif /* CACHE_MVS */

    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            int x_mb = mb_x << mi_ctx->log2_mb_size;
            int y_mb = mb_y << mi_ctx->log2_mb_size;

            block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];

            mi_ctx->clusters[0].sum[0] += block->mvs[0][0];
            mi_ctx->clusters[0].sum[1] += block->mvs[0][1];

            if (mi_ctx->mc_mode == MC_MODE_AOBMC)
                block->sbad = get_sbad(&mi_ctx->me_ctx, x_mb, y_mb, x_mb + block->mvs[0][0], y_mb + block->mvs[0][1]);
        }

    mi_ctx->clusters[0].nb = mi_ctx->b_count;
}

static int var_size_bme(MIContext *mi_ctx, Block *block, int x_mb, int y_mb, int n)
{
    uint8_t *data_prev = mi_ctx->frames[1].avf->data[0];
    uint8_t *data_next = mi_ctx->frames[2].avf->data[0];
    int linesize = mi_ctx->frames[0].avf->linesize[0];
    uint64_t cost_sb, cost_old = 0;
    int mv_x, mv_y, i, j;
    int x, y;
    int ret;

    mv_x = block->mvs[0][0];
    mv_y = block->mvs[0][1];

    data_prev += (y_mb + mv_y) * linesize;
    data_next += (y_mb - mv_y) * linesize;

    av_assert0(y_mb - FFABS(mv_y) >= 0 && y_mb + FFABS(mv_y) < mi_ctx->frames[0].avf->height);
    av_assert0(x_mb - FFABS(mv_x) >= 0 && x_mb + FFABS(mv_x) < mi_ctx->frames[0].avf->width);

    for (j = 0; j < 1 << n; j++)
        for (i = 0; i < 1 << n; i++)
            cost_old += FFABS(data_prev[x_mb + mv_x + i + j * linesize] - data_next[x_mb - mv_x + i + j * linesize]);

    if (!cost_old) {
        block->sb = 0;
        return 0;
    }

    if (!block->subs)
        block->subs = av_mallocz_array(4, sizeof(Block));
    if (!block->subs)
        return AVERROR(ENOMEM);

    block->sb = 1;

    for (y = 0; y < 2; y++)
        for (x = 0; x < 2; x++) {
            Block *sb = &block->subs[x + y * 2];
            int mv[2] = {x_mb + block->mvs[0][0], y_mb + block->mvs[0][1]};
            int mb_size = mi_ctx->me_ctx.mb_size;
            mi_ctx->me_ctx.mb_size = 1 << (n - 1);

            cost_sb = ff_me_search_ds(&mi_ctx->me_ctx, x_mb, y_mb, mv);
            mv_x = mv[0] - x_mb;
            mv_y = mv[1] - y_mb;

            mi_ctx->me_ctx.mb_size = mb_size;

            if (
                cost_sb < cost_old / 4) {

                sb->mvs[0][0] = mv_x;
                sb->mvs[0][1] = mv_y;
                //if (sb->mvs[0][0]/2 != block->mvs[0][0]/2 || sb->mvs[0][1]/2 != block->mvs[0][1]/2) av_log(0, 0, "new mv: (%d, %d) vs (%d, %d)\n", sb->mvs[0][0], sb->mvs[0][1], block->mvs[0][0], block->mvs[0][1]);

                if (n > 1) {
                    if (ret = var_size_bme(mi_ctx, sb, x_mb + (x << (n - 1)), y_mb + (y << (n - 1)), n - 1))
                        return ret;
                } else
                    sb->sb = 0;
            } else {
                block->sb = 0;
                return 0;
            }
        }

    return 0;
}

static int cluster_mvs(MIContext *mi_ctx)
{
    int changed, c, c_max = 0;
    int mb_x, mb_y, x, y;
    int mv_x, mv_y, avg_x, avg_y, dx, dy;
    int d, ret;
    Block *block;
    Cluster *cluster, *cluster_new;

    do {
        changed = 0;
        for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
            for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];
                c = block->cid;
                cluster = &mi_ctx->clusters[c];
                mv_x = block->mvs[0][0];
                mv_y = block->mvs[0][1];

                if (cluster->nb < 2)
                    continue;

                avg_x = cluster->sum[0] / cluster->nb;
                avg_y = cluster->sum[1] / cluster->nb;
                dx = avg_x - mv_x;
                dy = avg_y - mv_y;

                if (FFABS(avg_x - mv_x) > CLUSTER_THRESHOLD ||
                    FFABS(avg_y - mv_y) > CLUSTER_THRESHOLD) {

                    for (d = 1; d < 5; d++)
                        for (y = FFMAX(mb_y - d, 0); y < FFMIN(mb_y + d + 1, mi_ctx->b_height); y++)
                            for (x = FFMAX(mb_x - d, 0); x < FFMIN(mb_x + d + 1, mi_ctx->b_width); x++) {
                                Block *b = &mi_ctx->int_blocks[x + y * mi_ctx->b_width];
                                //if ((x - mb_x) && (y - mb_y))
                                //    continue;
                                if (b->cid > c) {
                                    c = b->cid;
                                    goto break1;
                                }
                            }

                    break1:

                    if (c == block->cid)
                        c = c_max + 1;

                    if (c >= NB_CLUSTERS) {
                        continue;
                    }

                    cluster_new = &mi_ctx->clusters[c];
                    cluster_new->sum[0] += mv_x;
                    cluster_new->sum[1] += mv_y;
                    cluster->sum[0] -= mv_x;
                    cluster->sum[1] -= mv_y;
                    cluster_new->nb++;
                    cluster->nb--;

                    c_max = FFMAX(c_max, c);
                    mi_ctx->cls_max = c_max;
                    block->cid = c;

                    changed = 1;
                }
            }
    } while (changed);

    /* find boundaries */
    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
            block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];
            for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++)
                for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
                    dx = x - mb_x;
                    dy = y - mb_y;

                    if ((x - mb_x) && (y - mb_y) || !dx && !dy)
                        continue;

                    if (!mb_x || !mb_y || mb_x == mi_ctx->b_width - 1 || mb_y == mi_ctx->b_height - 1)
                        continue; //TODO test if frame boundaries don't introduce artifacts

                    //if (mi_ctx->int_blocks[x + y * mi_ctx->b_width].cid == 0) {
                    if (block->cid != mi_ctx->int_blocks[x + y * mi_ctx->b_width].cid) {
                        if (!dx && block->cid == mi_ctx->int_blocks[x + (mb_y - dy) * mi_ctx->b_width].cid ||
                            !dy && block->cid == mi_ctx->int_blocks[(mb_x - dx) + y * mi_ctx->b_width].cid) {
                            if (ret = var_size_bme(mi_ctx, block, mb_x << mi_ctx->log2_mb_size, mb_y << mi_ctx->log2_mb_size, mi_ctx->log2_mb_size))
                                return ret;
                        }
                    }
                }
        }

    return 0;
}

static int inject_frame(AVFilterLink *inlink, AVFrame *avf_in)
{
    AVFilterContext *ctx = inlink->dst;
    MIContext *mi_ctx = ctx->priv;
    Frame frame_tmp, *frame;

    av_frame_free(&mi_ctx->frames[0].avf);
    frame_tmp = mi_ctx->frames[0];
    memmove(&mi_ctx->frames[0], &mi_ctx->frames[1], sizeof(mi_ctx->frames[0]) * (NB_FRAMES - 1));
    mi_ctx->frames[NB_FRAMES - 1] = frame_tmp;
    mi_ctx->frames[NB_FRAMES - 1].avf = avf_in;
    frame = &mi_ctx->frames[NB_FRAMES - 1];

    if (mi_ctx->mi_mode == MI_MODE_MCI) {
        if (mi_ctx->me_mode == ME_MODE_BIDIRECTIONAL) {
            int mb_x, mb_y, dir;

            if (mi_ctx->frames[1].avf) {
                for (dir = 0; dir < 2; dir++) {
                    mi_ctx->me_ctx.linesize = mi_ctx->frames[2].avf->linesize[0];
                    mi_ctx->me_ctx.data_cur = mi_ctx->frames[2].avf->data[0];
                    mi_ctx->me_ctx.data_ref = mi_ctx->frames[dir ? 3 : 1].avf->data[0];

                    for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
                        for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {

                            search_mv(mi_ctx, mi_ctx->frames[2].blocks, mb_x, mb_y, dir);
                        }
                }
            }

            /*
            AVFrameSideData *sd = av_frame_get_side_data(frame->avf, AV_FRAME_DATA_MOTION_VECTORS);

            for (i = 0; i < mi_ctx->b_count; i++)
                for (dir = 0; dir < 2; dir++) {
                    frame->blocks[i].mvs[dir][0] = 0;
                    frame->blocks[i].mvs[dir][1] = 0;
                }

            if (sd) {
                AVMotionVector *mvs = (AVMotionVector *) sd->data;

                for (i = 0; i < sd->size / sizeof(AVMotionVector); i++) {
                    AVMotionVector *mv = &mvs[i];
                    int mb_i, mb_y, mb_x;

                    mb_x = av_clip((mv->dst_x) >> mi_ctx->log2_mb_size, 0, mi_ctx->b_width - 1);
                    mb_y = av_clip((mv->dst_y) >> mi_ctx->log2_mb_size, 0, mi_ctx->b_height - 1);

                    dir = mv->source > 0;
                    mb_i = mb_x + mi_ctx->b_width * mb_y;

                    frame->blocks[mb_i].mvs[dir][0] = (mv->src_x - mv->dst_x);
                    frame->blocks[mb_i].mvs[dir][1] = (mv->src_y - mv->dst_y);

                }
            }*/
        }

        if (mi_ctx->me_mode == ME_MODE_BILATERAL) {
            int i, ret;

            if (!mi_ctx->frames[0].avf)
                return 0;

            // reset clusters
            for (i = 0; i < NB_CLUSTERS; i++) {
                mi_ctx->clusters[i].sum[0] = 0;
                mi_ctx->clusters[i].sum[1] = 0;
                mi_ctx->clusters[i].nb = 0;
            }

            mi_ctx->me_ctx.linesize = mi_ctx->frames[0].avf->linesize[0];
            mi_ctx->me_ctx.data_cur = mi_ctx->frames[1].avf->data[0];
            mi_ctx->me_ctx.data_ref = mi_ctx->frames[2].avf->data[0];

            bilateral_me(mi_ctx);

            if (mi_ctx->vsbmc)
                if (ret = cluster_mvs(mi_ctx))
                    return ret;
        }
    }

    return 0;
}

static int get_roughness(MIContext *mi_ctx, int mb_x, int mb_y, int mv_x, int mv_y, int dir)
{
    int x, y;
    int roughness = INT_MAX;

    for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++)
        for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
            int d, mv_x1, mv_y1;
            int dir1 = dir;

            if (dir < 0) {
                if (x == mb_x && y == mb_y)
                    continue;

                mv_x *= 2; mv_y *= 2;
                mv_x1 = mi_ctx->int_blocks[x + y * mi_ctx->b_width].mvs[0][0] * 2;
                mv_y1 = mi_ctx->int_blocks[x + y * mi_ctx->b_width].mvs[0][1] * 2;
            } else {
                if (x == mb_x && y == mb_y)
                    dir1 = 1 - dir;

                mv_x1 = mi_ctx->frames[2 - dir].blocks[x + y * mi_ctx->b_width].mvs[dir1][0];
                mv_y1 = mi_ctx->frames[2 - dir].blocks[x + y * mi_ctx->b_width].mvs[dir1][1];
                if (dir != dir1) {
                    mv_x1 = -mv_x1;
                    mv_y1 = -mv_y1;
                }
            }
            d = FFABS(mv_x - mv_x1) + FFABS(mv_y - mv_y1);
            roughness = FFMIN(roughness, d);
        }

    return roughness;
}

#define PIXEL_INIT(b_weight, mv_x, mv_y)\
    do {\
        pixel->refs[pixel->nb] = 1;\
        pixel->weights[pixel->nb] = b_weight * (ALPHA_MAX - alpha);\
        pixel->mvs[pixel->nb][0] = av_clip((mv_x * alpha) / ALPHA_MAX, x_min, x_max);\
        pixel->mvs[pixel->nb][1] = av_clip((mv_y * alpha) / ALPHA_MAX, y_min, y_max);\
        pixel->nb++;\
        pixel->refs[pixel->nb] = 2;\
        pixel->weights[pixel->nb] = b_weight * alpha;\
        pixel->mvs[pixel->nb][0] = av_clip(-mv_x * (ALPHA_MAX - alpha) / ALPHA_MAX, x_min, x_max);\
        pixel->mvs[pixel->nb][1] = av_clip(-mv_y * (ALPHA_MAX - alpha) / ALPHA_MAX, y_min, y_max);\
        pixel->nb++;\
    } while(0)

static void obmc(MIContext *mi_ctx, int alpha)
{
    int x, y;
    int width = mi_ctx->frames[0].avf->width;
    int height = mi_ctx->frames[0].avf->height;
    int mb_y, mb_x, dir;

    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
            mi_ctx->pixels[x + y * width].nb = 0;

    for (dir = 0; dir < 2; dir++)
        for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
            for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                int mv_x = mi_ctx->frames[2 - dir].blocks[mb_x + mb_y * mi_ctx->b_width].mvs[dir][0];
                int mv_y = mi_ctx->frames[2 - dir].blocks[mb_x + mb_y * mi_ctx->b_width].mvs[dir][1];
                int start_x, start_y;
                int startc_x, startc_y, endc_x, endc_y;

                if (get_roughness(mi_ctx, mb_x, mb_y, mv_x, mv_y, dir) > ROUGHNESS)
                    continue;

                start_x = (mb_x << mi_ctx->log2_mb_size) - mi_ctx->mb_size / 2;
                start_y = (mb_y << mi_ctx->log2_mb_size) - mi_ctx->mb_size / 2;

                startc_x = av_clip(start_x, 0, width - 1);
                startc_y = av_clip(start_y, 0, height - 1);
                endc_x = av_clip(start_x + (2 << mi_ctx->log2_mb_size), 0, width - 1);
                endc_y = av_clip(start_y + (2 << mi_ctx->log2_mb_size), 0, height - 1);

                if (dir) {
                    mv_x = -mv_x;
                    mv_y = -mv_y;
                }

                for (y = startc_y; y < endc_y; y++) {
                    int y_min = -y;
                    int y_max = height - y - 1;
                    for (x = startc_x; x < endc_x; x++) {
                        int x_min = -x;
                        int x_max = width - x - 1;
                        int obmc_weight = obmc_tab_linear[4 - mi_ctx->log2_mb_size][(x - start_x) + ((y - start_y) << (mi_ctx->log2_mb_size + 1))];
                        Pixel *pixel = &mi_ctx->pixels[x + y * width];

                        PIXEL_INIT(obmc_weight, mv_x, mv_y);
                    }
                }
            }
}

static void interpolate_pixels(MIContext *mi_ctx, int alpha, AVFrame *avf_out)
{
    int x, y, plane;

    for (plane = 0; plane < mi_ctx->nb_planes; plane++) {
        int width = avf_out->width;
        int height = avf_out->height;

        if (plane == 1 || plane == 2) {
            width = mi_ctx->chroma_width;
            height = mi_ctx->chroma_height;
        }

        for (y = 0; y < height; y++)
            for (x = 0; x < width; x++) {
                int i;
                int weight_sum = 0;
                int v = 0;
                Pixel *pixel;
                if (plane == 1 || plane == 2) //FIXME optimize
                    pixel = &mi_ctx->pixels[(x << mi_ctx->chroma_h_shift) + (y << mi_ctx->chroma_v_shift) * avf_out->width];
                else
                    pixel = &mi_ctx->pixels[x + y * avf_out->width];

                //FIXME optimize
                for (i = 0; i < pixel->nb; i++)
                    weight_sum += pixel->weights[i];

                if (!weight_sum || !pixel->nb) {
                    pixel->weights[0] = ALPHA_MAX - alpha;
                    pixel->refs[0] = 1;
                    pixel->mvs[0][0] = 0;
                    pixel->mvs[0][1] = 0;
                    pixel->weights[1] = alpha;
                    pixel->refs[1] = 2;
                    pixel->mvs[1][0] = 0;
                    pixel->mvs[1][1] = 0;
                    pixel->nb = 2;

                    weight_sum = ALPHA_MAX;
                }

                for (i = 0; i < pixel->nb; i++) {
                    int is_chroma = plane == 1 || plane == 2; //FIXME
                    int x_mv = x + (pixel->mvs[i][0] >> is_chroma);
                    int y_mv = y + (pixel->mvs[i][1] >> is_chroma);
                    Frame *frame = &mi_ctx->frames[pixel->refs[i]];

                    v += pixel->weights[i] * frame->avf->data[plane][x_mv + y_mv * frame->avf->linesize[plane]];
                }

                avf_out->data[plane][x + y * avf_out->linesize[plane]] = ROUNDED_DIV(v, weight_sum);
            }
    }
}

static void var_size_bmc(MIContext *mi_ctx, Block *block, int x_mb, int y_mb, int n, int alpha)
{
    int sb_x, sb_y;
    int width = mi_ctx->frames[0].avf->width;
    int height = mi_ctx->frames[0].avf->height;

    for (sb_y = 0; sb_y < 2; sb_y++)
        for (sb_x = 0; sb_x < 2; sb_x++) {
            Block *sb = &block->subs[sb_x + sb_y * 2];

            if (sb->sb)
                var_size_bmc(mi_ctx, sb, x_mb + (sb_x << (n - 1)), y_mb + (sb_y << (n - 1)), n - 1, alpha);
            else {
                int x, y;
                int mv_x = sb->mvs[0][0] * 2;
                int mv_y = sb->mvs[0][1] * 2;

                int start_x = x_mb + (sb_x << (n - 1));
                int start_y = y_mb + (sb_y << (n - 1));
                int end_x = start_x + (1 << (n - 1));
                int end_y = start_y + (1 << (n - 1));

                for (y = start_y; y < end_y; y++)  {
                    int y_min = -y;
                    int y_max = height - y - 1;
                    for (x = start_x; x < end_x; x++) {
                        int x_min = -x;
                        int x_max = width - x - 1;
                        Pixel *pixel = &mi_ctx->pixels[x + y * width];

                        PIXEL_INIT(PX_WEIGHT_MAX, mv_x, mv_y);
                    }
                }
            }
        }
}

static void obmc_bilateral(MIContext *mi_ctx, Block *block, int mb_x, int mb_y, int alpha)
{
    int x, y;
    int width = mi_ctx->frames[0].avf->width;
    int height = mi_ctx->frames[0].avf->height;

    Block *nb;
    int nb_x, nb_y;
    uint64_t sbads[9];

    int mv_x = block->mvs[0][0] * 2;
    int mv_y = block->mvs[0][1] * 2;
    int start_x, start_y;
    int startc_x, startc_y, endc_x, endc_y;

    if (mi_ctx->mc_mode == MC_MODE_AOBMC)
        for (nb_y = FFMAX(0, mb_y - 1); nb_y < FFMIN(mb_y + 2, mi_ctx->b_height); nb_y++)
            for (nb_x = FFMAX(0, mb_x - 1); nb_x < FFMIN(mb_x + 2, mi_ctx->b_width); nb_x++) {
                int x_nb = nb_x << mi_ctx->log2_mb_size;
                int y_nb = nb_y << mi_ctx->log2_mb_size;

                if (nb_x - mb_x || nb_y - mb_y)
                    sbads[nb_x - mb_x + 1 + (nb_y - mb_y + 1) * 3] = get_sbad(&mi_ctx->me_ctx, x_nb, y_nb, x_nb + block->mvs[0][0], y_nb + block->mvs[0][1]);
            }

    start_x = (mb_x << mi_ctx->log2_mb_size) - mi_ctx->mb_size / 2;
    start_y = (mb_y << mi_ctx->log2_mb_size) - mi_ctx->mb_size / 2;

    startc_x = av_clip(start_x, 0, width - 1);
    startc_y = av_clip(start_y, 0, height - 1);
    endc_x = av_clip(start_x + (2 << mi_ctx->log2_mb_size), 0, width - 1);
    endc_y = av_clip(start_y + (2 << mi_ctx->log2_mb_size), 0, height - 1);

    for (y = startc_y; y < endc_y; y++) {
        int y_min = -y;
        int y_max = height - y - 1;
        for (x = startc_x; x < endc_x; x++) {
            int x_min = -x;
            int x_max = width - x - 1;
            int obmc_weight = obmc_tab_linear[4 - mi_ctx->log2_mb_size][(x - start_x) + ((y - start_y) << (mi_ctx->log2_mb_size + 1))];
            Pixel *pixel = &mi_ctx->pixels[x + y * width];

            if (mi_ctx->mc_mode == MC_MODE_AOBMC) {
                nb_x = (((x - start_x) >> (mi_ctx->log2_mb_size - 1)) * 2 - 3) / 2;
                nb_y = (((y - start_y) >> (mi_ctx->log2_mb_size - 1)) * 2 - 3) / 2;

                if (nb_x || nb_y) {
                    uint64_t sbad = sbads[nb_x + 1 + (nb_y + 1) * 3];
                    nb = &mi_ctx->int_blocks[mb_x + nb_x + (mb_y + nb_y) * mi_ctx->b_width];

                    if (sbad && sbad != UINT64_MAX && nb->sbad != UINT64_MAX) {
                        int phi = av_clip(ALPHA_MAX * nb->sbad / sbad, 0, ALPHA_MAX);
                        obmc_weight = obmc_weight * phi / ALPHA_MAX;
                    }
                }
            }

            PIXEL_INIT(obmc_weight, mv_x, mv_y);
        }
    }
}

static void interpolate(AVFilterLink *inlink, AVFrame *avf_out)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi_ctx = ctx->priv;
    int x, y;
    int plane, alpha;
    int64_t pts;

    pts = av_rescale(avf_out->pts, (int64_t) ALPHA_MAX * outlink->time_base.num * inlink->time_base.den,
                                   (int64_t)             outlink->time_base.den * inlink->time_base.num);

    alpha = (pts - mi_ctx->frames[1].avf->pts * ALPHA_MAX) / (mi_ctx->frames[2].avf->pts - mi_ctx->frames[1].avf->pts);
    alpha = av_clip(alpha, 0, ALPHA_MAX);

    if (alpha == 0 || alpha == ALPHA_MAX) {
        av_frame_copy(avf_out, alpha ? mi_ctx->frames[2].avf : mi_ctx->frames[1].avf);
        return;
    }

    switch(mi_ctx->mi_mode) {
        case MI_MODE_DUP:
            av_frame_copy(avf_out, alpha > ALPHA_MAX / 2 ? mi_ctx->frames[2].avf : mi_ctx->frames[1].avf);

            break;
        case MI_MODE_BLEND:
            for (plane = 0; plane < mi_ctx->nb_planes; plane++) {
                int width = avf_out->width;
                int height = avf_out->height;

                if (plane == 1 || plane == 2) {
                    width = mi_ctx->chroma_width;
                    height = mi_ctx->chroma_height;
                }

                for (y = 0; y < height; y++) {
                    for (x = 0; x < width; x++) {
                        avf_out->data[plane][x + y * avf_out->linesize[plane]] =
                                          alpha  * mi_ctx->frames[2].avf->data[plane][x + y * mi_ctx->frames[2].avf->linesize[plane]] +
                            ((ALPHA_MAX - alpha) * mi_ctx->frames[1].avf->data[plane][x + y * mi_ctx->frames[1].avf->linesize[plane]] + 512) >> 10;
                    }
                }
            }

            break;
        case MI_MODE_MCI:
            if (mi_ctx->me_mode == ME_MODE_BIDIRECTIONAL) {
                obmc(mi_ctx, alpha);
                interpolate_pixels(mi_ctx, alpha, avf_out);
            } else if (mi_ctx->me_mode == ME_MODE_BILATERAL) {
                int mb_x, mb_y;
                Block *block;

            #if DEBUG_CLUSTERING
                int dx, dy;
                av_frame_copy(avf_out, mi_ctx->frames[2].avf);

                for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
                    for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                        int i = 0; int flag = 0;
                        block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];
                        for (y = FFMAX(mb_y - 1, 0); y < FFMIN(mb_y + 2, mi_ctx->b_height); y++)
                            for (x = FFMAX(mb_x - 1, 0); x < FFMIN(mb_x + 2, mi_ctx->b_width); x++) {
                                if ((x - mb_x) && (y - mb_y))
                                    continue;

                                if (block->cid != mi_ctx->int_blocks[x + y * mi_ctx->b_width].cid) {
                                    dx = x - mb_x;
                                    dy = y - mb_y;
                                    if (!dx && (block->cid == mi_ctx->int_blocks[x + (mb_y - dy) * mi_ctx->b_width].cid)) {
                                        if ((block->cid == mi_ctx->int_blocks[x - 1 + (mb_y - dy) * mi_ctx->b_width].cid) ||
                                            (block->cid == mi_ctx->int_blocks[x + 1 + (mb_y - dy) * mi_ctx->b_width].cid))
                                            flag = 1;
                                    } else if (!dy && (block->cid == mi_ctx->int_blocks[(mb_x - dx) + y * mi_ctx->b_width].cid)) {
                                        if ((block->cid == mi_ctx->int_blocks[(mb_x - dx) + (y - 1) * mi_ctx->b_width].cid) ||
                                            (block->cid == mi_ctx->int_blocks[(mb_x - dx) + (y - 1) * mi_ctx->b_width].cid))
                                            flag = 1;
                                    }
                                    i++;
                                }
                            }

                        if (block->cid > 0) {
                        //if (block->cid == mi_ctx->cls) {
                            for (y = mb_y * mi_ctx->mb_size; y < (mb_y + 1) * mi_ctx->mb_size; y++)
                                for (x = mb_x * mi_ctx->mb_size; x < (mb_x + 1) * mi_ctx->mb_size; x++) {
                                    if (block->sb) {
                                        avf_out->data[0][x + y * avf_out->linesize[0]] = 255;
                                    } else {
                                        int ins = block->cid % 5;
                                        //avf_out->data[0][x + y * avf_out->linesize[0]] /= (ins < 2 ? 2 : ins);
                                        block->mvs[0][0] = 0;
                                        block->mvs[0][1] = 0;
                                    }
                                }
                            //block->mvs[0][0] = 0;
                            //block->mvs[0][1] = 0;
                        } else {
                            //set MVs to 0
                            block->mvs[0][0] = 0;
                            block->mvs[0][1] = 0;
                        }
                    }

                mi_ctx->cls++;
                if (mi_ctx->cls >= mi_ctx->cls_max)
                    mi_ctx->cls = 1;
                for (int c = 0; c < mi_ctx->cls_max; c++) {
                    if (mi_ctx->clusters[mi_ctx->cls].nb < 2) {
                        mi_ctx->cls++;
                        if (mi_ctx->cls >= mi_ctx->cls_max) {
                            mi_ctx->cls = 0;
                            break;
                        }
                    }
                }

            #else
                for (y = 0; y < mi_ctx->frames[0].avf->height; y++)
                    for (x = 0; x < mi_ctx->frames[0].avf->width; x++)
                        mi_ctx->pixels[x + y * mi_ctx->frames[0].avf->width].nb = 0;

                for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
                    for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                        block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];

                        if (block->sb)
                            var_size_bmc(mi_ctx, block, mb_x << mi_ctx->log2_mb_size, mb_y << mi_ctx->log2_mb_size, mi_ctx->log2_mb_size, alpha);
                        obmc_bilateral(mi_ctx, block, mb_x, mb_y, alpha);

                    }

                interpolate_pixels(mi_ctx, alpha, avf_out);

            #endif

            #if EXPORT_MVS
                /* export MVs */
                AVFrameSideData *sd = av_frame_new_side_data(avf_out, AV_FRAME_DATA_MOTION_VECTORS, 4 * mi_ctx->b_count * sizeof(AVMotionVector));
                AVMotionVector *mv;
                int count = 0;

                mv = (AVMotionVector *) sd->data;
                for (mb_y = 0; mb_y < mi_ctx->b_height; mb_y++)
                    for (mb_x = 0; mb_x < mi_ctx->b_width; mb_x++) {
                        block = &mi_ctx->int_blocks[mb_x + mb_y * mi_ctx->b_width];

                        if (block->sb) {
                            int mb_size = mi_ctx->mb_size / 2;
                            for (y = 0; y < 2; y++)
                                for (x = 0; x < 2; x++) {
                                    mv->source = -1;
                                    mv->w = mb_size;
                                    mv->h = mb_size;
                                    mv->src_x = mb_size / 2 * (x ? 3 : 1) + (mb_x << mi_ctx->log2_mb_size) + block->subs[x + y * 2].mvs[0][0] * 2;
                                    mv->src_y = mb_size / 2 * (y ? 3 : 1) + (mb_y << mi_ctx->log2_mb_size) + block->subs[x + y * 2].mvs[0][1] * 2;
                                    mv->dst_x = mb_size / 2 * (x ? 3 : 1) + (mb_x << mi_ctx->log2_mb_size);
                                    mv->dst_y = mb_size / 2 * (y ? 3 : 1) + (mb_y << mi_ctx->log2_mb_size);
                                    mv++;
                                    count++;
                                }
                        } else {
                            mv->source = -1;
                            mv->w = mi_ctx->mb_size;
                            mv->h = mi_ctx->mb_size;
                            mv->src_x = mi_ctx->mb_size / 2 + (mb_x << mi_ctx->log2_mb_size) + block->mvs[0][0] * 2;
                            mv->src_y = mi_ctx->mb_size / 2 + (mb_y << mi_ctx->log2_mb_size) + block->mvs[0][1] * 2;
                            mv->dst_x = mi_ctx->mb_size / 2 + (mb_x << mi_ctx->log2_mb_size);
                            mv->dst_y = mi_ctx->mb_size / 2 + (mb_y << mi_ctx->log2_mb_size);
                            mv++;
                            count++;
                        }
                }

                for (; count < 4 * mi_ctx->b_count; count++) {
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

static int filter_frame(AVFilterLink *inlink, AVFrame *avf_in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi_ctx = ctx->priv;
    int ret;

    if (avf_in->pts == AV_NOPTS_VALUE) {
        ret = ff_filter_frame(ctx->outputs[0], avf_in);
        return ret;
    }

    if (!mi_ctx->frames[NB_FRAMES - 1].avf || avf_in->pts < mi_ctx->frames[NB_FRAMES - 1].avf->pts) {
        av_log(ctx, AV_LOG_VERBOSE, "Initializing out pts from input pts %"PRId64"\n", avf_in->pts);
        mi_ctx->out_pts = av_rescale_q(avf_in->pts, inlink->time_base, outlink->time_base);
    }

    if (!mi_ctx->frames[NB_FRAMES - 1].avf)
        if (ret = inject_frame(inlink, av_frame_clone(avf_in)))
            return ret;

    if (ret = inject_frame(inlink, avf_in))
        return ret;

    if (!mi_ctx->frames[0].avf)
        return 0;

    for (;;) {
        AVFrame *avf_out;

        if (av_compare_ts(mi_ctx->out_pts, outlink->time_base, mi_ctx->frames[2].avf->pts, inlink->time_base) > 0)
            break;

        if (!(avf_out = ff_get_video_buffer(ctx->outputs[0], inlink->w, inlink->h)))
            return AVERROR(ENOMEM);

        av_frame_copy_props(avf_out, mi_ctx->frames[NB_FRAMES - 1].avf);
        avf_out->pts = mi_ctx->out_pts++;

        interpolate(inlink, avf_out);

        if ((ret = ff_filter_frame(ctx->outputs[0], avf_out)) < 0)
            return ret;
    }
    return 0;
}

static av_cold void free_blocks(Block *block, int sb)
{
    if (block->subs)
        free_blocks(block->subs, 1);
    if (sb)
        av_freep(&block);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MIContext *mi_ctx = ctx->priv;
    int i, m;

    av_freep(&mi_ctx->pixels);
    if (mi_ctx->int_blocks)
        for (m = 0; m < mi_ctx->b_count; m++)
            free_blocks(&mi_ctx->int_blocks[m], 0);
    av_freep(&mi_ctx->int_blocks);

    for (i = 0; i < NB_FRAMES; i++) {
        Frame *frame = &mi_ctx->frames[i];
        av_freep(&frame->blocks);
        av_frame_free(&frame->avf);
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
