/**
 * Copyright (C) 2016 Davinder Singh (DSM_) <ds.mudhar<@gmail.com>
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
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/motion_vector.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

enum MEMethod {
    ME_METHOD_DS            = 1,
    ME_METHOD_EPZS          = 2,
    ME_METHOD_ESA           = 3,
    ME_METHOD_FSS           = 4,
    ME_METHOD_HEXBS         = 5,
    ME_METHOD_NTSS          = 6,
    ME_METHOD_TDLS          = 7,
    ME_METHOD_TSS           = 8,
    ME_METHOD_UMH           = 9,
};

typedef struct MEContext {
    const AVClass *class;
    AVMotionEstContext me_ctx;
    enum MEMethod method;               ///< motion estimation method

    int mb_size;                        ///< macroblock size
    int search_param;                   ///< search parameter
    int b_width, b_height, b_count;
    int log2_mb_size;

    AVFrame *prev, *cur, *next;

    int (*mv_table[3])[2][2];           ///< motion vectors of current & prev 2 frames
} MEContext;

#define OFFSET(x) offsetof(MEContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption mestimate_options[] = {
    { "method", "specify motion estimation method", OFFSET(method), AV_OPT_TYPE_INT, {.i64 = ME_METHOD_ESA}, ME_METHOD_DS, ME_METHOD_UMH, FLAGS, "method" },
        CONST("ds",   "diamond search",              ME_METHOD_DS,   "method"),
        CONST("epzs", "enhanced predictive zonal search",   ME_METHOD_EPZS, "method"),
        CONST("esa",  "exhaustive search",           ME_METHOD_ESA,  "method"),
        CONST("fss",  "four step search",            ME_METHOD_FSS,  "method"),
        CONST("hexbs",  "hexagon-based search",       ME_METHOD_HEXBS,  "method"),
        CONST("ntss", "new three step search",       ME_METHOD_NTSS, "method"),
        CONST("tdls", "two dimensional logarithmic search", ME_METHOD_TDLS, "method"),
        CONST("tss",  "three step search",           ME_METHOD_TSS,  "method"),
        CONST("umh",  "uneven multi-hexagon search", ME_METHOD_UMH,  "method"),
    { "mb_size", "specify macroblock size", OFFSET(mb_size), AV_OPT_TYPE_INT, {.i64 = 16}, 8, INT_MAX, FLAGS },
    { "search_param", "specify search parameter", OFFSET(search_param), AV_OPT_TYPE_INT, {.i64 = 7}, 4, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(mestimate);

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
        AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRAP,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static uint64_t get_sad(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int x_mv, int y_mv)
{
    const int linesize = me_ctx->linesize;
    uint8_t *data_ref = me_ctx->data_ref;
    uint8_t *data_cur = me_ctx->data_cur;
    uint64_t sad = 0;
    int i, j;

    data_ref += y_mv * linesize;
    data_cur += y_mb * linesize;

    for (j = 0; j < me_ctx->mb_size; j++)
        for (i = 0; i < me_ctx->mb_size; i++)
            sad += FFABS(data_ref[x_mv + i + j * linesize] - data_cur[x_mb + i + j * linesize]);

    return sad;
}

void ff_me_init_context(AVMotionEstContext *me_ctx, int mb_size, int search_param,
                        int width, int height)
{
    me_ctx->width = width;
    me_ctx->height = height;
    me_ctx->mb_size = mb_size;
    me_ctx->search_param = search_param;
    me_ctx->get_cost = &get_sad;
    me_ctx->x_max = width - mb_size + 1;
    me_ctx->y_max = height - mb_size + 1;
}

static int config_input(AVFilterLink *inlink)
{
    MEContext *s = inlink->dst->priv;
    int i;

    s->log2_mb_size = av_ceil_log2_c(s->mb_size);
    s->mb_size = 1 << s->log2_mb_size;

    s->b_width  = AV_CEIL_RSHIFT(inlink->w, s->log2_mb_size); //XXX how's inlink->w different from frame->width
    s->b_height = AV_CEIL_RSHIFT(inlink->h, s->log2_mb_size);
    s->b_count = s->b_width * s->b_height;

    for (i = 0; i < 3; i++) {
        s->mv_table[i] = av_mallocz_array(s->b_count, sizeof(*s->mv_table[0]));
        if (!s->mv_table[i])
            return AVERROR(ENOMEM);
    }

    ff_me_init_context(&s->me_ctx, s->mb_size, s->search_param, inlink->w, inlink->h);

    return 0;
}

#define COST_MV(x, y)\
{\
    cost = me_ctx->get_cost(me_ctx, x_mb, y_mb, x, y);\
    if (cost < cost_min) {\
        cost_min = cost;\
        mv[0] = x;\
        mv[1] = y;\
    }\
}

#define COST_4_MV(x, y)\
if (x >= x_min && x <= x_max && y >= y_min && y <= y_max)\
    COST_MV(x, y)

static void search_mv_esa(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    for (y = y_min; y <= y_max; y++)
        for (x = x_min; x <= x_max; x++)
            COST_MV(x, y)
}

static void search_mv_tss(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    int step = ROUNDED_DIV(me_ctx->search_param, 2);
    int i;

    int square[8][2] = {{0,-1}, {0,1}, {-1,0}, {1,0}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};

    mv[0] = x_mb;
    mv[1] = y_mb;

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    do {
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 8; i++)
            COST_4_MV(x + square[i][0] * step, y + square[i][1] * step)

        step = step / 2;

    } while (step > 0);
}

static void search_mv_ntss(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    int step = ROUNDED_DIV(me_ctx->search_param, 2);
    int first_step = 1;
    int i;

    int square[8][2] = {{0,-1}, {0,1}, {-1,0}, {1,0}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};

    mv[0] = x_mb;
    mv[1] = y_mb;

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    do {
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 8; i++)
            COST_4_MV(x + square[i][0] * step, y + square[i][1] * step)

        // addition to TSS in NTSS
        if (first_step) {

            for (i = 0; i < 8; i++)
                COST_4_MV(x + square[i][0], y + square[i][1])

            if (x == mv[0] && y == mv[1])
                return;

            if (FFABS(x - mv[0]) <= 1 && FFABS(y - mv[1]) <= 1) {
                x = mv[0];
                y = mv[1];

                for (i = 0; i < 8; i++)
                    COST_4_MV(x + square[i][0], y + square[i][1])
                return;
            }

            first_step = 0;
        }

        step = step / 2;

    } while (step > 0);
}

static void search_mv_tdls(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    int step = ROUNDED_DIV(me_ctx->search_param, 2);
    int i;

    int dia2[4][2] = {{-1, 0}, { 0,-1},
                      { 1, 0}, { 0, 1}};

    mv[0] = x_mb;
    mv[1] = y_mb;

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    do {
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 4; i++)
            COST_4_MV(x + dia2[i][0] * step, y + dia2[i][1] * step)

        if (x == mv[0] && y == mv[1])
            step = step / 2;

    } while (step > 0);
}

static void search_mv_fss(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    int step = 2;
    int i;

    int square[8][2] = {{0,-1}, {0,1}, {-1,0}, {1,0}, {-1,-1}, {-1,1}, {1,-1}, {1,1}};

    mv[0] = x_mb;
    mv[1] = y_mb;

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    do {
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 8; i++)
            COST_4_MV(x + square[i][0] * step, y + square[i][1] * step)

        if (x == mv[0] && y == mv[1])
            step = step / 2;

    } while (step > 0);
}

static void search_mv_ds(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    int i;
    int dir_x, dir_y;

    int dia[8][2] = {{-2, 0}, {-1,-1}, { 0,-2}, { 1,-1},
                     { 2, 0}, { 1, 1}, { 0, 2}, {-1, 1}};
    int dia2[4][2] = {{-1, 0}, { 0,-1},
                      { 1, 0}, { 0, 1}};

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    x = x_mb; y = y_mb;
    dir_x = dir_y = 0;

    do {
        x = mv[0];
        y = mv[1];

    #if 1
        for (i = 0; i < 8; i++)
            COST_4_MV(x + dia[i][0], y + dia[i][1]);
    #else
        /*
           this one is slightly faster than above version,
           it skips previously examined 3 or 5 locations based on prev origin.
        */
        if (dir_x <= 0)
            COST_4_MV(x - 2, y)
        if (dir_x <= 0 && dir_y <= 0)
            COST_4_MV(x - 1, y - 1)
        if (dir_y <= 0)
            COST_4_MV(x, y - 2)
        if (dir_x >= 0 && dir_y <= 0)
            COST_4_MV(x + 1, y - 1)
        if (dir_x >= 0)
            COST_4_MV(x + 2, y)
        if (dir_x >= 0 && dir_y >= 0)
            COST_4_MV(x + 1, y + 1)
        if (dir_y >= 0)
            COST_4_MV(x, y + 2)
        if (dir_x <= 0 && dir_y >= 0)
            COST_4_MV(x - 1, y + 1)

        dir_x = mv[0] - x;
        dir_y = mv[1] - y;
    #endif

    } while (x != mv[0] || y != mv[1]);

    for (i = 0; i < 4; i++)
        COST_4_MV(x + dia2[i][0], y + dia2[i][1])
}

void search_mv_epzs(AVMotionEstContext *me_ctx, int pred[11][2], int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    uint64_t threshold;
    int i;

    int dia[8][2] = {{-1, 0}, {-1,-1}, { 0,-1}, { 1,-1},
                     { 1, 0}, { 1, 1}, { 0, 1}, {-1, 1}};

    cost_min = UINT64_MAX;

    COST_4_MV(pred[0][0], pred[0][1])

    if (cost_min > 256) {
        threshold = cost_min;
        for (i = 1; i < 9; i++) {
            if (i >= 1 && i <= 6)
                threshold = FFMIN(cost_min, threshold);

            COST_4_MV(pred[i][0], pred[i][1])

            if (cost_min <= 1.2 * threshold + 128)
                break;
        }
    }

    do {
        x = mv[0];
        y = mv[1];

        if (FFABS(mv[0] - x_mb) > me_ctx->search_param || FFABS(mv[1] - y_mb) > me_ctx->search_param)
            break;

        for (i = 0; i < 8; i++) {
            if (x + dia[i][0] >= 0 && x + dia[i][0] < me_ctx->x_max &&
                y + dia[i][1] >= 0 && y + dia[i][1] < me_ctx->y_max)
                COST_MV(x + dia[i][0], y + dia[i][1]);
        }

    } while (x != mv[0] || y != mv[1]);
}

static void search_mv_hexbs(AVMotionEstContext *me_ctx, int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min;
    int i;

    int hex2[6][2] = {{-2, 0}, {-1,-2}, {-1, 2},
                      { 1,-2}, { 1, 2}, { 2, 0}};

    int dia2[4][2] = {{-1, 0}, { 0,-1},
                      { 1, 0}, { 0, 1}};

    if (!(cost_min = me_ctx->get_cost(me_ctx, x_mb, y_mb, x_mb, y_mb)))
        return;

    do {
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 6; i++)
            COST_4_MV(x + hex2[i][0], y + hex2[i][1]);

    } while (x != mv[0] || y != mv[1]);

    for (i = 0; i < 4; i++)
        COST_4_MV(x + dia2[i][0], y + dia2[i][1])
}

void search_mv_umh(AVMotionEstContext *me_ctx, int pred[5][2], int x_mb, int y_mb, int *mv)
{
    int x, y;
    int x_min = FFMAX(0, x_mb - me_ctx->search_param);
    int y_min = FFMAX(0, y_mb - me_ctx->search_param);
    int x_max = FFMIN(x_mb + me_ctx->search_param, me_ctx->x_max - 1);
    int y_max = FFMIN(y_mb + me_ctx->search_param, me_ctx->y_max - 1);
    uint64_t cost, cost_min = UINT64_MAX;
    int d, i;
    int end_x, end_y;

    int hex[16][2] = {{-4,-2}, {-4,-1}, {-4, 0}, {-4, 1}, {-4, 2},
                      { 4,-2}, { 4,-1}, { 4, 0}, { 4, 1}, { 4, 2},
                      {-2, 3}, { 0, 4}, { 2, 3},
                      {-2,-3}, { 0,-4}, { 2,-3}};
    int hex2[6][2] = {{-2, 0}, {-1,-2}, {-1, 2},
                      { 1,-2}, { 1, 2}, { 2, 0}};
    int dia2[4][2] = {{-1, 0}, { 0,-1},
                      { 1, 0}, { 0, 1}};

    for (i = 0; i < 5; i++)
        COST_4_MV(pred[i][0], pred[i][1])

    // Unsymmetrical-cross Search
    x = mv[0];
    y = mv[1];
    for (d = 1; d <= me_ctx->search_param; d += 2) { //TODO test
        COST_4_MV(x - d, y)
        COST_4_MV(x + d, y)
        if (d <= me_ctx->search_param / 2) {
            COST_4_MV(x, y - d)
            COST_4_MV(x, y + d)
        }
    }

    // Uneven Multi-Hexagon-Grid Search
    end_x = FFMIN(mv[0] + 2, x_max);
    end_y = FFMIN(mv[1] + 2, y_max);
    for (y = FFMAX(y_min, mv[1] - 2); y <= end_y; y++)
        for (x = FFMAX(x_min, mv[0] - 2); x <= end_x; x++)
            COST_4_MV(x, y)

    x = mv[0];
    y = mv[1];
    for (d = 1; d <= me_ctx->search_param / 4; d++)
        for (i = 1; i < 16; i++)
            COST_4_MV(x + hex[i][0] * d, y + hex[i][1] * d)

    // Extended Hexagon-based Search
    do {
        x = mv[0];
        y = mv[1];

        for (i = 0; i < 6; i++)
            COST_4_MV(x + hex2[i][0], y + hex2[i][1]);

    } while (x != mv[0] || y != mv[1]);

    for (i = 0; i < 4; i++)
        COST_4_MV(x + dia2[i][0], y + dia2[i][1]);
}

static void add_mv_data(AVMotionVector *mv, int mb_size,
                        int x, int y, int x_mv, int y_mv, int dir)
{
    mv->w = mb_size;
    mv->h = mb_size;
    mv->dst_x = x + (mb_size >> 1);
    mv->dst_y = y + (mb_size >> 1);
    mv->src_x = x_mv + (mb_size >> 1);
    mv->src_y = y_mv + (mb_size >> 1);
    mv->source = dir ? 1 : -1;
    mv->flags = 0;
}

#define SEARCH_MV(method)\
    do {\
        for (mb_y = 0; mb_y < s->b_height; mb_y++)\
            for (mb_x = 0; mb_x < s->b_width; mb_x++) {\
                const int x_mb = mb_x << s->log2_mb_size;\
                const int y_mb = mb_y << s->log2_mb_size;\
                int mv[2] = {x_mb, y_mb};\
                search_mv_##method(me_ctx, x_mb, y_mb, mv);\
                add_mv_data(((AVMotionVector *) sd->data) + mv_count++, me_ctx->mb_size, x_mb, y_mb, mv[0], mv[1], dir);\
            }\
    } while (0)

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;
    AVMotionEstContext *me_ctx = &s->me_ctx;
    AVFrameSideData *sd;
    AVFrame *out;
    int mb_x, mb_y, dir;
    int mv_x, mv_y;
    int32_t mv_count = 0;

    av_assert0(frame->pts != AV_NOPTS_VALUE); //FIXME if (AV_NOPTS_VALUE) return 0?

    av_frame_free(&s->prev);
    s->prev = s->cur;
    s->cur  = s->next;
    s->next = frame;

    s->mv_table[2] = memcpy(s->mv_table[2], s->mv_table[1], sizeof(*s->mv_table[1]) * s->b_count);
    s->mv_table[1] = memcpy(s->mv_table[1], s->mv_table[0], sizeof(*s->mv_table[0]) * s->b_count);

    if (!s->cur) {
        s->cur = av_frame_clone(frame);
        if (!s->cur)
            return AVERROR(ENOMEM);
    }

    if (!s->prev)
        return 0;

    out = av_frame_clone(s->cur);
    if (!out)
        return AVERROR(ENOMEM);

    sd = av_frame_new_side_data(out, AV_FRAME_DATA_MOTION_VECTORS, 2 * s->b_count * sizeof(AVMotionVector));
    if (!sd)
        return AVERROR(ENOMEM);

    me_ctx->data_cur = s->cur->data[0];
    me_ctx->linesize = s->cur->linesize[0];

    for (dir = 0; dir < 2; dir++) {
        me_ctx->data_ref = (dir ? s->next : s->prev)->data[0];

        if (s->method == ME_METHOD_DS)
            SEARCH_MV(ds);
        else if (s->method == ME_METHOD_ESA)
            SEARCH_MV(esa);
        else if (s->method == ME_METHOD_FSS)
            SEARCH_MV(fss);
        else if (s->method == ME_METHOD_NTSS)
            SEARCH_MV(ntss);
        else if (s->method == ME_METHOD_TDLS)
            SEARCH_MV(tdls);
        else if (s->method == ME_METHOD_TSS)
            SEARCH_MV(tss);
        else if (s->method == ME_METHOD_HEXBS)
            SEARCH_MV(hexbs);
        else if (s->method == ME_METHOD_UMH) {
            for (mb_y = 0; mb_y < s->b_height; mb_y++)
                for (mb_x = 0; mb_x < s->b_width; mb_x++) {
                    const int mb_i = mb_x + mb_y * s->b_width;
                    const int x_mb = mb_x << s->log2_mb_size;
                    const int y_mb = mb_y << s->log2_mb_size;
                    int mv[2] = {x_mb, y_mb};
                    int pred[5][2];

                    pred[1][0] = x_mb;
                    pred[1][1] = y_mb;

                    /* according to rules specified in paper, when nb is out of frame */

                    //left mb in current frame
                    if (mb_x != 0) {
                        pred[2][0] = s->mv_table[0][mb_x - 1 + mb_y * s->b_width][dir][0];
                        pred[2][1] = s->mv_table[0][mb_x - 1 + mb_y * s->b_width][dir][1];
                    } else {
                        pred[2][0] = x_mb;
                        pred[2][1] = y_mb;
                    }

                    //top mb in current frame
                    if (mb_y != 0) {
                        pred[3][0] = s->mv_table[0][mb_x + (mb_y - 1) * s->b_width][dir][0];
                        pred[3][1] = s->mv_table[0][mb_x + (mb_y - 1) * s->b_width][dir][1];
                        //top-right mb in current frame
                        if (mb_x < s->b_width - 1) {
                            pred[4][0] = s->mv_table[0][mb_x + 1 + (mb_y - 1) * s->b_width][dir][0];
                            pred[4][1] = s->mv_table[0][mb_x + 1 + (mb_y - 1) * s->b_width][dir][1];
                        } else if (mb_x != 0) {
                            //top-left mb in current frame
                            pred[4][0] = s->mv_table[0][mb_x - 1 + (mb_y - 1) * s->b_width][dir][0];
                            pred[4][1] = s->mv_table[0][mb_x - 1 + (mb_y - 1) * s->b_width][dir][1];
                        } else {
                            pred[4][0] = pred[2][0];
                            pred[4][1] = pred[2][1];
                        }
                    } else {
                        pred[3][0] = pred[2][0];
                        pred[3][1] = pred[2][1];
                        pred[4][0] = pred[2][0];
                        pred[4][1] = pred[2][1];
                    }

                    //median predictor
                    pred[0][0] = mid_pred(pred[2][0], pred[3][0], pred[4][0]);
                    pred[0][1] = mid_pred(pred[2][1], pred[3][1], pred[4][1]);

                    search_mv_umh(me_ctx, pred, x_mb, y_mb, mv);

                    s->mv_table[0][mb_i][dir][0] = mv[0];
                    s->mv_table[0][mb_i][dir][1] = mv[1];
                    add_mv_data(((AVMotionVector *) sd->data) + mv_count++, me_ctx->mb_size, x_mb, y_mb, mv[0], mv[1], dir);
                }

        } else if (s->method == ME_METHOD_EPZS) {

            for (mb_y = 0; mb_y < s->b_height; mb_y++)
                for (mb_x = 0; mb_x < s->b_width; mb_x++) {
                    const int mb_i = mb_x + mb_y * s->b_width;
                    const int x_mb = mb_x << s->log2_mb_size;
                    const int y_mb = mb_y << s->log2_mb_size;
                    int mv[2] = {x_mb, y_mb};
                    int pred[11][2];
                    int p;

                    for (p = 0; p < 11; p++) {
                        pred[p][0] = x_mb;
                        pred[p][1] = y_mb;
                    }

                    //left mb in current frame
                    if (mb_x != 0) {
                        pred[2][0] = s->mv_table[0][mb_x - 1 + mb_y * s->b_width][dir][0];
                        pred[2][1] = s->mv_table[0][mb_x - 1 + mb_y * s->b_width][dir][1];
                    }

                    //top mb in current frame
                    if (mb_y != 0) {
                        pred[3][0] = s->mv_table[0][mb_x + (mb_y - 1) * s->b_width][dir][0];
                        pred[3][1] = s->mv_table[0][mb_x + (mb_y - 1) * s->b_width][dir][1];
                    }

                    //top-right mb in current frame
                    if (mb_y != 0 && mb_x < s->b_width - 1) {
                        pred[4][0] = s->mv_table[0][mb_x + 1 + (mb_y - 1) * s->b_width][dir][0];
                        pred[4][1] = s->mv_table[0][mb_x + 1 + (mb_y - 1) * s->b_width][dir][1];
                    }

                    //median predictor
                    pred[0][0] = mid_pred(pred[2][0], pred[3][0], pred[4][0]);
                    pred[0][1] = mid_pred(pred[2][1], pred[3][1], pred[4][1]);

                    //collocated mb in prev frame
                    pred[5][0] = s->mv_table[1][mb_i][dir][0];
                    pred[5][1] = s->mv_table[1][mb_i][dir][1];

                    //accelerator motion vector of collocated block in prev frame
                    pred[6][0] = s->mv_table[1][mb_i][dir][0] + (s->mv_table[1][mb_i][dir][0] - s->mv_table[2][mb_i][dir][0]);
                    pred[6][1] = s->mv_table[1][mb_i][dir][1] + (s->mv_table[1][mb_i][dir][1] - s->mv_table[2][mb_i][dir][1]);
                    pred[6][0] = av_clip(pred[6][0], 0, me_ctx->x_max - 1);
                    pred[6][1] = av_clip(pred[6][1], 0, me_ctx->y_max - 1);

                    //left mb in prev frame
                    if (mb_x != 0) {
                        pred[7][0] = s->mv_table[1][mb_x - 1 + mb_y * s->b_width][dir][0];
                        pred[7][1] = s->mv_table[1][mb_x - 1 + mb_y * s->b_width][dir][1];
                    }

                    //top mb in prev frame
                    if (mb_y != 0) {
                        pred[8][0] = s->mv_table[1][mb_x + (mb_y - 1) * s->b_width][dir][0];
                        pred[8][1] = s->mv_table[1][mb_x + (mb_y - 1) * s->b_width][dir][1];
                    }

                    //right mb in prev frame
                    if (mb_x < s->b_width - 1) {
                        pred[9][0] = s->mv_table[1][mb_x + 1 + mb_y * s->b_width][dir][0];
                        pred[9][1] = s->mv_table[1][mb_x + 1 + mb_y * s->b_width][dir][1];
                    }

                    //bottom mb in prev frame
                    if (mb_y < s->b_height - 1) {
                        pred[10][0] = s->mv_table[1][mb_x + (mb_y + 1) * s->b_width][dir][0];
                        pred[10][1] = s->mv_table[1][mb_x + (mb_y + 1) * s->b_width][dir][1];
                    }

                    search_mv_epzs(me_ctx, pred, x_mb, y_mb, mv);

                    s->mv_table[0][mb_i][dir][0] = mv[0];
                    s->mv_table[0][mb_i][dir][1] = mv[1];
                    add_mv_data(((AVMotionVector *) sd->data) + mv_count++, s->mb_size, x_mb, y_mb, mv[0], mv[1], dir);
                }
        }
    }

    return ff_filter_frame(ctx->outputs[0], out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MEContext *s = ctx->priv;

    av_frame_free(&s->prev);
    av_frame_free(&s->cur);
    av_frame_free(&s->next);
}

static const AVFilterPad mestimate_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
        .config_props  = config_input,
    },
    { NULL }
};

static const AVFilterPad mestimate_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_mestimate = {
    .name          = "mestimate",
    .description   = NULL_IF_CONFIG_SMALL("Generate motion vectors."),
    .priv_size     = sizeof(MEContext),
    .priv_class    = &mestimate_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = mestimate_inputs,
    .outputs       = mestimate_outputs,
};
