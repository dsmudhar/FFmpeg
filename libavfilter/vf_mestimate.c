/**
 *
 * Developed by Davinder Singh (DSM_ / @dsmudhar) during GSoC 2016
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
    ME_METHOD_DIA           = 1,
    ME_METHOD_ESA           = 2,
    ME_METHOD_FSS           = 3,
    ME_METHOD_HEX           = 4,
    ME_METHOD_NTSS          = 5,
    ME_METHOD_STAR          = 6,
    ME_METHOD_TDLS          = 7,
    ME_METHOD_TSS           = 8,
    ME_METHOD_UMH           = 9,
};

typedef struct MEContext {
    AVFrame *prev, *cur, *next;
    int mb_size;
    int search_param;
    int64_t (*get_cost)(struct MEContext *me_ctx, int x_mb, int y_mb,
                        int mv_x, int mv_y, int dir);
    int width;
    int height;
} MEContext;

typedef struct MEFilterContext {
    const AVClass *class;
    MEContext *me_ctx;
    enum MEMethod method;               ///< motion estimation method

    int mb_size;                        ///< macroblock size
    int search_param;                   ///< search parameter
    int b_width, b_height;
    int log2_mb_size;

    AVMotionVector *mvs;                ///< motion vectors for current frame
    int32_t mv_count;                   ///< current frame motion vector count
} MEFilterContext;

#define OFFSET(x) offsetof(MEFilterContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption mestimate_options[] = {
    { "method", "specify motion estimation method", OFFSET(method), AV_OPT_TYPE_INT, {.i64 = ME_METHOD_ESA}, ME_METHOD_DIA, /*TODO*/ME_METHOD_UMH, FLAGS, "method" },
        CONST("dia",  "diamond search",              ME_METHOD_DIA,  "method"),
        CONST("esa",  "exhaustive search",           ME_METHOD_ESA,  "method"),
        CONST("fss",  "four step search",            ME_METHOD_FSS,  "method"),
        CONST("hex",  "hexagonal search",            ME_METHOD_HEX,  "method"),
        CONST("ntss", "new three step search",       ME_METHOD_NTSS, "method"),
        CONST("star", "star search",                 ME_METHOD_STAR, "method"),
        CONST("tdls", "2D logarithmic search",       ME_METHOD_TDLS, "method"),
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

static void init_me_context(MEContext *me_ctx, int width, int height,
                            int mb_size, int search_param)
{
    me_ctx->prev = me_ctx->cur = me_ctx->next = NULL;
    me_ctx->width = width; //XXX how's inlink->w different from frame->width
    me_ctx->height = height;
    me_ctx->mb_size = mb_size;
    me_ctx->search_param = search_param;
}

static int config_input(AVFilterLink *inlink)
{
    MEFilterContext *s = inlink->dst->priv;
    MEContext *me_ctx;

    s->log2_mb_size = ff_log2(s->mb_size); //HACK round off first?
    s->mb_size = 1 << s->log2_mb_size;

    s->b_width  = AV_CEIL_RSHIFT(inlink->w, s->log2_mb_size);
    s->b_height = AV_CEIL_RSHIFT(inlink->h, s->log2_mb_size);

    s->mvs = av_malloc_array(s->b_width * s->b_height, 2 * sizeof(AVMotionVector));
    if (!s->mvs)
        return AVERROR(ENOMEM);

    me_ctx = av_malloc(sizeof(MEContext));
    if (!me_ctx)
        return AVERROR(ENOMEM);

    s->me_ctx = me_ctx;
    init_me_context(me_ctx, inlink->w, inlink->h, s->mb_size, s->search_param);

    return 0;
}

static uint64_t get_mad(MEContext *me_ctx, int x_mb, int y_mb, int mv_x, int mv_y, int direction)
{
    uint8_t *buf_ref = (direction ? me_ctx->next : me_ctx->prev)->data[0];
    uint8_t *buf_cur = me_ctx->cur->data[0];
    int linesize = me_ctx->cur->linesize[0];
    int64_t mad = 0;
    int i, j;

    av_assert0(linesize == (direction ? me_ctx->next : me_ctx->prev)->linesize[0]);

    buf_ref += (y_mb + mv_y) * linesize;
    buf_cur += y_mb * linesize;

    for (j = 0; j < me_ctx->mb_size; j++)
        for (i = 0; i < me_ctx->mb_size; i++)
            mad += FFABS(buf_ref[x_mb + mv_x + i + j * linesize] - buf_cur[x_mb + i + j * linesize]);

    return mad;
}

static void search_mv_esa(MEContext *me_ctx, int x_mb, int y_mb, int *mv_x, int *mv_y, int dir)
{
    int x, y;
    int p = me_ctx->search_param;
    int start_x = av_clip(x_mb - p, 0, x_mb);
    int start_y = av_clip(y_mb - p, 0, y_mb);
    int end_x = av_clip(x_mb + p + 1, 0, me_ctx->width - me_ctx->mb_size);
    int end_y = av_clip(y_mb + p + 1, 0, me_ctx->height - me_ctx->mb_size);
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(me_ctx, x_mb, y_mb, 0, 0, dir)))
        return;

    for (y = start_y; y < end_y; y++)
        for (x = start_x; x < end_x; x++) {
            if ((cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir)) < cost_min) {
                cost_min = cost;
                *mv_x = x - x_mb;
                *mv_y = y - y_mb;
            } else if (cost == cost_min) {
                int mv_x1 = x - x_mb;
                int mv_y1 = y - y_mb;
                if (mv_x1 * mv_x1 + mv_y1 * mv_y1 < *mv_x * *mv_x + *mv_y * *mv_y) {
                    *mv_x = mv_x1;
                    *mv_y = mv_y1;
                }
            }
        }
}

static void search_mv_tss(MEContext *me_ctx, int x_mb, int y_mb, int *mv_x, int *mv_y, int dir)
{
    int x, y;
    int start_x, start_y, end_x, end_y;
    int step = ROUNDED_DIV(me_ctx->search_param, 2);
    int x_min_cost = x_mb, y_min_cost = y_mb;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(me_ctx, x_mb, y_mb, 0, 0, dir)))
        return;

    do {
        start_x = av_clip(x_min_cost - step, 0, x_min_cost);
        start_y = av_clip(y_min_cost - step, 0, y_min_cost);
        end_x = av_clip(x_min_cost + step + 1, 0, me_ctx->width - me_ctx->mb_size);
        end_y = av_clip(y_min_cost + step + 1, 0, me_ctx->height - me_ctx->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                //if (x == start_x + step && y == start_y + step) //FIXME
                //    continue;

                cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                if (!cost) {
                    *mv_x = x - x_mb;
                    *mv_y = y - y_mb;
                    return;
                } else if (cost < cost_min) {
                    cost_min = cost;
                    x_min_cost = x;
                    y_min_cost = y;
                }
            }
        }
        step = step / 2;

    } while (step > 0);

    *mv_x = x_min_cost - x_mb;
    *mv_y = y_min_cost - y_mb;
}

static void search_mv_ntss(MEContext *me_ctx, int x_mb, int y_mb, int *mv_x, int *mv_y, int dir)
{
    int x, y;
    int start_x, start_y, end_x, end_y;
    int step = ROUNDED_DIV(me_ctx->search_param, 2);
    int x_min_cost = x_mb, y_min_cost = y_mb;
    uint64_t cost, cost_min;
    int first_step = 1;

    if (!(cost_min = get_mad(me_ctx, x_mb, y_mb, 0, 0, dir)))
        return;

    do {
        start_x = av_clip(x_min_cost - step, 0, x_min_cost);
        start_y = av_clip(y_min_cost - step, 0, y_min_cost);
        end_x = av_clip(x_min_cost + step + 1, 0, me_ctx->width - me_ctx->mb_size);
        end_y = av_clip(y_min_cost + step + 1, 0, me_ctx->height - me_ctx->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                //if (x == start_x + step && y == start_y + step) //FIXME
                //    continue;

                cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                if (cost < cost_min) {
                    cost_min = cost;
                    x_min_cost = x;
                    y_min_cost = y;
                } else if (!cost && !first_step) {
                    *mv_x = x - x_mb;
                    *mv_y = y - y_mb;
                    return;
                }
            }
        }

        // addition to TSS in NTSS
        if (first_step) {
            int mv_x1, mv_y1;
            start_x = av_clip(x_mb - 1, 0, x_mb);
            start_y = av_clip(y_mb - 1, 0, y_mb);
            end_x = av_clip(x_mb + 2, 0, me_ctx->width - me_ctx->mb_size);
            end_y = av_clip(y_mb + 2, 0, me_ctx->height - me_ctx->mb_size);

            for (y = start_y; y < end_y; y++) {
                for (x = start_x; x < end_x; x++) {

                    if (x == x_mb && y == y_mb)
                        continue;

                    cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                    if (!cost) {
                        *mv_x = x - x_mb;
                        *mv_y = y - y_mb;
                        return;
                    } else if (cost < cost_min) {
                        cost_min = cost;
                        x_min_cost = x;
                        y_min_cost = y;
                    }
                }
            }

            mv_x1 = x_min_cost - x_mb;
            mv_y1 = y_min_cost - y_mb;

            if (!mv_x1 && !mv_y1)
                return;

            if (FFABS(mv_x1) <= 1 && FFABS(mv_y1) <= 1) {
                int i, j;
                start_x = x_mb + mv_x1;
                start_y = y_mb + mv_y1;
                // no need to clip here, since block size is >= 2
                for (j = -1; j <= 1; j++) {
                    y = start_y + j;
                    for (i = -1; i <= 1; i++) {
                        if ((1 - i * mv_x1) && (1 - j * mv_y1))
                            continue;

                        x = start_x + i;
                        cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                        if (!cost) {
                            *mv_x = x - x_mb;
                            *mv_y = y - y_mb;
                            return;
                        } else if (cost < cost_min) {
                            cost_min = cost;
                            x_min_cost = x;
                            y_min_cost = y;
                        }
                    }
                }

                *mv_x = x_min_cost - x_mb;
                *mv_y = y_min_cost - y_mb;
                return;
            }

            first_step = 0;
        }

        step = step / 2;

    } while (step > 0);

    *mv_x = x_min_cost - x_mb;
    *mv_y = y_min_cost - y_mb;
}

static void search_mv_tdls(MEContext *me_ctx, int x_mb, int y_mb, int *mv_x, int *mv_y, int dir)
{
    int x, y;
    int x_orig = x_mb, y_orig = y_mb;
    int start_x, start_y, end_x, end_y;
    int step = ROUNDED_DIV(me_ctx->search_param, 2);
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(me_ctx, x_mb, y_mb, 0, 0, dir)))
        return;

    do {
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_y = av_clip(y_orig + step + 1, 0, me_ctx->height - me_ctx->mb_size);

        for (y = start_y; y < end_y; y += step) {

            if (y == start_y + step) {
                start_x = av_clip(x_orig - step, 0, x_orig);
                end_x = av_clip(x_orig + step + 1, 0, me_ctx->width - me_ctx->mb_size);
            } else {
                start_x = x_orig;
                end_x = x_orig + 1;
            }

            for (x = start_x; x < end_x; x += step) {

                if (y == y_orig && x == x_orig)
                    continue;

                cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                if (!cost) {
                    *mv_x = x - x_mb;
                    *mv_y = y - y_mb;
                    return;
                } else if (cost < cost_min) {
                    cost_min = cost;
                    x_min_cost = x;
                    y_min_cost = y;
                }
            }
        }

        if (y_min_cost == y_orig || FFABS(y_min_cost - y_mb) >= me_ctx->search_param) { //FIXME
            step = step / 2;
        } else {
            x_orig = x_min_cost;
            y_orig = y_min_cost;
        }

    } while (step > 0);

    *mv_x = x_min_cost - x_mb;
    *mv_y = y_min_cost - y_mb;
}

static void search_mv_fss(MEContext *me_ctx, int x_mb, int y_mb, int *mv_x, int *mv_y, int dir)
{
    int x, y;
    int x_orig = x_mb, y_orig = y_mb;
    int start_x, start_y, end_x, end_y;
    int step = 2;
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(me_ctx, x_mb, y_mb, 0, 0, dir)))
        return;

    do {
        start_x = av_clip(x_orig - step, 0, x_orig);
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_x = av_clip(x_orig + step + 1, 0, me_ctx->width - me_ctx->mb_size);
        end_y = av_clip(y_orig + step + 1, 0, me_ctx->height - me_ctx->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                // skips already checked current origin
                if (x == x_orig && y == y_orig)
                    continue;

                cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                if (!cost) {
                    *mv_x = x - x_mb;
                    *mv_y = y - y_mb;
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

        if (x_min_cost == x_orig && y_min_cost == y_orig)
            step = 1;
        else {
            int i, j, mv_x1, mv_y1;
            do {
                mv_x1 = x_min_cost - x_orig;
                mv_y1 = y_min_cost - y_orig;

                x_orig = x_min_cost;
                y_orig = y_min_cost;

                for (j = -step; j <= step; j += step) { //FIXME clip
                    y = y_orig + j;
                    for (i = -step; i <= step; i += step) {
                        if ((4 - i * mv_x1) && (4 - j * mv_y1))
                            continue;

                        x = x_orig + i;
                        cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                        if (!cost) {
                            *mv_x = x - x_mb;
                            *mv_y = y - y_mb;
                            return;
                        } else if (cost < cost_min) {
                            cost_min = cost;
                            x_min_cost = x;
                            y_min_cost = y;
                        }
                    }
                }

                if (x_min_cost == x_orig && y_min_cost == y_orig ||
                    FFABS(x_min_cost - x_mb) >= me_ctx->search_param - 1 || FFABS(y_min_cost - y_mb) >= me_ctx->search_param - 1)
                    step = 1;

            } while (step > 1);
        }

    } while (step > 0);

    *mv_x = x_min_cost - x_mb;
    *mv_y = y_min_cost - y_mb;
}

static void search_mv_dia(MEContext *me_ctx, int x_mb, int y_mb, int *mv_x, int *mv_y, int dir)
{
    int x, y;
    int x_orig = x_mb, y_orig = y_mb;
    int start_x, start_y, end_x, end_y;
    int step = 2;
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(me_ctx, x_mb, y_mb, 0, 0, dir)))
        return;

    do {
        start_x = av_clip(x_orig - step, 0, x_orig);
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_x = av_clip(x_orig + step + 1, 0, me_ctx->width - me_ctx->mb_size);
        end_y = av_clip(y_orig + step + 1, 0, me_ctx->height - me_ctx->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                // skips already checked current origin
                if (x == x_orig && y == y_orig)
                    continue;

                if (FFABS(x_orig - x) + FFABS(y_orig - y) != step)
                    continue;

                cost = get_mad(me_ctx, x_mb, y_mb, x - x_mb, y - y_mb, dir);
                if (!cost) {
                    *mv_x = x - x_mb;
                    *mv_y = y - y_mb;
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
            FFABS(x_min_cost - x_mb) >= me_ctx->search_param || FFABS(y_min_cost - y_mb) >= me_ctx->search_param)
            step = 1;
        else {
            //TODO skip repeated
            x_orig = x_min_cost;
            y_orig = y_min_cost;
        }

    } while (step > 0);

    *mv_x = x_min_cost - x_mb;
    *mv_y = y_min_cost - y_mb;
}

static void add_mv_data(AVMotionVector *mv, int mb_size,
                        int x, int y, int mv_x, int mv_y, int dir)
{
    mv->w = mb_size;
    mv->h = mb_size;
    mv->dst_x = x + (mb_size >> 1);
    mv->dst_y = y + (mb_size >> 1);
    mv->src_x = x + mv_x + (mb_size >> 1);
    mv->src_y = y + mv_y + (mb_size >> 1);
    mv->source = dir ? 1 : -1;
    mv->flags = 0;
}

#define SEARCH_MV(method)\
    for (mb_y = 0; mb_y < s->b_height; mb_y++)\
        for (mb_x = 0; mb_x < s->b_width; mb_x++)\
            for (dir = 0; dir < 2; dir++) {\
                mv_x = 0; mv_y = 0;\
                search_mv_##method(me_ctx, mb_x << s->log2_mb_size, mb_y << s->log2_mb_size, &mv_x, &mv_y, dir);\
                add_mv_data(s->mvs + s->mv_count++, s->mb_size, mb_x << s->log2_mb_size, mb_y << s->log2_mb_size, mv_x, mv_y, dir);\
            }

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MEFilterContext *s = ctx->priv;
    MEContext *me_ctx = s->me_ctx;
    AVFrameSideData *sd;
    int mb_x, mb_y;
    int mv_x, mv_y;
    int8_t dir;

    av_assert0(frame->pts != AV_NOPTS_VALUE); //FIXME if (AV_NOPTS_VALUE) return 0?

    av_frame_free(&me_ctx->prev);
    me_ctx->prev = me_ctx->cur;
    me_ctx->cur  = me_ctx->next;
    me_ctx->next = frame;
    s->mv_count = 0;

    if (!me_ctx->cur) {
        me_ctx->cur = av_frame_clone(frame);
        if (!me_ctx->cur)
            return AVERROR(ENOMEM);
    }

    if (!me_ctx->prev)
        return 0;

    switch (s->method) {
        case ME_METHOD_ESA:
            /* exhaustive search */
            SEARCH_MV(esa);
            break;
        case ME_METHOD_TSS:
            /* three step search */
            SEARCH_MV(tss);
            break;
        case ME_METHOD_NTSS:
            /* new three step search */
            SEARCH_MV(ntss);
            break;
        case ME_METHOD_TDLS:
            /* two dimensional logarithmic search */
            SEARCH_MV(tdls);
            break;
        case ME_METHOD_FSS:
            /* four step search */
            SEARCH_MV(fss);
            break;
        case ME_METHOD_DIA:
            /* diamond search */
            SEARCH_MV(dia);
            break;
    }

    AVFrame *out = av_frame_clone(me_ctx->cur);
    if (!out)
        return AVERROR(ENOMEM);
    out->pts = me_ctx->next->pts;

    if (s->mv_count) {
        sd = av_frame_new_side_data(out, AV_FRAME_DATA_MOTION_VECTORS, s->mv_count * sizeof(AVMotionVector));
        if (!sd)
            return AVERROR(ENOMEM);
        memcpy(sd->data, s->mvs, s->mv_count * sizeof(AVMotionVector));
    }

    return ff_filter_frame(ctx->outputs[0], out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MEFilterContext *s = ctx->priv;
    MEContext *me_ctx = s->me_ctx;

    av_frame_free(&me_ctx->prev);
    av_frame_free(&me_ctx->cur);
    av_frame_free(&me_ctx->next);
    av_freep(&s->me_ctx);
    av_freep(&s->mvs);
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
    .priv_size     = sizeof(MEFilterContext),
    .priv_class    = &mestimate_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = mestimate_inputs,
    .outputs       = mestimate_outputs,
};
