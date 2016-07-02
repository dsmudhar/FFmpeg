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
    const AVClass *class;
    AVMotionVector *mvs;                ///< motion vectors for current frame
    AVFrame *prev, *cur, *next;         ///< previous, current, next frames
    enum MEMethod method;               ///< motion estimation method
    int mb_size;                        ///< macroblock size
    int search_param;                   ///< search parameter
    int32_t mv_count;                   ///< current frame motion vector count

} MEContext;

#define OFFSET(x) offsetof(MEContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption mestimate_options[] = {
    { "method", "specify motion estimation method", OFFSET(method), AV_OPT_TYPE_INT, {.i64 = ME_METHOD_ESA}, ME_METHOD_DIA, /*TODO*/INT_MAX, FLAGS, "method" },
        CONST("esa", "exhaustive search", 0, "method"),
        //TODO add other me method options
    { "mb_size", "specify macroblock size", OFFSET(mb_size), AV_OPT_TYPE_INT, {.i64 = 16}, 2, INT_MAX, FLAGS },
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

static int config_input(AVFilterLink *inlink)
{
    MEContext *s = inlink->dst->priv;
    int nb_blocks_y = inlink->h / s->mb_size;
    int nb_blocks_x = inlink->w / s->mb_size;

    s->mvs = av_malloc_array(nb_blocks_x * nb_blocks_y, 2 * sizeof(AVMotionVector));
    if (!s->mvs)
        return AVERROR(ENOMEM);

    return 0;
}

static uint64_t get_mad(MEContext *s, int x_cur, int y_cur, int x_sb, int y_sb, int direction)
{
    // dir = 0 => source = -1 => forward prediction  => frame k - 1 as reference
    // dir = 1 => source = +1 => backward prediction => frame k + 1 as reference
    uint8_t *buf_ref = (direction ? s->next : s->prev)->data[0];
    uint8_t *buf_cur = s->cur->data[0];
    int stride_ref = (direction ? s->next : s->prev)->linesize[0];
    int stride_cur = s->cur->linesize[0];
    int64_t mad = 0;
    int x, y;

    buf_ref += y_sb * stride_ref;
    buf_cur += y_cur * stride_cur;

    for (y = 0; y < s->mb_size; y++)
        for (x = 0; x < s->mb_size; x++) {
            int diff = buf_ref[y * stride_ref + x + x_sb] - buf_cur[y * stride_cur + x + x_cur];
            mad += FFABS(diff);
        }

    return mad;
}

#define MIN_VECTOR(dx0, dy0, dx1, dy1)\
do {\
    if (dx1 * dx1 + dy1 * dy1 < dx0 * dx0 + dy0 * dy0) {\
        dx0 = dx1;\
        dy0 = dy1;\
    }\
} while(0)

static void search_mv_esa(AVFilterLink *inlink, int x_cur, int y_cur, int *dx, int *dy, int dir)
{
    MEContext *s = inlink->dst->priv;

    int x, y;
    int p = s->search_param;
    int start_x = av_clip(x_cur - p, 0, x_cur);
    int start_y = av_clip(y_cur - p, 0, y_cur);
    int end_x = av_clip(x_cur + p + 1, 0, inlink->w - s->mb_size);
    int end_y = av_clip(y_cur + p + 1, 0, inlink->h - s->mb_size);
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(s, x_cur, y_cur, x_cur, y_cur, dir))) {
        *dx = 0;
        *dy = 0;
        return;
    }

    for (y = start_y; y < end_y; y++)
        for (x = start_x; x < end_x; x++) {
            if ((cost = get_mad(s, x_cur, y_cur, x, y, dir)) < cost_min) {
                cost_min = cost;
                *dx = x - x_cur;
                *dy = y - y_cur;
            } else if (cost == cost_min)
                MIN_VECTOR(*dx, *dy, x - x_cur, y - y_cur);
        }
}

static void search_mv_tss(AVFilterLink *inlink, int x_cur, int y_cur, int *dx, int *dy, int dir)
{
    MEContext *s = inlink->dst->priv;

    int x, y;
    int start_x, start_y, end_x, end_y;
    int step = ROUNDED_DIV(s->search_param, 2);
    int x_min_cost = x_cur, y_min_cost = y_cur;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(s, x_cur, y_cur, x_cur, y_cur, dir))) {
        *dx = 0;
        *dy = 0;
        return;
    }

    do {
        start_x = av_clip(x_min_cost - step, 0, x_min_cost);
        start_y = av_clip(y_min_cost - step, 0, y_min_cost);
        end_x = av_clip(x_min_cost + step + 1, 0, inlink->w - s->mb_size);
        end_y = av_clip(y_min_cost + step + 1, 0, inlink->h - s->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                //if (x == start_x + step && y == start_y + step) //FIXME
                //    continue;

                cost = get_mad(s, x_cur, y_cur, x, y, dir);
                if (!cost) {
                    *dx = x - x_cur;
                    *dy = y - y_cur;
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

    *dx = x_min_cost - x_cur;
    *dy = y_min_cost - y_cur;
}

static void search_mv_ntss(AVFilterLink *inlink, int x_cur, int y_cur, int *dx, int *dy, int dir)
{
    MEContext *s = inlink->dst->priv;

    int x, y;
    int start_x, start_y, end_x, end_y;
    int step = ROUNDED_DIV(s->search_param, 2);
    int x_min_cost = x_cur, y_min_cost = y_cur;
    uint64_t cost, cost_min;
    int first_step = 1;

    if (!(cost_min = get_mad(s, x_cur, y_cur, x_cur, y_cur, dir))) {
        *dx = 0;
        *dy = 0;
        return;
    }

    do {
        start_x = av_clip(x_min_cost - step, 0, x_min_cost);
        start_y = av_clip(y_min_cost - step, 0, y_min_cost);
        end_x = av_clip(x_min_cost + step + 1, 0, inlink->w - s->mb_size);
        end_y = av_clip(y_min_cost + step + 1, 0, inlink->h - s->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                //if (x == start_x + step && y == start_y + step) //FIXME
                //    continue;

                cost = get_mad(s, x_cur, y_cur, x, y, dir);
                if (cost < cost_min) {
                    cost_min = cost;
                    x_min_cost = x;
                    y_min_cost = y;
                } else if (!cost && !first_step) {
                    *dx = x - x_cur;
                    *dy = y - y_cur;
                    return;
                }
            }
        }

        // addition to TSS in NTSS
        if (first_step) {
            int dx1, dy1;
            start_x = av_clip(x_cur - 1, 0, x_cur);
            start_y = av_clip(y_cur - 1, 0, y_cur);
            end_x = av_clip(x_cur + 2, 0, inlink->w - s->mb_size);
            end_y = av_clip(y_cur + 2, 0, inlink->h - s->mb_size);

            for (y = start_y; y < end_y; y++) {
                for (x = start_x; x < end_x; x++) {

                    if (x == x_cur && y == y_cur)
                        continue;

                    cost = get_mad(s, x_cur, y_cur, x, y, dir);
                    if (!cost) {
                        *dx = x - x_cur;
                        *dy = y - y_cur;
                        return;
                    } else if (cost < cost_min) {
                        cost_min = cost;
                        x_min_cost = x;
                        y_min_cost = y;
                    }
                }
            }

            dx1 = x_min_cost - x_cur;
            dy1 = y_min_cost - y_cur;

            if (!dx1 && !dy1) {
                *dx = 0;
                *dy = 0;
                return;
            }

            if (FFABS(dx1) <= 1 && FFABS(dy1) <= 1) {
                int i, j;
                start_x = x_cur + dx1;
                start_y = y_cur + dy1;
                // no need to clip here, since block size is >= 2
                for (j = -1; j <= 1; j++) {
                    y = start_y + j;
                    for (i = -1; i <= 1; i++) {
                        if ((1 - i * dx1) && (1 - j * dy1))
                            continue;

                        x = start_x + i;
                        cost = get_mad(s, x_cur, y_cur, x, y, dir);
                        if (!cost) {
                            *dx = x - x_cur;
                            *dy = y - y_cur;
                            return;
                        } else if (cost < cost_min) {
                            cost_min = cost;
                            x_min_cost = x;
                            y_min_cost = y;
                        }
                    }
                }

                *dx = x_min_cost - x_cur;
                *dy = y_min_cost - y_cur;
                return;
            }

            first_step = 0;
        }

        step = step / 2;

    } while (step > 0);

    *dx = x_min_cost - x_cur;
    *dy = y_min_cost - y_cur;
}

static void search_mv_tdls(AVFilterLink *inlink, int x_cur, int y_cur, int *dx, int *dy, int dir)
{
    MEContext *s = inlink->dst->priv;

    int x, y;
    int x_orig = x_cur, y_orig = y_cur;
    int start_x, start_y, end_x, end_y;
    int step = ROUNDED_DIV(s->search_param, 2);
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(s, x_cur, y_cur, x_cur, y_cur, dir))) {
        *dx = 0;
        *dy = 0;
        return;
    }

    do {
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_y = av_clip(y_orig + step + 1, 0, inlink->h - s->mb_size);

        for (y = start_y; y < end_y; y += step) {

            if (y == start_y + step) {
                start_x = av_clip(x_orig - step, 0, x_orig);
                end_x = av_clip(x_orig + step + 1, 0, inlink->w - s->mb_size);
            } else {
                start_x = x_orig;
                end_x = x_orig + 1;
            }

            for (x = start_x; x < end_x; x += step) {

                if (y == y_orig && x == x_orig)
                    continue;

                cost = get_mad(s, x_cur, y_cur, x, y, dir);
                if (!cost) {
                    *dx = x - x_cur;
                    *dy = y - y_cur;
                    return;
                } else if (cost < cost_min) {
                    cost_min = cost;
                    x_min_cost = x;
                    y_min_cost = y;
                }
            }
        }

        if (y_min_cost == y_orig || FFABS(y_min_cost - y_cur) >= s->search_param) { //FIXME
            step = step / 2;
        } else {
            x_orig = x_min_cost;
            y_orig = y_min_cost;
        }

    } while (step > 0);

    *dx = x_min_cost - x_cur;
    *dy = y_min_cost - y_cur;
}

static void search_mv_fss(AVFilterLink *inlink, int x_cur, int y_cur, int *dx, int *dy, int dir)
{
    MEContext *s = inlink->dst->priv;

    int x, y;
    int x_orig = x_cur, y_orig = y_cur;
    int start_x, start_y, end_x, end_y;
    int step = 2;
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(s, x_cur, y_cur, x_cur, y_cur, dir))) {
        *dx = 0;
        *dy = 0;
        return;
    }

    do {
        start_x = av_clip(x_orig - step, 0, x_orig);
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_x = av_clip(x_orig + step + 1, 0, inlink->w - s->mb_size);
        end_y = av_clip(y_orig + step + 1, 0, inlink->h - s->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                // skips already checked current origin
                if (x == x_orig && y == y_orig)
                    continue;

                cost = get_mad(s, x_cur, y_cur, x, y, dir);
                if (!cost) {
                    *dx = x - x_cur;
                    *dy = y - y_cur;
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
            int i, j, dx1, dy1;
            do {
                dx1 = x_min_cost - x_orig;
                dy1 = y_min_cost - y_orig;

                x_orig = x_min_cost;
                y_orig = y_min_cost;

                for (j = -step; j <= step; j += step) { //FIXME clip
                    y = y_orig + j;
                    for (i = -step; i <= step; i += step) {
                        if ((4 - i * dx1) && (4 - j * dy1))
                            continue;

                        x = x_orig + i;
                        cost = get_mad(s, x_cur, y_cur, x, y, dir);
                        if (!cost) {
                            *dx = x - x_cur;
                            *dy = y - y_cur;
                            return;
                        } else if (cost < cost_min) {
                            cost_min = cost;
                            x_min_cost = x;
                            y_min_cost = y;
                        }
                    }
                }

                if (x_min_cost == x_orig && y_min_cost == y_orig ||
                    FFABS(x_min_cost - x_cur) >= s->search_param - 1 || FFABS(y_min_cost - y_cur) >= s->search_param - 1)
                    step = 1;

            } while (step > 1);
        }

    } while (step > 0);

    *dx = x_min_cost - x_cur;
    *dy = y_min_cost - y_cur;
}

static void search_mv_dia(AVFilterLink *inlink, int x_cur, int y_cur, int *dx, int *dy, int dir)
{
    MEContext *s = inlink->dst->priv;

    int x, y;
    int x_orig = x_cur, y_orig = y_cur;
    int start_x, start_y, end_x, end_y;
    int step = 2;
    int x_min_cost = x_orig, y_min_cost = y_orig;
    uint64_t cost, cost_min;

    if (!(cost_min = get_mad(s, x_cur, y_cur, x_cur, y_cur, dir))) {
        *dx = 0;
        *dy = 0;
        return;
    }

    do {
        start_x = av_clip(x_orig - step, 0, x_orig);
        start_y = av_clip(y_orig - step, 0, y_orig);
        end_x = av_clip(x_orig + step + 1, 0, inlink->w - s->mb_size);
        end_y = av_clip(y_orig + step + 1, 0, inlink->h - s->mb_size);

        for (y = start_y; y < end_y; y += step) {
            for (x = start_x; x < end_x; x += step) {

                // skips already checked current origin
                if (x == x_orig && y == y_orig)
                    continue;

                if (FFABS(x_orig - x) + FFABS(y_orig - y) != step)
                    continue;

                cost = get_mad(s, x_cur, y_cur, x, y, dir);
                if (!cost) {
                    *dx = x - x_cur;
                    *dy = y - y_cur;
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
            FFABS(x_min_cost - x_cur) >= s->search_param || FFABS(y_min_cost - y_cur) >= s->search_param)
            step = 1;
        else {
            //TODO skip repeated
            x_orig = x_min_cost;
            y_orig = y_min_cost;
        }

    } while (step > 0);

    *dx = x_min_cost - x_cur;
    *dy = y_min_cost - y_cur;
}

static void add_mv_data(AVMotionVector *mv, int mb_size,
                        int x, int y, int dx, int dy, int dir)
{
    mv->w = mb_size;
    mv->h = mb_size;
    mv->dst_x = x + (mb_size >> 1);
    mv->dst_y = y + (mb_size >> 1);
    mv->src_x = x + dx + (mb_size >> 1);
    mv->src_y = y + dy + (mb_size >> 1);
    mv->source = dir ? 1 : -1;
    mv->flags = 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;
    AVFrameSideData *sd;
    int x, y;
    int8_t dir;
    int dx, dy;

    av_assert0(frame->pts != AV_NOPTS_VALUE); //FIXME if (AV_NOPTS_VALUE) return 0?

    av_frame_free(&s->prev);
    s->mv_count = 0;
    s->prev = s->cur;
    s->cur  = s->next;
    s->next = frame;

    if (s->cur) {

        switch (s->method) {
            case ME_METHOD_ESA:
                /* exhaustive search */
                for (y = 0; y < inlink->h; y += s->mb_size)
                    for (x = 0; x < inlink->w; x += s->mb_size)
                        for (dir = 0; dir < 2; dir++) {
                            dx = 0; dy = 0;
                            search_mv_esa(inlink, x, y, &dx, &dy, dir);
                            add_mv_data(s->mvs + s->mv_count++, s->mb_size, x, y, dx, dy, dir);
                        }
                break;
            case ME_METHOD_TSS:
                /* three step search */
                for (y = 0; y < inlink->h; y += s->mb_size)
                    for (x = 0; x < inlink->w; x += s->mb_size)
                        for (dir = 0; dir < 2; dir++) {
                            dx = 0; dy = 0;
                            search_mv_tss(inlink, x, y, &dx, &dy, dir);
                            add_mv_data(s->mvs + s->mv_count++, s->mb_size, x, y, dx, dy, dir);
                        }
                break;
            case ME_METHOD_NTSS:
                /* new three step search */
                for (y = 0; y < inlink->h; y += s->mb_size)
                    for (x = 0; x < inlink->w; x += s->mb_size)
                        for (dir = 0; dir < 2; dir++) {
                            dx = 0; dy = 0;
                            search_mv_ntss(inlink, x, y, &dx, &dy, dir);
                            add_mv_data(s->mvs + s->mv_count++, s->mb_size, x, y, dx, dy, dir);
                        }
                break;
            case ME_METHOD_TDLS:
                /* two dimensional logarithmic search */
                for (y = 0; y < inlink->h; y += s->mb_size)
                    for (x = 0; x < inlink->w; x += s->mb_size)
                        for (dir = 0; dir < 2; dir++) {
                            dx = 0; dy = 0;
                            search_mv_tdls(inlink, x, y, &dx, &dy, dir);
                            add_mv_data(s->mvs + s->mv_count++, s->mb_size, x, y, dx, dy, dir);
                        }
                break;
            case ME_METHOD_FSS:
                /* four step search */
                for (y = 0; y < inlink->h; y += s->mb_size)
                    for (x = 0; x < inlink->w; x += s->mb_size)
                        for (dir = 0; dir < 2; dir++) {
                            dx = 0; dy = 0;
                            search_mv_fss(inlink, x, y, &dx, &dy, dir);
                            add_mv_data(s->mvs + s->mv_count++, s->mb_size, x, y, dx, dy, dir);
                        }
                break;
            case ME_METHOD_DIA:
                /* diamond search */
                for (y = 0; y < inlink->h; y += s->mb_size)
                    for (x = 0; x < inlink->w; x += s->mb_size)
                        for (dir = 0; dir < 2; dir++) {
                            dx = 0; dy = 0;
                            search_mv_dia(inlink, x, y, &dx, &dy, dir);
                            add_mv_data(s->mvs + s->mv_count++, s->mb_size, x, y, dx, dy, dir);
                        }
                break;
        }

    } else { // no vectors will be generated if cloned, so skipping for first frame (s->prev, s->next are null)
        s->cur = av_frame_clone(s->next);
        if (!s->cur)
            return AVERROR(ENOMEM);
    }

    AVFrame *out = av_frame_clone(s->cur);
    if (!out)
        return AVERROR(ENOMEM);
    out->pts = s->next->pts;

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
    MEContext *s = ctx->priv;

    av_frame_free(&s->prev);
    av_frame_free(&s->cur );
    av_frame_free(&s->next);
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
    .priv_size     = sizeof(MEContext),
    .priv_class    = &mestimate_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = mestimate_inputs,
    .outputs       = mestimate_outputs,
};
