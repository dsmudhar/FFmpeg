/**
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
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/motion_vector.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

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
        CONST("esa",   "exhaustive search",                  ME_METHOD_ESA,   "method"),
        CONST("tss",   "three step search",                  ME_METHOD_TSS,   "method"),
        CONST("tdls",  "two dimensional logarithmic search", ME_METHOD_TDLS,  "method"),
        CONST("ntss",  "new three step search",              ME_METHOD_NTSS,  "method"),
        CONST("fss",   "four step search",                   ME_METHOD_FSS,   "method"),
        CONST("ds",    "diamond search",                     ME_METHOD_DS,    "method"),
        CONST("hexbs", "hexagon-based search",               ME_METHOD_HEXBS, "method"),
        CONST("epzs",  "enhanced predictive zonal search",   ME_METHOD_EPZS,  "method"),
        CONST("umh",   "uneven multi-hexagon search",        ME_METHOD_UMH,   "method"),
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
                ff_me_search_##method(me_ctx, x_mb, y_mb, mv);\
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
    int ret;

    if (frame->pts == AV_NOPTS_VALUE) {
        ret = ff_filter_frame(ctx->outputs[0], frame);
        return ret;
    }

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

                    ff_me_search_umh(me_ctx, pred, x_mb, y_mb, mv);

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

                    ff_me_search_epzs(me_ctx, pred, x_mb, y_mb, mv);

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
