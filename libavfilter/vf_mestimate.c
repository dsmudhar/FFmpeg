/**
 * 
 * Developed by Davinder Singh (DSM_ / @dsmudhar) during GSoC 2016
 * As qualification task: Basic but working motion estimation filter
 * 
 * Used vf_w3fdif.c as base, needed two frames for bi-directional prediction.
 *
 * The filter uses block matching exhaustive search algorithm
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
    AVMotionVector *mvs; ///< motion vectors
    AVFrame *prev, *cur, *next;  ///< previous, current, next frames
    int block_size; ///< block size
    int reg_size; ///< search region
    int32_t mv_count; ///< no of motion vectors per frame

} MEContext;

#define OFFSET(x) offsetof(MEContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption mestimate_options[] = {
    { "block", "specify the block size", OFFSET(block_size), AV_OPT_TYPE_INT, {.i64=8}, 4, 32, FLAGS, "block" },
    { "search",  "specify search region", OFFSET(reg_size), AV_OPT_TYPE_INT, {.i64=7}, 4, 32, FLAGS, "search" },
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
    int nb_blocks_y = inlink->h / s->block_size;
    int nb_blocks_x = inlink->w / s->block_size;

    s->mvs = av_malloc_array(nb_blocks_x * nb_blocks_y, 2 * sizeof(AVMotionVector));
    if (!s->mvs)
        return AVERROR(ENOMEM);

    return 0;
}

static int64_t get_mse(MEContext *s, int width, int x_cur, int y_cur, int x_sb, int y_sb, int source)
{
    // source == -1 means forward prediction => frame k - 1 as reference
    uint8_t *buf_src = source == -1 ? s->prev->data[0] : s->next->data[0];
    uint8_t *buf_cur = s->cur->data[0];
    int64_t mse = 0;
    int i, j;
    
    for (i = 0; i < s->block_size; i++)
        for (j = 0; j < s->block_size; j++) {
            int64_t sb = ((int64_t) y_sb + i) * width + x_sb + j;
            int64_t cur = ((int64_t) y_cur + i) * width + x_cur + j;
            int diff = (int) buf_src[sb] - (int) buf_cur[cur];
            mse += pow(diff, 2);
        }
    
    return mse / pow(s->block_size, 2);
}

static void add_mv_data(AVMotionVector *mv, int block_size,
                  int dst_x, int dst_y, int src_x, int src_y,
                  int source)
{
    mv->w = block_size;
    mv->h = block_size;
    mv->dst_x = dst_x;
    mv->dst_y = dst_y;
    mv->src_x = src_x;
    mv->src_y = src_y;
    mv->source = source;
    mv->flags = 0;
}

static void get_motion_vector(AVFilterLink *inlink, int x_cur, int y_cur, int source)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;

    int i, j, x_sb, y_sb, dx = 0, dy = 0;
    int sign_i = 1, sign_j = 1;
    int y_sb_max = av_clip(y_cur + s->reg_size, 0, inlink->h - 1);
    int x_sb_max = av_clip(x_cur + s->reg_size, 0, inlink->w - 1);
    int64_t mse, mse_min = -1;

    for (i = 0; i < s->reg_size; i = sign_i ? -(i + 1) : -i, sign_i = !sign_i) {
        y_sb = y_cur + i;

        if (y_sb < 0 || y_sb > y_sb_max)
            continue;

        for (j = 0; j < s->reg_size; j = sign_j ? -(j + 1) : -j, sign_j = !sign_j) {
            x_sb = x_cur + j;

            if (x_sb < 0 || x_sb > x_sb_max)
                continue;
            
            if (mse_min == -1 || (mse = get_mse(s, inlink->w, x_cur, y_cur, x_sb, y_sb, source)) < mse_min) {
                mse_min = mse;
                dx = x_sb - x_cur;
                dy = y_sb - y_cur;
            }
        }
    }

    if (dx != 0 || dy != 0)
        add_mv_data(s->mvs + s->mv_count++, s->block_size, x_cur + dx, y_cur + dy, x_cur, y_cur, source);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;
    AVFrameSideData *sd;
    int i, x, y;

    av_frame_free(&s->prev);
    s->mv_count = 0;
    s->prev = s->cur;
    s->cur  = s->next;
    s->next = frame;

    if (s->cur) {
        for (y = 0; y < inlink->h; y += s->block_size)
            for (x = 0; x < inlink->w; x+= s->block_size)
                for (i = 0; i < 2; i++)
                    get_motion_vector(inlink, x, y, i == 0 ? -1 : 1);
    } else { // no vectors will be generated if cloned, so skipping for first frame (s->prev, s->next are null) 
        s->cur = av_frame_clone(s->next);
        if (!s->cur)
            return AVERROR(ENOMEM);
    }

    AVFrame *out = av_frame_clone(s->cur);
    if (!out)
        return AVERROR(ENOMEM);

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
    .description   = NULL_IF_CONFIG_SMALL("Generates motion vectors."),
    .priv_size     = sizeof(MEContext),
    .priv_class    = &mestimate_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = mestimate_inputs,
    .outputs       = mestimate_outputs,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL
};
