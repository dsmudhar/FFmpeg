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
    int linesize[4];      ///< bytes of pixel data per line for each plane
    int planeheight[4];   ///< height of each plane
    int nb_planes;

    AVFrame *prev, *cur, *next;  ///< previous, current, next frames
    int block_size; ///< block size
    int reg_size; ///< search region

} MEContext;

#define OFFSET(x) offsetof(MEContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption mestf_options[] = {
    { "block", "specify the block size", OFFSET(block_size), AV_OPT_TYPE_INT, {.i64=8}, 4, 32, FLAGS, "block" },
    { "search",  "specify seach region", OFFSET(reg_size), AV_OPT_TYPE_INT, {.i64=7}, 4, 32, FLAGS, "search" },
    { NULL }
};

AVFILTER_DEFINE_CLASS(mestf);

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
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    int ret;

    if ((ret = av_image_fill_linesizes(s->linesize, inlink->format, inlink->w)) < 0)
        return ret;

    s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = inlink->h;

    s->nb_planes = av_pix_fmt_count_planes(inlink->format);

    return 0;
}

static int64_t get_mse(MEContext *s, int x_cur, int y_cur, int x_sb, int y_sb)
{
    
    uint8_t *buf_cur = s->cur->data[0];
    uint8_t *buf_next = s->next->data[0];
    int64_t mse = 0;
    int i, j;
    
    for (i = 0; i < s->block_size; i++)
        for (j = 0; j < s->block_size; j++) {
            int64_t sb = ((int64_t) y_sb + i) * s->linesize[0] + x_sb + j;
            int64_t cur = ((int64_t) y_cur + i) * s->linesize[0] + x_cur + j;
            int64_t diff = (int) buf_next[sb] - (int) buf_cur[cur];
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

static AVMotionVector *get_motion_vector(AVFilterLink *inlink, int x_cur, int y_cur)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;
    AVMotionVector *mv = av_malloc(sizeof(AVMotionVector));

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

            mse = get_mse(s, x_cur, y_cur, x_sb, y_sb);
            if (mse_min == -1 || mse < mse_min) {

                mse_min = mse;
                dx = x_sb - x_cur;
                dy = y_sb - y_cur;

            }
        }
    }

    add_mv_data(mv, s->block_size, x_cur + dx, y_cur + dy, x_cur, y_cur, 0);

    return mv;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;
    int nb_blocks_y = inlink->h / s->block_size;
    int nb_blocks_x = s->linesize[0] / s->block_size;
    int x, y, mv_count = 0;
    AVMotionVector *mvs = av_malloc_array(nb_blocks_x * nb_blocks_y, sizeof(AVMotionVector));
    AVFrameSideData *sd;

    av_frame_free(&s->prev);
    s->prev = s->cur;
    s->cur  = s->next;
    s->next = frame;

    if (!s->cur) {
        s->cur = av_frame_clone(s->next);
        if (!s->cur)
            return AVERROR(ENOMEM);
    }

    for (y = 0; y < inlink->h; y += s->block_size) {
        for (x = 0; x < s->linesize[0]; x+= s->block_size) {
            AVMotionVector *mv = get_motion_vector(inlink, x, y);

            add_mv_data(mvs + mv_count, s->block_size, mv->dst_x, mv->dst_y, mv->src_x, mv->src_y, mv->source);
            av_freep(&mv);

            mv_count++;
        }
    }

    AVFrame *out = av_frame_clone(s->cur);
    if (!out)
        return AVERROR(ENOMEM);

    sd = av_frame_new_side_data(out, AV_FRAME_DATA_MOTION_VECTORS, mv_count * sizeof(AVMotionVector));
    memcpy(sd->data, mvs, mv_count * sizeof(AVMotionVector));
    
    av_freep(&mvs);
    av_frame_free(&s->prev);

    return ff_filter_frame(ctx->outputs[0], out);
    
} 

static av_cold void uninit(AVFilterContext *ctx)
{
    MEContext *s = ctx->priv;

    av_frame_free(&s->prev);
    av_frame_free(&s->cur );
    av_frame_free(&s->next);

}

static const AVFilterPad mestf_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
        .config_props  = config_input,
    },
    { NULL }
};

static const AVFilterPad mestf_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
    },
    { NULL }
};

AVFilter ff_vf_mestf = {
    .name          = "mestf",
    .description   = NULL_IF_CONFIG_SMALL("Generates motion vectors."),
    .priv_size     = sizeof(MEContext),
    .priv_class    = &mestf_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = mestf_inputs,
    .outputs       = mestf_outputs,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL
};
