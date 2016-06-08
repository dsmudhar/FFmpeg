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
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/motion_vector.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"

#define NB_INPUT_FRAMES 4
#define ALPHA_MAX 1024

enum MIMode {
    MI_MODE_NN           = 0,
    MI_MODE_LINEAR_IPOL  = 1,
    MI_MODE_GMC          = 2,
    MI_MODE_OBMC         = 3,
};

typedef struct InputFrame {
    AVFrame *f;
    // side data, if needed
} InputFrame;

typedef struct MIContext {
    const AVClass *class;
    AVMotionVector *mvs;
    AVFrame *prev, *cur, *next;
    enum MIMode mode;
    AVRational       frame_rate;

    InputFrame input[NB_INPUT_FRAMES];

    int64_t out_pts;

    int chroma_height;
    int chroma_width;
    int nb_planes;

} MIContext;

#define OFFSET(x) offsetof(MIContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption minterpolate_options[] = {
    { "mode",   "specify the interpolation mode", OFFSET(mode), AV_OPT_TYPE_INT, {.i64 = MI_MODE_NN}, 0, 3, FLAGS, "mode"},
    CONST("nn",         "", MI_MODE_NN,              "mode"),
    CONST("linear",     "", MI_MODE_LINEAR_IPOL,     "mode"),
    CONST("gmc",        "", MI_MODE_GMC,             "mode"),
    CONST("obmc",       "", MI_MODE_OBMC,            "mode"),

    { "fps",   "specify the frame rate", OFFSET(frame_rate), AV_OPT_TYPE_RATIONAL, {.dbl = 25}, 0, INT_MAX, FLAGS},
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
    MIContext *mi = inlink->dst->priv;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    mi->chroma_height = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    mi->chroma_width = AV_CEIL_RSHIFT(inlink->w, desc->log2_chroma_w);

    mi->nb_planes = av_pix_fmt_count_planes(inlink->format);
    
    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    MIContext *mi = outlink->src->priv;

    outlink->frame_rate = mi->frame_rate;
    outlink->time_base  = av_inv_q(mi->frame_rate);
    av_log(0,0, "FPS %d/%d\n", mi->frame_rate.num, mi->frame_rate.den);

    return 0;
}

static void interpolate(AVFilterLink *inlink, AVFrame *out)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi = ctx->priv;
    AVFrame *frame;
    int plane, alpha;
    int64_t pts;

    switch(mi->mode) {
    case MI_MODE_NN:
        pts = av_rescale_q(out->pts, outlink->time_base, inlink->time_base);
        if (FFABS(pts - mi->input[1].f->pts) < FFABS(pts - mi->input[2].f->pts)) {
            frame = mi->input[1].f;
        } else
            frame = mi->input[2].f;

        av_frame_copy(out, frame);

        break;
    case MI_MODE_LINEAR_IPOL:
        pts = av_rescale(out->pts,
                            outlink->time_base.num * (int64_t)ALPHA_MAX * inlink->time_base.den,
                            outlink->time_base.den * (int64_t)    inlink->time_base.num
                        );
        alpha = (pts - mi->input[1].f->pts*ALPHA_MAX)/ (mi->input[2].f->pts - mi->input[1].f->pts);
        alpha = av_clip(alpha, 0, ALPHA_MAX);

        for (plane=0; plane < mi->nb_planes; plane++) {
            int x, y;
            int w = out->width;
            int h = out->height;

            if (plane == 1 || plane == 2) {
                w = mi->chroma_width;
                h = mi->chroma_height;
            }

            for (y=0; y<h; y++) {
                for (x=0; x<w; x++) {
                    out->data[plane][ x + y*out->linesize[plane] ] =
                        ((ALPHA_MAX - alpha)*mi->input[1].f->data[plane][ x + y*mi->input[1].f->linesize[plane] ] +
                         alpha              *mi->input[2].f->data[plane][ x + y*mi->input[2].f->linesize[plane] ] + 512) >> 10;
                }
            }
        }

        break;
    }

}

static int inject_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi = ctx->priv;
    InputFrame tmp, *f;

    av_frame_free(&mi->input[0].f);
    tmp = mi->input[0];
    memmove(&mi->input[0], &mi->input[1], sizeof(mi->input[0]) * (NB_INPUT_FRAMES-1));
    mi->input[NB_INPUT_FRAMES-1] = tmp;
    mi->input[NB_INPUT_FRAMES-1].f = frame;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    MIContext *mi = ctx->priv;
    int ret;

    av_assert0(frame->pts != AV_NOPTS_VALUE); //FIXME

    if (!mi->input[NB_INPUT_FRAMES-1].f || frame->pts < mi->input[NB_INPUT_FRAMES-1].f->pts) {
        av_log(ctx, AV_LOG_VERBOSE, "Initializing outpts from input pts %"PRId64"\n", frame->pts);
        mi->out_pts = av_rescale_q(frame->pts, inlink->time_base, outlink->time_base);
    }

    if (!mi->input[NB_INPUT_FRAMES-1].f)
        inject_frame(inlink, av_frame_clone(frame));
    inject_frame(inlink, frame);

    if (!mi->input[0].f)
        return 0;

    for (;;) {
        AVFrame *out;

        if (av_compare_ts(mi->input[NB_INPUT_FRAMES/2].f->pts, inlink->time_base, mi->out_pts, outlink->time_base) < 0)
            break;

        if (!(out = ff_get_video_buffer(ctx->outputs[0], inlink->w, inlink->h)))
            return AVERROR(ENOMEM);

        av_frame_copy_props(out, mi->input[NB_INPUT_FRAMES-1].f); //TODO this also copy sidedata, why not move it to interpolate()?
        out->pts = mi->out_pts++;

        interpolate(inlink, out);

        ret = ff_filter_frame(ctx->outputs[0], out);
        if (ret < 0)
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
