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
    // int filter;           ///< 0 is simple, 1 is more complex
    // int deint;            ///< which frames to deinterlace
    int linesize[4];      ///< bytes of pixel data per line for each plane
    int planeheight[4];   ///< height of each plane
    // int field;            ///< which field are we on, 0 or 1
    // int eof;
     int nb_planes;
    // int32_t **work_line;  ///< lines we are calculating
    // int nb_threads;

    AVFrame *prev, *cur, *next;  ///< previous, current, next frames
    int block_size;
    int reg_size;

} MEContext;

#define OFFSET(x) offsetof(MEContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
#define CONST(name, help, val, unit) { name, help, 0, AV_OPT_TYPE_CONST, {.i64=val}, 0, 0, FLAGS, unit }

static const AVOption mestf_options[] = {
    { "block", "specify the block size", OFFSET(block_size), AV_OPT_TYPE_INT, {.i64=8}, 4, 32, FLAGS, "block" },
    // CONST("simple",  NULL, 0, "filter"),
    // CONST("complex", NULL, 1, "filter"),
    { "search",  "specify seach region", OFFSET(reg_size), AV_OPT_TYPE_INT, {.i64=7}, 4, 32, FLAGS, "search" },
    // CONST("all",        "deinterlace all frames",                       0, "deint"),
    // CONST("interlaced", "only deinterlace frames marked as interlaced", 1, "deint"),
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
    int ret;//, i;

    if ((ret = av_image_fill_linesizes(s->linesize, inlink->format, inlink->w)) < 0)
        return ret;

    s->planeheight[1] = s->planeheight[2] = AV_CEIL_RSHIFT(inlink->h, desc->log2_chroma_h);
    s->planeheight[0] = s->planeheight[3] = inlink->h;

    s->nb_planes = av_pix_fmt_count_planes(inlink->format);
    /*
    s->nb_threads = ctx->graph->nb_threads;
    s->work_line = av_calloc(s->nb_threads, sizeof(*s->work_line));
    if (!s->work_line)
        return AVERROR(ENOMEM);

    for (i = 0; i < s->nb_threads; i++) {
        s->work_line[i] = av_calloc(FFALIGN(s->linesize[0], 32), sizeof(*s->work_line[0]));
        if (!s->work_line[i])
            return AVERROR(ENOMEM);
    }*/

    return 0;
}

/*static int config_output(AVFilterLink *outlink)
{
    AVFilterLink *inlink = outlink->src->inputs[0];

    outlink->time_base.num = inlink->time_base.num;
    outlink->time_base.den = inlink->time_base.den * 2;
    outlink->frame_rate.num = inlink->frame_rate.num * 2;
    outlink->frame_rate.den = inlink->frame_rate.den;

    return 0;
}*/

/*
 * Filter coefficients from PH-2071, scaled by 256 * 128.
 * Each set of coefficients has a set for low-frequencies and high-frequencies.
 * n_coef_lf[] and n_coef_hf[] are the number of coefs for simple and more-complex.
 * It is important for later that n_coef_lf[] is even and n_coef_hf[] is odd.
 * coef_lf[][] and coef_hf[][] are the coefficients for low-frequencies
 * and high-frequencies for simple and more-complex mode.
 */
/*static const int8_t   n_coef_lf[2] = { 2, 4 };
static const int16_t coef_lf[2][4] = {{ 16384, 16384,     0,    0},
                                      {  -852, 17236, 17236, -852}};
static const int8_t   n_coef_hf[2] = { 3, 5 };
static const int16_t coef_hf[2][5] = {{ -2048,  4096, -2048,     0,    0},
                                      {  1016, -3801,  5570, -3801, 1016}};

typedef struct ThreadData {
    AVFrame *out, *cur, *adj;
    int plane;
} ThreadData;

static int deinterlace_slice(AVFilterContext *ctx, void *arg, int jobnr, int nb_jobs)
{
    MEContext *s = ctx->priv;
    ThreadData *td = arg;
    AVFrame *out = td->out;
    AVFrame *cur = td->cur;
    AVFrame *adj = td->adj;
    const int plane = td->plane;
    const int filter = s->filter;
    uint8_t *in_line, *in_lines_cur[5], *in_lines_adj[5];
    uint8_t *out_line, *out_pixel;
    int32_t *work_line, *work_pixel;
    uint8_t *cur_data = cur->data[plane];
    uint8_t *adj_data = adj->data[plane];
    uint8_t *dst_data = out->data[plane];
    const int linesize = s->linesize[plane];
    const int height   = s->planeheight[plane];
    const int cur_line_stride = cur->linesize[plane];
    const int adj_line_stride = adj->linesize[plane];
    const int dst_line_stride = out->linesize[plane];
    const int start = (height * jobnr) / nb_jobs;
    const int end = (height * (jobnr+1)) / nb_jobs;
    int j, y_in, y_out;

    //copy unchanged the lines of the field 
    y_out = start + (s->field == cur->top_field_first) - (start & 1);

    in_line  = cur_data + (y_out * cur_line_stride);
    out_line = dst_data + (y_out * dst_line_stride);

    while (y_out < end) {
        memcpy(out_line, in_line, linesize);
        y_out += 2;
        in_line  += cur_line_stride * 2;
        out_line += dst_line_stride * 2;
    }

    // interpolate other lines of the field
    y_out = start + (s->field != cur->top_field_first) - (start & 1);

    out_line = dst_data + (y_out * dst_line_stride);

    while (y_out < end) {
        // get low vertical frequencies from current field 
        for (j = 0; j < n_coef_lf[filter]; j++) {
            y_in = (y_out + 1) + (j * 2) - n_coef_lf[filter];

            while (y_in < 0)
                y_in += 2;
            while (y_in >= height)
                y_in -= 2;

            in_lines_cur[j] = cur_data + (y_in * cur_line_stride);
        }

        work_line = s->work_line[jobnr];
        switch (n_coef_lf[filter]) {
        case 2:
            filter_simple_low(work_line, in_lines_cur,
                                     coef_lf[filter], linesize);
            break;
        case 4:
            filter_complex_low(work_line, in_lines_cur,
                                      coef_lf[filter], linesize);
        }

        // get high vertical frequencies from adjacent fields
        for (j = 0; j < n_coef_hf[filter]; j++) {
            y_in = (y_out + 1) + (j * 2) - n_coef_hf[filter];

            while (y_in < 0)
                y_in += 2;
            while (y_in >= height)
                y_in -= 2;

            in_lines_cur[j] = cur_data + (y_in * cur_line_stride);
            in_lines_adj[j] = adj_data + (y_in * adj_line_stride);
        }

        work_line = s->work_line[jobnr];
        switch (n_coef_hf[filter]) {
        case 3:
            filter_simple_high(work_line, in_lines_cur, in_lines_adj,
                                      coef_hf[filter], linesize);
            break;
        case 5:
            filter_complex_high(work_line, in_lines_cur, in_lines_adj,
                                       coef_hf[filter], linesize);
        }

        // save scaled result to the output frame, scaling down by 256 * 128
        work_pixel = s->work_line[jobnr];
        out_pixel = out_line;

        filter_scale(out_pixel, work_pixel, linesize);

        // move on to next line
        y_out += 2;
        out_line += dst_line_stride * 2;
    }

    return 0;
}

static int filter(AVFilterContext *ctx, int is_second)
{
    MEContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *out, *adj;
    ThreadData td;
    int plane;

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out)
        return AVERROR(ENOMEM);
    av_frame_copy_props(out, s->cur);
    out->interlaced_frame = 0;

    if (!is_second) {
        if (out->pts != AV_NOPTS_VALUE)
            out->pts *= 2;
    } else {
        int64_t cur_pts  = s->cur->pts;
        int64_t next_pts = s->next->pts;

        if (next_pts != AV_NOPTS_VALUE && cur_pts != AV_NOPTS_VALUE) {
            out->pts = cur_pts + next_pts;
        } else {
            out->pts = AV_NOPTS_VALUE;
        }
    }

    adj = s->field ? s->next : s->prev;
    td.out = out; td.cur = s->cur; td.adj = adj;
    for (plane = 0; plane < s->nb_planes; plane++) {
        td.plane = plane;
        ctx->internal->execute(ctx, deinterlace_slice, &td, NULL, FFMIN(s->planeheight[plane], s->nb_threads));
    }

    s->field = !s->field;

    return ff_filter_frame(outlink, out);
}*/

static int64_t get_mse(MEContext *s, int x_cur, int y_cur, int x_sb, int y_sb)
{
    
    uint8_t *buf_cur = s->cur->data[0]; //TODO what about other panels?
    uint8_t *buf_next = s->next->data[0];
    int64_t mse = 0;
    int i, j;
    
    for (i = 0; i < s->block_size; i++) {
        for (j = 0; j < s->block_size; j++) {
            int64_t sb = ((int64_t) y_sb + i) * s->linesize[0] + x_sb + j;
            int64_t cur = ((int64_t) y_cur + i) * s->linesize[0] + x_cur + j;
            int64_t diff = (int) buf_next[sb] - (int) buf_cur[cur];
            mse += pow(diff, 2);

           // if (diff != 0)
//             printf("sx: %d, sy: %d, cx: %d, cy: %d, b_n: %d, b_c: %d\n", t_x_sb, t_y_sb,
            // t_x_cur, t_y_cur, buf_next[t_y_sb + t_x_sb], buf_cur[t_y_cur + t_x_cur]);
        }
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
    //mv->motion_x //XXX: what is this?
    //mv->motion_y //XXX: what is this?
    //mv->motion_scale XXX: what is this?
    mv->source = source; // TODO: confirm, +1 if next frame is used as reference, and -1, if prev?
    mv->flags = 0;
}

/**
 * @param start_x & @param start_y are top left corner of the current block
 * 
 * This implementation, in contrast to get_motion_vector_corner, starts comparing
 * blocks at same position.
 * 
 * @return motion vector for each block
 */
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

                av_log(s->class, AV_LOG_DEBUG, "mse: %d, x: %d, y: %d, sx: %d, sy: %d, dx, dy: %d, %d [before]\n", 
                    mse_min, x_cur, y_cur, x_sb, y_sb, dx, dy);

                mse_min = mse;
                dx = x_sb - x_cur;
                dy = y_sb - y_cur;

                /*if (dx != 0 || dy != 0)
                    av_log(s->class, AV_LOG_DEBUG, "mse: %d, x: %d, y: %d, sx: %d, sy: %d, dx, dy: %d, %d\n", 
                        mse, x_cur, y_cur, x_sb, y_sb, dx, dy);*/
            }
        }
    }

    add_mv_data(mv, s->block_size, x_cur + dx, y_cur + dy, x_cur, y_cur, 0);

    return mv;

}

/**
 * This function is not used.
 * 
 * This function start searching from corner of the search window. This doesn't
 * give accurate result as minimum value mean square error gets encountered first at wrong position.
 */
static void get_motion_vector_corner(AVFilterLink *inlink, int x_cur, int y_cur)
{
    AVFilterContext *ctx = inlink->dst;
    MEContext *s = ctx->priv;

    int x_sb, y_sb, dx = 0, dy = 0;

    int count = 0;

    // TEST: checking if min mse is encountered at same position before searching block from corner of search window.
    int64_t min_mse = get_mse(s, x_cur, y_cur, x_cur, y_cur);

    for (y_sb = av_clip(y_cur - s->reg_size, 0, inlink->h); y_sb < av_clip(y_cur + s->reg_size + 1, 0, inlink->h); y_sb++) {
        for (x_sb = av_clip(x_cur - s->reg_size, 0, inlink->w); x_sb < av_clip(x_cur + s->reg_size + 1, 0, inlink->w); x_sb++) {
            int64_t mse = get_mse(s, x_cur, y_cur, x_sb, y_sb);
            if (mse < min_mse) {

                min_mse = mse;
                dx = x_sb - x_cur;
                dy = y_sb - y_cur;
            }
        }
    }


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

    // if (out->pts != AV_NOPTS_VALUE)
    //     out->pts *= 2;
    return ff_filter_frame(ctx->outputs[0], out);
    
} 

/*static int request_frame(AVFilterLink *outlink)
{    
    AVFilterContext *ctx = outlink->src;
    MEContext *s = ctx->priv;
    int ret;

    if (s->eof)
        return AVERROR_EOF;

    ret = ff_request_frame(ctx->inputs[0]);

    if (ret == AVERROR_EOF && s->cur) {
        AVFrame *next = av_frame_clone(s->next);
        if (!next)
            return AVERROR(ENOMEM);
        next->pts = s->next->pts * 2 - s->cur->pts;
        filter_frame(ctx->inputs[0], next);
        s->eof = 1;
    } else if (ret < 0) {
        return ret;
    }

    return 0;
}*/

static av_cold void uninit(AVFilterContext *ctx)
{
    MEContext *s = ctx->priv;

    av_frame_free(&s->prev);
    av_frame_free(&s->cur );
    av_frame_free(&s->next);

/*  
    int i;
    for (i = 0; i < s->nb_threads; i++)
        av_freep(&s->work_line[i]);

    av_freep(&s->work_line);
*/
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
        // .config_props  = config_output,
        // .request_frame = request_frame,
    },
    { NULL }
};

AVFilter ff_vf_mestf = {
    .name          = "mestf",
    .description   = NULL_IF_CONFIG_SMALL("Motion Estimation filter. Does nothing yet."),
    .priv_size     = sizeof(MEContext),
    .priv_class    = &mestf_class,
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = mestf_inputs,
    .outputs       = mestf_outputs,
    .flags         = AVFILTER_FLAG_SUPPORT_TIMELINE_INTERNAL | AVFILTER_FLAG_SLICE_THREADS,
};
