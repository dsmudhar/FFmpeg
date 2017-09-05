/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
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

/*#define DWT_97 0

#define liftS lift
#define W_AM 3
#define W_AO 0
#define W_AS 1

#undef liftS
#define W_BM 1
#define W_BO 8
#define W_BS 4

#define W_CM 1
#define W_CO 0
#define W_CS 0

#define W_DM 3
#define W_DO 4
#define W_DS 3
*/

#include "libavutil/intmath.h"
#include "libavutil/libm.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/internal.h"
#include "snow_lavf.h"

//#include "libavcodec/rangecoder.h"
#include "libavcodec/mathops.h"

#include "libavcodec/mpegvideo.h"
#include "libavcodec/h263.h"

#define FF_ME_ITER 3

FF_DISABLE_DEPRECATION_WARNINGS

/*
static void snow_horizontal_compose97i(short *b, short *temp, int width)
{
    const int w2 = (width + 1) >> 1;
    int x;

    temp[0] = b[0] - ((3 * b[w2] + 2) >> 2);
    for (x = 1; x < (width >> 1); x++) {
        temp[2 * x]     = b[x] - ((3 * (b[x + w2 - 1] + b[x + w2]) + 4) >> 3);
        temp[2 * x - 1] = b[x + w2 - 1] - temp[2 * x - 2] - temp[2 * x];
    }
    if (width & 1) {
        temp[2 * x]     = b[x] - ((3 * b[x + w2 - 1] + 2) >> 2);
        temp[2 * x - 1] = b[x + w2 - 1] - temp[2 * x - 2] - temp[2 * x];
    } else
        temp[2 * x - 1] = b[x + w2 - 1] - 2 * temp[2 * x - 2];

    b[0] = temp[0] + ((2 * temp[0] + temp[1] + 4) >> 3);
    for (x = 2; x < width - 1; x += 2) {
        b[x]     = temp[x] + ((4 * temp[x] + temp[x - 1] + temp[x + 1] + 8) >> 4);
        b[x - 1] = temp[x - 1] + ((3 * (b[x - 2] + b[x])) >> 1);
    }
    if (width & 1) {
        b[x]     = temp[x] + ((2 * temp[x] + temp[x - 1] + 4) >> 3);
        b[x - 1] = temp[x - 1] + ((3 * (b[x - 2] + b[x])) >> 1);
    } else
        b[x - 1] = temp[x - 1] + 3 * b[x - 2];
}

static void vertical_compose97iH0(short *b0, short *b1, short *b2, int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] += (W_AM * (b0[i] + b2[i]) + W_AO) >> W_AS;
}

static void vertical_compose97iH1(short *b0, short *b1, short *b2, int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] -= (W_CM * (b0[i] + b2[i]) + W_CO) >> W_CS;
}

static void vertical_compose97iL0(short *b0, short *b1, short *b2, int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] += (W_BM * (b0[i] + b2[i]) + 4 * b1[i] + W_BO) >> W_BS;
}

static void vertical_compose97iL1(short *b0, short *b1, short *b2, int width)
{
    int i;

    for (i = 0; i < width; i++)
        b1[i] -= (W_DM * (b0[i] + b2[i]) + W_DO) >> W_DS;
}

static void spatial_compose97i_dy(LavfDWTCompose *cs, short *buffer,
                                  short *temp, int width, int height,
                                  int stride)
{
    int y        = cs->y;
    short *b0 = cs->b0;
    short *b1 = cs->b1;
    short *b2 = cs->b2;
    short *b3 = cs->b3;
    short *b4 = buffer + avpriv_mirror(y + 3, height - 1) * stride;
    short *b5 = buffer + avpriv_mirror(y + 4, height - 1) * stride;

    if (y + 3 < (unsigned)height)
        vertical_compose97iL1(b3, b4, b5, width);
    if (y + 2 < (unsigned)height)
        vertical_compose97iH1(b2, b3, b4, width);
    if (y + 1 < (unsigned)height)
        vertical_compose97iL0(b1, b2, b3, width);
    if (y + 0 < (unsigned)height)
        vertical_compose97iH0(b0, b1, b2, width);

    if (y - 1 < (unsigned)height)
        snow_horizontal_compose97i(b0, temp, width);
    if (y + 0 < (unsigned)height)
        snow_horizontal_compose97i(b1, temp, width);

    cs->b0  = b2;
    cs->b1  = b3;
    cs->b2  = b4;
    cs->b3  = b5;
    cs->y  += 2;
}

static void spatial_compose97i_init(LavfDWTCompose *cs, short *buffer, int height,
                                    int stride)
{
    cs->b0 = buffer + avpriv_mirror(-3 - 1, height - 1) * stride;
    cs->b1 = buffer + avpriv_mirror(-3,     height - 1) * stride;
    cs->b2 = buffer + avpriv_mirror(-3 + 1, height - 1) * stride;
    cs->b3 = buffer + avpriv_mirror(-3 + 2, height - 1) * stride;
    cs->y  = -3;
}

static void lavfsnow_spatial_idwt_init(LavfDWTCompose *cs, short *buffer, int width,
                                 int height, int stride, int type,
                                 int decomposition_count)
{
    int level;
    for (level = decomposition_count - 1; level >= 0; level--) {
        switch (type) {
        case DWT_97:
            spatial_compose97i_init(cs + level, buffer, height >> level,
                                    stride << level);
            break;
        }
    }
}

static void lavfsnow_spatial_idwt_slice(LavfDWTCompose *cs, short *buffer,
                                  short *temp, int width, int height,
                                  int stride, int type,
                                  int decomposition_count, int y)
{
    const int support = type == 1 ? 3 : 5;
    int level;
    if (type == 2)
        return;

    for (level = decomposition_count - 1; level >= 0; level--)
        while (cs[level].y <= FFMIN((y >> level) + support, height >> level)) {
            switch (type) {
            case DWT_97:
                spatial_compose97i_dy(cs + level, buffer, temp, width >> level,
                                      height >> level, stride << level);
                break;
            }
        }
}

static void lavfsnow_spatial_idwt(short *buffer, short *temp, int width, int height,
                     int stride, int type, int decomposition_count)
{
    LavfDWTCompose cs[MAX_DECOMPOSITIONS];
    int y;
    lavfsnow_spatial_idwt_init(cs, buffer, width, height, stride, type, decomposition_count);
    for (y = 0; y < height; y += 4)
        lavfsnow_spatial_idwt_slice(cs, buffer, temp, width, height, stride, type, decomposition_count, y);
}*/

/*static inline void put_rac_snow(RangeCoder *c, uint8_t *const state, int bit) {
    //put_rac(c, state, bit);
}*/

static int snow_get_buffer(LavfSnowContext *s, AVFrame *frame)
{
    int ret, i;
    int edges_needed = av_codec_is_encoder(s->avctx->codec);

    frame->width  = s->avctx->width ;
    frame->height = s->avctx->height;
    if (edges_needed) {
        frame->width  += 2 * EDGE_WIDTH;
        frame->height += 2 * EDGE_WIDTH;
    }
    if ((ret = ff_get_buffer(s->avctx, frame, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;
    if (edges_needed) {
        for (i = 0; frame->data[i]; i++) {
            int offset = (EDGE_WIDTH >> (i ? s->chroma_v_shift : 0)) *
                            frame->linesize[i] +
                            (EDGE_WIDTH >> (i ? s->chroma_h_shift : 0));
            frame->data[i] += offset;
        }
        frame->width  = s->avctx->width;
        frame->height = s->avctx->height;
    }

    return 0;
}

static av_cold int encode_init2(AVCodecContext *avctx, LavfSnowContext *s)
{
    int plane_index, ret;
    int i;
    MotionEstContext *mest_ctx = &s->mest_ctx;

/*#if FF_API_PRIVATE_OPT
    if (avctx->prediction_method)
        //s->pred = avctx->prediction_method;
#endif
*/
    /*if(//s->pred == DWT_97 &&
    (avctx->flags & AV_CODEC_FLAG_QSCALE) && avctx->global_quality == 0) {
        av_log(avctx, AV_LOG_ERROR, "The 9/7 wavelet is incompatible with lossless mode.\n");
        return -1;
    }*/
#if FF_API_MOTION_EST
    if (avctx->me_method == ME_ITER)
        s->motion_est = FF_ME_ITER;
#endif

    //s->spatial_decomposition_type= s->pred; //FIXME add decorrelator type r transform_type

    s->mv_scale       = (avctx->flags & AV_CODEC_FLAG_QPEL) ? 2 : 4;
    s->block_max_depth= (avctx->flags & AV_CODEC_FLAG_4MV ) ? 1 : 0;

    for(plane_index=0; plane_index<3; plane_index++){
        s->plane[plane_index].hcoeff[0]=  40;
        s->plane[plane_index].hcoeff[1]= -10;
        s->plane[plane_index].hcoeff[2]=   2;
        s->plane[plane_index].fast_mc= 1;
    }

    if ((ret = lavfsnow_common_init(avctx)) < 0) {
        return ret;
    }
    ff_mpegvideoencdsp_init(&s->mpvencdsp, avctx);

    lavfsnow_alloc_blocks(s);

    //s->version=0;

    //s->mpeg.avctx   = avctx;
    //s->mpeg.bit_rate= avctx->bit_rate;

    mest_ctx->temp      =
    mest_ctx->scratchpad= av_mallocz_array((avctx->width+64), 2*16*2*sizeof(uint8_t));
    mest_ctx->map       = av_mallocz(ME_MAP_SIZE*sizeof(uint32_t));
    mest_ctx->score_map = av_mallocz(ME_MAP_SIZE*sizeof(uint32_t));
    s->obmc_scratchpad= av_mallocz(MB_SIZE*MB_SIZE*12*sizeof(uint32_t));
    if (!mest_ctx->scratchpad || !mest_ctx->map || !mest_ctx->score_map || !s->obmc_scratchpad)
        return AVERROR(ENOMEM);

    //ff_h263_encode_init(&s->mpeg); //mv_penalty
    mest_ctx->mv_penalty = ff_h263_init_mv_penalty_and_fcode();

    s->max_ref_frames = av_clip(avctx->refs, 1, MAX_REF_FRAMES);

    /*if(avctx->flags&AV_CODEC_FLAG_PASS1){
        if(!avctx->stats_out)
            avctx->stats_out = av_mallocz(256);

        if (!avctx->stats_out)
            return AVERROR(ENOMEM);
    }
    if((avctx->flags&AV_CODEC_FLAG_PASS2) || !(avctx->flags&AV_CODEC_FLAG_QSCALE)){
        if(ff_rate_control_init(&s->mpeg) < 0)
            return -1;
    }*/
    s->pass1_rc= !(avctx->flags & (AV_CODEC_FLAG_QSCALE|AV_CODEC_FLAG_PASS2));

    switch(avctx->pix_fmt){
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUV410P:
            s->nb_planes = 3;
            //s->colorspace_type= 0;
            break;
        /*case AV_PIX_FMT_GRAY8:
            s->nb_planes = 1;
            s->colorspace_type = 1;
            break;*/
    default:
        av_log(avctx, AV_LOG_ERROR, "pixel format not supported\n");
        return -1;
    }
    avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_h_shift, &s->chroma_v_shift);

    ff_set_cmp(&s->mecc, s->mecc.me_cmp, s->avctx->me_cmp);
    ff_set_cmp(&s->mecc, s->mecc.me_sub_cmp, s->avctx->me_sub_cmp);

    s->input_avframe = av_frame_alloc();
    if (!s->input_avframe)
        return AVERROR(ENOMEM);

    if ((ret = snow_get_buffer(s, s->input_avframe)) < 0)
        return ret;

    if(s->motion_est == FF_ME_ITER){
        int size= s->b_width * s->b_height << 2*s->block_max_depth;
        for(i=0; i<s->max_ref_frames; i++){
            s->ref_mvs[i]= av_mallocz_array(size, sizeof(int16_t[2]));
            s->ref_scores[i]= av_mallocz_array(size, sizeof(uint32_t));
            if (!s->ref_mvs[i] || !s->ref_scores[i])
                return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static av_cold int encode_init(AVCodecContext *avctx)
{
    LavfSnowContext *s = avctx->priv_data;
    return encode_init2(avctx, s);
}

/*#define snow_slice_buffer_get_line(slice_buf, line_num)                     \
    ((slice_buf)->line[line_num] ? (slice_buf)->line[line_num]              \
                                 : snow_slice_buffer_load_line((slice_buf), \
                                                             (line_num)))

static short *snow_slice_buffer_load_line(lavf_slice_buffer *buf, int line)
{
    short *buffer;

    av_assert0(buf->data_stack_top >= 0);
    if (buf->line[line])
        return buf->line[line];

    buffer = buf->data_stack[buf->data_stack_top];
    buf->data_stack_top--;
    buf->line[line] = buffer;

    return buffer;
}*/

/*static void snow_inner_add_yblock(const uint8_t *obmc, const int obmc_stride, uint8_t * * block, int b_w, int b_h,
                              int src_x, int src_y, int src_stride, lavf_slice_buffer * sb, int add, uint8_t * dst8){
    int y, x;
    short * dst;
    for(y=0; y<b_h; y++){
        //FIXME ugly misuse of obmc_stride
        const uint8_t *obmc1= obmc + y*obmc_stride;
        const uint8_t *obmc2= obmc1+ (obmc_stride>>1);
        const uint8_t *obmc3= obmc1+ obmc_stride*(obmc_stride>>1);
        const uint8_t *obmc4= obmc3+ (obmc_stride>>1);
        dst = snow_slice_buffer_get_line(sb, src_y + y);
        for(x=0; x<b_w; x++){
            int v=   obmc1[x] * block[3][x + y*src_stride]
                    +obmc2[x] * block[2][x + y*src_stride]
                    +obmc3[x] * block[1][x + y*src_stride]
                    +obmc4[x] * block[0][x + y*src_stride];

            v <<= 8 - LOG2_OBMC_MAX;
            if(FRAC_BITS != 8){
                v >>= 8 - FRAC_BITS;
            }
            if(add){
                v += dst[x + src_x];
                v = (v + (1<<(FRAC_BITS-1))) >> FRAC_BITS;
                if(v&(~255)) v= ~(v>>31);
                dst8[x + y*src_stride] = v;
            }else{
                dst[x + src_x] -= v;
            }
        }
    }
}*/


static inline void pred_mv(LavfSnowContext *s, int *mx, int *my, int ref,
                           const LavfBlockNode *left, const LavfBlockNode *top, const LavfBlockNode *tr){
    if(s->ref_frames == 1){
        *mx = mid_pred(left->mx, top->mx, tr->mx);
        *my = mid_pred(left->my, top->my, tr->my);
    }else{
        const int *scale = lavfsnow_scale_mv_ref[ref];
        *mx = mid_pred((left->mx * scale[left->ref] + 128) >>8,
                       (top ->mx * scale[top ->ref] + 128) >>8,
                       (tr  ->mx * scale[tr  ->ref] + 128) >>8);
        *my = mid_pred((left->my * scale[left->ref] + 128) >>8,
                       (top ->my * scale[top ->ref] + 128) >>8,
                       (tr  ->my * scale[tr  ->ref] + 128) >>8);
    }
}

static av_always_inline int same_block(LavfBlockNode *a, LavfBlockNode *b){
    if((a->type&BLOCK_INTRA) && (b->type&BLOCK_INTRA)){
        return !((a->color[0] - b->color[0]) | (a->color[1] - b->color[1]) | (a->color[2] - b->color[2]));
    }else{
        return !((a->mx - b->mx) | (a->my - b->my) | (a->ref - b->ref) | ((a->type ^ b->type)&BLOCK_INTRA));
    }
}

//FIXME name cleanup (b_w, block_w, b_width stuff)
//XXX should we really inline it?
static av_always_inline void snow_add_yblock(LavfSnowContext *s, /*lavf_slice_buffer *sb, */short *dst, uint8_t *dst8, const uint8_t *obmc, int src_x, int src_y, int b_w, int b_h, int w, int h, int dst_stride, int src_stride, int obmc_stride, int b_x, int b_y, int add, int offset_dst, int plane_index){
    const int b_width = s->b_width  << s->block_max_depth;
    const int b_height= s->b_height << s->block_max_depth;
    const int b_stride= b_width;
    LavfBlockNode *lt= &s->block[b_x + b_y*b_stride];
    LavfBlockNode *rt= lt+1;
    LavfBlockNode *lb= lt+b_stride;
    LavfBlockNode *rb= lb+1;
    uint8_t *block[4];
    int sliced = 0;
    // When src_stride is large enough, it is possible to interleave the blocks.
    // Otherwise the blocks are written sequentially in the tmp buffer.
    int tmp_step= src_stride >= 7*MB_SIZE ? MB_SIZE : MB_SIZE*src_stride;
    uint8_t *tmp = s->scratchbuf;
    uint8_t *ptmp;
    int x,y;

    if(b_x<0){
        lt= rt;
        lb= rb;
    }else if(b_x + 1 >= b_width){
        rt= lt;
        rb= lb;
    }
    if(b_y<0){
        lt= lb;
        rt= rb;
    }else if(b_y + 1 >= b_height){
        lb= lt;
        rb= rt;
    }

    if(src_x<0){ //FIXME merge with prev & always round internal width up to *16
        obmc -= src_x;
        b_w += src_x;
        if(!sliced && !offset_dst)
            dst -= src_x;
        src_x=0;
    }
    if(src_x + b_w > w){
        b_w = w - src_x;
    }
    if(src_y<0){
        obmc -= src_y*obmc_stride;
        b_h += src_y;
        if(!sliced && !offset_dst)
            dst -= src_y*dst_stride;
        src_y=0;
    }
    if(src_y + b_h> h){
        b_h = h - src_y;
    }

    if(b_w<=0 || b_h<=0) return;

    if(!sliced && offset_dst)
        dst += src_x + src_y*dst_stride;
    dst8+= src_x + src_y*src_stride;
    //src += src_x + src_y*src_stride;

    ptmp= tmp + 3*tmp_step;
    block[0]= ptmp;
    ptmp+=tmp_step;
    lavfsnow_pred_block(s, block[0], tmp, src_stride, src_x, src_y, b_w, b_h, lt, plane_index, w, h);

    if(same_block(lt, rt)){
        block[1]= block[0];
    }else{
        block[1]= ptmp;
        ptmp+=tmp_step;
        lavfsnow_pred_block(s, block[1], tmp, src_stride, src_x, src_y, b_w, b_h, rt, plane_index, w, h);
    }

    if(same_block(lt, lb)){
        block[2]= block[0];
    }else if(same_block(rt, lb)){
        block[2]= block[1];
    }else{
        block[2]= ptmp;
        ptmp+=tmp_step;
        lavfsnow_pred_block(s, block[2], tmp, src_stride, src_x, src_y, b_w, b_h, lb, plane_index, w, h);
    }

    if(same_block(lt, rb) ){
        block[3]= block[0];
    }else if(same_block(rt, rb)){
        block[3]= block[1];
    }else if(same_block(lb, rb)){
        block[3]= block[2];
    }else{
        block[3]= ptmp;
        lavfsnow_pred_block(s, block[3], tmp, src_stride, src_x, src_y, b_w, b_h, rb, plane_index, w, h);
    }
    /*if(sliced){
        snow_inner_add_yblock(obmc, obmc_stride, block, b_w, b_h, src_x,src_y, src_stride, sb, add, dst8);
    }else{*/
        for(y=0; y<b_h; y++){
            //FIXME ugly misuse of obmc_stride
            const uint8_t *obmc1= obmc + y*obmc_stride;
            const uint8_t *obmc2= obmc1+ (obmc_stride>>1);
            const uint8_t *obmc3= obmc1+ obmc_stride*(obmc_stride>>1);
            const uint8_t *obmc4= obmc3+ (obmc_stride>>1);
            for(x=0; x<b_w; x++){
                int v=   obmc1[x] * block[3][x + y*src_stride]
                        +obmc2[x] * block[2][x + y*src_stride]
                        +obmc3[x] * block[1][x + y*src_stride]
                        +obmc4[x] * block[0][x + y*src_stride];

                v <<= 8 - LOG2_OBMC_MAX;
                if(FRAC_BITS != 8){
                    v >>= 8 - FRAC_BITS;
                }
                if(add){
                    v += dst[x + y*dst_stride];
                    v = (v + (1<<(FRAC_BITS-1))) >> FRAC_BITS;
                    if(v&(~255)) v= ~(v>>31);
                    dst8[x + y*src_stride] = v;
                }else{
                    dst[x + y*dst_stride] -= v;
                }
            }
        }
    //}
}

static inline void set_blocks(LavfSnowContext *s, int level, int x, int y, int l, int cb, int cr, int mx, int my, int ref, int type){
    const int w= s->b_width << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    const int block_w= 1<<rem_depth;
    const int block_h= 1<<rem_depth; //FIXME "w!=h"
    LavfBlockNode block;
    int i,j;

    block.color[0]= l;
    block.color[1]= cb;
    block.color[2]= cr;
    block.mx= mx;
    block.my= my;
    block.ref= ref;
    block.type= type;
    block.level= level;

    for(j=0; j<block_h; j++){
        for(i=0; i<block_w; i++){
            s->block[index + i + j*w]= block;
        }
    }
}

static inline void init_ref(MotionEstContext *c, uint8_t *src[3], uint8_t *ref[3], uint8_t *ref2[3], int x, int y, int ref_index){
    LavfSnowContext *s = c->avctx->priv_data;
    const int offset[3]= {
          y*c->  stride + x,
        ((y*c->uvstride + x)>>s->chroma_h_shift),
        ((y*c->uvstride + x)>>s->chroma_h_shift),
    };
    int i;
    for(i=0; i<3; i++){
        c->src[0][i]= src [i];
        c->ref[0][i]= ref [i] + offset[i];
    }
    av_assert2(!ref_index);
}

/* bitstream functions */

/*static inline void put_symbol(RangeCoder *c, uint8_t *state, int v, int is_signed){
    int i;

    if(v){
        const int a= FFABS(v);
        const int e= av_log2(a);
        const int el= FFMIN(e, 10);
        put_rac_snow(c, state+0, 0);

        for(i=0; i<el; i++){
            put_rac_snow(c, state+1+i, 1);  //1..10
        }
        for(; i<e; i++){
            put_rac_snow(c, state+1+9, 1);  //1..10
        }
        put_rac_snow(c, state+1+FFMIN(i,9), 0);

        for(i=e-1; i>=el; i--){
            put_rac_snow(c, state+22+9, (a>>i)&1); //22..31
        }
        for(; i>=0; i--){
            put_rac_snow(c, state+22+i, (a>>i)&1); //22..31
        }

        if(is_signed)
            put_rac_snow(c, state+11 + el, v < 0); //11..21
    }else{
        put_rac_snow(c, state+0, 1);
    }
}*/

//near copy & paste from dsputil, FIXME
static int pix_sum(uint8_t * pix, int line_size, int w, int h)
{
    int s, i, j;

    s = 0;
    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j++) {
            s += pix[0];
            pix ++;
        }
        pix += line_size - w;
    }
    return s;
}

/*static int pix_norm1(uint8_t * pix, int line_size, int w)
{
    int s, i, j;
    uint32_t *sq = ff_square_tab + 256;

    s = 0;
    for (i = 0; i < w; i++) {
        for (j = 0; j < w; j ++) {
            s += sq[pix[0]];
            pix ++;
        }
        pix += line_size - w;
    }
    return s;
}*/

//TODO could be merged with one in motion_est.c
static inline int get_penalty_factor(int lambda, int lambda2, int type){
    switch(type&0xFF){
    default:
    case FF_CMP_SAD:
        return lambda>>FF_LAMBDA_SHIFT;
    case FF_CMP_DCT:
        return (3*lambda)>>(FF_LAMBDA_SHIFT+1);
    case FF_CMP_W53:
        return (4*lambda)>>(FF_LAMBDA_SHIFT);
    case FF_CMP_W97:
        return (2*lambda)>>(FF_LAMBDA_SHIFT);
    case FF_CMP_SATD:
    case FF_CMP_DCT264:
        return (2*lambda)>>FF_LAMBDA_SHIFT;
    case FF_CMP_RD:
    case FF_CMP_PSNR:
    case FF_CMP_SSE:
    case FF_CMP_NSSE:
        return lambda2>>FF_LAMBDA_SHIFT;
    case FF_CMP_BIT:
        return 1;
    }
}

//FIXME copy&paste
#define P_LEFT P[1]
#define P_TOP P[2]
#define P_TOPRIGHT P[3]
#define P_MEDIAN P[4]
#define P_MV1 P[9]
#define FLAG_QPEL   1 //must be 1

static void init_mpeg_struct(MotionEstContext * c) {
    MpegMEStruct * s = &c->mme_struct;
    s->mb_stride = 2;
    s->mb_x = 0;
    s->mb_y = 0;

    //s. pict_type; //done in encode()
    //s. mb_width, mb_height; //done in encode()
    s->width = c->avctx->width;
    s->height = c->avctx->height;

    /* not used in snow */
    s->end_mb_y = 0;
    s->first_slice_line = 0;
    s->mpv_flags = 0;
    s->pp_time = 0;
    s->pb_time = 0;
    s->mv_type = 0;
}

static int encode_q_branch(LavfSnowContext *s, int level, int x, int y){
    /*uint8_t p_buffer[1024];*/
    //uint8_t i_buffer[1024];
    /*uint8_t p_state[sizeof(s->block_state)];*/
    //uint8_t i_state[sizeof(s->block_state)];
    /*RangeCoder pc, ic; */
    /*uint8_t *pbbak= s->c.bytestream;*/
    /*uint8_t *pbbak_start= s->c.bytestream_start;*/
    int score, score2, /*iscore, i_len, p_len,*/ block_s, sum/*, base_bits*/;
    const int w= s->b_width  << s->block_max_depth;
    const int h= s->b_height << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    const int block_w= 1<<(LOG2_MB_SIZE - level);
    int trx= (x+1)<<rem_depth;
    int try= (y+1)<<rem_depth;
    const LavfBlockNode *left  = x ? &s->block[index-1] : &null_block;
    const LavfBlockNode *top   = y ? &s->block[index-w] : &null_block;
    const LavfBlockNode *right = trx<w ? &s->block[index+1] : &null_block;
    const LavfBlockNode *bottom= try<h ? &s->block[index+w] : &null_block;
    const LavfBlockNode *tl    = y && x ? &s->block[index-w-1] : left;
    const LavfBlockNode *tr    = y && trx<w && ((x&1)==0 || level==0) ? &s->block[index-w+(1<<rem_depth)] : tl; //FIXME use lt
    int pl = left->color[0];
    int pcb= left->color[1];
    int pcr= left->color[2];
    int pmx, pmy;
    int mx=0, my=0;
    int l,cr,cb;
    const int stride= s->current_avframe->linesize[0];
    const int uvstride= s->current_avframe->linesize[1];
    uint8_t *current_data[3]= { s->input_avframe->data[0] + (x + y*  stride)*block_w,
                                s->input_avframe->data[1] + ((x*block_w)>>s->chroma_h_shift) + ((y*uvstride*block_w)>>s->chroma_v_shift),
                                s->input_avframe->data[2] + ((x*block_w)>>s->chroma_h_shift) + ((y*uvstride*block_w)>>s->chroma_v_shift)};
    int P[10][2];
    int16_t last_mv[3][2];
    int qpel= !!(s->avctx->flags & AV_CODEC_FLAG_QPEL); //unused
    const int shift= 1+qpel;
    MotionEstContext *mest_ctx= &s->mest_ctx;
    /*int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);*/
    /*int mx_context= av_log2(2*FFABS(left->mx - top->mx));*/
    /*int my_context= av_log2(2*FFABS(left->my - top->my));*/
    /*int s_context= 2*left->level + 2*top->level + tl->level + tr->level;*/
    int ref, best_ref, ref_score, ref_mx, ref_my;

    //av_assert0(sizeof(s->block_state) >= 256);
    if(s->keyframe){
        set_blocks(s, level, x, y, pl, pcb, pcr, 0, 0, 0, BLOCK_INTRA);
        return 0;
    }

//    clip predictors / edge ?

    P_LEFT[0]= left->mx;
    P_LEFT[1]= left->my;
    P_TOP [0]= top->mx;
    P_TOP [1]= top->my;
    P_TOPRIGHT[0]= tr->mx;
    P_TOPRIGHT[1]= tr->my;

    last_mv[0][0]= s->block[index].mx;
    last_mv[0][1]= s->block[index].my;
    last_mv[1][0]= right->mx;
    last_mv[1][1]= right->my;
    last_mv[2][0]= bottom->mx;
    last_mv[2][1]= bottom->my;

    /*s->mpeg.mb_stride=2;
    s->mpeg.mb_x=
    s->mpeg.mb_y= 0;*/
    init_mpeg_struct(mest_ctx);
    mest_ctx->skip= 0;

    av_assert1(mest_ctx->  stride ==   stride);
    av_assert1(mest_ctx->uvstride == uvstride);

    mest_ctx->penalty_factor    = get_penalty_factor(s->lambda, s->lambda2, mest_ctx->avctx->me_cmp);
    mest_ctx->sub_penalty_factor= get_penalty_factor(s->lambda, s->lambda2, mest_ctx->avctx->me_sub_cmp);
    mest_ctx->mb_penalty_factor = get_penalty_factor(s->lambda, s->lambda2, mest_ctx->avctx->mb_cmp);
    mest_ctx->current_mv_penalty= mest_ctx->mv_penalty[/*s->mpeg.f_code=*/1] + MAX_DMV;

    mest_ctx->xmin = - x*block_w - 16+3;
    mest_ctx->ymin = - y*block_w - 16+3;
    mest_ctx->xmax = - (x+1)*block_w + (w<<(LOG2_MB_SIZE - s->block_max_depth)) + 16-3;
    mest_ctx->ymax = - (y+1)*block_w + (h<<(LOG2_MB_SIZE - s->block_max_depth)) + 16-3;

    if(P_LEFT[0]     > (mest_ctx->xmax<<shift)) P_LEFT[0]    = (mest_ctx->xmax<<shift);
    if(P_LEFT[1]     > (mest_ctx->ymax<<shift)) P_LEFT[1]    = (mest_ctx->ymax<<shift);
    if(P_TOP[0]      > (mest_ctx->xmax<<shift)) P_TOP[0]     = (mest_ctx->xmax<<shift);
    if(P_TOP[1]      > (mest_ctx->ymax<<shift)) P_TOP[1]     = (mest_ctx->ymax<<shift);
    if(P_TOPRIGHT[0] < (mest_ctx->xmin<<shift)) P_TOPRIGHT[0]= (mest_ctx->xmin<<shift);
    if(P_TOPRIGHT[0] > (mest_ctx->xmax<<shift)) P_TOPRIGHT[0]= (mest_ctx->xmax<<shift); //due to pmx no clip
    if(P_TOPRIGHT[1] > (mest_ctx->ymax<<shift)) P_TOPRIGHT[1]= (mest_ctx->ymax<<shift);

    P_MEDIAN[0]= mid_pred(P_LEFT[0], P_TOP[0], P_TOPRIGHT[0]);
    P_MEDIAN[1]= mid_pred(P_LEFT[1], P_TOP[1], P_TOPRIGHT[1]);

    if (!y) {
        mest_ctx->pred_x= P_LEFT[0];
        mest_ctx->pred_y= P_LEFT[1];
    } else {
        mest_ctx->pred_x = P_MEDIAN[0];
        mest_ctx->pred_y = P_MEDIAN[1];
    }

    score= INT_MAX;
    best_ref= 0;

    for(ref=0; ref<s->ref_frames; ref++){
        init_ref(mest_ctx, current_data, s->last_avframe[ref]->data, NULL, block_w*x, block_w*y, 0);

        ref_score= ff_epzs_motion_search(mest_ctx, &ref_mx, &ref_my, P, 0, /*ref_index*/ 0, last_mv,
                                         (1<<16)>>shift, level-LOG2_MB_SIZE+4, block_w);

        av_assert2(ref_mx >= mest_ctx->xmin);
        av_assert2(ref_mx <= mest_ctx->xmax);
        av_assert2(ref_my >= mest_ctx->ymin);
        av_assert2(ref_my <= mest_ctx->ymax);

        ref_score= mest_ctx->sub_motion_search(mest_ctx, &ref_mx, &ref_my, ref_score, 0, 0, level-LOG2_MB_SIZE+4, block_w);
        ref_score= ff_get_mb_score(mest_ctx, ref_mx, ref_my, 0, 0, level-LOG2_MB_SIZE+4, block_w, 0);
        ref_score+= 2*av_log2(2*ref)*mest_ctx->penalty_factor;
        if(s->ref_mvs[ref]){
            s->ref_mvs[ref][index][0]= ref_mx;
            s->ref_mvs[ref][index][1]= ref_my;
            s->ref_scores[ref][index]= ref_score;
        }
        if(score > ref_score){
            score= ref_score;
            best_ref= ref;
            mx= ref_mx;
            my= ref_my;
        }
    }
    //FIXME if mb_cmp != SSE then intra cannot be compared currently and mb_penalty vs. lambda2

  //  subpel search
    /*base_bits= get_rac_count(&s->c) - 8*(s->c.bytestream - s->c.bytestream_start);
    pc= s->c;
    pc.bytestream_start=
    pc.bytestream= p_buffer; //FIXME end/start? and at the other stoo
    memcpy(p_state, s->block_state, sizeof(s->block_state));*/

    /*if(level!=s->block_max_depth)
        put_rac_snow(&pc, &p_state[4 + s_context], 1);
    put_rac_snow(&pc, &p_state[1 + left->type + top->type], 0);
    if(s->ref_frames > 1) put_symbol(&pc, &p_state[128 + 1024 + 32*ref_context], best_ref, 0);*/
    pred_mv(s, &pmx, &pmy, best_ref, left, top, tr);
    /*put_symbol(&pc, &p_state[128 + 32*(mx_context + 16*!!best_ref)], mx - pmx, 1);
    put_symbol(&pc, &p_state[128 + 32*(my_context + 16*!!best_ref)], my - pmy, 1);*/
    /*p_len= pc.bytestream - pc.bytestream_start;*/
    //THIS COULD BE CRITICAL:
    score += (s->lambda2/**(get_rac_count(&pc)-base_bits)*/)>>FF_LAMBDA_SHIFT;

    block_s= block_w*block_w;
    sum = pix_sum(current_data[0], stride, block_w, block_w);
    l= (sum + block_s/2)/block_s;
    /*iscore = pix_norm1(current_data[0], stride, block_w) - 2*l*sum + l*l*block_s;*/

    if (s->nb_planes > 2) {
        block_s= block_w*block_w>>(s->chroma_h_shift + s->chroma_v_shift);
        sum = pix_sum(current_data[1], uvstride, block_w>>s->chroma_h_shift, block_w>>s->chroma_v_shift);
        cb= (sum + block_s/2)/block_s;
    //    iscore += pix_norm1(&current_mb[1][0], uvstride, block_w>>1) - 2*cb*sum + cb*cb*block_s;
        sum = pix_sum(current_data[2], uvstride, block_w>>s->chroma_h_shift, block_w>>s->chroma_v_shift);
        cr= (sum + block_s/2)/block_s;
    //    iscore += pix_norm1(&current_mb[2][0], uvstride, block_w>>1) - 2*cr*sum + cr*cr*block_s;
    }else
        cb = cr = 0;

    /*ic= s->c;
    ic.bytestream_start=
    ic.bytestream= i_buffer; //FIXME end/start? and at the other stoo*/
    //memcpy(i_state, s->block_state, sizeof(s->block_state));
    /*if(level!=s->block_max_depth)
        put_rac_snow(&ic, &i_state[4 + s_context], 1);
    put_rac_snow(&ic, &i_state[1 + left->type + top->type], 1);*/
    /*put_symbol(&ic, &i_state[32],  l-pl , 1);
    if (s->nb_planes > 2) {
        put_symbol(&ic, &i_state[64], cb-pcb, 1);
        put_symbol(&ic, &i_state[96], cr-pcr, 1);
    }*/
    /*i_len= ic.bytestream - ic.bytestream_start;*/
    /*iscore += (s->lambda2*(get_rac_count(&ic)-base_bits))>>FF_LAMBDA_SHIFT;

    av_assert1(iscore < 255*255*256 + s->lambda2*10);
    av_assert1(iscore >= 0);*/
    av_assert1(l>=0 && l<=255);
    av_assert1(pl>=0 && pl<=255);

    /*if(level==0){
        //int varc= iscore >> 8;
        int vard= score >> 8;
        if (vard <= 64 || vard < varc)
            mest_ctx->scene_change_score+= ff_sqrt(vard) - ff_sqrt(varc);
        else
            mest_ctx->scene_change_score+= s->mpeg.qscale;
    }*/

    if(level!=s->block_max_depth){
        /*put_rac_snow(&s->c, &s->block_state[4 + s_context], 0);*/
        score2 = encode_q_branch(s, level+1, 2*x+0, 2*y+0);
        score2+= encode_q_branch(s, level+1, 2*x+1, 2*y+0);
        score2+= encode_q_branch(s, level+1, 2*x+0, 2*y+1);
        score2+= encode_q_branch(s, level+1, 2*x+1, 2*y+1);
        score2+= s->lambda2>>FF_LAMBDA_SHIFT; //FIXME exact split overhead

        if(score2 < score/* && score2 < iscore*/)
            return score2;
    }

    //NO intra blocks
    /*if(iscore < score){
        pred_mv(s, &pmx, &pmy, 0, left, top, tr);
        memcpy(pbbak, i_buffer, i_len);
        s->c= ic;
        s->c.bytestream_start= pbbak_start;
        s->c.bytestream= pbbak + i_len;
        set_blocks(s, level, x, y, l, cb, cr, pmx, pmy, 0, BLOCK_INTRA);
        memcpy(s->block_state, i_state, sizeof(s->block_state));
        return iscore;
    }else{*/
        /*memcpy(pbbak, p_buffer, p_len);
        s->c= pc;
        s->c.bytestream_start= pbbak_start;
        s->c.bytestream= pbbak + p_len;*/
        set_blocks(s, level, x, y, pl, pcb, pcr, mx, my, best_ref, 0);
        /*memcpy(s->block_state, p_state, sizeof(s->block_state));*/
        return score;
    //}
}

static void encode_q_branch2(LavfSnowContext *s, int level, int x, int y){
    const int w= s->b_width  << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    int trx= (x+1)<<rem_depth;
    LavfBlockNode *b= &s->block[index];
    const LavfBlockNode *left  = x ? &s->block[index-1] : &null_block;
    const LavfBlockNode *top   = y ? &s->block[index-w] : &null_block;
    const LavfBlockNode *tl    = y && x ? &s->block[index-w-1] : left;
    const LavfBlockNode *tr    = y && trx<w && ((x&1)==0 || level==0) ? &s->block[index-w+(1<<rem_depth)] : tl; //FIXME use lt
    int pl = left->color[0];
    int pcb= left->color[1];
    int pcr= left->color[2];
    int pmx, pmy;
    /*int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);
    int mx_context= av_log2(2*FFABS(left->mx - top->mx)) + 16*!!b->ref;
    int my_context= av_log2(2*FFABS(left->my - top->my)) + 16*!!b->ref;
    int s_context= 2*left->level + 2*top->level + tl->level + tr->level;*/

    if(s->keyframe){
        set_blocks(s, level, x, y, pl, pcb, pcr, 0, 0, 0, BLOCK_INTRA);
        return;
    }

    if(level!=s->block_max_depth){
        /*if(same_block(b,b+1) && same_block(b,b+w) && same_block(b,b+w+1)){
            put_rac_snow(&s->c, &s->block_state[4 + s_context], 1);
        }else{*/
            /*put_rac_snow(&s->c, &s->block_state[4 + s_context], 0);*/
            encode_q_branch2(s, level+1, 2*x+0, 2*y+0);
            encode_q_branch2(s, level+1, 2*x+1, 2*y+0);
            encode_q_branch2(s, level+1, 2*x+0, 2*y+1);
            encode_q_branch2(s, level+1, 2*x+1, 2*y+1);
            return;
        //}
    }
    //NO intra blocks
    /*if(b->type & BLOCK_INTRA){
        pred_mv(s, &pmx, &pmy, 0, left, top, tr);
        put_rac_snow(&s->c, &s->block_state[1 + (left->type&1) + (top->type&1)], 1);
        put_symbol(&s->c, &s->block_state[32], b->color[0]-pl , 1);
        if (s->nb_planes > 2) {
            put_symbol(&s->c, &s->block_state[64], b->color[1]-pcb, 1);
            put_symbol(&s->c, &s->block_state[96], b->color[2]-pcr, 1);
        }
        set_blocks(s, level, x, y, b->color[0], b->color[1], b->color[2], pmx, pmy, 0, BLOCK_INTRA);
    }else{*/
        pred_mv(s, &pmx, &pmy, b->ref, left, top, tr);
        /*put_rac_snow(&s->c, &s->block_state[1 + (left->type&1) + (top->type&1)], 0);
        if(s->ref_frames > 1)
            put_symbol(&s->c, &s->block_state[128 + 1024 + 32*ref_context], b->ref, 0);
        put_symbol(&s->c, &s->block_state[128 + 32*mx_context], b->mx - pmx, 1);
        put_symbol(&s->c, &s->block_state[128 + 32*my_context], b->my - pmy, 1);*/
        set_blocks(s, level, x, y, pl, pcb, pcr, b->mx, b->my, b->ref, 0);
    //}
}

static void get_dc(LavfSnowContext *s, int mb_x, int mb_y, int plane_index){
    int i, x2, y2;
    LavfPlane *p= &s->plane[plane_index];
    const int block_size = MB_SIZE >> s->block_max_depth;
    const int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? lavfsnow_obmc_tab[s->block_max_depth+s->chroma_h_shift] : lavfsnow_obmc_tab[s->block_max_depth];
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    const int ref_stride= s->current_avframe->linesize[plane_index];
    //uint8_t *src= s-> input_avframe->data[plane_index];
    short *dst= (short*)s->obmc_scratchpad + plane_index*block_size*block_size*4; //FIXME change to unsigned
    const int b_stride = s->b_width << s->block_max_depth;
    const int w= p->width;
    const int h= p->height;
    int index= mb_x + mb_y*b_stride;
    LavfBlockNode *b= &s->block[index];
    LavfBlockNode backup= *b;
    /*int ab=0;
    int aa=0;*/

    av_assert2(s->chroma_h_shift == s->chroma_v_shift); //obmc stuff above

    b->type|= BLOCK_INTRA;
    b->color[plane_index]= 0;
    memset(dst, 0, obmc_stride*obmc_stride*sizeof(short));

    for(i=0; i<4; i++){
        int mb_x2= mb_x + (i &1) - 1;
        int mb_y2= mb_y + (i>>1) - 1;
        int x= block_w*mb_x2 + block_w/2;
        int y= block_h*mb_y2 + block_h/2;

        snow_add_yblock(s, dst + (i&1)*block_w + (i>>1)*obmc_stride*block_h, NULL, obmc,
                    x, y, block_w, block_h, w, h, obmc_stride, ref_stride, obmc_stride, mb_x2, mb_y2, 0, 0, plane_index);

        for(y2= FFMAX(y, 0); y2<FFMIN(h, y+block_h); y2++){
            for(x2= FFMAX(x, 0); x2<FFMIN(w, x+block_w); x2++){
                int index= x2-(block_w*mb_x - block_w/2) + (y2-(block_h*mb_y - block_h/2))*obmc_stride;
                //int obmc_v= obmc[index];
                int d;
                /*if(y<0) obmc_v += obmc[index + block_h*obmc_stride];
                if(x<0) obmc_v += obmc[index + block_w];
                if(y+block_h>h) obmc_v += obmc[index - block_h*obmc_stride];
                if(x+block_w>w) obmc_v += obmc[index - block_w];*/
                //FIXME precalculate this or simplify it somehow else

                d = -dst[index] + (1<<(FRAC_BITS-1));
                dst[index] = d;
                //ab += (src[x2 + y2*ref_stride] - (d>>FRAC_BITS)) * obmc_v;
                //aa += obmc_v * obmc_v; //FIXME precalculate this
            }
        }
    }
    *b= backup;

    //return av_clip_uint8( ROUNDED_DIV(ab<<LOG2_OBMC_MAX, aa) ); //FIXME we should not need clipping
}

static inline int get_block_bits(LavfSnowContext *s, int x, int y, int w){
    const int b_stride = s->b_width << s->block_max_depth;
    const int b_height = s->b_height<< s->block_max_depth;
    int index= x + y*b_stride;
    const LavfBlockNode *b     = &s->block[index];
    const LavfBlockNode *left  = x ? &s->block[index-1] : &null_block;
    const LavfBlockNode *top   = y ? &s->block[index-b_stride] : &null_block;
    const LavfBlockNode *tl    = y && x ? &s->block[index-b_stride-1] : left;
    const LavfBlockNode *tr    = y && x+w<b_stride ? &s->block[index-b_stride+w] : tl;
    int dmx, dmy;
//  int mx_context= av_log2(2*FFABS(left->mx - top->mx));
//  int my_context= av_log2(2*FFABS(left->my - top->my));

    if(x<0 || x>=b_stride || y>=b_height)
        return 0;
/*
1            0      0
01X          1-2    1
001XX        3-6    2-3
0001XXX      7-14   4-7
00001XXXX   15-30   8-15
*/
//FIXME try accurate rate
//FIXME intra and inter predictors if surrounding blocks are not the same type
    av_assert0(!(b->type & BLOCK_INTRA));
    /*if(b->type & BLOCK_INTRA){
        return 3+2*( av_log2(2*FFABS(left->color[0] - b->color[0]))
                   + av_log2(2*FFABS(left->color[1] - b->color[1]))
                   + av_log2(2*FFABS(left->color[2] - b->color[2])));
    }else{*/
        pred_mv(s, &dmx, &dmy, b->ref, left, top, tr);
        dmx-= b->mx;
        dmy-= b->my;
        return 2*(1 + av_log2(2*FFABS(dmx)) //FIXME kill the 2* can be merged in lambda
                    + av_log2(2*FFABS(dmy))
                    + av_log2(2*b->ref));
    //}
}

static int get_block_rd(LavfSnowContext *s, int mb_x, int mb_y, int plane_index, uint8_t (*obmc_edged)[MB_SIZE * 2]){
    LavfPlane *p= &s->plane[plane_index];
    const int block_size = MB_SIZE >> s->block_max_depth;
    const int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    const int ref_stride= s->current_avframe->linesize[plane_index];
    uint8_t *dst= s->current_avframe->data[plane_index];
    uint8_t *src= s->  input_avframe->data[plane_index];
    short *pred= (short*)s->obmc_scratchpad + plane_index*block_size*block_size*4;
    uint8_t *cur = s->scratchbuf;
    uint8_t *tmp = s->emu_edge_buffer;
    const int b_stride = s->b_width << s->block_max_depth;
    const int b_height = s->b_height<< s->block_max_depth;
    const int w= p->width;
    const int h= p->height;
    int distortion;
    int rate= 0;
    const int penalty_factor= get_penalty_factor(s->lambda, s->lambda2, s->avctx->me_cmp);
    int sx= block_w*mb_x - block_w/2;
    int sy= block_h*mb_y - block_h/2;
    int x0= FFMAX(0,-sx);
    int y0= FFMAX(0,-sy);
    int x1= FFMIN(block_w*2, w-sx);
    int y1= FFMIN(block_h*2, h-sy);
    int i,x,y;

    av_assert2(s->chroma_h_shift == s->chroma_v_shift); //obmc and square assumtions below chckinhg only block_w

    lavfsnow_pred_block(s, cur, tmp, ref_stride, sx, sy, block_w*2, block_h*2, &s->block[mb_x + mb_y*b_stride], plane_index, w, h);

    for(y=y0; y<y1; y++){
        const uint8_t *obmc1= obmc_edged[y];
        const short *pred1 = pred + y*obmc_stride;
        uint8_t *cur1 = cur + y*ref_stride;
        uint8_t *dst1 = dst + sx + (sy+y)*ref_stride;
        for(x=x0; x<x1; x++){
#if FRAC_BITS >= LOG2_OBMC_MAX
            int v = (cur1[x] * obmc1[x]) << (FRAC_BITS - LOG2_OBMC_MAX);
#else
            int v = (cur1[x] * obmc1[x] + (1<<(LOG2_OBMC_MAX - FRAC_BITS-1))) >> (LOG2_OBMC_MAX - FRAC_BITS);
#endif
            v = (v + pred1[x]) >> FRAC_BITS;
            if(v&(~255)) v= ~(v>>31);
            dst1[x] = v;
        }
    }

    /* copy the regions where obmc[] = (uint8_t)256 */
    if(LOG2_OBMC_MAX == 8
        && (mb_x == 0 || mb_x == b_stride-1)
        && (mb_y == 0 || mb_y == b_height-1)){
        if(mb_x == 0)
            x1 = block_w;
        else
            x0 = block_w;
        if(mb_y == 0)
            y1 = block_h;
        else
            y0 = block_h;
        for(y=y0; y<y1; y++)
            memcpy(dst + sx+x0 + (sy+y)*ref_stride, cur + x0 + y*ref_stride, x1-x0);
    }

    if(block_w==16){
        //DISABLED CODE FROM GIT LOG
        /* FIXME rearrange dsputil to fit 32x32 cmp functions */
        /* FIXME check alignment of the cmp wavelet vs the encoding wavelet */
        /* FIXME cmps overlap but do not cover the wavelet's whole support.
         * So improving the score of one block is not strictly guaranteed
         * to improve the score of the whole frame, thus iterative motion
         * estimation does not always converge. */

        /*if(s->avctx->me_cmp == FF_CMP_W97) distortion = ff_w97_32_c(&s->m, src + sx + sy*ref_stride, dst + sx + sy*ref_stride, ref_stride, 32);
        else if(s->avctx->me_cmp == FF_CMP_W53) distortion = ff_w53_32_c(&s->m, src + sx + sy*ref_stride, dst + sx + sy*ref_stride, ref_stride, 32);
        else{*/
        distortion = 0;
        for(i=0; i<4; i++){
            int off = sx+16*(i&1) + (sy+16*(i>>1))*ref_stride;
            distortion += s->mecc.me_cmp[0](&s->mest_ctx, src + off, dst + off, ref_stride, 16);
        }
        /*}*/
    }else{
        av_assert2(block_w==8);
        distortion = s->mecc.me_cmp[0](&s->mest_ctx, src + sx + sy*ref_stride, dst + sx + sy*ref_stride, ref_stride, block_w*2);
    }

    if(plane_index==0){
        for(i=0; i<4; i++){
/* ..RRr
 * .RXx.
 * rxx..
 */
            rate += get_block_bits(s, mb_x + (i&1) - (i>>1), mb_y + (i>>1), 1);
        }
        if(mb_x == b_stride-2)
            rate += get_block_bits(s, mb_x + 1, mb_y + 1, 1);
    }
    //av_log(NULL, AV_LOG_ERROR, "distortion: %d\n", distortion);
    return distortion + rate*penalty_factor;
}

static int get_4block_rd(LavfSnowContext *s, int mb_x, int mb_y, int plane_index){
    int i, y2;
    LavfPlane *p= &s->plane[plane_index];
    const int block_size = MB_SIZE >> s->block_max_depth;
    const int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? lavfsnow_obmc_tab[s->block_max_depth+s->chroma_h_shift] : lavfsnow_obmc_tab[s->block_max_depth];
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    const int ref_stride= s->current_avframe->linesize[plane_index];
    uint8_t *dst= s->current_avframe->data[plane_index];
    uint8_t *src= s-> input_avframe->data[plane_index];
    //FIXME zero_dst is const but add_yblock changes dst if add is 0 (this is never the case for dst=zero_dst
    // const has only been removed from zero_dst to suppress a warning
    static short zero_dst[4096]; //FIXME
    const int b_stride = s->b_width << s->block_max_depth;
    const int w= p->width;
    const int h= p->height;
    int distortion= 0;
    int rate= 0;
    const int penalty_factor= get_penalty_factor(s->lambda, s->lambda2, s->avctx->me_cmp);

    av_assert2(s->chroma_h_shift == s->chroma_v_shift); //obmc and square assumtions below

    for(i=0; i<9; i++){
        int mb_x2= mb_x + (i%3) - 1;
        int mb_y2= mb_y + (i/3) - 1;
        int x= block_w*mb_x2 + block_w/2;
        int y= block_h*mb_y2 + block_h/2;

        snow_add_yblock(s, zero_dst, dst, obmc,
                   x, y, block_w, block_h, w, h, /*dst_stride*/0, ref_stride, obmc_stride, mb_x2, mb_y2, 1, 1, plane_index);

        //FIXME find a cleaner/simpler way to skip the outside stuff
        for(y2= y; y2<0; y2++)
            memcpy(dst + x + y2*ref_stride, src + x + y2*ref_stride, block_w);
        for(y2= h; y2<y+block_h; y2++)
            memcpy(dst + x + y2*ref_stride, src + x + y2*ref_stride, block_w);
        if(x<0){
            for(y2= y; y2<y+block_h; y2++)
                memcpy(dst + x + y2*ref_stride, src + x + y2*ref_stride, -x);
        }
        if(x+block_w > w){
            for(y2= y; y2<y+block_h; y2++)
                memcpy(dst + w + y2*ref_stride, src + w + y2*ref_stride, x+block_w - w);
        }

        av_assert1(block_w== 8 || block_w==16);
        distortion += s->mecc.me_cmp[block_w==8](&s->mest_ctx, src + x + y*ref_stride, dst + x + y*ref_stride, ref_stride, block_h);
    }

    if(plane_index==0){
        LavfBlockNode *b= &s->block[mb_x+mb_y*b_stride];
        int merged= same_block(b,b+1) && same_block(b,b+b_stride) && same_block(b,b+b_stride+1);

/* ..RRRr
 * .RXXx.
 * .RXXx.
 * rxxx.
 */
        if(merged)
            rate = get_block_bits(s, mb_x, mb_y, 2);
        for(i=merged?4:0; i<9; i++){
            static const int dxy[9][2] = {{0,0},{1,0},{0,1},{1,1},{2,0},{2,1},{-1,2},{0,2},{1,2}};
            rate += get_block_bits(s, mb_x + dxy[i][0], mb_y + dxy[i][1], 1);
        }
    }
    return distortion + rate*penalty_factor;
}

static av_always_inline int check_block(LavfSnowContext *s, int mb_x, int mb_y, int p[3], int intra, uint8_t (*obmc_edged)[MB_SIZE * 2], int *best_rd){
    const int b_stride= s->b_width << s->block_max_depth;
    LavfBlockNode *block= &s->block[mb_x + mb_y * b_stride];
    LavfBlockNode backup= *block;
    unsigned value;
    int rd, index;

    av_assert2(mb_x>=0 && mb_y>=0);
    av_assert2(mb_x<b_stride);

    /*if(intra){
        block->color[0] = p[0];
        block->color[1] = p[1];
        block->color[2] = p[2];
        block->type |= BLOCK_INTRA;
    }else{*/
        index= (p[0] + 31*p[1]) & (ME_CACHE_SIZE-1);
        value= s->me_cache_generation + (p[0]>>10) + (p[1]<<6) + (block->ref<<12);
        if(s->me_cache[index] == value)
            return 0;
        s->me_cache[index]= value;

        block->mx= p[0];
        block->my= p[1];
        block->type &= ~BLOCK_INTRA;
    //}

    rd= get_block_rd(s, mb_x, mb_y, 0, obmc_edged) + s->intra_penalty * !!intra;

//FIXME chroma
    if(rd < *best_rd){
        *best_rd= rd;
        return 1;
    }else{
        *block= backup;
        return 0;
    }
}

/* special case for int[2] args we discard afterwards,
 * fixes compilation problem with gcc 2.95 */
static av_always_inline int check_block_inter(LavfSnowContext *s, int mb_x, int mb_y, int p0, int p1, uint8_t (*obmc_edged)[MB_SIZE * 2], int *best_rd){
    int p[2] = {p0, p1};
    return check_block(s, mb_x, mb_y, p, 0, obmc_edged, best_rd);
}

static av_always_inline int check_4block_inter(LavfSnowContext *s, int mb_x, int mb_y, int p0, int p1, int ref, int *best_rd){
    const int b_stride= s->b_width << s->block_max_depth;
    LavfBlockNode *block= &s->block[mb_x + mb_y * b_stride];
    LavfBlockNode backup[4];
    unsigned value;
    int rd, index;

    /* We don't initialize backup[] during variable declaration, because
     * that fails to compile on MSVC: "cannot convert from 'BlockNode' to
     * 'int16_t'". */
    backup[0] = block[0];
    backup[1] = block[1];
    backup[2] = block[b_stride];
    backup[3] = block[b_stride + 1];

    av_assert2(mb_x>=0 && mb_y>=0);
    av_assert2(mb_x<b_stride);
    av_assert2(((mb_x|mb_y)&1) == 0);

    index= (p0 + 31*p1) & (ME_CACHE_SIZE-1);
    value= s->me_cache_generation + (p0>>10) + (p1<<6) + (block->ref<<12);
    if(s->me_cache[index] == value)
        return 0;
    s->me_cache[index]= value;

    block->mx= p0;
    block->my= p1;
    block->ref= ref;
    block->type &= ~BLOCK_INTRA;
    block[1]= block[b_stride]= block[b_stride+1]= *block;

    rd= get_4block_rd(s, mb_x, mb_y, 0);

//FIXME chroma
    if(rd < *best_rd){
        *best_rd= rd;
        return 1;
    }else{
        block[0]= backup[0];
        block[1]= backup[1];
        block[b_stride]= backup[2];
        block[b_stride+1]= backup[3];
        return 0;
    }
}

static void iterative_me(LavfSnowContext *s){
    int pass, mb_x, mb_y;
    const int b_width = s->b_width  << s->block_max_depth;
    const int b_height= s->b_height << s->block_max_depth;
    const int b_stride= b_width;
    //int color[3];

    {
        //RangeCoder r = s->c;
        //uint8_t state[sizeof(s->block_state)];
        //memcpy(state, s->block_state, sizeof(s->block_state));
        for(mb_y= 0; mb_y<s->b_height; mb_y++)
            for(mb_x= 0; mb_x<s->b_width; mb_x++)
                encode_q_branch(s, 0, mb_x, mb_y);
        //s->c = r;
        //memcpy(s->block_state, state, sizeof(s->block_state));
    }

    for(pass=0; pass<25; pass++){
        int change= 0;

        for(mb_y= 0; mb_y<b_height; mb_y++){
            for(mb_x= 0; mb_x<b_width; mb_x++){
                int dia_change, i, j, ref;
                int best_rd= INT_MAX, ref_rd;
                LavfBlockNode backup, ref_b;
                const int index= mb_x + mb_y * b_stride;
                LavfBlockNode *block= &s->block[index];
                LavfBlockNode *tb =                   mb_y            ? &s->block[index-b_stride  ] : NULL;
                LavfBlockNode *lb = mb_x                              ? &s->block[index         -1] : NULL;
                LavfBlockNode *rb = mb_x+1<b_width                    ? &s->block[index         +1] : NULL;
                LavfBlockNode *bb =                   mb_y+1<b_height ? &s->block[index+b_stride  ] : NULL;
                LavfBlockNode *tlb= mb_x           && mb_y            ? &s->block[index-b_stride-1] : NULL;
                LavfBlockNode *trb= mb_x+1<b_width && mb_y            ? &s->block[index-b_stride+1] : NULL;
                LavfBlockNode *blb= mb_x           && mb_y+1<b_height ? &s->block[index+b_stride-1] : NULL;
                LavfBlockNode *brb= mb_x+1<b_width && mb_y+1<b_height ? &s->block[index+b_stride+1] : NULL;
                const int b_w= (MB_SIZE >> s->block_max_depth);
                uint8_t obmc_edged[MB_SIZE * 2][MB_SIZE * 2];

                //av_log(NULL, AV_LOG_ERROR, "%d, %d, pass=%d, key: %d\n", mb_x, mb_y, pass, s->keyframe);
                av_assert0(!(block->type&BLOCK_INTRA)); //TODO remove

                if(pass && (block->type & BLOCK_OPT))
                    continue;
                block->type |= BLOCK_OPT;

                backup= *block;

                if(!s->me_cache_generation)
                    memset(s->me_cache, 0, sizeof(s->me_cache));
                s->me_cache_generation += 1<<22;

                //FIXME precalculate
                {
                    int x, y;
                    for (y = 0; y < b_w * 2; y++)
                        memcpy(obmc_edged[y], lavfsnow_obmc_tab[s->block_max_depth] + y * b_w * 2, b_w * 2);
                    if(mb_x==0)
                        for(y=0; y<b_w*2; y++)
                            memset(obmc_edged[y], obmc_edged[y][0] + obmc_edged[y][b_w-1], b_w);
                    if(mb_x==b_stride-1)
                        for(y=0; y<b_w*2; y++)
                            memset(obmc_edged[y]+b_w, obmc_edged[y][b_w] + obmc_edged[y][b_w*2-1], b_w);
                    if(mb_y==0){
                        for(x=0; x<b_w*2; x++)
                            obmc_edged[0][x] += obmc_edged[b_w-1][x];
                        for(y=1; y<b_w; y++)
                            memcpy(obmc_edged[y], obmc_edged[0], b_w*2);
                    }
                    if(mb_y==b_height-1){
                        for(x=0; x<b_w*2; x++)
                            obmc_edged[b_w*2-1][x] += obmc_edged[b_w][x];
                        for(y=b_w; y<b_w*2-1; y++)
                            memcpy(obmc_edged[y], obmc_edged[b_w*2-1], b_w*2);
                    }
                }

                //skip stuff outside the picture
                if(mb_x==0 || mb_y==0 || mb_x==b_width-1 || mb_y==b_height-1){
                    uint8_t *src= s->  input_avframe->data[0];
                    uint8_t *dst= s->current_avframe->data[0];
                    const int stride= s->current_avframe->linesize[0];
                    const int block_w= MB_SIZE >> s->block_max_depth;
                    const int block_h= MB_SIZE >> s->block_max_depth;
                    const int sx= block_w*mb_x - block_w/2;
                    const int sy= block_h*mb_y - block_h/2;
                    const int w= s->plane[0].width;
                    const int h= s->plane[0].height;
                    int y;

                    for(y=sy; y<0; y++)
                        memcpy(dst + sx + y*stride, src + sx + y*stride, block_w*2);
                    for(y=h; y<sy+block_h*2; y++)
                        memcpy(dst + sx + y*stride, src + sx + y*stride, block_w*2);
                    if(sx<0){
                        for(y=sy; y<sy+block_h*2; y++)
                            memcpy(dst + sx + y*stride, src + sx + y*stride, -sx);
                    }
                    if(sx+block_w*2 > w){
                        for(y=sy; y<sy+block_h*2; y++)
                            memcpy(dst + w + y*stride, src + w + y*stride, sx+block_w*2 - w);
                    }
                }

                // intra(black) = neighbors' contribution to the current block
                for(i=0; i < s->nb_planes; i++)
                    get_dc(s, mb_x, mb_y, i);//color[i]= get_dc(s, mb_x, mb_y, i);


                // get previous score (cannot be cached due to OBMC)
                /*if(pass > 0 && (block->type&BLOCK_INTRA)){
                    int color0[3]= {block->color[0], block->color[1], block->color[2]};
                    check_block(s, mb_x, mb_y, color0, 1, obmc_edged, &best_rd);
                }else*/
                    check_block_inter(s, mb_x, mb_y, block->mx, block->my, obmc_edged, &best_rd);

                ref_b= *block;
                ref_rd= best_rd;
                for(ref=0; ref < s->ref_frames; ref++){
                    int16_t (*mvr)[2]= &s->ref_mvs[ref][index];

                    if(s->ref_scores[ref][index] > s->ref_scores[ref_b.ref][index]*3/2) //FIXME tune threshold
                        continue;
                    block->ref= ref;
                    best_rd= INT_MAX;

                    check_block_inter(s, mb_x, mb_y, mvr[0][0], mvr[0][1], obmc_edged, &best_rd);
                    check_block_inter(s, mb_x, mb_y, 0, 0, obmc_edged, &best_rd);
                    if(tb)
                        check_block_inter(s, mb_x, mb_y, mvr[-b_stride][0], mvr[-b_stride][1], obmc_edged, &best_rd);
                    if(lb)
                        check_block_inter(s, mb_x, mb_y, mvr[-1][0], mvr[-1][1], obmc_edged, &best_rd);
                    if(rb)
                        check_block_inter(s, mb_x, mb_y, mvr[1][0], mvr[1][1], obmc_edged, &best_rd);
                    if(bb)
                        check_block_inter(s, mb_x, mb_y, mvr[b_stride][0], mvr[b_stride][1], obmc_edged, &best_rd);

                    /* fullpel ME */
                    //FIXME avoid subpel interpolation / round to nearest integer
                    do{
                        int newx = block->mx;
                        int newy = block->my;
                        int dia_size = s->iterative_dia_size ? s->iterative_dia_size : FFMAX(s->avctx->dia_size, 1);
                        dia_change=0;
                        for(i=0; i < dia_size; i++){
                            for(j=0; j<i; j++){
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx+4*(i-j), newy+(4*j), obmc_edged, &best_rd);
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx-4*(i-j), newy-(4*j), obmc_edged, &best_rd);
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx-(4*j), newy+4*(i-j), obmc_edged, &best_rd);
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx+(4*j), newy-4*(i-j), obmc_edged, &best_rd);
                            }
                        }
                    }while(dia_change);
                    /* subpel ME */
                    do{
                        static const int square[8][2]= {{+1, 0},{-1, 0},{ 0,+1},{ 0,-1},{+1,+1},{-1,-1},{+1,-1},{-1,+1},};
                        dia_change=0;
                        for(i=0; i<8; i++)
                            dia_change |= check_block_inter(s, mb_x, mb_y, block->mx+square[i][0], block->my+square[i][1], obmc_edged, &best_rd);
                    }while(dia_change);
                    //FIXME or try the standard 2 pass qpel or similar

                    mvr[0][0]= block->mx;
                    mvr[0][1]= block->my;
                    if(ref_rd > best_rd){
                        ref_rd= best_rd;
                        ref_b= *block;
                    }
                }
                best_rd= ref_rd;
                *block= ref_b;
                //CRITICAL 1:
                //check_block(s, mb_x, mb_y, color, 0, obmc_edged, &best_rd);
                //FIXME RD style color selection
                if(!same_block(block, &backup)){
                    //av_assert0(0>0);
                    if(tb ) tb ->type &= ~BLOCK_OPT;
                    if(lb ) lb ->type &= ~BLOCK_OPT;
                    if(rb ) rb ->type &= ~BLOCK_OPT;
                    if(bb ) bb ->type &= ~BLOCK_OPT;
                    if(tlb) tlb->type &= ~BLOCK_OPT;
                    if(trb) trb->type &= ~BLOCK_OPT;
                    if(blb) blb->type &= ~BLOCK_OPT;
                    if(brb) brb->type &= ~BLOCK_OPT;
                    change ++;
                }
            }
        }
        av_log(s->avctx, AV_LOG_DEBUG, "pass:%d changed:%d\n", pass, change);
        if(!change)
            break;
    }

    if(s->block_max_depth == 1){
        int change= 0;
        for(mb_y= 0; mb_y<b_height; mb_y+=2){
            for(mb_x= 0; mb_x<b_width; mb_x+=2){
                int i;
                int best_rd, init_rd;
                const int index= mb_x + mb_y * b_stride;
                LavfBlockNode *b[4];

                b[0]= &s->block[index];
                b[1]= b[0]+1;
                b[2]= b[0]+b_stride;
                b[3]= b[2]+1;
                if(same_block(b[0], b[1]) &&
                   same_block(b[0], b[2]) &&
                   same_block(b[0], b[3]))
                    continue;

                if(!s->me_cache_generation)
                    memset(s->me_cache, 0, sizeof(s->me_cache));
                s->me_cache_generation += 1<<22;

                init_rd= best_rd= get_4block_rd(s, mb_x, mb_y, 0);

                //FIXME more multiref search?
                check_4block_inter(s, mb_x, mb_y,
                                   (b[0]->mx + b[1]->mx + b[2]->mx + b[3]->mx + 2) >> 2,
                                   (b[0]->my + b[1]->my + b[2]->my + b[3]->my + 2) >> 2, 0, &best_rd);

                for(i=0; i<4; i++)
                    if(!(b[i]->type&BLOCK_INTRA))
                        check_4block_inter(s, mb_x, mb_y, b[i]->mx, b[i]->my, b[i]->ref, &best_rd);

                if(init_rd != best_rd)
                    change++;
            }
        }
        av_log(s->avctx, AV_LOG_ERROR, "pass:4mv changed:%d\n", change*4);
    }
}

static void encode_blocks(LavfSnowContext *s){
    int x, y;
    int w= s->b_width;
    int h= s->b_height;

    if(s->motion_est == FF_ME_ITER && !s->keyframe)
        iterative_me(s);

    for(y=0; y<h; y++){
        /*if(s->c.bytestream_end - s->c.bytestream < w*MB_SIZE*MB_SIZE*3){ //FIXME nicer limit
            av_log(s->avctx, AV_LOG_ERROR, "encoded frame too large\n");
            return;
        }*/
        for(x=0; x<w; x++){
            if(s->motion_est == FF_ME_ITER)
                encode_q_branch2(s, 0, x, y);
            else
                encode_q_branch (s, 0, x, y);
        }
    }
}

/*static void snow_reset_contexts(LavfSnowContext *s){ //FIXME better initial contexts
    int plane_index, level, orientation;

    for(plane_index=0; plane_index<3; plane_index++){
        for(level=0; level<MAX_DECOMPOSITIONS; level++){
            for(orientation=level ? 1:0; orientation<4; orientation++){
                memset(s->plane[plane_index].band[level][orientation].state, MID_STATE, sizeof(s->plane[plane_index].band[level][orientation].state));
            }
        }
    }
    memset(s->header_state, MID_STATE, sizeof(s->header_state));
    memset(s->block_state, MID_STATE, sizeof(s->block_state));
}*/

/*static void encode_qlogs(LavfSnowContext *s){
    int plane_index, level, orientation;

    for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
        for(level=0; level<s->spatial_decomposition_count; level++){
            for(orientation=level ? 1:0; orientation<4; orientation++){
                if(orientation==2) continue;
                put_symbol(&s->c, s->header_state, s->plane[plane_index].band[level][orientation].qlog, 1);
            }
        }
    }
}

static void encode_header(LavfSnowContext *s) {
    int plane_index, i;
    uint8_t kstate[32];

    memset(kstate, MID_STATE, sizeof(kstate));

    put_rac_snow(&s->c, kstate, s->keyframe);
    if(s->keyframe || s->always_reset){
        snow_reset_contexts(s);
        s->last_spatial_decomposition_type=
        s->last_qlog=
        s->last_qbias=
        s->last_mv_scale=
        s->last_block_max_depth= 0;
        for(plane_index=0; plane_index<2; plane_index++){
            LavfPlane *p= &s->plane[plane_index];
            p->last_htaps=0;
            memset(p->last_hcoeff, 0, sizeof(p->last_hcoeff));
        }
    }
    if(s->keyframe){
        put_symbol(&s->c, s->header_state, s->version, 0);
        put_rac_snow(&s->c, s->header_state, s->always_reset);
        put_symbol(&s->c, s->header_state, s->temporal_decomposition_type, 0);
        put_symbol(&s->c, s->header_state, s->temporal_decomposition_count, 0);
        put_symbol(&s->c, s->header_state, s->spatial_decomposition_count, 0);
        put_symbol(&s->c, s->header_state, s->colorspace_type, 0);
        if (s->nb_planes > 2) {
            put_symbol(&s->c, s->header_state, s->chroma_h_shift, 0);
            put_symbol(&s->c, s->header_state, s->chroma_v_shift, 0);
        }
        put_rac_snow(&s->c, s->header_state, s->spatial_scalability);
        //put_rac_snow(&s->c, s->header_state, s->rate_scalability);
        put_symbol(&s->c, s->header_state, s->max_ref_frames-1, 0);

        encode_qlogs(s);
    }

    if(!s->keyframe){
        int update_mc=0;
        for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
            LavfPlane *p= &s->plane[plane_index];
            update_mc |= p->last_htaps   != p->htaps;
            update_mc |= !!memcmp(p->last_hcoeff, p->hcoeff, sizeof(p->hcoeff));
        }
        put_rac_snow(&s->c, s->header_state, update_mc);
        if(update_mc){
            for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
                LavfPlane *p= &s->plane[plane_index];
                put_symbol(&s->c, s->header_state, p->htaps/2-1, 0);
                for(i= p->htaps/2; i; i--)
                    put_symbol(&s->c, s->header_state, FFABS(p->hcoeff[i]), 0);
            }
        }
        if(s->last_spatial_decomposition_count != s->spatial_decomposition_count){
            put_rac_snow(&s->c, s->header_state, 1);
            put_symbol(&s->c, s->header_state, s->spatial_decomposition_count, 0);
            encode_qlogs(s);
        }else
            put_rac_snow(&s->c, s->header_state, 0);
    }

    put_symbol(&s->c, s->header_state, s->spatial_decomposition_type - s->last_spatial_decomposition_type, 1);
    put_symbol(&s->c, s->header_state, s->qlog            - s->last_qlog    , 1);
    put_symbol(&s->c, s->header_state, s->mv_scale        - s->last_mv_scale, 1);
    put_symbol(&s->c, s->header_state, s->qbias           - s->last_qbias   , 1);
    put_symbol(&s->c, s->header_state, s->block_max_depth - s->last_block_max_depth, 1);
}

static void update_last_header_values(LavfSnowContext *s){
    int plane_index;

    if(!s->keyframe){
        for(plane_index=0; plane_index<2; plane_index++){
            LavfPlane *p= &s->plane[plane_index];
            p->last_htaps  = p->htaps;
            memcpy(p->last_hcoeff, p->hcoeff, sizeof(p->hcoeff));
        }
    }

    s->last_spatial_decomposition_type  = s->spatial_decomposition_type;
    s->last_qlog                        = s->qlog;
    s->last_qbias                       = s->qbias;
    s->last_mv_scale                    = s->mv_scale;
    s->last_block_max_depth             = s->block_max_depth;
    s->last_spatial_decomposition_count = s->spatial_decomposition_count;
}
*/

static int qscale2qlog(int qscale){
    return lrint(QROOT*log2(qscale / (float)FF_QP2LAMBDA))
           + 61*QROOT/8; ///< 64 > 60
}

/*static void calculate_visual_weight(LavfSnowContext *s, LavfPlane *p){
    int width = p->width;
    int height= p->height;
    int level, orientation, x, y;

    for(level=0; level<s->spatial_decomposition_count; level++){
        for(orientation=level ? 1 : 0; orientation<4; orientation++){
            LavfSubBand *b= &p->band[level][orientation];
            short *ibuf= b->ibuf;
            int64_t error=0;

            memset(s->spatial_idwt_buffer, 0, sizeof(*s->spatial_idwt_buffer)*width*height);
            ibuf[b->width/2 + b->height/2*b->stride]= 256*16;
            lavfsnow_spatial_idwt(s->spatial_idwt_buffer, s->temp_idwt_buffer, width, height, width, s->spatial_decomposition_type, s->spatial_decomposition_count);
        }
    }
}*/

//#define USE_HALFPEL_PLANE 1

/* removed: s->halfpel_plane are not used anywhere */
/*static int halfpel_interpol(LavfSnowContext *s, uint8_t *halfpel[4][4], AVFrame *frame){
    int p,x,y;

    for(p=0; p < s->nb_planes; p++){
        int is_chroma= !!p;
        int w= is_chroma ? AV_CEIL_RSHIFT(s->avctx->width,  s->chroma_h_shift) : s->avctx->width;
        int h= is_chroma ? AV_CEIL_RSHIFT(s->avctx->height, s->chroma_v_shift) : s->avctx->height;
        int ls= frame->linesize[p];
        uint8_t *src= frame->data[p];

        halfpel[1][p] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
        halfpel[2][p] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
        halfpel[3][p] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
        if (!halfpel[1][p] || !halfpel[2][p] || !halfpel[3][p]) {
            av_freep(&halfpel[1][p]);
            av_freep(&halfpel[2][p]);
            av_freep(&halfpel[3][p]);
            return AVERROR(ENOMEM);
        }
        halfpel[1][p] += EDGE_WIDTH * (1 + ls);
        halfpel[2][p] += EDGE_WIDTH * (1 + ls);
        halfpel[3][p] += EDGE_WIDTH * (1 + ls);

        halfpel[0][p]= src;
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= y*ls + x;

                halfpel[1][p][i]= (20*(src[i] + src[i+1]) - 5*(src[i-1] + src[i+2]) + (src[i-2] + src[i+3]) + 16 )>>5;
            }
        }
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= y*ls + x;

                halfpel[2][p][i]= (20*(src[i] + src[i+ls]) - 5*(src[i-ls] + src[i+2*ls]) + (src[i-2*ls] + src[i+3*ls]) + 16 )>>5;
            }
        }
        src= halfpel[1][p];
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= y*ls + x;

                halfpel[3][p][i]= (20*(src[i] + src[i+ls]) - 5*(src[i-ls] + src[i+2*ls]) + (src[i-2*ls] + src[i+3*ls]) + 16 )>>5;
            }
        }

//FIXME border!
    }
    return 0;
}*/

/*static void snow_release_buffer(LavfSnowContext *s)
{
    if(s->last_avframe[s->max_ref_frames-1]->data[0]){
        av_frame_unref(s->last_avframe[s->max_ref_frames-1]);
        for(i=0; i<9; i++)
            if(s->halfpel_plane[s->max_ref_frames-1][1+i/3][i%3]) {
                av_free(s->halfpel_plane[s->max_ref_frames-1][1+i/3][i%3] - EDGE_WIDTH*(1+s->current_avframe->linesize[i%3]));
                s->halfpel_plane[s->max_ref_frames-1][1+i/3][i%3] = NULL;
            }
    }
}*/

static int snow_frame_start(LavfSnowContext *s){
    AVFrame *tmp;
    int i, ret;

    //snow_release_buffer:
    if(s->last_avframe[s->max_ref_frames-1]->data[0])
        av_frame_unref(s->last_avframe[s->max_ref_frames-1]);

    tmp = s->last_avframe[s->max_ref_frames-1];
    for(i=s->max_ref_frames-1; i>0; i--)
        s->last_avframe[i] = s->last_avframe[i-1];
    /*memmove(s->halfpel_plane+1, s->halfpel_plane, (s->max_ref_frames-1)*sizeof(void*)*4*4);
    if(USE_HALFPEL_PLANE && s->current_avframe->data[0]) {
        if((ret = halfpel_interpol(s, s->halfpel_plane[0], s->current_avframe)) < 0)
            return ret;
    }*/
    s->last_avframe[0] = s->current_avframe;
    s->current_avframe = tmp;

    if(s->keyframe){
        s->ref_frames= 0;
    }else{
        int i;
        for(i=0; i<s->max_ref_frames && s->last_avframe[i]->data[0]; i++)
            if(i && s->last_avframe[i-1]->key_frame)
                break;
        s->ref_frames= i;
        if(s->ref_frames==0){
            av_log(s->avctx,AV_LOG_ERROR, "No reference frames\n");
            return AVERROR_INVALIDDATA;
        }
    }
    if ((ret = snow_get_buffer(s, s->current_avframe)) < 0)
        return ret;

    s->current_avframe->key_frame= s->keyframe;

    return 0;
}

static int encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                        const AVFrame *pict, int *got_packet)
{
    return 0;
}

int lavfsnow_encode_frame(AVCodecContext *avctx, const AVFrame *avframe, /*int *got_packet,*/ int *flags)
{
    LavfSnowContext *s = avctx->priv_data;
    MotionEstContext *mest_ctx = &s->mest_ctx;
    //av_assert0(avctx==s->avctx);
    //RangeCoder * const c= &s->c;
    AVFrame *pic;
    const int width= s->avctx->width;
    const int height= s->avctx->height;
    int /*level, orientation,*/ plane_index, i, y, ret;
    //uint8_t rc_header_bak[sizeof(s->header_state)];
    //uint8_t rc_block_bak[sizeof(s->block_state)];
    *flags = 0;
    //AVPacket *pkt = av_packet_alloc();

    /*if ((ret = ff_alloc_packet2(avctx, pkt, s->b_width*s->b_height*MB_SIZE*MB_SIZE*3 + AV_INPUT_BUFFER_MIN_SIZE, 0)) < 0)
        return ret;*/

    //ff_init_range_encoder(c, pkt->data, pkt->size);
    //ff_build_rac_states(c, (1LL<<32)/20, 256-8);

    for(i=0; i < s->nb_planes; i++){
        int hshift= i ? s->chroma_h_shift : 0;
        int vshift= i ? s->chroma_v_shift : 0;
        for(y=0; y<AV_CEIL_RSHIFT(height, vshift); y++)
            memcpy(&s->input_avframe->data[i][y * s->input_avframe->linesize[i]], &avframe->data[i][y * avframe->linesize[i]], AV_CEIL_RSHIFT(width, hshift));
        s->mpvencdsp.draw_edges(s->input_avframe->data[i], s->input_avframe->linesize[i], AV_CEIL_RSHIFT(width, hshift), AV_CEIL_RSHIFT(height, vshift), EDGE_WIDTH >> hshift, EDGE_WIDTH >> vshift, EDGE_TOP | EDGE_BOTTOM);
    }
    pic = s->input_avframe;
    //pic->pict_type = avframe->pict_type;
    pic->quality = avframe->quality;

    //s->mpeg.picture_number= avctx->frame_number;
    s->keyframe= avctx->gop_size==0 || avctx->frame_number % avctx->gop_size == 0;
    /*mest_ctx->mme_struct.pict_type =*/ pic->pict_type = s->keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    if(s->pass1_rc && avctx->frame_number == 0)
        pic->quality = 2*FF_QP2LAMBDA;
    if (pic->quality) {
        s->lambda = pic->quality * 3/2;
    }
    av_assert0( qscale2qlog(pic->quality) >= 0 ); //test
    if (/*qscale2qlog(pic->quality) < 0 || */(!pic->quality && (avctx->flags & AV_CODEC_FLAG_QSCALE))) {
        //av_assert0(0>0);
        s->lambda = 0;
    }//else keep previous frame's qlog until after motion estimation

    if (s->current_avframe->data[0]
#if FF_API_EMU_EDGE
        && !(s->avctx->flags&CODEC_FLAG_EMU_EDGE)
#endif
        ) {
        int w = s->avctx->width;
        int h = s->avctx->height;

        s->mpvencdsp.draw_edges(s->current_avframe->data[0], s->current_avframe->linesize[0], w, h, EDGE_WIDTH, EDGE_WIDTH, EDGE_TOP | EDGE_BOTTOM);
        if (s->current_avframe->data[2]) {
            s->mpvencdsp.draw_edges(s->current_avframe->data[1], s->current_avframe->linesize[1], w>>s->chroma_h_shift, h>>s->chroma_v_shift, EDGE_WIDTH>>s->chroma_h_shift, EDGE_WIDTH>>s->chroma_v_shift, EDGE_TOP | EDGE_BOTTOM);
            s->mpvencdsp.draw_edges(s->current_avframe->data[2], s->current_avframe->linesize[2], w>>s->chroma_h_shift, h>>s->chroma_v_shift, EDGE_WIDTH>>s->chroma_h_shift, EDGE_WIDTH>>s->chroma_v_shift, EDGE_TOP | EDGE_BOTTOM);
        }
    }

    snow_frame_start(s);
    av_frame_unref(avctx->coded_frame);
    ret = av_frame_ref(avctx->coded_frame, s->current_avframe);
    if (ret < 0)
        return ret;

    /*s->mpeg.current_picture_ptr= &s->mpeg.current_picture;
    s->mpeg.current_picture.f = s->current_avframe;
    s->mpeg.current_picture.f->pts = avframe->pts;*/
    if(pic->pict_type == AV_PICTURE_TYPE_P){
        int block_width = (width +15)>>4;
        int block_height= (height+15)>>4;
        //int stride= s->current_avframe->linesize[0];

        av_assert0(s->current_avframe->data[0]);
        av_assert0(s->last_avframe[0]->data[0]);

        //s->mpeg.avctx = s->avctx;
        /*s->mpeg.last_picture.f = s->last_avframe[0];
        s->mpeg.new_picture.f = s->input_avframe;
        s->mpeg.last_picture_ptr = &s->mpeg.last_picture;*/

        //needed in ff_init_me: motion_est, me_method, codec_id, qdsp, hdsp, no_rounding, linesize, uvlinesize, mb_width
        //s->mpeg.linesize = stride; //needed in ff_init_me; 
        //s->mpeg.uvlinesize = s->current_avframe->linesize[1]; //needed in ff_init_me;
        //s->mpeg.width = width;
        //s->mpeg.height = height;

        //s->mpeg.mb_width =               //needed in ff_init_me;
        mest_ctx->mme_struct.mb_width = block_width;
        mest_ctx->mme_struct.mb_height = block_height;

        //s->mpeg.mb_stride = s->mpeg.mb_width+1;
        //s->mpeg.b8_stride = 2*s->mpeg.mb_width+1;
        //s->mpeg.f_code=1;
        //mest_ctx->mme_struct.pict_type = pic->pict_type;
/*#if FF_API_MOTION_EST
        //s->mpeg.me_method = s->avctx->me_method; //needed in ff_init_me;
#endif*/
        //s->mpeg.motion_est = s->motion_est; //needed in ff_init_me;
        //mest_ctx->scene_change_score = 0;
        mest_ctx->dia_size = avctx->dia_size;
        //s->mpeg.quarter_sample = (s->avctx->flags & AV_CODEC_FLAG_QPEL) != 0;
        //s->mpeg.out_format = FMT_H263;
        //s->mpeg.unrestricted_mv = 1;

        //s->mpeg.lambda = s->lambda;
        //s->mpeg.qscale= (s->lambda*139 + FF_LAMBDA_SCALE*64) >> (FF_LAMBDA_SHIFT + 7);
        s->lambda2 = /*s->mpeg.lambda2 =*/ (s->lambda*s->lambda + FF_LAMBDA_SCALE/2) >> FF_LAMBDA_SHIFT;

        mest_ctx->mec_ctx = s->mecc; //move
        //s->mpeg.qdsp = s->qdsp; //move //needed in ff_init_me;
        //s->mpeg.hdsp = s->hdsp; //needed in ff_init_me;
        mest_ctx->avctx = avctx;
        //ff_init_me(mest_ctx, &s->mpeg);
        ff_init_me2(mest_ctx, &s->hdsp, &s->qdsp, s->current_avframe->linesize[0], s->current_avframe->linesize[1], 0, block_width);
        //s->hdsp = s->mpeg.hdsp;
        s->mecc = mest_ctx->mec_ctx;
        /* from ff_init_me: */ //TODO merge,remove
        /*if(stride){
            mest_ctx->stride  = stride;
            mest_ctx->uvstride= s->current_avframe->linesize[1];
        }else{
            mest_ctx->stride  = 16*block_width + 32;
            mest_ctx->uvstride=  8*block_width + 16;
        }*/
    }

    /*if(s->pass1_rc){
        memcpy(rc_header_bak, s->header_state, sizeof(s->header_state));
        memcpy(rc_block_bak, s->block_state, sizeof(s->block_state));
    }*/

    /*s->spatial_decomposition_count= 5;

    while(   !(width >>(s->chroma_h_shift + s->spatial_decomposition_count))
          || !(height>>(s->chroma_v_shift + s->spatial_decomposition_count)))
        s->spatial_decomposition_count--;

    if (s->spatial_decomposition_count <= 0) {
        av_log(avctx, AV_LOG_ERROR, "Resolution too low\n");
        return AVERROR(EINVAL);
    }*/

    mest_ctx->mme_struct.pict_type = pic->pict_type;
    //s->qbias = pic->pict_type == AV_PICTURE_TYPE_P ? 2 : 0;

    lavfsnow_common_init_after_header(s);

    /*if(s->last_spatial_decomposition_count != s->spatial_decomposition_count){
        for(plane_index=0; plane_index < s->nb_planes; plane_index++){
            calculate_visual_weight(s, &s->plane[plane_index]);
        }
    }*/

    //encode_header(s);
    //if(s->keyframe/* || s->always_reset*/) //from encode_header:
        //memset(s->block_state, MID_STATE, sizeof(s->block_state));//snow_reset_contexts(s);

    //s->mpeg.misc_bits = 8*pkt->data;//(s->c.bytestream - s->c.bytestream_start);
    encode_blocks(s);
    //s->mpeg.mv_bits = 8*pkt->data;//(s->c.bytestream - s->c.bytestream_start) - s->mpeg.misc_bits;

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        LavfPlane *p = &s->plane[plane_index];
        int w = p->width;
        int h = p->height;
        int x, y;
        //int bits= put_bits_count(&s->c.pb);

        //ME/MC only
        //if(pic->pict_type == AV_PICTURE_TYPE_I)
            for(y = 0; y < h; y++)
                for(x = 0; x < w; x++)
                    s->current_avframe->data[plane_index][y*s->current_avframe->linesize[plane_index] + x] = avframe->data[plane_index][y*avframe->linesize[plane_index] + x];
        /*}else{
            memset(s->spatial_idwt_buffer, 0, sizeof(short)*w*h);
            predict_plane(s, s->spatial_idwt_buffer, plane_index, 1);
        }*/

    }

    //update_last_header_values(s);

    //snow_release_buffer(s):
    if(s->last_avframe[s->max_ref_frames-1]->data[0])
        av_frame_unref(s->last_avframe[s->max_ref_frames-1]);

    s->current_avframe->coded_picture_number = avctx->frame_number;
    s->current_avframe->pict_type = pic->pict_type;
    s->current_avframe->quality = pic->quality;
    //s->mpeg.frame_bits = 8*(s->c.bytestream - s->c.bytestream_start);
    //s->mpeg.p_tex_bits = s->mpeg.frame_bits - s->mpeg.misc_bits - s->mpeg.mv_bits;
    /*s->mpeg.current_picture.f->display_picture_number =
    s->mpeg.current_picture.f->coded_picture_number   = avctx->frame_number;
    s->mpeg.current_picture.f->quality                = pic->quality;*/
    //s->mpeg.total_bits += 8*(s->c.bytestream - s->c.bytestream_start);
    /*if(s->pass1_rc)
        if (ff_rate_estimate_qscale(&s->mpeg, 0) < 0)
            return -1;*/
    /*if(avctx->flags&AV_CODEC_FLAG_PASS1)
        ff_write_pass1_stats(&s->mpeg);*/
    //s->mpeg.last_pict_type = s->mpeg.pict_type;
    /*avctx->frame_bits = s->mpeg.frame_bits;
    avctx->mv_bits = s->mpeg.mv_bits;
    avctx->misc_bits = s->mpeg.misc_bits;
    avctx->p_tex_bits = s->mpeg.p_tex_bits;*/

    //av_packet_free(&pkt);
    //pkt->size = ff_rac_terminate(c);
    if (s->current_avframe->key_frame)
        *flags |= AV_PKT_FLAG_KEY;
    //*got_packet = 1;

    return 0;
}

static av_cold void lavfsnow_common_end(LavfSnowContext *s)
{
    //int plane_index, level, orientation, i;
    int i;
    MotionEstContext *mest_ctx = &s->mest_ctx;

    //av_freep(&s->spatial_dwt_buffer);
    //av_freep(&s->temp_dwt_buffer);
    //av_freep(&s->spatial_idwt_buffer);
    //av_freep(&s->temp_idwt_buffer);
    //av_freep(&s->run_buffer);

    mest_ctx->temp= NULL;
    av_freep(&mest_ctx->scratchpad);
    av_freep(&mest_ctx->map);
    av_freep(&mest_ctx->score_map);
    av_freep(&s->obmc_scratchpad);

    av_freep(&s->block);
    av_freep(&s->scratchbuf);
    av_freep(&s->emu_edge_buffer);

    for(i=0; i<MAX_REF_FRAMES; i++){
        av_freep(&s->ref_mvs[i]);
        av_freep(&s->ref_scores[i]);
        if(s->last_avframe[i] && s->last_avframe[i]->data[0]) {
            av_assert0(s->last_avframe[i]->data[0] != s->current_avframe->data[0]);
        }
        av_frame_free(&s->last_avframe[i]);
    }

    /*for(plane_index=0; plane_index < MAX_PLANES; plane_index++){
        for(level=MAX_DECOMPOSITIONS-1; level>=0; level--){
            for(orientation=level ? 1 : 0; orientation<4; orientation++){
                LavfSubBand *b= &s->plane[plane_index].band[level][orientation];

                av_freep(&b->x_coeff);
            }
        }
    }*/
    av_frame_free(&s->mconly_avframe);
    av_frame_free(&s->current_avframe);
}

static av_cold int encode_end(AVCodecContext *avctx)
{
    LavfSnowContext *s = avctx->priv_data;

    lavfsnow_common_end(s);
    //ff_rate_control_uninit(&s->mpeg);
    av_frame_free(&s->input_avframe);
    av_freep(&avctx->stats_out);

    return 0;
}
FF_ENABLE_DEPRECATION_WARNINGS

#define OFFSET(x) offsetof(LavfSnowContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    //FF_MPV_COMMON_OPTS
    /*{"motion_est", "motion estimation algorithm", OFFSET(motion_est), AV_OPT_TYPE_INT, {.i64 = FF_ME_EPZS }, FF_ME_ZERO, FF_ME_ITER, VE, "motion_est" },
    { "zero", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_ZERO }, 0, 0, VE, "motion_est" },
    { "epzs", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_EPZS }, 0, 0, VE, "motion_est" },
    { "xone", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_XONE }, 0, 0, VE, "motion_est" },
    { "iter", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_ITER }, 0, 0, VE, "motion_est" },*/
    //{ "memc_only",      "Only do ME/MC (I frames -> ref, P frame -> ME+MC).",   OFFSET(memc_only), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
    //{ "no_bitstream",   "Skip final bitstream writeout.",                    OFFSET(no_bitstream), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
    { "intra_penalty",       "Penalty for intra blocks in block decission",      OFFSET(intra_penalty), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, VE },
    { "iterative_dia_size",  "Dia size for the iterative ME",          OFFSET(iterative_dia_size), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, VE },
    //{ "sc_threshold",   "Scene change threshold",                   OFFSET(sc_threshold), AV_OPT_TYPE_INT, { .i64 = 0 }, INT_MIN, INT_MAX, VE },
    //{ "pred",           "Spatial decomposition type",                                OFFSET(pred), AV_OPT_TYPE_INT, { .i64 = 0 }, DWT_97, DWT_97, VE, "pred" },
    //    { "dwt97", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, INT_MIN, INT_MAX, VE, "pred" },
    { NULL },
};

static const AVClass lavfsnow_class = {
    .class_name = "lavfsnow encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec snow_encoder = {
    .type           = AVMEDIA_TYPE_VIDEO,
    .priv_data_size = sizeof(LavfSnowContext),
    .init           = encode_init,
    .encode2        = encode_frame,
    .close          = encode_end,
    .pix_fmts       = (const enum AVPixelFormat[]){
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NONE
    },
    .priv_class     = &lavfsnow_class,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
};
