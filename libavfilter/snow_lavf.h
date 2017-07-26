/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (C) 2006 Robert Edele <yartrebo@earthlink.net>
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

#ifndef AVCODEC_SNOW_H
#define AVCODEC_SNOW_H

#include "libavutil/motion_vector.h"

#include "libavcodec/hpeldsp.h"
#include "libmotion/me_cmp.h"
#include "libavcodec/qpeldsp.h"
#include "snow_dwt_lavf.h"

#include "libavcodec/rangecoder.h"
#include "libavcodec/mathops.h"

#define FF_MPV_OFFSET(x) (offsetof(MpegEncContext, x) + offsetof(SnowContext, mpeg))
#include "libavcodec/mpegvideo.h"
#include "libavcodec/h264qpel.h"

#define MID_STATE 128

#define MAX_PLANES 4
#define QSHIFT 5
#define QROOT (1<<QSHIFT)
#define LOSSLESS_QLOG -128
#define FRAC_BITS 4
#define MAX_REF_FRAMES 8

#define LOG2_OBMC_MAX 8
#define OBMC_MAX (1<<(LOG2_OBMC_MAX))
typedef struct BlockNode{
    int16_t mx;                 ///< Motion vector component X, see mv_scale
    int16_t my;                 ///< Motion vector component Y, see mv_scale
    uint8_t ref;                ///< Reference frame index
    uint8_t color[3];           ///< Color for intra
    uint8_t type;               ///< Bitfield of BLOCK_*
//#define TYPE_SPLIT    1
#define BLOCK_INTRA   1         ///< Intra block, inter otherwise
#define BLOCK_OPT     2         ///< Block needs no checks in this round of iterative motion estiation
//#define TYPE_NOCOLOR  4
    uint8_t level; //FIXME merge into type?
}BlockNode;

static const BlockNode null_block= { //FIXME add border maybe
    .color= {128,128,128},
    .mx= 0,
    .my= 0,
    .ref= 0,
    .type= 0,
    .level= 0,
};

#define LOG2_MB_SIZE 4
#define MB_SIZE (1<<LOG2_MB_SIZE)
#define ENCODER_EXTRA_BITS 4
#define HTAPS_MAX 8

typedef struct x_and_coeff{
    int16_t x;
    uint16_t coeff;
} x_and_coeff;

typedef struct SubBand{
    int level;
    int stride;
    int width;
    int height;
    int qlog;        ///< log(qscale)/log[2^(1/6)]
    DWTELEM *buf;
    IDWTELEM *ibuf;
    int buf_x_offset;
    int buf_y_offset;
    int stride_line; ///< Stride measured in lines, not pixels.
    x_and_coeff * x_coeff;
    struct SubBand *parent;
    uint8_t state[/*7*2*/ 7 + 512][32];
}SubBand;

typedef struct Plane{
    int width;
    int height;
    SubBand band[MAX_DECOMPOSITIONS][4];

    int htaps;
    int8_t hcoeff[HTAPS_MAX/2];
    int diag_mc;
    int fast_mc;

    int last_htaps;
    int8_t last_hcoeff[HTAPS_MAX/2];
    int last_diag_mc;
}Plane;

typedef struct SnowContext{
    AVClass *class;
    AVCodecContext *avctx;
    RangeCoder c;
    MECmpContext mecc;
    HpelDSPContext hdsp;
    QpelDSPContext qdsp;
    VideoDSPContext vdsp;
    H264QpelContext h264qpel;
    MpegvideoEncDSPContext mpvencdsp;
    //SnowDWTContext dwt;
    AVFrame *input_picture;              ///< new_picture with the internal linesizes
    AVFrame *current_picture;
    AVFrame *last_picture[MAX_REF_FRAMES];
    uint8_t *halfpel_plane[MAX_REF_FRAMES][4][4];
    AVFrame *mconly_picture;
//     uint8_t q_context[16];
    uint8_t header_state[32];
    uint8_t block_state[128 + 32*128];
    int keyframe;
    int always_reset;
    int version;
    int spatial_decomposition_type;
    int last_spatial_decomposition_type;
    int temporal_decomposition_type;
    int spatial_decomposition_count;
    int last_spatial_decomposition_count;
    int temporal_decomposition_count;
    int max_ref_frames;
    int ref_frames;
    int16_t (*ref_mvs[MAX_REF_FRAMES])[2];
    uint32_t *ref_scores[MAX_REF_FRAMES];
    DWTELEM *spatial_dwt_buffer;
    DWTELEM *temp_dwt_buffer;
    IDWTELEM *spatial_idwt_buffer;
    IDWTELEM *temp_idwt_buffer;
    int *run_buffer;
    int colorspace_type;
    int chroma_h_shift;
    int chroma_v_shift;
    int spatial_scalability;
    int qlog;
    int last_qlog;
    int lambda;
    int lambda2;
    int pass1_rc;
    int mv_scale;
    int last_mv_scale;
    int qbias;
    int last_qbias;
#define QBIAS_SHIFT 3
    int b_width;
    int b_height;
    int block_max_depth;
    int last_block_max_depth;
    int nb_planes;
    Plane plane[MAX_PLANES];
    BlockNode *block;
#define ME_CACHE_SIZE 1024
    unsigned me_cache[ME_CACHE_SIZE];
    unsigned me_cache_generation;
    slice_buffer sb;
    int memc_only;
    int no_bitstream;
    int intra_penalty;
    int motion_est;
    int iterative_dia_size;
    int sc_threshold;

    MpegEncContext mpeg; // needed for motion estimation, should not be used for anything else, the idea is to eventually make the motion estimation independent of MpegEncContext, so this will be removed then (FIXME/XXX)

    uint8_t *scratchbuf;
    uint8_t *emu_edge_buffer;

    AVMotionVector *avmv;
    int avmv_index;
    uint64_t encoding_error[AV_NUM_DATA_POINTERS];

    int pred;
}SnowContext;

/* common code */

int ff_snow_common_init(AVCodecContext *avctx);
int ff_snow_common_init_after_header(AVCodecContext *avctx);
void ff_snow_common_end(SnowContext *s);
void ff_snow_release_buffer(AVCodecContext *avctx);
void ff_snow_reset_contexts(SnowContext *s);
int ff_snow_alloc_blocks(SnowContext *s);
int ff_snow_frame_start(SnowContext *s);
void ff_snow_pred_block(SnowContext *s, uint8_t *dst, uint8_t *tmp, ptrdiff_t stride,
                     int sx, int sy, int b_w, int b_h, const BlockNode *block,
                     int plane_index, int w, int h);
int ff_snow_get_buffer(SnowContext *s, AVFrame *frame);
int ff_get_mvs_snow(AVCodecContext *avctx, int16_t (*mvs)[2], int8_t *refs, int w, int h);
/* common inline functions */
//XXX doublecheck all of them should stay inlined

const int8_t ff_quant3bA[256];
const uint8_t * const ff_obmc_tab[4];

/* runtime generated tables */
uint8_t ff_qexp[QROOT];
int ff_scale_mv_ref[MAX_REF_FRAMES][MAX_REF_FRAMES];

#endif /* AVCODEC_SNOW_H */
