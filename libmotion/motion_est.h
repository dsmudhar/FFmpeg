/*
 * Motion estimation
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

#ifndef AVCODEC_MOTION_EST_H
#define AVCODEC_MOTION_EST_H

#include <stdint.h>

#include "libavcodec/avcodec.h"
#include "libavcodec/hpeldsp.h"
#include "libavcodec/qpeldsp.h"
#include "me_cmp.h"

struct MpegEncContext;

#if ARCH_IA64 // Limit static arrays to avoid gcc failing "short data segment overflowed"
#define MAX_MV 1024
#else
#define MAX_MV 4096
#endif
#define MAX_DMV (2*MAX_MV)
#define ME_MAP_SIZE 64

#define FF_ME_ZERO 0
#define FF_ME_EPZS 1
#define FF_ME_XONE 2

/**
 * members of MpegEncContext needed in ME.
 */
typedef struct MpegMEStruct
{
    /* need in ff_epzs_motion_search() */
    int mb_stride;             ///< mb_width+1 used for some arrays to allow simple addressing of left & top MBs without sig11
    int mb_x, mb_y;
    int end_mb_y;              ///< end   mb_y of this thread (so current thread should process start_mb_y <= row < end_mb_y)
    int pict_type;             ///< AV_PICTURE_TYPE_I, AV_PICTURE_TYPE_P, AV_PICTURE_TYPE_B, ...
    int first_slice_line;      ///< used in MPEG-4 too to handle resync markers
    int mb_width, mb_height;   ///< number of MBs horizontally & vertically
    int mpv_flags;             ///< flags set by private options

    /* used in cmp_direct_inline() */
    uint16_t pp_time;               ///< time distance between the last 2 p,s,i frames
    uint16_t pb_time;               ///< time distance between the last b and p,s,i frame
/*
#define MV_TYPE_16X16       0   ///< 1 vector for the whole mb
#define MV_TYPE_8X8         1   ///< 4 vectors (H.263, MPEG-4 4MV)
#define MV_TYPE_16X8        2   ///< 2 vectors, one per 16x8 block
#define MV_TYPE_FIELD       3   ///< 2 vectors, one per field
#define MV_TYPE_DMV         4   ///< 2 vectors, special mpeg2 Dual Prime Vectors*/
    int mv_type;
    int width, height;          ///< picture size. must be a multiple of 16
    /*int mb_x, mb_y;*/

    int direct_inline; ///< used to confirm cmp_direct_inline vars has been set.
} MpegMEStruct;
/**
 * Motion estimation context.
 */
typedef struct MotionEstContext {
    AVCodecContext *avctx;
    int skip;                       ///< set if ME is skipped for the current MB
    int co_located_mv[4][2];        ///< mv from last P-frame for direct mode ME
    int direct_basis_mv[4][2];
    uint8_t *scratchpad;            /**< data area for the ME algo, so that
                                     * the ME does not need to malloc/free. */
    uint8_t *best_mb;
    uint8_t *temp_mb[2];
    uint8_t *temp;
    int best_bits;
    uint32_t *map;                  ///< map to avoid duplicate evaluations
    uint32_t *score_map;            ///< map to store the scores
    unsigned map_generation;
    int pre_penalty_factor;
    int penalty_factor;             /**< an estimate of the bits required to
                                     * code a given mv value, e.g. (1,0) takes
                                     * more bits than (0,0). We have to
                                     * estimate whether any reduction in
                                     * residual is worth the extra bits. */
    int sub_penalty_factor;
    int mb_penalty_factor;
    int flags;
    int sub_flags;
    int mb_flags;
    int pre_pass;                   ///< = 1 for the pre pass
    int dia_size;
    int xmin;
    int xmax;
    int ymin;
    int ymax;
    int pred_x;
    int pred_y;
    uint8_t *src[4][4];
    uint8_t *ref[4][4];
    int stride;
    int uvstride;
    /* temp variables for picture complexity calculation */
    int64_t mc_mb_var_sum_temp;
    int64_t mb_var_sum_temp;
    int scene_change_score;

    op_pixels_func(*hpel_put)[4];
    op_pixels_func(*hpel_avg)[4];
    qpel_mc_func(*qpel_put)[16];
    qpel_mc_func(*qpel_avg)[16];
    uint8_t (*mv_penalty)[MAX_DMV * 2 + 1]; ///< bit amount needed to encode a MV
    uint8_t *current_mv_penalty;
    int (*sub_motion_search)(struct MotionEstContext *c,
                             int *mx_ptr, int *my_ptr, int dmin,
                             int src_index, int ref_index,
                             int size, int h);

    /* AIATMIIOMPV */
    MECmpContext mec_ctx;
    struct MpegEncContext *mpeg_ctx;
    MpegMEStruct mme_struct; ///< plan is - to copy required things from MpegEncContext, test everything, remove this.MpegEncContext, merge context and this struct;
} MotionEstContext;

static inline int ff_h263_round_chroma(int x)
{
    //FIXME static or not?
    static const uint8_t h263_chroma_roundtab[16] = {
    //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
        0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1,
    };
    return h263_chroma_roundtab[x & 0xf] + (x >> 3);
}

int ff_init_me(MotionEstContext *mest_ctx, struct MpegEncContext *s);
int ff_init_me2(MotionEstContext *c, HpelDSPContext *hdsp, QpelDSPContext *qdsp, int linesize, int uvlinesize, int no_rounding, int mb_size);

int avpriv_motion_get_penalty_factor(int lambda, int lambda2, int type);

void ff_estimate_p_frame_motion(struct MpegEncContext *s, int mb_x, int mb_y);
void ff_estimate_b_frame_motion(struct MpegEncContext *s, int mb_x, int mb_y);

int ff_pre_estimate_p_frame_motion(struct MpegEncContext *s,
                                   int mb_x, int mb_y);

/* temporary function used to copy required stuff to mme, to test that it doesn't break things
   temporary, until we find better location for members */
void ff_epzs_copy_stuff(MotionEstContext *c);
int ff_epzs_motion_search(MotionEstContext *c, int *mx_ptr, int *my_ptr,
                          int P[10][2], int src_index, int ref_index,
                          int16_t (*last_mv)[2], int ref_mv_scale, int size,
                          int h);

int ff_get_mb_score(MotionEstContext *c, int mx, int my, int src_index,
                    int ref_index, int size, int h, int add_rate);

int ff_get_best_fcode(struct MpegEncContext *s,
                      int16_t (*mv_table)[2], int type);

void ff_fix_long_p_mvs(struct MpegEncContext *s);
void ff_fix_long_mvs(struct MpegEncContext *s, uint8_t *field_select_table,
                     int field_select, int16_t (*mv_table)[2], int f_code,
                     int type, int truncate);

#endif /* AVCODEC_MOTION_EST_H */
