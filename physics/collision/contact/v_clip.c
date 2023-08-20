/**
 * @file v_clip.c
 *     Author:    _              _
 *               / \   _ __ ___ (_)  __ _ ___
 *              / _ \ | '_ ` _ \| |/ _` / __|
 *             / ___ \| | | | | | | (_| \__ \
 *            /_/   \_\_| |_| |_|_|\__,_|___/  2023/2/28
 *
 * @example
 *
 * @development_log
 *
 */
#include "opus/vg/vg_utils.h"
#include "opus/vg/vg_color.h"
#include <stdio.h>

#include <string.h>
#include "opus/physics/physics.h"
#include "opus/physics/physics_private.h"
#include "opus/math/geometry.h"

#define OPUS_VCLIP_NO_EDGE ((uint64_t) -1)
#define S_ (0)
#define E_ (1)
#define PREV_(_i, _n) ((_i) == 0 ? (_n) - 1: (_i) - 1)
#define NEXT_(_i, _n) (((_i) + 1) % (_n))

static opus_vec2 VCLIP_intersect_(opus_vec2 a1, opus_vec2 a2, opus_vec2 b1, opus_vec2 b2)
{
	opus_vec2 p;
	opus_real t1, t2;
	/* if we call this function, we already know it will intersect */
	geo_ll_intersect(a1.x, a1.y, a2.x, a2.y, b1.x, b1.y, b2.x, b2.y, &p.x, &p.y, &t1, &t2);
	return p;
}

/**
 * @brief
 * @param inc_s
 * @param inc_e
 * @param ref_s
 * @param ref_e
 * @param A
 * @param B
 * @param ta
 * @param tb
 * @param normal
 * @return is reference edge on A
 */
static int VCLIP_polygon_polygon_find_clip_edge_(opus_vec2 *inc_s, opus_vec2 *inc_e,
                                                 opus_vec2 *ref_s, opus_vec2 *ref_e,
                                                 uint64_t      incident_indices[2],
                                                 opus_polygon *A, opus_polygon *B,
                                                 opus_mat2d ta, opus_mat2d tb,
                                                 opus_vec2 normal)
{
	uint64_t  index_sa, index_sb; /* index of support point */
	opus_vec2 sa, sb;             /* support point of A and B */

	uint64_t  idx1, idx2;
	opus_vec2 p1, p2;
	opus_real r1, r2;

	/* get support point */
	sa = A->_.get_support((void *) A, ta, normal, &index_sa);
	sb = B->_.get_support((void *) B, tb, opus_vec2_neg(normal), &index_sb);
	sa = opus_mat2d_pre_mul_vec(ta, sa);
	sb = opus_mat2d_pre_mul_vec(tb, sb);

	idx1 = (B->n + index_sb - 1) % B->n;
	idx2 = (index_sb + 1) % B->n;
	p1   = B->vertices[idx1]; /* prev */
	p2   = B->vertices[idx2]; /* next */
	p1   = opus_mat2d_pre_mul_vec(tb, p1);
	p2   = opus_mat2d_pre_mul_vec(tb, p2);
	r1   = opus_vec2_dot(opus_vec2_to(p1, sb), normal);
	r2   = opus_vec2_dot(opus_vec2_to(p2, sb), normal);
	if (opus_abs(r1) < opus_abs(r2)) {
		*inc_s = p1;
		*inc_e = sb;

		incident_indices[S_] = idx1;
		incident_indices[E_] = index_sb;
	} else {
		*inc_s = sb;
		*inc_e = p2;

		incident_indices[S_] = index_sb;
		incident_indices[E_] = idx2;
	}

	p1 = A->vertices[PREV_(index_sa, A->n)]; /* prev */
	p2 = A->vertices[NEXT_(index_sa, A->n)]; /* next */
	p1 = opus_mat2d_pre_mul_vec(ta, p1);
	p2 = opus_mat2d_pre_mul_vec(ta, p2);
	r1 = opus_vec2_dot(opus_vec2_to(p1, sa), normal);
	r2 = opus_vec2_dot(opus_vec2_to(p2, sa), normal);
	if (opus_abs(r1) < opus_abs(r2)) {
		*ref_s = p1;
		*ref_e = sa;
	} else {
		*ref_s = sa;
		*ref_e = p2;
	}

	/* compare which edge is more perpendicular to normal */
	r1 = opus_abs(opus_vec2_dot(opus_vec2_to(*ref_s, *ref_e), normal));
	r2 = opus_abs(opus_vec2_dot(opus_vec2_to(*inc_s, *inc_e), normal));

	/* swap, reference edge should be more perpendicular to normal */
	if (r1 > r2) {
		opus_vec2_swap(inc_s, ref_s);
		opus_vec2_swap(inc_e, ref_e);
		return 0;
	}

	return 1;
}

static opus_clip_result VCLIP_polygon_polygon_(opus_overlap_result overlap)
{
	opus_clip_result result = {0};

	opus_polygon *A = (void *) overlap.A;
	opus_polygon *B = (void *) overlap.B;

	uint64_t  incident_indices[2];
	uint64_t  id[2][2];
	opus_vec2 inc_s, inc_e; /* incident edge */
	opus_vec2 ref_s, ref_e; /* reference edge */
	opus_vec2 ref_n;        /* normal of reference edge */

	int       is_s_outside, is_e_outside;
	opus_vec2 p1, p2;
	int       vs, ve;
	int       is_ref_on_A;

	/* get reference edge and incident edge */
	is_ref_on_A = VCLIP_polygon_polygon_find_clip_edge_(&inc_s, &inc_e, &ref_s, &ref_e,
	                                                    incident_indices,
	                                                    A, B, overlap.transform_a, overlap.transform_b,
	                                                    overlap.normal);
	OPUS_ASSERT(is_ref_on_A);

	id[S_][0] = id[S_][1] = incident_indices[S_];
	id[E_][0] = id[E_][1] = incident_indices[E_];

	/* notice that both clip edges have a valid direction (CCW order) */
	ref_n = opus_vec2_norm(opus_vec2_to(ref_s, ref_e));

	/* 1st filter field, clip left and right */
	vs = opus_voronoi_region_(ref_s, ref_e, inc_s);
	ve = opus_voronoi_region_(ref_s, ref_e, inc_e);
	if (vs == -1) {
		inc_s     = VCLIP_intersect_(ref_s, opus_vec2_add(ref_s, overlap.normal), inc_s, inc_e);
		id[S_][1] = NEXT_(id[S_][1], B->n);
	}
	if (ve == -1) {
		inc_e     = VCLIP_intersect_(ref_s, opus_vec2_add(ref_s, overlap.normal), inc_s, inc_e);
		id[E_][0] = PREV_(id[E_][0], B->n);
	}

	vs = opus_voronoi_region_(ref_s, ref_e, inc_s);
	ve = opus_voronoi_region_(ref_s, ref_e, inc_e);
	if (vs == 1) {
		inc_s     = VCLIP_intersect_(ref_e, opus_vec2_add(ref_e, overlap.normal), inc_s, inc_e);
		id[S_][1] = NEXT_(id[S_][1], B->n);
	}
	if (ve == 1) {
		inc_e     = VCLIP_intersect_(ref_e, opus_vec2_add(ref_e, overlap.normal), inc_s, inc_e);
		id[E_][0] = PREV_(id[E_][0], B->n);
	}

	/* 2nd filter field, clip inner and outer */
	p1           = opus_vec2_add(ref_s, opus_vec2_skewT(ref_n));
	is_s_outside = !opus_is_point_on_same_side(ref_s, ref_e, p1, inc_s);
	is_e_outside = !opus_is_point_on_same_side(ref_s, ref_e, p1, inc_e);

	if (is_s_outside && !is_e_outside) {
		inc_s = VCLIP_intersect_(ref_s, ref_e, inc_s, inc_e);
		if (id[S_][0] != id[S_][1]) id[S_][1] = NEXT_(id[S_][1], B->n);
	}
	if (!is_s_outside && is_e_outside) {
		inc_e = VCLIP_intersect_(ref_s, ref_e, inc_s, inc_e);
		if (id[E_][0] != id[E_][1]) id[E_][0] = PREV_(id[S_][0], B->n);
	}
	//	OPUS_ASSERT(!(is_s_outside && is_e_outside));

	/* last we project incident edge points on reference edge */
	p1 = opus_nearest_point_on_line(ref_s, ref_e, inc_s);
	p2 = opus_nearest_point_on_line(ref_s, ref_e, inc_e);

	/* we want [0] is the point on A, and [1] is the point on B */
	/* just keep it in order */
	result.n_support      = 2;
	result.supports[0][0] = p1;
	result.supports[0][1] = inc_s;
	result.supports[1][0] = p2;
	result.supports[1][1] = inc_e;

	result.A = (void *) A;
	result.B = (void *) B;
	memcpy(result.contact_id, id, sizeof(id));

	return result;
}

opus_clip_result VCLIP_polygon_circle_(opus_overlap_result overlap)
{
	opus_clip_result result = {0};

	opus_polygon *A;
	opus_circle  *B;
	opus_mat2d    ta, tb;

	opus_vec2 support_a, center, support_b, p1, p2;
	opus_vec2 ref_s, ref_e;
	opus_real r1, r2;
	uint64_t  index_support;

	OPUS_ASSERT(overlap.A->type == OPUS_SHAPE_POLYGON);
	OPUS_ASSERT(overlap.B->type == OPUS_SHAPE_CIRCLE);

	A = (void *) overlap.A;
	opus_mat2d_copy(ta, overlap.transform_a);
	B = (void *) overlap.B;
	opus_mat2d_copy(tb, overlap.transform_b);

	support_a = A->_.get_support((void *) A, ta, overlap.normal, &index_support);
	support_a = opus_mat2d_pre_mul_vec(ta, support_a);

	/* get support of the circle */
	center    = opus_mat2d_pre_mul_vec(tb, opus_vec2_(0, 0)); /* real center of the circle */
	support_b = opus_vec2_neg(overlap.normal);
	support_b = opus_vec2_scale(support_b, B->radius);
	support_b = opus_vec2_add(support_b, center);

	/* get reference edge of the polygon */
	p1 = A->vertices[(A->n + index_support - 1) % A->n]; /* prev */
	p2 = A->vertices[(index_support + 1) % A->n];        /* next */
	p1 = opus_mat2d_pre_mul_vec(ta, p1);
	p2 = opus_mat2d_pre_mul_vec(ta, p2);
	r1 = opus_vec2_dot(opus_vec2_to(p1, support_a), overlap.normal);
	r2 = opus_vec2_dot(opus_vec2_to(p2, support_a), overlap.normal);
	if (opus_abs(r1) < opus_abs(r2)) {
		ref_s = p1;
		ref_e = support_a;
	} else {
		ref_s = support_a;
		ref_e = p2;
	}

	extern plutovg_t *pl;
	opus_pl_line_vec(pl, ref_s, ref_e);
	plutovg_set_source_rgb(pl, COLOR_RED);
	plutovg_set_line_width(pl, 2);
	plutovg_stroke(pl);

	result.A              = (void *) A;
	result.B              = (void *) B;
	result.n_support      = 1;
	result.supports[0][0] = opus_nearest_point_on_line(ref_s, ref_e, support_b);
	result.supports[0][1] = support_b;

	return result;
}

/**
 * @brief Sutherland Hodgman polygon clipping method
 * @param overlap
 * @return
 */
opus_clip_result opus_VCLIP(opus_overlap_result overlap)
{
	opus_clip_result r0 = {0};
	opus_shape      *A = overlap.A, *B = overlap.B;

	if (!overlap.is_overlap) return r0;

	if (A->type == OPUS_SHAPE_POLYGON && B->type == A->type)
		return VCLIP_polygon_polygon_(overlap);
	if (A->type == OPUS_SHAPE_POLYGON && B->type == OPUS_SHAPE_CIRCLE)
		return VCLIP_polygon_circle_(overlap);
	if (A->type == OPUS_SHAPE_CIRCLE && B->type == OPUS_SHAPE_POLYGON)
		return VCLIP_polygon_circle_(overlap);
	return r0;
}
