/**
 * @file polygon.c
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

#include <stdint.h>
#include <stdlib.h>
#include "opus/physics/physics.h"
#include "opus/physics/physics_private.h"
#include "opus/utils/utils.h"
#include "opus/math/polygon/polygon.h"


opus_polygon *opus_shape_polygon_init(opus_polygon *polygon, opus_vec2 *vertices, size_t n, opus_vec2 center)
{
	opus_vec2 t;

	if (polygon) {
		polygon->_.type   = OPUS_SHAPE_POLYGON;
		polygon->center   = center;
		polygon->n        = n;
		polygon->vertices = OPUS_CALLOC(n, sizeof(opus_vec2));
		memcpy(polygon->vertices, vertices, sizeof(opus_vec2) * n);

		polygon->_.get_support  = (void *)opus_shape_polygon_get_support; /*FIXME*/
		polygon->_.get_inertia  = (void *)opus_shape_polygon_get_inertia;
		polygon->_.update_bound = (void *)opus_shape_polygon_update_bound;
		polygon->_.get_area     = (void *)opus_shape_polygon_get_area;
		polygon->_.set_center   = (void *)opus_shape_polygon_set_center;

		/* in case the center of the polygon is not the same as that given by user */
		opus_make_ccw(polygon->vertices, polygon->n);
		opus_center(polygon->vertices, polygon->n, &t);
		opus_translate(polygon->vertices, polygon->n, t, opus_vec2_(-1, -1));

		opus_shape_polygon_update_bound((void *) polygon, 0, opus_vec2_(0, 0));
	}
	return polygon;
}

opus_polygon *opus_shape_polygon_create(opus_vec2 *vertices, size_t n, opus_vec2 center)
{
	return opus_shape_polygon_init(OPUS_CALLOC(1, sizeof(opus_polygon)), vertices, n, center);
}

void opus_shape_polygon_done(opus_polygon *polygon)
{
	if (polygon->vertices) free(polygon->vertices);
	polygon->vertices = NULL;
}

void opus_shape_polygon_destroy(opus_polygon *polygon)
{
	opus_shape_polygon_done(polygon);
	free(polygon);
}

/**
 * @brief Return the furthest vertex in the direction
 * @param vertices
 * @param n
 * @param dir
 * @return
 */
opus_vec2 opus_shape_polygon_get_support(opus_shape *shape, opus_mat2d transform, opus_vec2 dir, uint64_t *index)
{
	opus_polygon *polygon = (void *) shape;

	size_t     n        = polygon->n;
	opus_vec2 *vertices = polygon->vertices, p;

	size_t    i, max_i = 0;
	opus_real max = -OPUS_REAL_MAX, dot;

	for (i = 0; i < n; i++) {
		p   = opus_mat2d_pre_mul_vec(transform, opus_vec2_add(vertices[i], polygon->center));
		dot = opus_vec2_dot(p, dir);
		if (dot > max) {
			max   = dot;
			max_i = i;
		}
	}

	*index = max_i;

	return vertices[max_i];
}

opus_real opus_shape_polygon_get_inertia(opus_shape *shape, opus_real mass)
{
	opus_polygon *polygon = (void *) shape;
	return opus_inertia(polygon->vertices, polygon->n, mass);
}

void opus_shape_polygon_update_bound(opus_shape *shape, opus_real rotation, opus_vec2 position)
{
	opus_polygon *polygon = (void *) shape;

	opus_mat2d transform;
	size_t     i;
	opus_real  x, y;
	opus_real  min_x = OPUS_REAL_MAX, min_y = OPUS_REAL_MAX, max_x = -OPUS_REAL_MAX, max_y = -OPUS_REAL_MAX;

	opus_mat2d_rotate_about(transform, (float) rotation, position);
	for (i = 0; i < polygon->n; i++) {
		x = polygon->vertices[i].x + polygon->center.x;
		y = polygon->vertices[i].y + polygon->center.y;

		opus_mat2d_pre_mul_xy(&x, &y, transform, x, y);

		if (x < min_x) min_x = x;
		if (x > max_x) max_x = x;

		if (y < min_y) min_y = y;
		if (y > max_y) max_y = y;
	}

	shape->bound.min.x = min_x;
	shape->bound.min.y = min_y;
	shape->bound.max.x = max_x;
	shape->bound.max.y = max_y;
}

opus_real opus_shape_polygon_get_area(opus_shape *shape)
{
	opus_polygon *polygon = (void *) shape;
	return opus_area(polygon->vertices, polygon->n, 0);
}

void opus_shape_polygon_set_center(opus_shape *shape, opus_vec2 center)
{
	opus_polygon *polygon;
	polygon         = (void *) shape;
	polygon->center = center;
}
