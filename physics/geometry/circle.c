/**
 * @file circle.c
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
#include <stdlib.h>
#include "opus/physics/physics.h"
#include "opus/physics/physics_private.h"


opus_circle *opus_shape_circle_init(opus_circle *circle, opus_vec2 center, opus_real radius)
{
	if (circle) {
		circle->_.type         = OPUS_SHAPE_CIRCLE;
		circle->_.get_support  = (void *)opus_shape_circle_get_support; /*FIXME*/
		circle->_.get_inertia  = (void *)opus_shape_circle_get_inertia;
		circle->_.update_bound = (void *)opus_shape_circle_update_bound;
		circle->_.get_area     = (void *)opus_shape_circle_get_area;
		circle->_.set_center   = (void *)opus_shape_circle_set_center;

		circle->center = center;
		circle->radius = radius;

		opus_shape_circle_update_bound((void *) circle, 0, opus_vec2_(0, 0));
	}
	return circle;
}

opus_circle *opus_shape_circle_create(opus_vec2 center, opus_real radius)
{
	return opus_shape_circle_init(malloc(sizeof(opus_circle)), center, radius);
}

void opus_shape_circle_done(opus_circle *circle)
{
}

void opus_shape_circle_destroy(opus_circle *circle)
{
	free(circle);
}

opus_vec2 opus_shape_circle_get_support(opus_shape *shape, opus_mat2d transform, opus_vec2 dir, uint64_t *index)
{
	opus_circle *circle = (void *) shape;
	opus_vec2    center;

	*index = 0;
	center = opus_mat2d_pre_mul_vec(transform, circle->center);
	opus_vec2_set_length(&dir, circle->radius);

	return opus_vec2_add(dir, center);
}

opus_real opus_shape_circle_get_inertia(opus_shape *shape, opus_real mass)
{
	opus_circle *circle = (void *) shape;
	return mass * circle->radius * circle->radius / 2;
}

void opus_shape_circle_update_bound(opus_shape *shape, opus_real rotation, opus_vec2 position)
{
	opus_circle *circle = (void *) shape;
	shape->bound.min.x  = -circle->radius + position.x + circle->center.x;
	shape->bound.min.y  = -circle->radius + position.y + circle->center.y;
	shape->bound.max.x  = circle->radius + position.x + circle->center.x;
	shape->bound.max.y  = circle->radius + position.y + circle->center.y;
}

opus_real opus_shape_circle_get_area(opus_shape *shape)
{
	opus_circle *circle = (void *) shape;
	return OPUS_PI * circle->radius * circle->radius;
}

void opus_shape_circle_set_center(opus_shape *shape, opus_vec2 center)
{
	opus_circle *circle = (void *) shape;
	circle->center      = center;
}
