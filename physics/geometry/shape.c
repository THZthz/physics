/**
 * @file shape.c
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

#include "opus/physics/physics.h"
#include "opus/utils/utils.h"

#define MAX_(a, b) ((a) > (b) ? (a) : (b))

unsigned int OPUS_physics_shape_max_size = sizeof(opus_shape);


void opus_shape_get_max_struct_size_(void)
{
	OPUS_physics_shape_max_size = MAX_(OPUS_physics_shape_max_size, sizeof(opus_polygon));
	OPUS_physics_shape_max_size = MAX_(OPUS_physics_shape_max_size, sizeof(opus_circle));
}

void opus_shape_destroy(opus_shape *shape)
{
	switch (shape->type) {
		case OPUS_SHAPE_POLYGON:
			opus_shape_polygon_destroy((opus_polygon *) shape);
			break;
		case OPUS_SHAPE_CIRCLE:
			opus_shape_circle_destroy((opus_circle *) shape);
			break;
		default:
			OPUS_ERROR("opus_shape_destroy::no such type of shape[%d]\n", shape->type);
	}
}
