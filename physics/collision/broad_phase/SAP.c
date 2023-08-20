/**
 * @file SAP.c
 *     Author:    _              _
 *               / \   _ __ ___ (_)  __ _ ___
 *              / _ \ | '_ ` _ \| |/ _` / __|
 *             / ___ \| | | | | | | (_| \__ \
 *            /_/   \_\_| |_| |_|_|\__,_|___/  2023/2/28
 *
 * @brief sweep and prune
 *
 */

#include "opus/data_structure/array.h"
#include "opus/physics/physics.h"
#include "opus/physics/physics_private.h"

static int bodies_sort_x_(const void *a, const void *b)
{
	opus_body *ba = *(opus_body **) a;
	opus_body *bb = *(opus_body **) b;
	return opus_sign(ba->bound.min.x - bb->bound.min.x);
}

void opus_SAP(opus_body **bodies, size_t n, opus_sap_cb callback, void *data)
{
	size_t i, j, k, l;

	opus_body *A, *B, *part_a, *part_b;

	opus_mat2d ta, tb, ta1, tb1;
	opus_aabb  ba, bb;

	uint64_t na, nb; /* length of parts of A and B */

	/* if no bodies exist, exit */
	if (bodies == NULL) return;

	/* update AABB */
	for (i = 0; i < n; i++)
		opus_body_update_bound(bodies[i]);

	/* sort bodies by X in ascending order */
	qsort(bodies, n, sizeof(opus_body *), bodies_sort_x_);
	for (i = 0; i < n; i++) {
		A  = bodies[i];
		na = opus_arr_len(A->parts);

		for (j = i + 1; j < n; j++) {
			B  = bodies[j];
			nb = opus_arr_len(B->parts);

			ba = A->shape->bound;
			bb = B->shape->bound;

			/* X-axis: we have already sorted all the bodies in X axis */
			if (bb.min.x > ba.max.x) break;

			/* Y-axis: check AABB bounding box to check the two bodies can collide */
			if (ba.max.y < bb.min.y || ba.min.y > bb.max.y) continue;

			/* two static bodies can not collide */
			if (A->type == OPUS_BODY_STATIC && B->type == OPUS_BODY_STATIC) continue;

			/* check bitmask to determine if the two can collide */
			if (!(A->bitmask & B->bitmask)) continue;

			/* account for compound bodies */
			for (k = 0; k < na; k++) {
				part_a = A->parts[k];
				for (l = 0; l < nb; l++) {
					part_b = B->parts[l];

					/* calculate rigid body transformation matrix */
					opus_body_get_transform(part_a, ta);
					opus_body_get_transform(part_b, tb);

					/* check AABB overlap */
					if (!opus_aabb_is_overlap(&part_a->shape->bound, &part_b->shape->bound)) continue;

					callback(part_a, part_b, ta, tb, data);
				}
			}
		}
	}
}
