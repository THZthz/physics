/**
 * @file sleeping.c
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
#include "opus/physics/physics_private.h"
#include "opus/data_structure/array.h"

void opus_sleeping_wake_up(opus_body *body)
{
	if (!opus_body_is_static(body)) {
		body->is_sleeping   = 0;
		body->sleep_counter = 0;
	}
}

void opus_sleeping_fall_asleep(opus_physics_world *world, opus_body *body)
{
	if (!opus_body_is_kinematic(body) && opus_body_is_jointed(body)) {
		body->is_sleeping   = 1;
		body->sleep_counter = world->body_sleep_delay_counter;
		opus_vec2_set(&body->velocity, 0, 0);
		body->angular_velocity = 0;
		body->motion           = 0;
	}
}

void opus_sleeping_update(opus_physics_world *world, opus_real dt)
{
	uint64_t i;

	opus_body *body;
	opus_real  min_motion, max_motion;
	opus_real  time_factor;
	opus_real  sleep_threshold;

	if (!world->enable_sleeping) return;

	time_factor     = dt * dt * dt;
	sleep_threshold = time_factor * world->body_sleep_motion_threshold;
	for (i = 0; i < opus_arr_len(world->bodies); i++) {
		body = world->bodies[i];

		/* an external force is applied */
		if (!opus_vec2_equal(body->force, opus_vec2_(0, 0))) {
			opus_sleeping_wake_up(body);
			continue;
		}

		/* biased average motion estimation between frames */
		min_motion        = opus_min(body->motion, body->prev_motion);
		max_motion        = opus_max(body->motion, body->prev_motion);
		body->prev_motion = body->motion;
		body->motion      = world->body_min_motion_bias * min_motion +
		               (1 - world->body_min_motion_bias) * max_motion;

		if (body->is_sleeping) continue;

//		printf("%f %f\n", body->motion, sleep_threshold);

		if (world->body_sleep_counter_threshold && body->motion < sleep_threshold) {
			body->sleep_counter++;

			if (body->sleep_counter >= world->body_sleep_counter_threshold)
				opus_sleeping_fall_asleep(world, body);
		} else {
			if (body->sleep_counter > 0)
				body->sleep_counter--;
		}
	}
}

void opus_sleeping_before_resolution(opus_physics_world *world)
{
	uint64_t i;

	opus_contacts *contacts;
	opus_body     *A, *B;

	if (!world->enable_sleeping) return;
}

void opus_sleeping_after_collision(opus_physics_world *world, opus_real dt)
{
	int      has_active_contact;
	uint64_t i, j, n;

	opus_contacts *contacts;
	opus_contact  *contact;
	opus_body     *A, *B, *sleeping_body, *moving_body;
	opus_real      wake_threshold, time_factor;

	if (!world->enable_sleeping) return;

	time_factor    = dt * dt * dt;
	wake_threshold = world->body_wake_motion_threshold * time_factor;
	opus_hashmap_foreach_start(&world->contacts, contacts, i)
	{
		contacts = *(opus_contacts **) contacts;

		n = opus_arr_len(contacts->contacts);
		if (n == 0) continue;
		has_active_contact = 0;
		for (j = 0; j < n; j++) {
			contact = contacts->contacts[j];
			if (contact->is_active) {
				has_active_contact = 1;
			}
		}
		if (!has_active_contact) continue;

		A = contacts->A;
		B = contacts->B;

		if (A->is_sleeping && B->is_sleeping) continue;
		if (opus_body_is_static(A) || opus_body_is_static(B)) continue;

		if (A->is_sleeping || B->is_sleeping) {
			sleeping_body = (A->is_sleeping && !opus_body_is_static(A)) ? A : B,
			moving_body   = sleeping_body == A ? B : A;

			if (!opus_body_is_static(sleeping_body) && moving_body->motion > wake_threshold) {
				opus_sleeping_wake_up(sleeping_body);
			}
		}
	}
	opus_hashmap_foreach_end();
}
