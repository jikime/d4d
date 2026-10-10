/* SPDX-License-Identifier: Apache-2.0 */
#include "walk_range_filter.h"
#include <math.h>
#include <string.h>

void walk_range_reset(struct walk_range_filter *filter)
{
    memset(filter, 0, sizeof(*filter));
}

static float median3(float a, float b, float c)
{
    float t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

enum walk_range_state walk_range_update(struct walk_range_filter *filter,
        float raw_metres, int64_t sample_ms, int64_t now_ms,
        struct walk_range_output *output)
{
    if (!filter || !output || !isfinite(raw_metres) || raw_metres < 0.0f ||
        raw_metres > 9999.0f || sample_ms < 0 || now_ms < sample_ms ||
        now_ms - sample_ms > WALK_RANGE_MAX_AGE_MS ||
        (filter->seen_sample && sample_ms <= filter->last_sample_ms) ||
        (filter->have_output && now_ms < filter->output_wall_ms))
        return WALK_RANGE_INVALID;

    if (filter->seen_sample &&
        sample_ms - filter->last_sample_ms > WALK_RANGE_WINDOW_GAP_MS) {
        filter->count = filter->next = 0;
    }
    filter->last_sample_ms = sample_ms;
    filter->seen_sample = true;
    filter->samples[filter->next] = raw_metres;
    filter->next = (filter->next + 1U) % 3U;
    if (filter->count < 3U) ++filter->count;
    if (filter->count < 3U) return WALK_RANGE_WARMING;

    const float target = median3(filter->samples[0], filter->samples[1], filter->samples[2]);
    float value = target, limit = 0.0f;
    bool limited = false;
    if (filter->have_output) {
        int64_t dt = sample_ms - filter->output_sample_ms;
        const int64_t wall_dt = now_ms - filter->output_wall_ms;
        if (wall_dt < dt) dt = wall_dt;
        /* A larger budget can never be used in our [0,9999] domain. Cap before
         * float conversion, not by resetting the anchor after a long outage. */
        if (dt > 5000000) dt = 5000000;
        limit = WALK_RANGE_SPEED_MPS * ((float)dt / 1000.0f);
        const float delta = target - filter->metres;
        if (delta > limit) { value = filter->metres + limit; limited = true; }
        else if (delta < -limit) { value = filter->metres - limit; limited = true; }
    }
    filter->metres = value;
    filter->have_output = true;
    filter->output_sample_ms = sample_ms;
    filter->output_wall_ms = now_ms;
    *output = (struct walk_range_output) {
        .metres = value, .median_metres = target,
        .limit_metres = limit, .limited = limited,
    };
    return WALK_RANGE_READY;
}
