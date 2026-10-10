/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WALK_RANGE_FILTER_H
#define WALK_RANGE_FILTER_H
#include <stdbool.h>
#include <stdint.h>

/* Stationary recovered drone + walking searcher. A configurable assumption,
 * NOT measured user speed or a correction of CS absolute-distance bias. */
#define WALK_RANGE_SPEED_MPS 2.0f
#define WALK_RANGE_WINDOW_GAP_MS 3000
#define WALK_RANGE_MAX_AGE_MS 1500

struct walk_range_filter {
    float samples[3];
    float metres;
    int64_t last_sample_ms, output_sample_ms, output_wall_ms;
    uint8_t count, next;
    bool seen_sample, have_output;
};
enum walk_range_state { WALK_RANGE_INVALID, WALK_RANGE_WARMING, WALK_RANGE_READY };
struct walk_range_output {
    float metres, median_metres, limit_metres;
    bool limited;
};
void walk_range_reset(struct walk_range_filter *filter);
/* Single owner (main thread). sample_ms is captured when a valid CS estimate
 * is produced; now_ms is the UI service time. Repeated/out-of-order, invalid,
 * future and stale samples do not mutate state. Three fresh samples warm up.
 * After a gap, discard the median window but retain the same-peer anchor.
 * Rate bound uses the smaller of elapsed observation and display time, so a
 * queued burst cannot consume multiple seconds of movement instantly. */
enum walk_range_state walk_range_update(struct walk_range_filter *filter,
        float raw_metres, int64_t sample_ms, int64_t now_ms,
        struct walk_range_output *output);
#endif
