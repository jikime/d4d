/* SPDX-License-Identifier: Apache-2.0 */
#include "walk_range_filter_test.h"
#include "../src/walk_range_filter.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define NEAR(a, b) (fabsf((a) - (b)) <= 0.0001f)

static int seed(struct walk_range_filter *filter, float metres, int64_t base_ms)
{
    struct walk_range_output output;
    walk_range_reset(filter);
    CHECK(walk_range_update(filter, metres, base_ms + 1000, base_ms + 1000,
                            &output) == WALK_RANGE_WARMING);
    CHECK(!filter->have_output);
    CHECK(walk_range_update(filter, metres, base_ms + 2000, base_ms + 2000,
                            &output) == WALK_RANGE_WARMING);
    CHECK(!filter->have_output);
    CHECK(walk_range_update(filter, metres, base_ms + 3000, base_ms + 3000,
                            &output) == WALK_RANGE_READY);
    CHECK(filter->have_output && filter->count == 3);
    CHECK(NEAR(output.metres, metres) && NEAR(output.median_metres, metres));
    CHECK(!output.limited);
    return 0;
}

static int test_warmup_and_independence(void)
{
    struct walk_range_filter filter, other;
    struct walk_range_output output;
    walk_range_reset(&filter);
    memset(&output, 0x5a, sizeof(output));
    struct walk_range_output unchanged;
    memcpy(&unchanged, &output, sizeof(output));
    CHECK(walk_range_update(&filter, 20.0f, 0, 0, &output) == WALK_RANGE_WARMING);
    CHECK(memcmp(&output, &unchanged, sizeof(output)) == 0);
    CHECK(walk_range_update(&filter, 0.0f, 1000, 1000, &output) == WALK_RANGE_WARMING);
    CHECK(memcmp(&output, &unchanged, sizeof(output)) == 0);
    CHECK(walk_range_update(&filter, 21.0f, 2000, 2000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 20.0f) && !output.limited);
    struct walk_range_filter before;
    memcpy(&before, &filter, sizeof(filter));
    int err = seed(&other, 1.0f, 0);
    if (err) return err;
    CHECK(memcmp(&filter, &before, sizeof(filter)) == 0);
    CHECK(NEAR(other.metres, 1.0f));
    return 0;
}

static int test_single_outlier(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    int err = seed(&filter, 20.0f, 0);
    if (err) return err;
    CHECK(walk_range_update(&filter, 0.0f, 4000, 4000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 20.0f) && NEAR(output.median_metres, 20.0f));
    CHECK(!output.limited);
    CHECK(walk_range_update(&filter, 20.0f, 5000, 5000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 20.0f));
    CHECK(walk_range_update(&filter, 20.0f, 6000, 6000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 20.0f));
    return 0;
}

static int test_persistent_changes(void)
{
    const float starts[] = {20.0f, 20.0f, 0.0f};
    const float targets[] = {5.0f, 0.0f, 20.0f};
    for (unsigned scenario = 0; scenario < 3; ++scenario) {
        struct walk_range_filter filter;
        struct walk_range_output output;
        int err = seed(&filter, starts[scenario], 0);
        if (err) return err;
        float expected = starts[scenario];
        for (unsigned step = 0; step < 13; ++step) {
            int64_t now = 4000 + (int64_t)step * 1000;
            CHECK(walk_range_update(&filter, targets[scenario], now, now,
                                    &output) == WALK_RANGE_READY);
            /* The first changed sample is removed by median-of-three. */
            if (step > 0) {
                float delta = targets[scenario] - expected;
                if (delta > 2.0f) expected += 2.0f;
                else if (delta < -2.0f) expected -= 2.0f;
                else expected = targets[scenario];
            }
            CHECK(NEAR(output.metres, expected));
            CHECK(NEAR(output.limit_metres, 2.0f));
        }
        CHECK(NEAR(output.metres, targets[scenario]));
        CHECK(!output.limited);
    }
    return 0;
}

static int test_fractional_time(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    int err = seed(&filter, 20.0f, 0);
    if (err) return err;
    CHECK(walk_range_update(&filter, 0.0f, 4000, 4000, &output) == WALK_RANGE_READY);
    CHECK(walk_range_update(&filter, 0.0f, 4500, 4500, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 19.0f) && NEAR(output.limit_metres, 1.0f));
    CHECK(output.limited);
    CHECK(walk_range_update(&filter, 0.0f, 4625, 4625, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 18.75f) && NEAR(output.limit_metres, 0.25f));
    CHECK(walk_range_update(&filter, 0.0f, 4626, 4626, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 18.748f) && NEAR(output.limit_metres, 0.002f));
    return 0;
}

static int expect_invalid(struct walk_range_filter *filter, float raw,
                          int64_t sample_ms, int64_t now_ms)
{
    struct walk_range_filter before;
    memcpy(&before, filter, sizeof(before));
    struct walk_range_output output;
    memset(&output, 0x35, sizeof(output));
    struct walk_range_output unchanged;
    memcpy(&unchanged, &output, sizeof(output));
    CHECK(walk_range_update(filter, raw, sample_ms, now_ms,
                            &output) == WALK_RANGE_INVALID);
    CHECK(memcmp(filter, &before, sizeof(before)) == 0);
    CHECK(memcmp(&output, &unchanged, sizeof(output)) == 0);
    return 0;
}

static int test_invalid_inputs(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    int err = seed(&filter, 20.0f, 0);
    if (err) return err;
    const struct { float raw; int64_t sample_ms, now_ms; } cases[] = {
        {NAN, 4000, 4000}, {INFINITY, 4000, 4000}, {-INFINITY, 4000, 4000},
        {-0.01f, 4000, 4000}, {10000.0f, 4000, 4000},
        {10.0f, -1, 4000}, {10.0f, 4001, 4000},
        {10.0f, 4000, 5501}, /* 1501-ms-old sample. */
        {10.0f, 3000, 4000}, {10.0f, 2999, 4000},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        err = expect_invalid(&filter, cases[i].raw, cases[i].sample_ms, cases[i].now_ms);
        if (err) return err;
    }
    struct walk_range_filter before;
    memcpy(&before, &filter, sizeof(filter));
    CHECK(walk_range_update(&filter, 10.0f, 4000, 4000, NULL) == WALK_RANGE_INVALID);
    CHECK(memcmp(&filter, &before, sizeof(filter)) == 0);
    CHECK(walk_range_update(NULL, 10.0f, 4000, 4000, &output) == WALK_RANGE_INVALID);
    /* Exact age boundary is accepted, but a backwards UI clock is not. */
    CHECK(walk_range_update(&filter, 20.0f, 4000, 5500, &output) == WALK_RANGE_READY);
    err = expect_invalid(&filter, 10.0f, 4500, 5400);
    if (err) return err;
    CHECK(walk_range_update(&filter, 9999.0f, 5500, 5500, &output) == WALK_RANGE_READY);
    return 0;
}

static int test_gap_rewarm_keeps_anchor(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    int err = seed(&filter, 20.0f, 0);
    if (err) return err;
    CHECK(walk_range_update(&filter, 40.0f, 7001, 7001, &output) == WALK_RANGE_WARMING);
    CHECK(filter.have_output && filter.count == 1 && NEAR(filter.metres, 20.0f));
    CHECK(filter.output_sample_ms == 3000 && filter.output_wall_ms == 3000);
    CHECK(walk_range_update(&filter, 40.0f, 8001, 8001, &output) == WALK_RANGE_WARMING);
    CHECK(filter.count == 2 && NEAR(filter.metres, 20.0f));
    CHECK(walk_range_update(&filter, 40.0f, 9001, 9001, &output) == WALK_RANGE_READY);
    CHECK(output.limited && NEAR(output.limit_metres, 12.002f));
    CHECK(NEAR(output.metres, 32.002f));

    err = seed(&filter, 20.0f, 0);
    if (err) return err;
    CHECK(walk_range_update(&filter, 20.0f, 6000, 6000, &output) == WALK_RANGE_READY);
    CHECK(filter.count == 3); /* Exactly 3000 ms does not discard the window. */

    /* Long observation gaps legitimately allow movement, without anchor reset. */
    CHECK(walk_range_update(&filter, 0.0f, 26000, 26000, &output) == WALK_RANGE_WARMING);
    CHECK(walk_range_update(&filter, 0.0f, 27000, 27000, &output) == WALK_RANGE_WARMING);
    CHECK(walk_range_update(&filter, 0.0f, 28000, 28000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 0.0f) && !output.limited);
    return 0;
}

static int test_queued_burst_wall_bound(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    int err = seed(&filter, 20.0f, 0);
    if (err) return err;
    CHECK(walk_range_update(&filter, 0.0f, 4500, 6000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 20.0f));
    CHECK(walk_range_update(&filter, 0.0f, 5500, 6000, &output) == WALK_RANGE_READY);
    CHECK(output.limited && NEAR(output.metres, 20.0f) && NEAR(output.limit_metres, 0.0f));
    CHECK(walk_range_update(&filter, 0.0f, 6000, 6000, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 20.0f) && NEAR(output.limit_metres, 0.0f));
    CHECK(walk_range_update(&filter, 0.0f, 6250, 6250, &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 19.5f) && NEAR(output.limit_metres, 0.5f));
    return 0;
}

static int test_64_bit_uptime(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    const int64_t base = INT64_C(4294967296) - 3500;
    int err = seed(&filter, 20.0f, base);
    if (err) return err;
    CHECK(walk_range_update(&filter, 0.0f, base + 4000, base + 4000,
                            &output) == WALK_RANGE_READY);
    CHECK(walk_range_update(&filter, 0.0f, base + 5000, base + 5000,
                            &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 18.0f) && NEAR(output.limit_metres, 2.0f));
    CHECK(filter.output_sample_ms > INT64_C(4294967296));
    /* A huge outage cannot overflow elapsed-time arithmetic or float budget. */
    const int64_t far = INT64_MAX - 10000;
    CHECK(walk_range_update(&filter, 9999.0f, far, far, &output) == WALK_RANGE_WARMING);
    CHECK(walk_range_update(&filter, 9999.0f, far + 1000, far + 1000,
                            &output) == WALK_RANGE_WARMING);
    CHECK(walk_range_update(&filter, 9999.0f, far + 2000, far + 2000,
                            &output) == WALK_RANGE_READY);
    CHECK(NEAR(output.metres, 9999.0f) && isfinite(output.limit_metres));
    return 0;
}

static uint32_t next_random(uint32_t *state)
{
    *state = *state * UINT32_C(1664525) + UINT32_C(1013904223);
    return *state;
}

static int test_randomized_bound_invariants(void)
{
    struct walk_range_filter filter;
    struct walk_range_output output;
    uint32_t random = UINT32_C(0x54c5a11);
    int64_t sample_ms = INT64_C(4294967296) + 3000;
    int64_t wall_ms = sample_ms;
    int err = seed(&filter, 20.0f, sample_ms - 3000);
    if (err) return err;
    for (unsigned i = 0; i < 1000; ++i) {
        int64_t step_ms = (int64_t)(next_random(&random) % 1500U) + 1;
        if (i % 97U == 96U) step_ms += 4000;
        sample_ms += step_ms;
        int64_t now_ms = sample_ms + (int64_t)(next_random(&random) % 501U);
        if (now_ms < wall_ms) now_ms = wall_ms;
        wall_ms = now_ms;
        float raw = (float)(next_random(&random) % 100001U) / 1000.0f;
        struct walk_range_filter before;
        memcpy(&before, &filter, sizeof(filter));
        enum walk_range_state state = walk_range_update(&filter, raw, sample_ms,
                                                        now_ms, &output);
        CHECK(state == WALK_RANGE_READY || state == WALK_RANGE_WARMING);
        CHECK(filter.last_sample_ms == sample_ms);
        if (state == WALK_RANGE_WARMING) {
            CHECK(filter.have_output && NEAR(filter.metres, before.metres));
            CHECK(filter.output_sample_ms == before.output_sample_ms);
            CHECK(filter.output_wall_ms == before.output_wall_ms);
        } else {
            int64_t dt = sample_ms - before.output_sample_ms;
            int64_t wall_dt = now_ms - before.output_wall_ms;
            if (wall_dt < dt) dt = wall_dt;
            float bound = WALK_RANGE_SPEED_MPS * ((float)dt / 1000.0f);
            CHECK(isfinite(output.metres) && output.metres >= 0.0f && output.metres <= 9999.0f);
            CHECK(fabsf(output.metres - before.metres) <= bound + 0.0001f);
            CHECK(NEAR(output.limit_metres, bound));
            float low = fminf(before.metres, output.median_metres);
            float high = fmaxf(before.metres, output.median_metres);
            CHECK(output.metres >= low - 0.0001f && output.metres <= high + 0.0001f);
            CHECK(NEAR(filter.metres, output.metres));
        }
        if (i % 33U == 0) {
            err = expect_invalid(&filter, raw, sample_ms, now_ms);
            if (err) return err;
        }
    }
    return 0;
}

int walk_range_selftest(void)
{
    static int (*const tests[])(void) = {
        test_warmup_and_independence, test_single_outlier, test_persistent_changes,
        test_fractional_time, test_invalid_inputs, test_gap_rewarm_keeps_anchor,
        test_queued_burst_wall_bound, test_64_bit_uptime, test_randomized_bound_invariants,
    };
    for (unsigned i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
        int err = tests[i]();
        if (err) return err;
    }
    return 0;
}
