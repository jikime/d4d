/* SPDX-License-Identifier: Apache-2.0
 * Single-threaded Zephyr HAL for the existing CEVA/Hillcrest SH2 driver.
 * The current hardware was identified as BNO08x-compatible, SW part 10004148.
 * A >=3 g magnitude is a BENCH heuristic, not a validated crash classifier.
 */
#include "nu54_imu.h"
#include "nu54_peripherals.h"
#include "vendor/sh2/sh2.h"
#include "vendor/sh2/sh2_err.h"
#include "vendor/sh2/sh2_SensorValue.h"
#include <math.h>
#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(nu54_imu, LOG_LEVEL_INF);
#define IMU_ADDRESS 0x4b
#define FRESH_MS 250U
#define WARMUP_MS 1000U
#define WARMUP_SAMPLES 10U
static const struct device *bus;
static struct nu54_imu_snapshot state;
static bool opened, accept_reports, have_sample, have_sequence;
static bool impact_latched, impact_pending, reset_pending;
static bool hal_reset_succeeded;
static uint8_t previous_sequence;
static uint32_t last_sample_ms, warmup_start_ms, warmup_count;
/* Per-explicit-begin diagnostics only. Never print SHTP sensor payloads and
 * never add I2C transactions merely to probe/report diagnostics. */
static struct {
    uint32_t read_calls, read_errors, write_calls, write_errors;
    uint32_t empty_headers, invalid_headers, header_mismatches;
    uint32_t read_deadlines, packets, reset_attempts;
    int last_read_error, last_write_error, open_status, product_status;
    uint16_t first_total;
    uint8_t first_channel, first_sequence;
    bool first_seen, open_called, product_called;
    uint16_t mismatch_expected_total, mismatch_observed_total, mismatch_position;
    uint8_t mismatch_expected_channel, mismatch_observed_channel;
    uint8_t mismatch_expected_sequence, mismatch_observed_sequence;
    bool mismatch_seen, mismatch_first;
} diag;

static void report_diagnostics(const char *stage)
{
    LOG_INF("IMU_DIAG stage=%s reset_attempts=%u reset_ack=%u open_called=%u open_status=%d product_called=%u product_status=%d",
            stage, diag.reset_attempts, hal_reset_succeeded, diag.open_called,
            diag.open_status, diag.product_called, diag.product_status);
    LOG_INF("IMU_DIAG read_calls=%u read_errors=%u last_read_error=%d write_calls=%u write_errors=%u last_write_error=%d",
            diag.read_calls, diag.read_errors, diag.last_read_error,
            diag.write_calls, diag.write_errors, diag.last_write_error);
    LOG_INF("IMU_DIAG empty_headers=%u invalid_headers=%u header_mismatches=%u read_deadlines=%u packets=%u",
            diag.empty_headers, diag.invalid_headers, diag.header_mismatches,
            diag.read_deadlines, diag.packets);
    LOG_INF("IMU_DIAG first_nonempty_header_seen=%u total=%u channel=%u sequence=%u payload_logged=0",
            diag.first_seen, diag.first_total, diag.first_channel, diag.first_sequence);
    if (diag.mismatch_seen) {
        LOG_INF("IMU_DIAG header_delta first_chunk=%u position=%u expected_total=%u observed_total=%u expected_channel=%u observed_channel=%u",
                diag.mismatch_first, diag.mismatch_position, diag.mismatch_expected_total,
                diag.mismatch_observed_total, diag.mismatch_expected_channel, diag.mismatch_observed_channel);
        LOG_INF("IMU_DIAG header_delta expected_sequence=%u observed_sequence=%u action=vendor_chunk_assembly payload_logged=0",
                diag.mismatch_expected_sequence, diag.mismatch_observed_sequence);
    }
}

static uint32_t now_ms(void) { return k_uptime_get_32(); }
static uint32_t hal_time(sh2_Hal_t *self)
{
    ARG_UNUSED(self);
    return (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks());
}

static int hal_open(sh2_Hal_t *self)
{
    ARG_UNUSED(self);
    /* This is a BNO08x executable-channel software reset, never a PMIC write.
     * It also restarts a previous stream on explicit user rearm. */
    const uint8_t reset[] = {5, 0, 1, 0, 1};
    hal_reset_succeeded = false;
    for (unsigned attempt = 0; attempt < 5; ++attempt) {
        ++diag.reset_attempts;
        ++diag.write_calls;
        int err = i2c_write(bus, reset, sizeof(reset), IMU_ADDRESS);
        if (err == 0) {
            hal_reset_succeeded = true;
            k_msleep(300);
            return 0;
        }
        ++diag.write_errors;
        diag.last_write_error = err;
        k_msleep(30);
    }
    return SH2_ERR_IO;
}

static void hal_close(sh2_Hal_t *self) { ARG_UNUSED(self); }

static int hal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned capacity, uint32_t *timestamp)
{
    ARG_UNUSED(self);
    uint8_t header[4];
    ++diag.read_calls;
    int err = i2c_read(bus, header, sizeof(header), IMU_ADDRESS);
    if (err) {
        ++diag.read_errors;
        diag.last_read_error = err;
        k_msleep(1);
        return 0;
    }
    const uint16_t total = ((uint16_t)header[0] | ((uint16_t)header[1] << 8)) & 0x7fffU;
    if (total == 0) { ++diag.empty_headers; k_msleep(1); return 0; }
    if (!diag.first_seen) {
        diag.first_seen = true;
        diag.first_total = total;
        diag.first_channel = header[2];
        diag.first_sequence = header[3];
    }
    if (total < 4 || total > capacity || total > SH2_HAL_MAX_TRANSFER_IN || header[2] > 5) {
        ++diag.invalid_headers;
        ++state.invalid;
        state.failure = "invalid_SHTP_header";
        k_msleep(1);
        return 0;
    }
    const uint32_t started = now_ms();
    *timestamp = hal_time(self); /* Host reception time; no wired INT signal. */
    unsigned remaining = total, position = 0;
    bool first = true;
    while (remaining) {
        if (now_ms() - started > 100U) {
            ++diag.read_deadlines;
            ++state.invalid;
            return 0;
        }
        uint8_t chunk[32];
        const unsigned repeated = first ? 0U : 4U;
        const unsigned count = MIN(remaining + repeated, sizeof(chunk));
        ++diag.read_calls;
        err = i2c_read(bus, chunk, count, IMU_ADDRESS);
        if (err) {
            ++diag.read_errors;
            diag.last_read_error = err;
            return 0;
        }
        if (count < 4) {
            ++state.invalid;
            return 0;
        }
        /* Match the hardware-tested SparkFun SH2 HAL: keep the first data
         * transaction's header and skip later four-byte I2C headers. The
         * initial four-byte peek is a length probe, not an invariant channel/
         * sequence snapshot for every subsequent transaction. Enforcing that
         * extra equality discarded every observed boot advertisement on this
         * sensor. Retain the delta as bounded diagnostics, not a fatal test;
         * the SH2 SHTP layer validates the assembled transfer's real header. */
        if (chunk[2] != header[2] || chunk[3] != header[3]) {
            ++diag.header_mismatches;
            if (!diag.mismatch_seen) {
                diag.mismatch_seen = true;
                diag.mismatch_first = first;
                diag.mismatch_position = position;
                diag.mismatch_expected_total = total;
                diag.mismatch_observed_total =
                    ((uint16_t)chunk[0] | ((uint16_t)chunk[1] << 8)) & 0x7fffU;
                diag.mismatch_expected_channel = header[2];
                diag.mismatch_observed_channel = chunk[2];
                diag.mismatch_expected_sequence = header[3];
                diag.mismatch_observed_sequence = chunk[3];
            }
        }
        const unsigned copied = count - repeated;
        if (copied == 0 || copied > remaining || position > capacity - copied) {
            ++state.invalid;
            return 0;
        }
        memcpy(buffer + position, chunk + repeated, copied);
        position += copied;
        remaining -= copied;
        first = false;
    }
    ++diag.packets;
    return (int)total;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned length)
{
    ARG_UNUSED(self);
    if (!length || length > SH2_HAL_MAX_TRANSFER_OUT) return SH2_ERR_BAD_PARAM;
    /* SH2 retries a zero return indefinitely. A completed I2C failure must be
     * a negative error, not 'temporarily busy', so unplug cannot hang forever. */
    ++diag.write_calls;
    int err = i2c_write(bus, buffer, length, IMU_ADDRESS);
    if (err) {
        ++diag.write_errors;
        diag.last_write_error = err;
    }
    return err == 0 ? (int)length : SH2_ERR_IO;
}

static sh2_Hal_t hal = {
    .open = hal_open, .close = hal_close, .read = hal_read,
    .write = hal_write, .getTimeUs = hal_time,
};

static void warmup_restart(bool reset_sequence)
{
    state.armed = false;
    have_sample = false;
    warmup_count = 0;
    warmup_start_ms = now_ms();
    if (reset_sequence) have_sequence = false;
    /* Never erase an already latched impact because a sensor reset occurs. */
}

static void async_event(void *cookie, sh2_AsyncEvent_t *event)
{
    ARG_UNUSED(cookie);
    if (event && event->eventId == SH2_RESET) reset_pending = true;
}

static void sensor_event(void *cookie, sh2_SensorEvent_t *event)
{
    ARG_UNUSED(cookie);
    if (!accept_reports || !state.ready || !event || event->reportId != SH2_ACCELEROMETER) return;
    sh2_SensorValue_t value = {0};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK || value.sensorId != SH2_ACCELEROMETER) {
        ++state.invalid;
        warmup_restart(false);
        state.failure = "invalid_accel_report";
        return;
    }
    const uint32_t now = now_ms();
    if (have_sample && now - last_sample_ms >= FRESH_MS) warmup_restart(true);
    if (have_sequence) {
        const uint8_t step = (uint8_t)(value.sequence - previous_sequence);
        if (step == 0) return;
        if (step >= 128) {
            ++state.invalid;
            warmup_restart(true);
            state.failure = "out_of_order_accel_report";
            return;
        }
        state.sequence_gaps += step - 1U;
    }
    previous_sequence = value.sequence;
    have_sequence = true;
    const float x = value.un.accelerometer.x / 9.80665f;
    const float y = value.un.accelerometer.y / 9.80665f;
    const float z = value.un.accelerometer.z / 9.80665f;
    const float magnitude = sqrtf(x*x + y*y + z*z);
    if (!isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(magnitude)) {
        ++state.invalid;
        warmup_restart(false);
        state.failure = "nonfinite_accel_report";
        return;
    }
    state.ax_g = x; state.ay_g = y; state.az_g = z; state.magnitude_g = magnitude;
    if (magnitude > state.peak_g) state.peak_g = magnitude;
    ++state.samples;
    if (!have_sample) warmup_start_ms = now;
    if (warmup_count < WARMUP_SAMPLES) ++warmup_count;
    last_sample_ms = now;
    have_sample = true;
    state.failure = "none";
    if (!impact_latched && warmup_count >= WARMUP_SAMPLES && now - warmup_start_ms >= WARMUP_MS)
        state.armed = true;
    if (state.armed && magnitude >= NU54_IMPACT_THRESHOLD_G) {
        impact_latched = impact_pending = true;
        state.armed = false;
    }
}

static bool set_report(uint32_t interval_us)
{
    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = interval_us;
    return sh2_setSensorConfig(SH2_ACCELEROMETER, &config) == SH2_OK;
}

static bool supported_part(uint32_t part)
{
    return part == 10003608U || part == 10003676U || part == 10004148U || part == 10004563U;
}

bool nu54_imu_begin(void)
{
    if (opened) nu54_imu_stop();
    memset(&diag, 0, sizeof(diag));
    hal_reset_succeeded = false;
    memset(&state, 0, sizeof(state));
    state.address = IMU_ADDRESS;
    state.ax_g = state.ay_g = state.az_g = state.magnitude_g = NAN;
    state.age_ms = UINT32_MAX;
    state.failure = "initializing";
    accept_reports = impact_latched = impact_pending = reset_pending = false;
    warmup_restart(true);
    bus = nu54_qwiic_bus();
    int err = nu54_qwiic_prepare();
    if (err) {
        state.failure = "qwiic_not_ready";
        LOG_ERR("IMU QWIIC prepare failed: %d", err);
        report_diagnostics("prepare_failed");
        return false;
    }
    diag.open_called = true;
    err = sh2_open(&hal, async_event, NULL);
    diag.open_status = err;
    opened = err == SH2_OK;
    if (err != SH2_OK) { state.failure = "SH2_open_failed"; goto failed; }
    /* This vendored SHTP version ignores the HAL open return. Verify it here. */
    if (!hal_reset_succeeded) { state.failure = "sensor_reset_write_failed"; goto failed; }
    sh2_ProductIds_t ids = {0};
    diag.product_called = true;
    diag.product_status = sh2_getProdIds(&ids);
    if (diag.product_status != SH2_OK) { state.failure = "no_SH2_product_response"; goto failed; }
    for (unsigned i = 0; i < ids.numEntries; ++i) {
        if (supported_part(ids.entry[i].swPartNumber)) {
            state.part = ids.entry[i].swPartNumber;
            break;
        }
    }
    if (!state.part) { state.failure = "unsupported_SW_part"; goto failed; }
    sh2_setSensorCallback(sensor_event, NULL);
    reset_pending = false;
    if (!set_report(NU54_IMU_INTERVAL_US)) { state.failure = "enable_report_failed"; goto failed; }
    state.ready = accept_reports = true;
    state.failure = "none";
    warmup_restart(true);
    LOG_INF("IMU addr=0x4b part=%u requested_hz=125 warmup_ms=1000 threshold_mg=3000", state.part);
    report_diagnostics("ready");
    return true;
failed:
    report_diagnostics(state.failure);
    if (opened) {
        sh2_setSensorCallback(NULL, NULL);
        sh2_close();
    }
    opened = false;
    LOG_ERR("IMU initialization failed: %s", state.failure);
    return false;
}

void nu54_imu_poll(void)
{
    if (!opened || !state.ready) return;
    if (have_sample && now_ms() - last_sample_ms >= FRESH_MS) {
        warmup_restart(true);
        state.failure = "sensor_data_stale";
    }
    sh2_service();
    if (reset_pending) {
        reset_pending = false;
        ++state.resets;
        state.ready = accept_reports = false;
        warmup_restart(true);
        state.ready = set_report(NU54_IMU_INTERVAL_US);
        accept_reports = state.ready;
        state.failure = state.ready ? "none" : "report_reenable_failed";
        warmup_restart(true);
    }
}

bool nu54_imu_take_impact(void)
{
    const bool pending = impact_pending;
    impact_pending = false;
    return pending;
}

void nu54_imu_stop(void)
{
    accept_reports = false;
    state.ready = state.armed = false;
    if (!opened) return;
    const bool reports_off = set_report(0);
    const bool sleep_requested = sh2_devSleep() == SH2_OK;
    state.failure = reports_off && sleep_requested ? "stopped_sleep_requested" : "stop_request_partial_failure";
    sh2_setSensorCallback(NULL, NULL);
    sh2_close();
    opened = false;
    LOG_INF("IMU report_disable=%d sleep_request=%d sleep_current_unverified=1", reports_off, sleep_requested);
}

struct nu54_imu_snapshot nu54_imu_snapshot(void)
{
    struct nu54_imu_snapshot result = state;
    result.age_ms = have_sample ? now_ms() - last_sample_ms : UINT32_MAX;
    result.fresh = state.ready && have_sample && result.age_ms < FRESH_MS;
    result.armed = state.armed && result.fresh;
    if (state.ready && have_sample && !result.fresh) result.failure = "sensor_data_stale";
    return result;
}
