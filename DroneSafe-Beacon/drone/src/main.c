/*
 * Standard connected CS reflector derived from Nordic ras_reflector v3.4.1.
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Modifications: authenticated NU54 recovery state machine, 2026.
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * No open pairing, fixed public device name, fake range, or CS Test fallback.
 * Radio emissions remain observable. Flash/debug extraction resistance is not
 * enabled on this borrowed development board. Not flight or battery qualified.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <bluetooth/services/ras.h>
#include "secure_pair.h"
#include "usb_console.h"
#include "nu54_peripherals.h"
#include "nu54_imu.h"

#define BEACON_PERIOD_MS 1000
#define CS_ACTIVITY_TIMEOUT_MS 30000
#define CONNECT_ADV_RETRY_MS 250

static struct bt_le_ext_adv *connect_adv, *beacon_adv;
static struct bt_conn *peer;
static struct k_spinlock peer_lock;
static atomic_t connected, config_ready, cs_secured, stream_on, need_resync;
static atomic_t manual_trigger, manual_rearm, status_requested, test_drop;
static atomic_t plot_request;
static atomic_t adv_completions, beacon_busy;
static atomic_t link_generation;
static atomic_t last_result_ms, last_cs_activity_ms;
static atomic_t cs_aborts, cs_good_subevents;
static atomic_t last_procedure_abort, last_subevent_abort;
static uint32_t adv_attempts, resync_count, next_beacon, connected_at;
static bool recovery, connect_adv_running, defaults_set, procedure_set;
static bool roles_disabled;
static uint8_t saved_recovery;
static uint32_t trigger_at;
static uint32_t beacon_started;
static uint32_t connect_adv_retry;
static bool plot_mode;
static uint32_t next_plot;
static atomic_t imu_restore_busy;
static struct k_thread imu_restore_thread;
K_THREAD_STACK_DEFINE(imu_restore_stack, 3072);

static void restore_sensor_stop(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);
    const uint32_t started = k_uptime_get_32();
    /* An MCU-only reset need not reset the independently powered IMU. Reopen
     * its protocol once, then disable its reports and request sleep. This runs
     * outside main/system workqueue: advertising, USB and LED work continue.
     * The HAL bounds every I2C transfer (100 ms), reset retries and product
     * query (2 s); do not kill this thread while it could hold an I2C lock.
     * Main defers rearm and never reads the SH2 state until busy is cleared. */
    const bool initialized = nu54_imu_begin();
    if (initialized) nu54_imu_stop();
    const struct nu54_imu_snapshot snapshot = nu54_imu_snapshot();
    const bool stopped = initialized && snapshot.failure &&
                         !strcmp(snapshot.failure, "stopped_sleep_requested");
    printk("IMU_RESTORE cleanup_done=1 init_ok=%u stop_requested=%u elapsed_ms=%u failure=%s advertising_continues=1 sleep_current_unverified=1\n",
           initialized, stopped, k_uptime_get_32() - started,
           snapshot.failure ? snapshot.failure : "unknown");
    /* Atomic release publishes the completed state to the sole future SH2
     * owner (main). This worker performs no further sensor access. */
    atomic_clear(&imu_restore_busy);
}

static int recovery_load(const char *name, size_t len, settings_read_cb read, void *arg)
{
    if (strcmp(name, "latched") || len != 1) return -ENOENT;
    int n = read(arg, &saved_recovery, 1);
    if (n != 1) return -EIO;
    saved_recovery = saved_recovery == 1;
    return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(recovery_settings, "recovery", NULL, recovery_load, NULL, NULL);

static struct bt_conn *peer_ref(void)
{
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    struct bt_conn *p = peer ? bt_conn_ref(peer) : NULL;
    k_spin_unlock(&peer_lock, key);
    return p;
}

static bool is_current(struct bt_conn *conn)
{
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    bool same = peer == conn;
    k_spin_unlock(&peer_lock, key);
    return same;
}

/* A callback can publish a timestamp after main took its 'now' snapshot.
 * Clamp that tiny negative age rather than treating it as a 49-day timeout.
 * All watchdog intervals are much shorter than the signed 32-bit half range.
 */
static uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
    const int32_t age = (int32_t)(now - then);
    return age > 0 ? (uint32_t)age : 0;
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (err) return;
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    if (peer) {
        k_spin_unlock(&peer_lock, key);
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }
    peer = bt_conn_ref(conn);
    k_spin_unlock(&peer_lock, key);
    atomic_clear(&config_ready);
    atomic_clear(&cs_secured);
    atomic_clear(&stream_on);
    atomic_clear(&need_resync);
    const uint32_t now = k_uptime_get_32();
    atomic_set(&last_result_ms, now);
    atomic_set(&last_cs_activity_ms, now);
    atomic_set(&connected, 1);
    atomic_inc(&link_generation);
    printk("LINK connected role=drone cs_authorized=0\n");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    k_spinlock_key_t key = k_spin_lock(&peer_lock);
    struct bt_conn *old = peer == conn ? peer : NULL;
    if (old) peer = NULL;
    k_spin_unlock(&peer_lock, key);
    if (!old) return;
    atomic_clear(&connected);
    atomic_clear(&stream_on);
    atomic_clear(&config_ready);
    atomic_clear(&cs_secured);
    atomic_clear(&need_resync);
    bt_conn_unref(old);
    printk("LINK disconnected role=drone reason=%u session_discarded=1 reboot=0\n", reason);
}

static void config_cb(struct bt_conn *conn, uint8_t status, struct bt_conn_le_cs_config *cfg)
{
    if (!is_current(conn)) return;
    if (status || !cfg || cfg->id != 0 || !secure_pair_is_authorized(conn)) {
        atomic_set(&need_resync, 1);
        return;
    }
    atomic_set(&config_ready, 1);
    atomic_set(&last_cs_activity_ms, k_uptime_get_32());
}

static void cs_security_cb(struct bt_conn *conn, uint8_t status)
{
    if (!is_current(conn)) return;
    if (status || !secure_pair_is_authorized(conn)) atomic_set(&need_resync, 1);
    else {
        atomic_set(&cs_secured, 1);
        atomic_set(&last_cs_activity_ms, k_uptime_get_32());
    }
}

static void procedure_cb(struct bt_conn *conn, uint8_t status,
                         struct bt_conn_le_cs_procedure_enable_complete *p)
{
    if (!is_current(conn)) return;
    if (status || !p || !secure_pair_is_authorized(conn)) {
        atomic_set(&need_resync, 1);
        return;
    }
    atomic_set(&stream_on, p->state == 1);
    atomic_set(&last_cs_activity_ms, k_uptime_get_32());
    printk("CS procedure_enabled=%u role=drone authenticated=1\n", p->state);
}

static void subevent_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *r)
{
    if (!is_current(conn) || !secure_pair_is_authorized(conn) || !r) return;
    if (r->header.config_id != 0) {
        atomic_set(&need_resync, 1);
        return;
    }
    const uint32_t now = k_uptime_get_32();
    atomic_set(&last_cs_activity_ms, now);
    if (r->header.subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED ||
        r->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
        /* Missing one CS sync or a controller scheduling conflict does not
         * mean the authenticated ACL is lost. A subevent-only abort can leave
         * recurring procedures running; a whole-procedure abort stops them
         * until the initiator coordinates disable/enable on the same ACL.
         * The initiator owns result assembly and bounded retries.
         * A 30-second no-activity watchdog remains as the reflector fallback.
         */
        atomic_inc(&cs_aborts);
        atomic_set(&last_procedure_abort, r->header.procedure_abort_reason);
        atomic_set(&last_subevent_abort, r->header.subevent_abort_reason);
        if (r->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
            atomic_clear(&stream_on);
        }
        printk("CS_ABORT role=drone procedure=%u subevent=%u counter=%u total=%ld acl_retained=1\n",
               (unsigned)r->header.procedure_abort_reason,
               (unsigned)r->header.subevent_abort_reason,
               r->header.procedure_counter, (long)atomic_get(&cs_aborts));
    } else {
        atomic_inc(&cs_good_subevents);
        atomic_set(&last_result_ms, now);
    }
}

BT_CONN_CB_DEFINE(drone_conn_callbacks) = {
    .connected = connected_cb, .disconnected = disconnected_cb,
    .le_cs_config_complete = config_cb,
    .le_cs_security_enable_complete = cs_security_cb,
    .le_cs_procedure_enable_complete = procedure_cb,
    .le_cs_subevent_data_available = subevent_cb,
};

static void beacon_sent(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info)
{
    ARG_UNUSED(adv);
    atomic_add(&adv_completions, info->num_sent);
    atomic_clear(&beacon_busy);
    if (info->num_sent) nu54_leds_advert_pulse();
}
static const struct bt_le_ext_adv_cb beacon_callbacks = {.sent = beacon_sent};

/* RAS UUID advertises a service category, NOT an owner's name or fixed ID.
 * The controller's private addresses and registered IRKs select the peer.
 * No sensor readings, key material or stable owner identifier are advertised.
 */
static const struct bt_data advertisement[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RANGING_SERVICE_VAL)),
};

static int advertising_init(void)
{
    struct bt_le_adv_param p = {
        .id = BT_ID_DEFAULT, .sid = 0, .options = BT_LE_ADV_OPT_CONN,
        .interval_min = 160, .interval_max = 160,
    };
    int err = bt_le_ext_adv_create(&p, NULL, &connect_adv);
    if (err) return err;
    err = bt_le_ext_adv_set_data(connect_adv, advertisement, ARRAY_SIZE(advertisement), NULL, 0);
    if (err) return err;
    secure_pair_set_oob_advertiser(connect_adv);
    p.sid = 1;
    p.options = BT_LE_ADV_OPT_NONE;
    p.interval_min = p.interval_max = 1600;
    err = bt_le_ext_adv_create(&p, &beacon_callbacks, &beacon_adv);
    if (err) return err;
    return bt_le_ext_adv_set_data(beacon_adv, advertisement, ARRAY_SIZE(advertisement), NULL, 0);
}

static void service_advertisements(uint32_t now)
{
    bool want_connect = !atomic_get(&connected) &&
                        (recovery || secure_pair_oob_advertising_needed());
    if (want_connect && !connect_adv_running && (int32_t)(now-connect_adv_retry) >= 0) {
        connect_adv_retry = now + CONNECT_ADV_RETRY_MS;
        int err = bt_le_ext_adv_start(connect_adv, BT_LE_EXT_ADV_START_DEFAULT);
        if (!err || err == -EALREADY) connect_adv_running = true;
        else printk("ADV connect_start_error=%d\n", err);
    } else if (!want_connect && connect_adv_running && (int32_t)(now-connect_adv_retry) >= 0) {
        connect_adv_retry = now + CONNECT_ADV_RETRY_MS;
        int err = bt_le_ext_adv_stop(connect_adv);
        if (!err || err == -EALREADY) connect_adv_running = false;
        else printk("ADV connect_stop_error=%d\n", err);
    }
    if (!recovery) return;
    if (atomic_get(&beacon_busy) && (uint32_t)(now-beacon_started) > 3000) {
        int err = bt_le_ext_adv_stop(beacon_adv);
        beacon_started = now;
        if (!err || err == -EALREADY) atomic_clear(&beacon_busy);
        printk("ADV completion_timeout stop=%d\n", err);
    }
    if ((int32_t)(now - next_beacon) >= 0 && !atomic_get(&beacon_busy)) {
        const struct bt_le_ext_adv_start_param once = {.num_events = 1};
        atomic_set(&beacon_busy, 1);
        int err = bt_le_ext_adv_start(beacon_adv, &once);
        next_beacon = now + BEACON_PERIOD_MS;
        if (!err) {
            beacon_started = now;
            ++adv_attempts;
        } else { atomic_clear(&beacon_busy); printk("ADV independent_start_error=%d\n", err); }
    }
}

static void close_session(struct bt_conn *conn, const char *reason)
{
    ++resync_count;
    if (atomic_get(&config_ready)) {
        struct bt_le_cs_procedure_enable_param stop = {.config_id = 0, .enable = 0};
        (void)bt_le_cs_procedure_enable(conn, &stop);
    }
    /* Disconnect terminates ACL and its CS context on both controllers. A lost
     * termination packet is bounded by link supervision on the other device.
     * Never reset RADIO registers behind the Bluetooth controller's back. */
    int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    printk("RESYNC role=drone reason=%s disconnect=%d count=%u\n", reason, err, resync_count);
    atomic_clear(&need_resync);
    atomic_clear(&stream_on);
    defaults_set = false;
    procedure_set = false;
}

static void trigger(const char *cause)
{
    if (recovery) return;
    recovery = true;
    plot_mode = false;
    trigger_at = k_uptime_get_32();
    next_beacon = trigger_at;
    saved_recovery = 1;
    int err = settings_save_one("recovery/latched", &saved_recovery, 1);
    nu54_leds_impact();
    nu54_imu_stop();
    printk("EVENT RECOVERY cause=%s persisted=%d classification=CRASH_SUSPECT_TEST_ONLY\n", cause, err == 0);
}

static void app_command(const char *line)
{
    if (!strcmp(line, "b")) atomic_set(&manual_trigger, 1);
    else if (!strcmp(line, "r")) atomic_set(&manual_rearm, 1);
    else if (!strcmp(line, "s") || !strcmp(line, "?")) atomic_set(&status_requested, 1);
    else if (!strcmp(line, "drop")) atomic_set(&test_drop, 1);
    else if (!strcmp(line, "p")) atomic_set(&plot_request, 1);
    else if (!strcmp(line, "m")) atomic_set(&plot_request, 2);
}

int main(void)
{
    int err = nu54_peripherals_init(NU54_ROLE_DRONE);
    printk("BOOT NU54V_SECURE_DRONE_reliability2 peripheral_init=%d connected_cs=1\n", err);
    err = bt_enable(NULL);
    if (err) { printk("FATAL bt_enable=%d\n", err); return 0; }
    err = settings_load();
    if (err) { printk("FATAL settings_load=%d\n", err); return 0; }
    err = secure_pair_init();
    if (err) { printk("FATAL auth_init=%d\n", err); return 0; }
    err = advertising_init();
    if (err) { printk("FATAL adv_init=%d\n", err); return 0; }
    err = usb_console_start(app_command);
    if (err) { printk("FATAL console_init=%d\n", err); return 0; }
    recovery = saved_recovery == 1;
    bool restore_sensor_pending = recovery;
    atomic_set(&imu_restore_busy, recovery);
    if (!recovery) nu54_imu_begin();
    else printk("EVENT RECOVERY cause=persisted_restore sensor_cleanup=deferred_once\n");
    bool was_connected = false;
    atomic_val_t seen_generation = 0;
    bool closing = false;
    uint32_t close_at = 0, next_status = 0;
    while (true) {
        uint32_t now = k_uptime_get_32();
        secure_pair_poll();
        atomic_val_t requested_plot = atomic_set(&plot_request, 0);
        if (requested_plot) plot_mode = requested_plot == 1 && !recovery;
        if (atomic_cas(&manual_trigger, 1, 0) || nu54_button_pressed()) trigger("manual_test");
        /* Keep a received 'r' pending while the one-shot restore worker owns
         * SH2. Never perform two independent HAL sessions concurrently. */
        if (!atomic_get(&imu_restore_busy) && atomic_cas(&manual_rearm, 1, 0)) {
            recovery = false;
            saved_recovery = 0;
            printk("EVENT REARM persist_err=%d\n", settings_save_one("recovery/latched", &saved_recovery, 1));
            bt_le_ext_adv_stop(beacon_adv);
            atomic_clear(&beacon_busy);
            struct bt_conn *c = peer_ref();
            if (c) { close_session(c, "rearm"); bt_conn_unref(c); }
            nu54_imu_begin();
        }
        if (!recovery) {
            nu54_imu_poll();
            if (nu54_imu_take_impact()) trigger("accel_threshold");
        }
        struct nu54_imu_snapshot imu = {
            .age_ms = UINT32_MAX, .failure = "restore_cleanup_running",
        };
        if (!atomic_get(&imu_restore_busy)) imu = nu54_imu_snapshot();
        nu54_leds_service(imu.armed, recovery);
        bool linked = atomic_get(&connected);
        atomic_val_t generation = atomic_get(&link_generation);
        if (linked && (!was_connected || generation != seen_generation)) {
            seen_generation = generation;
            connected_at = now;
            defaults_set = procedure_set = false;
            closing = false;
            roles_disabled = false;
            /* Connectable advertising ended automatically when a connection formed. */
            connect_adv_running = false;
        }
        if (!linked && was_connected) {
            defaults_set = procedure_set = false;
            closing = false;
            connect_adv_running = false;
        }
        was_connected = linked;
        struct bt_conn *conn = peer_ref();
        if (conn) {
            if (!roles_disabled) {
                const struct bt_le_cs_set_default_settings_param disabled = {
                    .enable_initiator_role = false, .enable_reflector_role = false,
                    .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
                    .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
                };
                err = bt_le_cs_set_default_settings(conn, &disabled);
                if (err) atomic_set(&need_resync, 1);
                else roles_disabled = true;
            }
            bool authorized = secure_pair_is_authorized(conn);
            if (atomic_cas(&test_drop, 1, 0)) atomic_set(&need_resync, 1);
            const uint32_t watchdog_now = k_uptime_get_32();
            if (!closing && ((elapsed_ms(watchdog_now, connected_at) > 15000 && !authorized) ||
                atomic_get(&need_resync) ||
                (defaults_set && elapsed_ms(watchdog_now,
                    (uint32_t)atomic_get(&last_cs_activity_ms)) >= CS_ACTIVITY_TIMEOUT_MS))) {
                close_session(conn, "failure_or_deadline");
                closing = true;
                close_at = now;
            }
            if (closing && now-close_at > 2000) {
                (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                close_at = now;
            }
            if (!closing && recovery && authorized && !defaults_set) {
                const struct bt_le_cs_set_default_settings_param defaults = {
                    .enable_initiator_role = false, .enable_reflector_role = true,
                    .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
                    .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
                };
                err = bt_le_cs_set_default_settings(conn, &defaults);
                if (err) atomic_set(&need_resync, 1);
                else {
                    defaults_set = true;
                    atomic_set(&last_cs_activity_ms, k_uptime_get_32());
                    printk("AUTH_CS_ALLOWED role=drone level=4\n");
                }
            }
            if (!closing && defaults_set && atomic_get(&config_ready) && !procedure_set) {
                const struct bt_le_cs_set_procedure_parameters_param params = {
                    .config_id = 0, .max_procedure_len = 1000,
                    .min_procedure_interval = 1, .max_procedure_interval = 100,
                    .max_procedure_count = 0, .min_subevent_len = 10000, .max_subevent_len = 75000,
                    .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
                    .phy = BT_LE_CS_PROCEDURE_PHY_1M, .tx_power_delta = 0x80,
                    .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
                    .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
                    .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
                };
                err = bt_le_cs_set_procedure_parameters(conn, &params);
                if (err) atomic_set(&need_resync, 1);
                else procedure_set = true;
            }
            bt_conn_unref(conn);
        }
        now = k_uptime_get_32();
        service_advertisements(now);
        if (restore_sensor_pending) {
            restore_sensor_pending = false;
            printk("IMU_RESTORE cleanup_started=1 independent_worker=1 attempts=1\n");
            (void)k_thread_create(&imu_restore_thread, imu_restore_stack,
                                 K_THREAD_STACK_SIZEOF(imu_restore_stack),
                                 restore_sensor_stop, NULL, NULL, NULL,
                                 K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
        }
        if (plot_mode && (int32_t)(now-next_plot) >= 0) {
            next_plot = now + 50;
            printk("ax_g:%.3f\tay_g:%.3f\taz_g:%.3f\tmag_g:%.3f\tthreshold_g:3\tvalid:%u\n",
                (double)imu.ax_g, (double)imu.ay_g, (double)imu.az_g, (double)imu.magnitude_g, imu.fresh);
        }
        if (atomic_cas(&status_requested, 1, 0) || (!plot_mode && (int32_t)(now-next_status) >= 0)) {
            next_status = now + 5000;
            printk("STATUS role=drone recovery=%u owner=%u linked=%u cs=%u cs_secure=%u beacon_period_ms=1000 adv_attempts=%u adv_events=%ld resync=%u imu_armed=%u samples=%u led=%u imu_restore_busy=%u cs_aborts=%ld cs_good=%ld abort_procedure=%ld abort_subevent=%ld cs_activity_age_ms=%u cs_result_age_ms=%u uptime_ms=%u\n",
                recovery, secure_pair_has_owner(), linked, (unsigned)atomic_get(&stream_on),
                (unsigned)atomic_get(&cs_secured), adv_attempts, (long)atomic_get(&adv_completions),
                resync_count, imu.armed, imu.samples, nu54_led_mask(),
                (unsigned)atomic_get(&imu_restore_busy),
                (long)atomic_get(&cs_aborts), (long)atomic_get(&cs_good_subevents),
                (long)atomic_get(&last_procedure_abort), (long)atomic_get(&last_subevent_abort),
                elapsed_ms(now, (uint32_t)atomic_get(&last_cs_activity_ms)),
                elapsed_ms(now, (uint32_t)atomic_get(&last_result_ms)), now);
            if (!recovery) printk("IMU ready=%u fresh=%u part=%u age_ms=%u mag_g=%.3f failure=%s\n",
                imu.ready, imu.fresh, imu.part, imu.age_ms, (double)imu.magnitude_g, imu.failure);
        }
        k_sleep(K_MSEC(5));
    }
}
