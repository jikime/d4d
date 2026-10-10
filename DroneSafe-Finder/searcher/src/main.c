/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2026 NU54V recovery bench application contributors
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 *
 * Derived from NCS v3.4.1 channel_sounding/ras_initiator.
 * Standard connected CS + encrypted RAS. No CS Test/raw-radio transport.
 * Only a registered LE Secure Connections authenticated peer may range.
 */
#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/cs_de.h>
#include "secure_pair.h"
#include "usb_console.h"
#include "nu54_peripherals.h"
#include "walk_range_filter.h"
#include "walk_range_filter_test.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nu54_searcher, LOG_LEVEL_INF);

#define CS_CONFIG_ID 0
#define MAX_AP CONFIG_BT_RAS_MAX_ANTENNA_PATHS
#define CHANNEL_INDEX_OFFSET 2
#define TONE_QI_OK_TONE_COUNT_THRESHOLD 15
#define LOCAL_PROCEDURE_MEM ((BT_RAS_MAX_STEPS_PER_PROCEDURE * sizeof(struct bt_le_cs_subevent_step)) + (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))
#define SETUP_TIMEOUT_MS 5000
#define RESULT_TIMEOUT_MS 12000 /* Try CS-only recovery before losing the ACL. */
#define CONNECTION_INTERVAL_UNITS 40 /* 50 ms: room for ACL + 16-ms CS subevent. */
#define PROCEDURE_INTERVAL_EVENTS 20 /* 20 * 50 ms = 1000 ms */

enum event_bit { EV_CONNECTED, EV_DISCONNECTED, EV_MTU, EV_DISCOVERED,
                 EV_FEATURES, EV_CAPABILITIES, EV_CONFIG, EV_CS_SECURITY,
                 EV_PROCEDURE_ENABLED, EV_SUBSCRIBED, EV_PROCEDURE_DISABLED };
static atomic_t events;
static atomic_t session_failed, link_live, peer_found, status_requested, test_drop;
static atomic_t scan_paused, scan_control_changed;
static atomic_t measurement_paused, cs_restart_in_progress, test_cs_restart;
static uint32_t cs_restarts, connection_attempts;
static atomic_t disconnections;
static uint32_t consecutive_restarts, max_result_gap_ms;
static bool have_good_range;
/* Main-thread display filter; never reset merely because the ACL/CS restarts. */
static struct walk_range_filter walk_filter;
static bt_addr_le_t walk_filter_owner;
static bool have_filter_owner, have_filtered_range;
static int64_t last_filtered_ms;
static uint32_t filtered_ranges, limited_ranges, stale_ui_ranges;
static atomic_t beacon_rx, beacon_seen, last_beacon_ms, last_beacon_rssi;
static struct bt_conn *connection; /* Only main owns the create reference. */
/* Callback identity is published atomically; callbacks never read main's
 * mutable owned pointer. The controller/RAS holds callback references. */
static atomic_ptr_t callback_connection;
static atomic_t create_pending;
static bt_addr_le_t candidate;
static uint32_t epoch, local_epoch, ras_feature_bits;
static uint16_t local_counter, local_procedure_counter, last_completed_procedure;
static uint16_t peer_counter;
static uint32_t peer_epoch;
static bool have_local_counter, local_complete, peer_requested;
static bool peer_complete, peer_ready, have_completed_procedure;
static int64_t last_good_ms;
static atomic_ptr_t failure_reason = (void *)"none";
struct range_message { uint32_t epoch; uint16_t counter; float metres; int64_t sample_ms; };
K_MSGQ_DEFINE(range_messages, sizeof(struct range_message), 4, 4);
static struct bt_conn_le_cs_config cs_config;
NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, LOCAL_PROCEDURE_MEM);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, BT_RAS_PROCEDURE_MEM);
/* RAS real-time API resets its output immediately after the callback returns. */
NET_BUF_SIMPLE_DEFINE_STATIC(completed_peer_steps, BT_RAS_PROCEDURE_MEM);
static uint16_t m_n_iqs[MAX_AP][CS_DE_NUM_CHANNELS];
static cs_de_report_t m_cs_de_report;
/* These buffers/counters have one writer: serialized Bluetooth RX callbacks.
 * Main resets them only before publishing a connection or after disconnection.
 * UI consumes immutable messages, never these mutable assembly buffers. */
static uint16_t parse_expected_steps, parse_received_steps;
static bool parse_final_subevent;
/* A lost measurement is not a lost authenticated connection.  These assembly
 * fields, like the buffers above, are written only by Bluetooth RX callbacks. */
static atomic_t cs_restart_requested, measurement_drops, range_successes;
static bool have_seen_local_procedure, local_discarded, measurement_wait_local = true;
static uint16_t newest_local_procedure, peer_request_counter;
static bool parse_measurement_aborted;

/* Returns 2 for the intrinsically ambiguous half-range distance. */
static int counter12_order(uint16_t incoming, uint16_t reference)
{
    uint16_t delta = (incoming - reference) & 0x0fffU;
    return delta == 0 ? 0 : delta == 0x0800U ? 2 : delta < 0x0800U ? 1 : -1;
}

static void discard_peer_assembly(void)
{
    peer_complete = peer_ready = false;
    net_buf_simple_reset(&completed_peer_steps);
    /* latest_peer_steps belongs to the SDK while notifications are active.
     * Its real-time completion path resets it AFTER our callback returns. */
}

/* Call only at a Bluetooth RX stop/enable barrier, or after disconnection. */
static void reset_measurement_assembly(bool reset_watermarks)
{
    have_local_counter = local_complete = local_discarded = false;
    peer_requested = false;
    discard_peer_assembly();
    net_buf_simple_reset(&latest_local_steps);
    measurement_wait_local = true;
    parse_measurement_aborted = false;
    if (reset_watermarks) {
        have_seen_local_procedure = false;
        have_completed_procedure = false;
    }
}

static void measurement_drop(const char *reason)
{
    atomic_inc(&measurement_drops);
    LOG_WRN("MEASUREMENT_DROP reason=%s counter=%u total=%ld acl_retained=1",
            reason, local_counter, (long)atomic_get(&measurement_drops));
}

static void discard_local_measurement(const char *reason)
{
    if (!local_discarded) measurement_drop(reason);
    local_discarded = true;
    local_complete = false;
    have_local_counter = false;
    net_buf_simple_reset(&latest_local_steps);
    if ((peer_ready || peer_complete) && peer_counter == local_counter)
        discard_peer_assembly();
}

static bool current_link(struct bt_conn *conn)
{
    return conn != NULL && conn == atomic_ptr_get(&callback_connection) && atomic_get(&link_live);
}
static bool authorized_link(struct bt_conn *conn)
{
    return current_link(conn) && !atomic_get(&scan_paused) && !atomic_get(&session_failed) &&
           secure_pair_is_authorized(conn);
}
static void abort_session(const char *reason)
{
    if (atomic_cas(&session_failed, 0, 1)) {
        atomic_ptr_set(&failure_reason, (void *)reason);
        LOG_WRN("SESSION_ABORT epoch=%u reason=%s", epoch, reason);
    }
}
static void app_command(const char *line)
{
    if (!strcmp(line, "s") || !strcmp(line, "S") || !strcmp(line, "?"))
        atomic_set(&status_requested, 1);
    else if (!strcmp(line, "drop")) atomic_set(&test_drop, 1);
    else if (!strcmp(line, "retry")) atomic_set(&test_cs_restart, 1);
    else if (!strcmp(line, "pause")) {
        atomic_set(&scan_paused, 1);
        atomic_set(&scan_control_changed, 1);
    } else if (!strcmp(line, "resume")) {
        atomic_clear(&scan_paused);
        atomic_set(&scan_control_changed, 1);
    }
}
static void service_ui(void)
{
    secure_pair_poll();
    if (atomic_get(&scan_paused) && connection) abort_session("usb_pause_scan_only");
    if (atomic_cas(&scan_control_changed, 1, 0)) {
        LOG_INF("SCAN_CONTROL paused=%d passive_scan_retained=1",
                (int)atomic_get(&scan_paused));
        atomic_set(&status_requested, 1);
    }
    if (atomic_cas(&test_drop, 1, 0) && connection) abort_session("usb_resync_test");
    if (atomic_cas(&test_cs_restart, 1, 0) && authorized_link(connection))
        atomic_set(&cs_restart_requested, 1);
    struct range_message message;
    while (k_msgq_get(&range_messages, &message, K_NO_WAIT) == 0) {
        if (authorized_link(connection) && !atomic_get(&measurement_paused) && message.epoch == epoch) {
            const int64_t now = k_uptime_get();
            if (message.sample_ms > now || now-message.sample_ms > WALK_RANGE_MAX_AGE_MS ||
                (have_good_range && message.sample_ms <= last_good_ms)) {
                ++stale_ui_ranges;
                LOG_WRN("RANGE_UI_DROP reason=stale_or_reordered counter=%u", message.counter);
                continue;
            }
            bt_addr_le_t owner;
            if (!secure_pair_get_peer(&owner)) {
                abort_session("filter_owner_unavailable");
                continue;
            }
            if (!have_filter_owner || bt_addr_le_cmp(&walk_filter_owner, &owner)) {
                walk_range_reset(&walk_filter);
                bt_addr_le_copy(&walk_filter_owner, &owner);
                have_filter_owner = true;
                have_filtered_range = false;
            }
            if (have_good_range) {
                uint32_t gap = (uint32_t)(message.sample_ms - last_good_ms);
                if (gap > max_result_gap_ms) max_result_gap_ms = gap;
            }
            /* RF health uses received valid raw results, NOT display-filter
             * acceptance. Smoothing must not cause needless radio restarts. */
            last_good_ms = message.sample_ms;
            have_good_range = true;
            consecutive_restarts = 0;
            struct walk_range_output filtered;
            enum walk_range_state state = walk_range_update(&walk_filter,
                message.metres, message.sample_ms, now, &filtered);
            LOG_INF("RANGE_RAW epoch=%u counter=%u metres=%.3f authenticated=1 sample_ms=%lld filter_state=%u",
                    epoch, message.counter, (double)message.metres,
                    (long long)message.sample_ms, (unsigned)state);
            if (state == WALK_RANGE_READY) {
                ++filtered_ranges;
                if (filtered.limited) ++limited_ranges;
                last_filtered_ms = message.sample_ms;
                have_filtered_range = true;
                nu54_ui_range(filtered.metres); /* Both OLED and LEDs use this value. */
                LOG_INF("RANGE epoch=%u counter=%u metres=%.3f authenticated=1 raw_metres=%.3f median_metres=%.3f limited=%u limit_m=%.3f sample_ms=%lld",
                        epoch, message.counter, (double)filtered.metres,
                        (double)message.metres, (double)filtered.median_metres,
                        (unsigned)filtered.limited, (double)filtered.limit_metres,
                        (long long)message.sample_ms);
            }
        }
    }
    if (atomic_cas(&status_requested, 1, 0)) {
        uint32_t age = atomic_get(&beacon_seen) ?
            k_uptime_get_32() - (uint32_t)atomic_get(&last_beacon_ms) : UINT32_MAX;
        LOG_INF("STATUS epoch=%u linked=%d authorized=%d failed=%d reason=%s paused=%d "
                "beacon_rx=%u last_beacon_age_ms=%u beacon_rssi_dbm=%d "
                "beacon_payload_authenticated=0 ranges=%u drops=%u cs_restarts=%u "
                "connect_attempts=%u disconnections=%u range_age_ms=%u max_gap_ms=%u",
                epoch, (int)atomic_get(&link_live),
                connection ? secure_pair_is_authorized(connection) : 0,
                (int)atomic_get(&session_failed), (const char *)atomic_ptr_get(&failure_reason),
                (int)atomic_get(&scan_paused), (uint32_t)atomic_get(&beacon_rx),
                age, atomic_get(&beacon_seen) ? (int)atomic_get(&last_beacon_rssi) : 127,
                (uint32_t)atomic_get(&range_successes), (uint32_t)atomic_get(&measurement_drops),
                cs_restarts, connection_attempts, (uint32_t)atomic_get(&disconnections),
                (uint32_t)(have_good_range ? k_uptime_get()-last_good_ms : UINT32_MAX),
                max_result_gap_ms);
        LOG_INF("WALK_FILTER speed_mps=%.1f median_n=3 outputs=%u limited=%u stale_ui=%u display_age_ms=%u",
                (double)WALK_RANGE_SPEED_MPS, filtered_ranges, limited_ranges, stale_ui_ranges,
                (uint32_t)(have_filtered_range ? k_uptime_get()-last_filtered_ms : UINT32_MAX));
    }
}
static int wait_event(enum event_bit event, int timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;
    while (k_uptime_get() < deadline) {
        service_ui();
        if (atomic_get(&session_failed) || atomic_test_bit(&events, EV_DISCONNECTED))
            return -ECONNRESET;
        if (atomic_test_and_clear_bit(&events, event)) return 0;
        k_sleep(K_MSEC(20));
    }
    abort_session("setup_timeout");
    return -ETIMEDOUT;
}

static bool m_is_tone_quality_ok(uint16_t num_iqs[CS_DE_NUM_CHANNELS], uint8_t channel_map[10])
{
	uint8_t ok_tones_count = 0;

	for (uint8_t i = 0; i < CS_DE_NUM_CHANNELS; ++i) {
		if (BT_LE_CS_CHANNEL_BIT_GET(channel_map, i + CHANNEL_INDEX_OFFSET) &&
		    num_iqs[i] >= 1) {
			ok_tones_count += 1;
		}
	}
	return (ok_tones_count >= TONE_QI_OK_TONE_COUNT_THRESHOLD);
}

static void cumulate_mean(float *avg, float new_value, uint16_t *N)
{
	float a = 1.0f / (*N);
	float b = 1.0f - a;
	*avg = a * new_value + b * (*avg);
}

static void extract_pcts(cs_de_report_t *p_report, uint8_t channel_index,
			 uint8_t antenna_permutation_index,
			 struct bt_hci_le_cs_step_data_tone_info *local_tone_info,
			 struct bt_hci_le_cs_step_data_tone_info *remote_tone_info)
{

	for (uint8_t tone_index = 0; tone_index < p_report->n_ap; tone_index++) {
		int antenna_path = bt_le_cs_get_antenna_path(p_report->n_ap,
							     antenna_permutation_index, tone_index);
		if (antenna_path < 0 || antenna_path >= MAX_AP || channel_index >= CS_DE_NUM_CHANNELS) {
			LOG_WRN("Invalid antenna path.");
			return;
		}

		if (local_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH ||
		    remote_tone_info[tone_index].quality_indicator !=
			    BT_HCI_LE_CS_TONE_QUALITY_HIGH) {
			return;
		}

		struct bt_le_cs_iq_sample local_iq =
			bt_le_cs_parse_pct(local_tone_info[tone_index].phase_correction_term);
		struct bt_le_cs_iq_sample remote_iq =
			bt_le_cs_parse_pct(remote_tone_info[tone_index].phase_correction_term);

		m_n_iqs[antenna_path][channel_index]++;

		if (m_n_iqs[antenna_path][channel_index] == 1) {
			p_report->iq_tones[antenna_path].i_local[channel_index] = local_iq.i;
			p_report->iq_tones[antenna_path].q_local[channel_index] = local_iq.q;
			p_report->iq_tones[antenna_path].i_remote[channel_index] = remote_iq.i;
			p_report->iq_tones[antenna_path].q_remote[channel_index] = remote_iq.q;
		} else {
			cumulate_mean(&p_report->iq_tones[antenna_path].i_local[channel_index],
				      local_iq.i, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].q_local[channel_index],
				      local_iq.q, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].i_remote[channel_index],
				      remote_iq.i, &m_n_iqs[antenna_path][channel_index]);
			cumulate_mean(&p_report->iq_tones[antenna_path].q_remote[channel_index],
				      remote_iq.q, &m_n_iqs[antenna_path][channel_index]);
		}
	}
}

static void extract_rtt_timings(cs_de_report_t *p_report,
				struct bt_hci_le_cs_step_data_mode_1 *local_rtt_data,
				struct bt_hci_le_cs_step_data_mode_1 *peer_rtt_data)
{
	if (local_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    local_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    local_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE ||
	    peer_rtt_data->packet_quality_aa_check !=
		    BT_HCI_LE_CS_PACKET_QUALITY_AA_CHECK_SUCCESSFUL ||
	    peer_rtt_data->packet_rssi == BT_HCI_LE_CS_PACKET_RSSI_NOT_AVAILABLE ||
	    peer_rtt_data->tod_toa_reflector == BT_HCI_LE_CS_TIME_DIFFERENCE_NOT_AVAILABLE) {
		return;
	}

	if (p_report->role == BT_CONN_LE_CS_ROLE_INITIATOR) {
		p_report->rtt_accumulated_half_ns +=
			local_rtt_data->toa_tod_initiator - peer_rtt_data->tod_toa_reflector;
	} else {
		p_report->rtt_accumulated_half_ns +=
			peer_rtt_data->toa_tod_initiator - local_rtt_data->tod_toa_reflector;
	}

	p_report->rtt_count++;
}

static bool process_ranging_header(struct ras_ranging_header *ranging_header, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;
    if (ranging_header->antenna_paths_mask != BIT(0) ||
        ranging_header->config_id != CS_CONFIG_ID ||
        ranging_header->ranging_counter != local_counter) {
        abort_session("invalid_ras_ranging_header");
        return false;
    }
    p_report->n_ap = 1;
    return true;
}

static bool process_subevent_header(struct ras_subevent_header *header, void *user_data)
{
    ARG_UNUSED(user_data);
    if (parse_final_subevent || parse_received_steps != parse_expected_steps) {
        abort_session("invalid_ras_subevent_order");
        return false;
    }
    if ((header->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_COMPLETE &&
         header->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_INCOMPLETE &&
         header->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_ABORTED) ||
        (header->subevent_done_status != BT_CONN_LE_CS_SUBEVENT_COMPLETE &&
         header->subevent_done_status != BT_CONN_LE_CS_SUBEVENT_ABORTED) ||
        (header->ranging_abort_reason > 3 && header->ranging_abort_reason != 15) ||
        (header->subevent_abort_reason > 3 && header->subevent_abort_reason != 15)) {
        abort_session("invalid_ras_status_or_abort_reason");
        return false;
    }
    if (header->subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED ||
        header->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED) {
        LOG_WRN("REMOTE_CS_ABORT procedure=%u subevent=%u counter=%u acl_retained=1",
                header->ranging_abort_reason, header->subevent_abort_reason, local_counter);
        parse_measurement_aborted = true;
        if (header->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED)
            atomic_set(&cs_restart_requested, 1);
        return false;
    }
    if (header->subevent_done_status != BT_CONN_LE_CS_SUBEVENT_COMPLETE ||
        header->subevent_abort_reason || header->ranging_abort_reason ||
        (header->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_COMPLETE &&
         header->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_INCOMPLETE)) {
        abort_session("invalid_ras_subevent_header");
        return false;
    }
    parse_expected_steps += header->num_steps_reported;
    parse_final_subevent = header->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE;
    return true;
}

static bool process_step_data(struct bt_le_cs_subevent_step *local_step,
			      struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	cs_de_report_t *p_report = (cs_de_report_t *)user_data;

    ++parse_received_steps;
    if (local_step && peer_step && local_step->mode == 0 && peer_step->mode == 0)
        return true; /* Calibration steps do not enter the PBR/RTT estimator. */
    if (!local_step || !peer_step || !local_step->data || !peer_step->data ||
        local_step->mode != peer_step->mode || local_step->channel != peer_step->channel ||
        local_step->channel < CHANNEL_INDEX_OFFSET ||
        local_step->channel - CHANNEL_INDEX_OFFSET >= CS_DE_NUM_CHANNELS) {
        abort_session("invalid_step_metadata");
        return false;
    }
    size_t min_len = 0;
    if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1)
        min_len = sizeof(struct bt_hci_le_cs_step_data_mode_1);
    else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2)
        min_len = sizeof(struct bt_hci_le_cs_step_data_mode_2) +
                  p_report->n_ap * sizeof(struct bt_hci_le_cs_step_data_tone_info);
    else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3)
        min_len = sizeof(struct bt_hci_le_cs_step_data_mode_3) +
                  p_report->n_ap * sizeof(struct bt_hci_le_cs_step_data_tone_info);
    else {
        abort_session("unsupported_step_mode");
        return false;
    }
    if (local_step->data_len < min_len || peer_step->data_len < min_len) {
        abort_session("truncated_step_data");
        return false;
    }

	if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		struct bt_hci_le_cs_step_data_mode_2 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_2 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index, local_step_data->tone_info,
			     peer_step_data->tone_info);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_1) {
		struct bt_hci_le_cs_step_data_mode_1 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_1 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_1 *)peer_step->data;

		extract_rtt_timings(p_report, local_step_data, peer_step_data);
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
		struct bt_hci_le_cs_step_data_mode_3 *local_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)local_step->data;
		struct bt_hci_le_cs_step_data_mode_3 *peer_step_data =
			(struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data;

		extract_pcts(p_report, local_step->channel - CHANNEL_INDEX_OFFSET,
			     local_step_data->antenna_permutation_index, local_step_data->tone_info,
			     peer_step_data->tone_info);

		extract_rtt_timings(p_report,
				    (struct bt_hci_le_cs_step_data_mode_1 *)local_step_data,
				    (struct bt_hci_le_cs_step_data_mode_1 *)peer_step_data);
	}

	return true;
}

/* The SDK's void parser returns before our step callback for a peer-aborted
 * step (RAS Step_Mode bit 7).  Bounded preflight distinguishes that normal RF
 * failure from a malformed/truncated successful report; no estimate is made
 * from either.  Copies of net_buf_simple consume no shared bytes. */
static int preflight_peer_steps(void)
{
    struct net_buf_simple peer = completed_peer_steps;
    struct net_buf_simple local = latest_local_steps;
    if (peer.len < sizeof(struct ras_ranging_header)) return -EINVAL;
    struct ras_ranging_header *rh = net_buf_simple_pull_mem(&peer, sizeof(*rh));
    if (rh->config_id != CS_CONFIG_ID || rh->ranging_counter != local_counter ||
        rh->antenna_paths_mask != BIT(0)) return -EINVAL;
    bool final_seen = false;
    while (peer.len) {
        if (final_seen || peer.len < sizeof(struct ras_subevent_header)) return -EINVAL;
        struct ras_subevent_header *sh = net_buf_simple_pull_mem(&peer, sizeof(*sh));
        if (sh->subevent_done_status == BT_CONN_LE_CS_SUBEVENT_ABORTED ||
            sh->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_ABORTED)
            return 0; /* Existing header callback logs the reason and drops it. */
        if (sh->subevent_done_status != BT_CONN_LE_CS_SUBEVENT_COMPLETE ||
            sh->subevent_abort_reason || sh->ranging_abort_reason ||
            (sh->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_COMPLETE &&
             sh->ranging_done_status != BT_CONN_LE_CS_PROCEDURE_INCOMPLETE)) return -EINVAL;
        final_seen = sh->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE;
        for (unsigned i = 0; i < sh->num_steps_reported; ++i) {
            if (local.len < 3 || peer.len < 1) return -EINVAL;
            uint8_t mode = net_buf_simple_pull_u8(&local);
            (void)net_buf_simple_pull_u8(&local); /* SDK supplies peer channel from local. */
            uint8_t local_len = net_buf_simple_pull_u8(&local);
            uint8_t peer_mode = net_buf_simple_pull_u8(&peer);
            if (!local_len || local_len > local.len) return -EINVAL;
            /* RAS specification: when bit 7 is set, bits 0..6 are not valid
             * mode data. This is an aborted measurement, not a usable step. */
            if (peer_mode & BIT(7)) return 1;
            if (peer_mode != mode) return -EINVAL;
            size_t peer_len = local_len;
            if (mode == 0) peer_len = cs_config.role == BT_CONN_LE_CS_ROLE_INITIATOR ?
                sizeof(struct bt_hci_le_cs_step_data_mode_0_reflector) :
                sizeof(struct bt_hci_le_cs_step_data_mode_0_initiator);
            if (peer_len > peer.len) return -EINVAL;
            net_buf_simple_pull(&local, local_len);
            net_buf_simple_pull(&peer, peer_len);
        }
    }
    return final_seen && local.len == 0 ? 0 : -EINVAL;
}

static void finish_range_if_complete(struct bt_conn *conn)
{
    if (!authorized_link(conn) || atomic_get(&measurement_paused)) return;
    /* RAS notification and local HCI results may arrive in either order. */
    if (!peer_complete || !have_local_counter || !local_complete) return;
    if (peer_epoch != epoch || local_epoch != epoch) {
        abort_session("ras_result_mismatch_or_error");
        return;
    }
    int order = counter12_order(peer_counter, local_counter);
    if (order == 2) { abort_session("ambiguous_ras_counter"); return; }
    if (order < 0) { discard_peer_assembly(); return; }
    if (order > 0) { discard_local_measurement("peer_advanced_before_local"); return; }
    if (!latest_local_steps.len) {
        discard_local_measurement("no_complete_local_steps");
        return;
    }
    int preflight = preflight_peer_steps();
    if (preflight > 0) { discard_local_measurement("peer_step_aborted"); return; }
    if (preflight < 0) { abort_session("invalid_ras_structure"); return; }
    memset(&m_cs_de_report, 0, sizeof(m_cs_de_report));
    m_cs_de_report.role = cs_config.role;
    memset(m_n_iqs, 0, sizeof(m_n_iqs));
    parse_expected_steps = parse_received_steps = 0;
    parse_final_subevent = false;
    parse_measurement_aborted = false;
    bt_ras_rreq_rd_subevent_data_parse(&completed_peer_steps, &latest_local_steps,
        cs_config.role, process_ranging_header, process_subevent_header,
        process_step_data, &m_cs_de_report);
    if (atomic_get(&session_failed)) return;
    if (parse_measurement_aborted) {
        discard_local_measurement("peer_subevent_aborted");
        return;
    }
    /* The SDK parser returns void on malformed data. Do not turn a partially
     * parsed procedure into a seemingly valid distance estimate. */
    if (!parse_final_subevent || parse_expected_steps != parse_received_steps ||
        latest_local_steps.len || completed_peer_steps.len) {
        abort_session("incomplete_ras_parse");
        return;
    }
    for (uint8_t ap = 0; ap < m_cs_de_report.n_ap && ap < MAX_AP; ap++) {
        m_cs_de_report.distance_estimates[ap].ifft = NAN;
        m_cs_de_report.distance_estimates[ap].phase_slope = NAN;
        m_cs_de_report.distance_estimates[ap].rtt = NAN;
        m_cs_de_report.distance_estimates[ap].best = NAN;
        m_cs_de_report.tone_quality[ap] = m_is_tone_quality_ok(m_n_iqs[ap],
            cs_config.channel_map) ? CS_DE_TONE_QUALITY_OK : CS_DE_TONE_QUALITY_BAD;
    }
    cs_de_quality_t quality = cs_de_calc(&m_cs_de_report);
    float metres = NAN;
    if (quality == CS_DE_QUALITY_OK && m_cs_de_report.n_ap == 1 &&
        m_cs_de_report.tone_quality[0] == CS_DE_TONE_QUALITY_OK) {
        metres = m_cs_de_report.distance_estimates[0].best;
        if (!isfinite(metres)) metres = m_cs_de_report.distance_estimates[0].ifft;
    }
    net_buf_simple_reset(&latest_local_steps);
    net_buf_simple_reset(&completed_peer_steps);
    if (!(ras_feature_bits & RAS_FEAT_REALTIME_RD))
        net_buf_simple_reset(&latest_peer_steps);
    local_complete = false;
    peer_complete = peer_ready = false;
    last_completed_procedure = local_procedure_counter;
    have_completed_procedure = true;
    have_local_counter = false; /* Reopen the buffers only after both resets. */
    local_discarded = true; /* Ignore duplicate local callbacks for this counter. */
    if (!isfinite(metres) || metres < 0.0f || metres > 9999.0f) {
        measurement_drop("distance_quality_failed");
        return;
    }
    const struct range_message message = {
        .epoch = epoch, .counter = local_counter, .metres = metres,
        .sample_ms = k_uptime_get(),
    };
    if (k_msgq_put(&range_messages, &message, K_NO_WAIT))
        abort_session("range_queue_full");
    else atomic_inc(&range_successes);
}

static void ranging_data_cb(struct bt_conn *conn, uint16_t counter, int err)
{
    if (!authorized_link(conn) || atomic_get(&measurement_paused)) return;
    if (counter & ~0x0fffU) { abort_session("invalid_ras_counter_bits"); return; }
    if (peer_requested && counter == peer_request_counter) peer_requested = false;
    if (err) {
        if (err == -ENODATA) {
            if (have_local_counter && counter == local_counter)
                discard_local_measurement("ras_missing_segments");
            else measurement_drop("ras_missing_segments");
        } else {
            abort_session("ras_transport_or_format_error");
        }
        return;
    }
    if (measurement_wait_local) return; /* No peer-first data across a restart. */
    int order = counter12_order(counter, bt_ras_rreq_get_ranging_counter(newest_local_procedure));
    if (order == 2) { abort_session("ambiguous_ras_counter"); return; }
    if (order < 0 || (order == 0 && local_discarded)) return;
    if (peer_complete || peer_ready) {
        int peer_order = counter12_order(counter, peer_counter);
        if (peer_order == 2) { abort_session("ambiguous_peer_counter"); return; }
        if (peer_order < 0 || (peer_order == 0 && peer_complete)) return;
    }
    if (latest_peer_steps.len < sizeof(struct ras_ranging_header) +
                                sizeof(struct ras_subevent_header) ||
        latest_peer_steps.len > completed_peer_steps.size) {
        abort_session("ras_peer_buffer_invalid");
        return;
    }
    net_buf_simple_reset(&completed_peer_steps);
    net_buf_simple_add_mem(&completed_peer_steps, latest_peer_steps.data, latest_peer_steps.len);
    peer_counter = counter;
    peer_epoch = epoch;
    peer_complete = true;
    finish_range_if_complete(conn);
}

static void request_peer_if_ready(struct bt_conn *conn)
{
    if (!authorized_link(conn) || atomic_get(&measurement_paused) ||
        (ras_feature_bits & RAS_FEAT_REALTIME_RD) ||
        !peer_ready || !have_local_counter || !local_complete || peer_requested) return;
    if (peer_epoch != epoch || local_epoch != epoch) {
        abort_session("ras_ready_counter_mismatch");
        return;
    }
    int order = counter12_order(peer_counter, local_counter);
    if (order == 2) { abort_session("ambiguous_ready_counter"); return; }
    if (order < 0) { discard_peer_assembly(); return; }
    if (order > 0) { discard_local_measurement("ras_ready_advanced"); return; }
    peer_requested = true;
    peer_request_counter = local_counter;
    int err = bt_ras_rreq_cp_get_ranging_data(conn, &latest_peer_steps,
                                              local_counter, ranging_data_cb);
    if (err) {
        peer_requested = false;
        if (err == -EBUSY || err == -ENODATA)
            discard_local_measurement("ras_get_temporarily_unavailable");
        else abort_session("ras_get_failed");
    }
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
    if (!authorized_link(conn) || atomic_get(&measurement_paused)) return;
    if (!result || result->header.config_id != CS_CONFIG_ID) {
        abort_session("invalid_local_cs_config");
        return;
    }
    uint8_t procedure_status = result->header.procedure_done_status;
    uint8_t subevent_status = result->header.subevent_done_status;
    if ((procedure_status != BT_CONN_LE_CS_PROCEDURE_COMPLETE &&
         procedure_status != BT_CONN_LE_CS_PROCEDURE_INCOMPLETE &&
         procedure_status != BT_CONN_LE_CS_PROCEDURE_ABORTED) ||
        (subevent_status != BT_CONN_LE_CS_SUBEVENT_COMPLETE &&
         subevent_status != BT_CONN_LE_CS_SUBEVENT_ABORTED)) {
        abort_session("invalid_local_cs_status");
        return;
    }
    uint16_t procedure_counter = result->header.procedure_counter;
    uint16_t counter = bt_ras_rreq_get_ranging_counter(procedure_counter);
    uint16_t delta = procedure_counter - newest_local_procedure;
    bool new_procedure = !have_seen_local_procedure || delta != 0;
    if (have_seen_local_procedure) {
        if (delta == 0x8000U) { abort_session("ambiguous_local_counter"); return; }
        if (delta > 0x8000U) return; /* Late HCI result from an older procedure. */
    }
    if (new_procedure) {
        if (have_local_counter) discard_local_measurement("next_procedure_before_pair_complete");
        newest_local_procedure = procedure_counter;
        have_seen_local_procedure = true;
        measurement_wait_local = false;
        if (peer_ready || peer_complete) {
            if (peer_epoch != epoch) { abort_session("invalid_peer_epoch"); return; }
            int order = counter12_order(peer_counter, counter);
            if (order == 2) { abort_session("ambiguous_queued_peer_counter"); return; }
            if (order < 0) discard_peer_assembly();
        }
        local_counter = counter;
        local_procedure_counter = procedure_counter;
        local_epoch = epoch;
        have_local_counter = true;
        local_complete = local_discarded = false;
        net_buf_simple_reset(&latest_local_steps);
    }
    if (procedure_status == BT_CONN_LE_CS_PROCEDURE_ABORTED ||
        subevent_status == BT_CONN_LE_CS_SUBEVENT_ABORTED) {
        LOG_WRN("LOCAL_CS_ABORT procedure=%u subevent=%u counter=%u acl_retained=1",
                result->header.procedure_abort_reason, result->header.subevent_abort_reason,
                procedure_counter);
        if (procedure_status == BT_CONN_LE_CS_PROCEDURE_ABORTED)
            atomic_set(&cs_restart_requested, 1);
        discard_local_measurement("local_cs_aborted");
        return;
    }
    /* One aborted subevent invalidates its entire measurement.  Continuing
     * pieces of that same procedure cannot be mistaken for a new assembly. */
    if (local_discarded || local_complete || !have_local_counter) return;
    if (local_epoch != epoch || local_procedure_counter != procedure_counter) {
        abort_session("invalid_local_epoch_or_counter");
        return;
    }
    if (result->step_data_buf) {
        uint16_t len = result->step_data_buf->len;
        if (len > net_buf_simple_tailroom(&latest_local_steps)) {
            abort_session("step_buffer_full");
            return;
        }
        net_buf_simple_add_mem(&latest_local_steps,
            net_buf_simple_pull_mem(result->step_data_buf, len), len);
    }
    if (procedure_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
        local_complete = true;
        request_peer_if_ready(conn);
        finish_range_if_complete(conn);
    }
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t counter)
{
    if (!authorized_link(conn) || atomic_get(&measurement_paused) || measurement_wait_local) return;
    if (counter & ~0x0fffU) { abort_session("invalid_ready_counter_bits"); return; }
    int order = counter12_order(counter, bt_ras_rreq_get_ranging_counter(newest_local_procedure));
    if (order == 2) { abort_session("ambiguous_ready_counter"); return; }
    if (order < 0 || (order == 0 && local_discarded)) return;
    if (peer_complete || peer_ready) {
        int peer_order = counter12_order(counter, peer_counter);
        if (peer_order == 2) { abort_session("ambiguous_queued_ready_counter"); return; }
        if (peer_order <= 0) return;
    }
    discard_peer_assembly();
    peer_counter = counter;
    peer_epoch = epoch;
    peer_ready = true;
    request_peer_if_ready(conn);
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t counter)
{
    if (!authorized_link(conn) || atomic_get(&measurement_paused)) return;
    if (counter & ~0x0fffU) { abort_session("invalid_overwritten_counter_bits"); return; }
    if (have_local_counter && counter == local_counter)
        discard_local_measurement("ras_data_overwritten");
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    ARG_UNUSED(params);
    if (!authorized_link(conn)) return;
    if (err) abort_session("mtu_failed");
    else atomic_set_bit(&events, EV_MTU);
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
    struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
    uint32_t expected_epoch = (uint32_t)(uintptr_t)context;
    bool accepted = authorized_link(conn) && expected_epoch == epoch;
    int err = accepted ? bt_ras_rreq_alloc_and_assign_handles(dm, conn) : -ECANCELED;
    int release_err = bt_gatt_dm_data_release(dm);
    if (!accepted) return;
    if (err || release_err) abort_session("ras_discovery_assign_failed");
    else atomic_set_bit(&events, EV_DISCOVERED);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
    if ((uint32_t)(uintptr_t)context == epoch && current_link(conn))
        abort_session("ras_service_missing");
}

static void discovery_error_cb(struct bt_conn *conn, int err, void *context)
{
    ARG_UNUSED(err);
    if ((uint32_t)(uintptr_t)context == epoch && current_link(conn))
        abort_session("ras_discovery_failed");
}

static struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_completed_cb,
    .service_not_found = discovery_service_not_found_cb,
    .error_found = discovery_error_cb,
};

static void ras_features_cb(struct bt_conn *conn, uint32_t features, int err)
{
    if (!authorized_link(conn)) return;
    if (err) abort_session("ras_features_failed");
    else { ras_feature_bits = features; atomic_set_bit(&events, EV_FEATURES); }
}

static void subscription_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_subscribe_params *params)
{
    ARG_UNUSED(params);
    if (!authorized_link(conn)) return;
    if (err) abort_session("ras_subscription_failed");
    else atomic_set_bit(&events, EV_SUBSCRIBED);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    /* A fast controller may invoke connected before bt_conn_le_create returns.
     * Publish that callback's identity while the outgoing creation is pending. */
    if (atomic_get(&create_pending))
        (void)atomic_ptr_cas(&callback_connection, NULL, conn);
    if (conn != atomic_ptr_get(&callback_connection)) return;
    if (err) {
        atomic_set_bit(&events, EV_DISCONNECTED);
        abort_session("connection_failed");
        return;
    }
    atomic_set(&link_live, 1);
    atomic_set_bit(&events, EV_CONNECTED);
    LOG_INF("CONNECTED epoch=%u awaiting_L4_owner_authorization=1", epoch);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (conn != atomic_ptr_get(&callback_connection)) return;
    atomic_inc(&disconnections);
    atomic_clear(&link_live);
    atomic_set_bit(&events, EV_DISCONNECTED);
    LOG_INF("DISCONNECTED epoch=%u reason=0x%02x session_discarded=1", epoch, reason);
    /* The main thread alone releases its bt_conn_le_create() reference. */
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    if (conn != atomic_ptr_get(&callback_connection)) return;
    if (err || level < BT_SECURITY_L4) abort_session("L4_security_failed");
    /* OOB module also verifies the registered owner before is_authorized=true. */
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    ARG_UNUSED(conn); ARG_UNUSED(param);
    return false; /* Keep the 50-ms ACL interval used for the 1-s CS cadence. */
}

static void phy_updated_cb(struct bt_conn *conn, struct bt_conn_le_phy_info *info)
{
    if (current_link(conn) && info)
        LOG_INF("ACL_PHY epoch=%u tx=%u rx=%u", epoch, info->tx_phy, info->rx_phy);
}

static void remote_capabilities_cb(struct bt_conn *conn, uint8_t status,
                                  struct bt_conn_le_cs_capabilities *params)
{
    ARG_UNUSED(params);
    if (!authorized_link(conn)) return;
    if (status) abort_session("cs_capability_failed");
    else atomic_set_bit(&events, EV_CAPABILITIES);
}

static void config_create_cb(struct bt_conn *conn, uint8_t status,
                            struct bt_conn_le_cs_config *config)
{
    if (!authorized_link(conn)) return;
    if (status || !config || config->id != CS_CONFIG_ID) abort_session("cs_config_failed");
    else { cs_config = *config; atomic_set_bit(&events, EV_CONFIG); }
}

static void cs_security_cb(struct bt_conn *conn, uint8_t status)
{
    if (!authorized_link(conn)) return;
    if (status) abort_session("cs_security_failed");
    else atomic_set_bit(&events, EV_CS_SECURITY);
}

static void procedure_enable_cb(struct bt_conn *conn, uint8_t status,
                               struct bt_conn_le_cs_procedure_enable_complete *params)
{
    if (!authorized_link(conn)) return;
    if (status || !params || params->config_id != CS_CONFIG_ID) {
        abort_session("cs_enable_failed_or_disabled");
        return;
    }
    if (!params->state) {
        atomic_set(&measurement_paused, 1);
        reset_measurement_assembly(false); /* Bluetooth RX serialization barrier. */
        atomic_set_bit(&events, EV_PROCEDURE_DISABLED);
        if (!atomic_get(&cs_restart_in_progress)) atomic_set(&cs_restart_requested, 1);
        LOG_INF("CS_DISABLED epoch=%u acl_retained=1", epoch);
        return;
    }
    reset_measurement_assembly(false);
    atomic_clear(&measurement_paused);
    LOG_INF("CS_ENABLED epoch=%u interval_events=%u procedure_count=%u tx_dbm=%d subevent_us=%u",
            epoch, params->procedure_interval, params->procedure_count,
            params->selected_tx_power, params->subevent_len);
    atomic_set_bit(&events, EV_PROCEDURE_ENABLED);
}

BT_CONN_CB_DEFINE(app_conn_callbacks) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .security_changed = security_changed_cb,
    .le_param_req = le_param_req,
    .le_phy_updated = phy_updated_cb,
    .le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
    .le_cs_config_complete = config_create_cb,
    .le_cs_security_enable_complete = cs_security_cb,
    .le_cs_procedure_enable_complete = procedure_enable_cb,
    .le_cs_subevent_data_available = subevent_result_cb,
};

static bool find_ras_uuid(struct bt_data *data, void *user_data)
{
    bool *found = user_data;
    if (data->type != BT_DATA_UUID16_SOME && data->type != BT_DATA_UUID16_ALL)
        return true;
    if (data->data_len & 1U) return true; /* Reject malformed UUID16 lists. */
    for (size_t offset = 0; offset + 1 < data->data_len; offset += 2) {
        if (sys_get_le16(data->data + offset) == BT_UUID_RANGING_SERVICE_VAL) {
            *found = true;
            return false;
        }
    }
    return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                    struct net_buf_simple *ad)
{
    /* Zephyr resolves known RPAs before delivering this callback. Match only
     * the enrolled identity, never an arbitrary RAS advertiser. This is NOT
     * payload authentication/decryption: plain advertisements can be spoofed. */
    if (!secure_pair_can_connect(addr)) return;
    bool has_ras = false;
    bt_data_parse(ad, find_ras_uuid, &has_ras);
    if (type == BT_GAP_ADV_TYPE_ADV_NONCONN_IND && has_ras && secure_pair_has_owner()) {
        atomic_set(&last_beacon_rssi, rssi);
        atomic_set(&last_beacon_ms, (atomic_val_t)k_uptime_get_32());
        atomic_set(&beacon_seen, 1);
        atomic_inc(&beacon_rx); /* Scan reports, not claimed advertising events. */
    }
    if (atomic_ptr_get(&callback_connection) || atomic_get(&create_pending) ||
        atomic_get(&scan_paused) || atomic_get(&peer_found) ||
        (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) ||
        (type == BT_GAP_ADV_TYPE_ADV_IND && !has_ras)) return;
    bt_addr_le_copy(&candidate, addr);
    atomic_set(&peer_found, 1);
}

static int restart_cs_only(void)
{
    if (!authorized_link(connection)) return -EACCES;
    atomic_set(&cs_restart_in_progress, 1);
    atomic_set(&measurement_paused, 1);
    atomic_clear_bit(&events, EV_PROCEDURE_DISABLED);
    atomic_clear_bit(&events, EV_PROCEDURE_ENABLED);
    k_msgq_purge(&range_messages);
    const struct bt_le_cs_procedure_enable_param stop = {.config_id=CS_CONFIG_ID, .enable=0};
    int err = bt_le_cs_procedure_enable(connection, &stop);
    if (!err) err = wait_event(EV_PROCEDURE_DISABLED, SETUP_TIMEOUT_MS);
    if (!err) {
        /* Do not reset the SDK-owned live RAS buffer. Let queued notifications
         * drain while RX callbacks reject samples; retain the full16 watermark. */
        int64_t settle_until = k_uptime_get() + 500;
        while (authorized_link(connection) && k_uptime_get() < settle_until) {
            service_ui(); k_sleep(K_MSEC(20));
        }
        if (!authorized_link(connection)) err = -ECONNRESET;
    }
    if (!err) {
        const struct bt_le_cs_procedure_enable_param start = {.config_id=CS_CONFIG_ID, .enable=1};
        err = bt_le_cs_procedure_enable(connection, &start);
        if (!err) err = wait_event(EV_PROCEDURE_ENABLED, SETUP_TIMEOUT_MS);
    }
    atomic_clear(&cs_restart_in_progress);
    ++cs_restarts;
    LOG_INF("CS_RESTART epoch=%u count=%u result=%d acl_retained=%u", epoch,
            cs_restarts, err, current_link(connection));
    return err;
}

static int configure_session(void)
{
    int err;
    /* Repeat the L4 + registered-owner gate before every CS control phase. */
    if (!authorized_link(connection)) return -EACCES;
    struct bt_conn_info link_info;
    if (!bt_conn_get_info(connection, &link_info) && link_info.le.phy)
        LOG_INF("ACL_CONFIG epoch=%u interval_us=%u tx_phy=%u rx_phy=%u", epoch,
                link_info.le.interval_us, link_info.le.phy->tx_phy, link_info.le.phy->rx_phy);
    static struct bt_gatt_exchange_params mtu = { .func = mtu_exchange_cb };
    err = bt_gatt_exchange_mtu(connection, &mtu);
    if (err || wait_event(EV_MTU, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb,
                          (void *)(uintptr_t)epoch);
    if (err || wait_event(EV_DISCOVERED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    if (!authorized_link(connection)) return -EACCES;
    const struct bt_le_cs_set_default_settings_param defaults = {
        .enable_initiator_role = true, .enable_reflector_role = false,
        .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
        .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
    };
    err = bt_le_cs_set_default_settings(connection, &defaults);
    if (err) return err;
    err = bt_ras_rreq_read_features(connection, ras_features_cb);
    if (err || wait_event(EV_FEATURES, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    err = bt_ras_rreq_subscription_change_cb_register(connection, subscription_cb);
    if (err) return err;
    if (ras_feature_bits & RAS_FEAT_REALTIME_RD) {
        err = bt_ras_rreq_realtime_rd_subscribe(connection, &latest_peer_steps, ranging_data_cb);
        if (err || wait_event(EV_SUBSCRIBED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    } else {
        err = bt_ras_rreq_rd_overwritten_subscribe(connection, ranging_data_overwritten_cb);
        if (err || wait_event(EV_SUBSCRIBED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
        err = bt_ras_rreq_rd_ready_subscribe(connection, ranging_data_ready_cb);
        if (err || wait_event(EV_SUBSCRIBED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
        err = bt_ras_rreq_on_demand_rd_subscribe(connection);
        if (err || wait_event(EV_SUBSCRIBED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
        err = bt_ras_rreq_cp_subscribe(connection);
        if (err || wait_event(EV_SUBSCRIBED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    }
    if (!authorized_link(connection)) return -EACCES;
    err = bt_le_cs_read_remote_supported_capabilities(connection);
    if (err || wait_event(EV_CAPABILITIES, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    struct bt_le_cs_create_config_params config = {
        .id = CS_CONFIG_ID,
        .mode = BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1,
        .min_main_mode_steps = 2, .max_main_mode_steps = 5,
        .main_mode_repetition = 0, .mode_0_steps = 3,
        .role = BT_CONN_LE_CS_ROLE_INITIATOR,
        .rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
        .cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
        .channel_map_repetition = 1,
        .channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
        .ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT, .ch3c_jump = 2,
    };
    bt_le_cs_set_valid_chmap_bits(config.channel_map);
    if (!authorized_link(connection)) return -EACCES;
    err = bt_le_cs_create_config(connection, &config,
                                 BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
    if (err || wait_event(EV_CONFIG, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    if (!authorized_link(connection)) return -EACCES;
    err = bt_le_cs_security_enable(connection);
    if (err || wait_event(EV_CS_SECURITY, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    const struct bt_le_cs_set_procedure_parameters_param procedure = {
        .config_id = CS_CONFIG_ID,
        .max_procedure_len = 800, /* 500 ms; below the 1-second cadence. */
        .min_procedure_interval = PROCEDURE_INTERVAL_EVENTS,
        .max_procedure_interval = PROCEDURE_INTERVAL_EVENTS,
        .max_procedure_count = 0, /* No lifetime/procedure-count limit. */
        .min_subevent_len = 16000, .max_subevent_len = 16000,
        .tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
        .phy = BT_LE_CS_PROCEDURE_PHY_1M, .tx_power_delta = 0x80,
        .preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
        .snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
        .snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
    };
    if (!authorized_link(connection)) return -EACCES;
    err = bt_le_cs_set_procedure_parameters(connection, &procedure);
    if (err) return err;
    const struct bt_le_cs_procedure_enable_param enable = { .config_id = CS_CONFIG_ID, .enable = 1 };
    if (!authorized_link(connection)) return -EACCES;
    err = bt_le_cs_procedure_enable(connection, &enable);
    if (err || wait_event(EV_PROCEDURE_ENABLED, SETUP_TIMEOUT_MS)) return err ? err : -ETIMEDOUT;
    return 0;
}

static void clear_session(void)
{
    atomic_clear(&events);
    atomic_clear(&session_failed);
    atomic_clear(&link_live);
    atomic_clear(&measurement_paused);
    atomic_clear(&cs_restart_in_progress);
    atomic_clear(&cs_restart_requested);
    have_good_range = false;
    have_filtered_range = false; /* Hide disconnected history; retain same-owner filter anchor. */
    consecutive_restarts = 0;
    reset_measurement_assembly(true); /* No live callbacks at a new ACL boundary. */
    atomic_ptr_set(&failure_reason, (void *)"none");
    ras_feature_bits = 0;
    have_local_counter = local_complete = peer_requested = false;
    peer_complete = peer_ready = have_completed_procedure = false;
    peer_epoch = 0;
    local_epoch = 0;
    k_msgq_purge(&range_messages);
    net_buf_simple_reset(&latest_local_steps);
    net_buf_simple_reset(&latest_peer_steps);
    net_buf_simple_reset(&completed_peer_steps);
    memset(&cs_config, 0, sizeof(cs_config));
}

static void close_session(void)
{
    if (!connection) return;
    if (!atomic_get(&session_failed))
        atomic_ptr_set(&failure_reason, (void *)"link_lost_or_authorization_removed");
    atomic_set(&session_failed, 1); /* Reject all late result callbacks immediately. */
    nu54_ui_no_signal((const char *)atomic_ptr_get(&failure_reason));
    if (atomic_get(&link_live)) {
        /* Disconnect is the session boundary: controller discards all CS state,
         * peer sees LL termination or its independent supervision timeout. */
        int err = bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        LOG_INF("SESSION_CLOSE epoch=%u disconnect_result=%d", epoch, err);
    } else if (!atomic_test_bit(&events, EV_DISCONNECTED)) {
        (void)bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
    int64_t deadline = k_uptime_get() + 6000;
    while (!atomic_test_bit(&events, EV_DISCONNECTED) && k_uptime_get() < deadline) {
        secure_pair_poll();
        k_sleep(K_MSEC(20));
    }
    /* Never create a second link while an old link/connection attempt lives.
     * Each retry is bounded; retain the reference until the callback confirms
     * termination, rather than freeing a live connection or rebooting. */
    while (!atomic_test_bit(&events, EV_DISCONNECTED)) {
        LOG_WRN("Disconnect still pending; retrying safely");
        (void)bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        for (unsigned i = 0; i < 50 && !atomic_test_bit(&events, EV_DISCONNECTED); i++) {
            secure_pair_poll();
            k_sleep(K_MSEC(20));
        }
    }
    /* RAS owns its own reference and frees its context in its disconnected
     * callback. Do not race that callback with a second main-thread free. */
    atomic_ptr_clear(&callback_connection);
    bt_conn_unref(connection);
    connection = NULL;
    clear_session();
}

int main(void)
{
    int err = nu54_peripherals_init(NU54_ROLE_SEARCHER);
    if (err) LOG_WRN("Peripheral init %d; Bluetooth remains usable", err);
    /* Isolated synthetic tests of the actual C filter; never feed the display
     * or radio with simulated distances and never modify the live filter. */
    int test_error = walk_range_selftest();
    LOG_INF("WALK_FILTER_SELFTEST pass=%u failed_line=%d", (unsigned)(test_error == 0), test_error);
    if (test_error) { nu54_ui_no_signal("FILTER SELFTEST FAIL"); return 0; }
    nu54_ui_status("BOOT");
    err = bt_enable(NULL);
    if (!err) err = settings_load();
    if (!err) err = secure_pair_init();
    if (err) {
        LOG_ERR("Secure Bluetooth initialization failed: %d", err);
        nu54_ui_no_signal("INIT ERROR");
        return 0; /* Fail closed; never scan or range without pairing storage. */
    }
    err = usb_console_start(app_command);
    if (err) {
        LOG_ERR("Trusted USB console unavailable: %d", err);
        nu54_ui_no_signal("USB ERROR");
        return 0;
    }
    LOG_INF("BOOT NU54_SECURE_SEARCHER_walkfilter1 period_ms=1000 encrypted_RAS=1 CS_TEST=0 tx_request_dbm=%d", CONFIG_BT_CTLR_TX_PWR_DBM);
    while (true) {
        clear_session();
        atomic_clear(&peer_found);
        nu54_ui_no_signal("SEARCHING");
        /* No duplicate filter: repeated independent beacons must be counted. */
        const struct bt_le_scan_param scan = {
            .type = BT_LE_SCAN_TYPE_PASSIVE, .options = BT_LE_SCAN_OPT_NONE,
            .interval = BT_GAP_SCAN_FAST_INTERVAL, .window = BT_GAP_SCAN_FAST_INTERVAL,
        };
        err = bt_le_scan_start(&scan, scan_cb);
        if (err) {
            LOG_WRN("Scan start failed: %d", err);
            for (unsigned i = 0; i < 50; i++) { service_ui(); k_sleep(K_MSEC(20)); }
            continue;
        }
        while (!atomic_get(&peer_found) || atomic_get(&scan_paused)) {
            if (atomic_get(&scan_paused)) atomic_clear(&peer_found);
            service_ui(); k_sleep(K_MSEC(20));
        }
        (void)bt_le_scan_stop();
        if (atomic_get(&scan_paused) || !secure_pair_can_connect(&candidate)) continue;
        ++epoch;
        if (!epoch) ++epoch;
        const struct bt_le_conn_param params = {
            .interval_min = CONNECTION_INTERVAL_UNITS, .interval_max = CONNECTION_INTERVAL_UNITS,
            .latency = 0, .timeout = 600, /* Survive short fades; declare lost after 6 s. */
        };
        if (atomic_get(&scan_paused)) continue;
        atomic_set(&create_pending, 1);
        ++connection_attempts;
        err = bt_conn_le_create(&candidate, BT_CONN_LE_CREATE_CONN, &params, &connection);
        if (!err) (void)atomic_ptr_cas(&callback_connection, NULL, connection);
        atomic_clear(&create_pending);
        if (err) {
            atomic_ptr_clear(&callback_connection);
            connection = NULL;
            LOG_WRN("Connect request failed: %d", err);
            k_sleep(K_MSEC(500));
            continue;
        }
        if (!wait_event(EV_CONNECTED, 5000)) {
            const struct bt_le_cs_set_default_settings_param disabled = {
                .enable_initiator_role = false, .enable_reflector_role = false,
                .cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
                .max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
            };
            err = bt_le_cs_set_default_settings(connection, &disabled);
            if (err) abort_session("preauth_cs_disable_failed");
            int64_t auth_deadline = k_uptime_get() + 15000;
            nu54_ui_status("AUTHENTICATING");
            while (current_link(connection) && !atomic_get(&session_failed) &&
                   !secure_pair_is_authorized(connection) && k_uptime_get() < auth_deadline) {
                service_ui(); k_sleep(K_MSEC(20));
            }
            if (!authorized_link(connection)) abort_session("authorization_timeout_or_failed");
            else {
                err = configure_session();
                if (err) { LOG_WRN("CS setup failed: %d", err); abort_session("cs_setup_failed"); }
                else {
                    last_good_ms = k_uptime_get();
                    int64_t next_ui_measurement = last_good_ms;
                    int64_t next_restart_allowed = last_good_ms;
                    int64_t last_cs_restart_ms = last_good_ms;
                    while (authorized_link(connection)) {
                        service_ui();
                        int64_t now = k_uptime_get();
                        if (now >= next_restart_allowed &&
                            (atomic_get(&cs_restart_requested) ||
                             (now-last_good_ms >= RESULT_TIMEOUT_MS &&
                              now-last_cs_restart_ms >= RESULT_TIMEOUT_MS))) {
                            atomic_clear(&cs_restart_requested);
                            if (++consecutive_restarts > 3) {
                                abort_session("persistent_cs_failure_after_retries"); break;
                            }
                            if (restart_cs_only()) {
                                abort_session("cs_only_restart_failed"); break;
                            }
                            last_cs_restart_ms = k_uptime_get();
                            next_restart_allowed = last_cs_restart_ms + 1000;
                        }
                        if (now >= next_ui_measurement) {
                            nu54_ui_range_wait((uint32_t)(have_filtered_range ?
                                k_uptime_get()-last_filtered_ms : UINT32_MAX));
                            next_ui_measurement = now + 1000;
                        }
                        k_sleep(K_MSEC(20));
                    }
                }
            }
        }
        close_session();
        for (unsigned i = 0; i < 25; i++) { service_ui(); k_sleep(K_MSEC(20)); }
    }
    return 0;
}


