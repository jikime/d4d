/* SPDX-License-Identifier: Apache-2.0
 * NU54 single-owner LESC OOB enrollment. Written for NCS v3.4.1 / sdk-zephyr
 * ncs-v3.4.1 public APIs; no Bluetooth stack implementation is copied here.
 * Never log OOB values, private keys, LTKs, or USB command bodies.
 */
#include "secure_pair.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_SMP_SC_ONLY), "Authenticated SC-only is required");
BUILD_ASSERT(IS_ENABLED(CONFIG_BT_SMP_APP_PAIRING_ACCEPT), "Pairing gate is required");
BUILD_ASSERT(IS_ENABLED(CONFIG_BT_SETTINGS), "Persistent Bluetooth bonds are required");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_BONDING_REQUIRED),
	     "New bonds are required by pairing_accept, not by reconnect security requests");
BUILD_ASSERT(CONFIG_BT_MAX_CONN == 1, "Enrollment module supports one connection");
BUILD_ASSERT(CONFIG_BT_MAX_PAIRED == 1, "Only the registered owner may be bonded");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_USE_DEBUG_KEYS), "Debug keys are forbidden");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_STORE_DEBUG_KEYS), "Debug bonds are forbidden");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_LOG_SNIFFER_INFO), "Key logging is forbidden");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_SMP_LOG_LEVEL_DBG), "SMP debug may expose secrets");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_KEYS_LOG_LEVEL_DBG), "Key debug logging is forbidden");
BUILD_ASSERT(!IS_ENABLED(CONFIG_LOG_PRINTK), "USB OOB must not enter the logging subsystem");
BUILD_ASSERT(IS_ENABLED(CONFIG_PRINTK_SYNC), "USB protocol lines must not interleave");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_KEYS_OVERWRITE_OLDEST), "Never replace an owner automatically");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_SMP_ALLOW_UNAUTH_OVERWRITE), "Never downgrade a bond");

#define ENROLLMENT_MS 120000
#define OOB_ADVERT_READY_MS 5000
#define OWNER_VERSION 1
#define OWNER_SETTING "nu54_pair/owner"

K_MUTEX_DEFINE(pair_lock);
static bool initialized;
static bool owner_present;
static bool storage_fault;
static bool window_open;
static bool armed;
static bool local_valid;
static bool remote_valid;
static bool oob_submitted;
static bool oob_preparing;
static uint32_t pairing_attempts;
static uint32_t authentication_failures;
static uint32_t oob_rejections;
static int64_t window_deadline;
static bt_addr_le_t owner_address;
static struct bt_le_oob local_oob;
static struct bt_le_oob remote_oob;
static struct bt_le_ext_adv *oob_advertiser;
static struct bt_conn *active_conn;

/* Volatile writes ensure secret clearing isn't optimized away. */
static void wipe(void *buffer, size_t length)
{
	volatile uint8_t *p = buffer;
	while (length--) {
		*p++ = 0;
	}
}

static bt_addr_le_t canonical_address(const bt_addr_le_t *address)
{
	bt_addr_le_t result = *address;
	if (result.type == BT_ADDR_LE_PUBLIC_ID) {
		result.type = BT_ADDR_LE_PUBLIC;
	} else if (result.type == BT_ADDR_LE_RANDOM_ID) {
		result.type = BT_ADDR_LE_RANDOM;
	}
	return result;
}

static bool address_equal(const bt_addr_le_t *a, const bt_addr_le_t *b)
{
	if (!a || !b) {
		return false;
	}
	bt_addr_le_t ca = canonical_address(a);
	bt_addr_le_t cb = canonical_address(b);
	return bt_addr_le_eq(&ca, &cb);
}

static bool live_window_locked(void)
{
	return window_open && k_uptime_get() < window_deadline && !storage_fault &&
	       !owner_present;
}

static void wipe_oob_locked(void)
{
	wipe(&local_oob, sizeof(local_oob));
	wipe(&remote_oob, sizeof(remote_oob));
	local_valid = false;
	remote_valid = false;
	oob_submitted = false;
}

static void close_window_locked(void)
{
	window_open = false;
	armed = false;
	bt_set_bondable(false);
	bt_le_oob_set_sc_flag(false);
	/* Zephyr retains OOB pointers through pairing. Do not invalidate material
	 * while the active connection might still be executing SMP. */
	if (!active_conn) {
		wipe_oob_locked();
	}
}

struct bond_summary {
	unsigned int count;
	bool owner_found;
};

static void count_bond(const struct bt_bond_info *info, void *context)
{
	struct bond_summary *summary = context;
	++summary->count;
	if (owner_present && address_equal(&info->addr, &owner_address)) {
		summary->owner_found = true;
	}
}

static struct bond_summary bonds_locked(void)
{
	struct bond_summary summary = {0};
	bt_foreach_bond(BT_ID_DEFAULT, count_bond, &summary);
	return summary;
}

static bool owner_bond_locked(void)
{
	return owner_present && !storage_fault &&
	       bt_le_bond_exists(BT_ID_DEFAULT, &owner_address);
}

/* A crash between bond persistence and owner persistence is intentionally not
 * auto-repaired. An orphan/extra bond must never silently become the owner. */
static int owner_settings_set(const char *name, size_t len,
			      settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "owner") != 0) {
		return -ENOENT;
	}
	uint8_t record[8] = {0};
	if (len != sizeof(record) || read_cb(cb_arg, record, sizeof(record)) != (ssize_t)sizeof(record) ||
	    record[0] != OWNER_VERSION || record[1] > BT_ADDR_LE_RANDOM) {
		storage_fault = true;
		/* Preserve fail-closed USB repair instead of making settings_load()
		 * abort before the trusted console can be started. */
		return 0;
	}
	owner_address.type = record[1];
	memcpy(owner_address.a.val, &record[2], sizeof(owner_address.a.val));
	owner_present = true;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(nu54_pair, "nu54_pair", NULL,
			       owner_settings_set, NULL, NULL);

static bool conn_matches_enrollment_locked(struct bt_conn *conn)
{
	struct bt_conn_info info;
	if (!live_window_locked() || !armed || !local_valid || !remote_valid ||
	    conn != active_conn || bt_conn_get_info(conn, &info) ||
	    info.type != BT_CONN_TYPE_LE || info.id != BT_ID_DEFAULT) {
		return false;
	}
	/* remote is the on-air setup address, not an identity substituted after
	 * key distribution. Bind the OOB package to the actual connection peer. */
	return address_equal(info.le.remote, &remote_oob.addr);
}

static bool link_is_l4(const struct bt_conn_info *info)
{
	return info->type == BT_CONN_TYPE_LE && info->id == BT_ID_DEFAULT &&
	       info->state == BT_CONN_STATE_CONNECTED &&
	       info->security.level == BT_SECURITY_L4 &&
	       info->security.enc_key_size == 16 &&
	       (info->security.flags & BT_SECURITY_FLAG_SC) != 0;
}

bool secure_pair_is_authorized(struct bt_conn *conn)
{
	if (!conn) {
		return false;
	}
	struct bt_conn_info info;
	if (bt_conn_get_info(conn, &info) || !link_is_l4(&info)) {
		return false;
	}
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool allowed = initialized && conn == active_conn && owner_bond_locked() &&
		       address_equal(info.le.dst, &owner_address);
	k_mutex_unlock(&pair_lock);
	return allowed;
}

static void disconnect_denied(struct bt_conn *conn)
{
	/* May run inside pairing_failed/security_changed. Calling auth_cancel
	 * there can recursively re-enter SMP failure processing. Disconnecting
	 * ends the untrusted link without re-entering that callback chain. */
	(void)bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static enum bt_security_err pairing_accept(struct bt_conn *conn,
				 const struct bt_conn_pairing_feat *const feat)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	/* Bonding (bit0), SC (bit3), 128-bit key and both-way OOB required.
	 * Never accept a new pairing for an existing owner, including a lost key. */
	bool state_ok = conn_matches_enrollment_locked(conn);
	bool accept = feat && state_ok &&
		      feat->oob_data_flag == 1 && feat->max_enc_key_size == 16 &&
		      (feat->auth_req & 0x09) == 0x09;
	if (accept) {
		++pairing_attempts;
	}
	k_mutex_unlock(&pair_lock);
	printk("SP DIAG PAIR accept=%u state=%u features=%u oob=%u key128=%u sc=%u bond=%u\n",
	       accept, state_ok, feat != NULL, feat && feat->oob_data_flag == 1,
	       feat && feat->max_enc_key_size == 16, feat && (feat->auth_req & 0x08) != 0,
	       feat && (feat->auth_req & 0x01) != 0);
	return accept ? BT_SECURITY_ERR_SUCCESS : BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
}

static void oob_requested(struct bt_conn *conn, struct bt_conn_oob_info *info)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool accept = conn_matches_enrollment_locked(conn) && info &&
		      info->type == BT_CONN_OOB_LE_SC &&
		      info->lesc.oob_config == BT_CONN_OOB_BOTH_PEERS;
	int err = -EPERM;
	if (accept) {
		oob_submitted = true;
		err = bt_le_oob_set_sc_data(conn, &local_oob.le_sc_data,
					 &remote_oob.le_sc_data);
		if (err) {
			oob_submitted = false;
		}
	}
	k_mutex_unlock(&pair_lock);
	printk("SP DIAG OOB_CHECK accept=%u both=%u error=%d\n", accept,
	       info && info->type == BT_CONN_OOB_LE_SC &&
	       info->lesc.oob_config == BT_CONN_OOB_BOTH_PEERS, err);
	if (err) {
		disconnect_denied(conn);
	}
}

static void unexpected_confirmation(struct bt_conn *conn)
{
	/* Just Works fallback is never enrollment authorization. */
	disconnect_denied(conn);
}

static void authentication_cancelled(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	k_mutex_lock(&pair_lock, K_FOREVER);
	close_window_locked();
	k_mutex_unlock(&pair_lock);
}

static const struct bt_conn_auth_cb authentication_callbacks = {
	.pairing_accept = pairing_accept,
	.oob_data_request = oob_requested,
	.pairing_confirm = unexpected_confirmation,
	.cancel = authentication_cancelled,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	struct bt_conn_info info;
	bool success = false;
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (bonded && oob_submitted && conn_matches_enrollment_locked(conn) &&
	    !bt_conn_get_info(conn, &info) && link_is_l4(&info) &&
	    (info.security.flags & BT_SECURITY_FLAG_OOB) != 0 &&
	    bt_le_bond_exists(BT_ID_DEFAULT, info.le.dst)) {
		bt_addr_le_t identity = canonical_address(info.le.dst);
		uint8_t record[8] = {OWNER_VERSION, identity.type};
		memcpy(&record[2], identity.a.val, sizeof(identity.a.val));
		/* Owner metadata contains no key. Stack settings own the encrypted-link
		 * bond keys. A write failure leaves this connection unauthorized. */
		int err = settings_save_one(OWNER_SETTING, record, sizeof(record));
		if (!err) {
			owner_address = identity;
			owner_present = true;
			success = true;
		}
	}
	if (!success) {
		storage_fault = true;
	}
	close_window_locked();
	/* Pairing is complete; the stack no longer needs these one-shot values. */
	wipe_oob_locked();
	k_mutex_unlock(&pair_lock);
	printk(success ? "SP EVENT ENROLLED\n" : "SP EVENT ENROLLMENT_REJECTED\n");
	if (!success) {
		disconnect_denied(conn);
	}
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool submitted = oob_submitted;
	bool current = conn == active_conn;
	bool was_open = live_window_locked();
	++authentication_failures;
	/* ncs-v3.4.1 reports an OOB confirm mismatch through pairing_failed
	 * (AUTH_FAIL); bt_le_oob_set_sc_data itself can return 0 after reporting
	 * that failure. Do not confuse -ENOMEM/API failures with crypto rejection. */
	if (conn == active_conn && oob_submitted && reason == BT_SECURITY_ERR_AUTH_FAIL) {
		++oob_rejections;
	}
	close_window_locked();
	k_mutex_unlock(&pair_lock);
	printk("SP DIAG FAILURE reason=%u submitted=%u current=%u window=%u\n",
	       reason, submitted, current, was_open);
	printk("SP EVENT PAIRING_FAILED\n");
	disconnect_denied(conn);
}

static void bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (id == BT_ID_DEFAULT && owner_present && address_equal(peer, &owner_address)) {
		/* Keep owner metadata: losing the key must not reopen enrollment. */
		storage_fault = true;
	}
	k_mutex_unlock(&pair_lock);
}

static struct bt_conn_auth_info_cb authentication_info = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
	.bond_deleted = bond_deleted,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}
	struct bt_conn_info info;
	bool info_ok = !bt_conn_get_info(conn, &info) && info.type == BT_CONN_TYPE_LE &&
		      info.id == BT_ID_DEFAULT;
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool ready = initialized;
	bool fault = storage_fault;
	bool busy = active_conn != NULL;
	bool owner = owner_present;
	bool bond = owner_bond_locked();
	bool was_open = live_window_locked();
	bool was_armed = armed;
	bool owner_match = info_ok && owner && address_equal(info.le.dst, &owner_address);
	bool enroll_match = info_ok && remote_valid &&
			    address_equal(info.le.remote, &remote_oob.addr);
	bool accept = info_ok && ready && !fault && !busy &&
		      ((bond && owner_match) || (was_open && was_armed && enroll_match));
	if (accept) {
		active_conn = bt_conn_ref(conn);
		oob_submitted = false;
	}
	k_mutex_unlock(&pair_lock);
	printk("SP DIAG CONNECT accept=%u info=%u ready=%u fault=%u busy=%u owner=%u "
	       "bond=%u window=%u armed=%u owner_match=%u enroll_match=%u\n", accept,
	       info_ok, ready, fault, busy, owner, bond, was_open, was_armed,
	       owner_match, enroll_match);
	int security_error = accept ? bt_conn_set_security(conn, BT_SECURITY_L4) : -EPERM;
	printk("SP DIAG SECURITY_REQUEST accepted=%u error=%d\n", accept, security_error);
	if (security_error) {
		disconnect_denied(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (active_conn == conn) {
		bt_conn_unref(active_conn);
		active_conn = NULL;
		/* One OOB package is for one enrollment attempt, not an indefinite
		 * retry credential. USB must explicitly open a fresh attempt. */
		close_window_locked();
		wipe_oob_locked();
	}
	k_mutex_unlock(&pair_lock);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	printk("SP DIAG SECURITY level=%u error=%u\n", level, err);
	if (err || level != BT_SECURITY_L4) {
		disconnect_denied(conn);
	}
}

BT_CONN_CB_DEFINE(secure_pair_connections) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

int secure_pair_init(void)
{
	if (initialized) {
		return -EALREADY;
	}
	bt_set_bondable(false);
	bt_le_oob_set_sc_flag(false);
	int err = bt_conn_auth_cb_register(&authentication_callbacks);
	if (err) {
		return err;
	}
	err = bt_conn_auth_info_cb_register(&authentication_info);
	if (err) {
		(void)bt_conn_auth_cb_register(NULL);
		return err;
	}
	k_mutex_lock(&pair_lock, K_FOREVER);
	struct bond_summary summary = bonds_locked();
	if ((owner_present && (summary.count != 1 || !summary.owner_found)) ||
	    (!owner_present && summary.count != 0)) {
		storage_fault = true;
	}
	initialized = true;
	k_mutex_unlock(&pair_lock);
	return 0;
}

bool secure_pair_has_owner(void)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool result = owner_present;
	k_mutex_unlock(&pair_lock);
	return result;
}

bool secure_pair_window_open(void)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool result = initialized && live_window_locked();
	k_mutex_unlock(&pair_lock);
	return result;
}

bool secure_pair_oob_advertising_needed(void)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool result = initialized && !storage_fault && !owner_present && oob_advertiser &&
		      (oob_preparing || live_window_locked());
	k_mutex_unlock(&pair_lock);
	return result;
}

bool secure_pair_ready_to_connect(void)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool result = initialized && !storage_fault &&
		      (owner_bond_locked() || (live_window_locked() && armed && remote_valid));
	k_mutex_unlock(&pair_lock);
	return result;
}

bool secure_pair_get_peer(bt_addr_le_t *out)
{
	if (!out) {
		return false;
	}
	k_mutex_lock(&pair_lock, K_FOREVER);
	bool result = false;
	if (initialized && owner_bond_locked()) {
		*out = owner_address;
		result = true;
	} else if (initialized && live_window_locked() && armed && remote_valid) {
		*out = remote_oob.addr;
		result = true;
	}
	k_mutex_unlock(&pair_lock);
	return result;
}

bool secure_pair_peer_allowed(const bt_addr_le_t *address)
{
	bt_addr_le_t expected;
	return address && secure_pair_get_peer(&expected) && address_equal(address, &expected);
}

void secure_pair_set_oob_advertiser(struct bt_le_ext_adv *adv)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	/* Do not switch the source address underneath an exported OOB package. */
	if (!window_open && !active_conn && !oob_preparing) {
		oob_advertiser = adv;
	}
	k_mutex_unlock(&pair_lock);
}

void secure_pair_poll(void)
{
	struct bt_conn *abort = NULL;
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (window_open && k_uptime_get() >= window_deadline) {
		close_window_locked();
		if (active_conn && !owner_present) {
			abort = bt_conn_ref(active_conn);
		}
		printk("SP EVENT WINDOW_EXPIRED\n");
	}
	k_mutex_unlock(&pair_lock);
	if (abort) {
		disconnect_denied(abort);
		bt_conn_unref(abort);
	}
}

#if defined(CONFIG_BT_EXT_ADV)
static int wait_for_oob_advertiser(struct bt_le_ext_adv *adv)
{
	int64_t deadline = k_uptime_get() + OOB_ADVERT_READY_MS;
	for (;;) {
		if (!secure_pair_oob_advertising_needed()) {
			return -ECANCELED;
		}
		struct bt_le_ext_adv_info info;
		int err = bt_le_ext_adv_get_info(adv, &info);
		if (err) {
			return err;
		}
		if (info.ext_adv_state == BT_LE_EXT_ADV_STATE_ENABLED) {
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}
		/* The USB command thread waits; the main loop owns start/stop. */
		k_sleep(K_MSEC(25));
	}
}
#endif

static int open_enrollment(void)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	struct bond_summary summary = bonds_locked();
	if (!initialized || storage_fault || owner_present || summary.count || active_conn ||
	    window_open || oob_preparing) {
		k_mutex_unlock(&pair_lock);
		return -EPERM;
	}
	wipe_oob_locked();
	oob_preparing = true;
	struct bt_le_ext_adv *adv = oob_advertiser;
	k_mutex_unlock(&pair_lock);

	/* OOB address generation can issue synchronous HCI commands. Never hold
	 * the application mutex while waiting for the Bluetooth RX thread: an
	 * unrelated connection callback also takes this mutex. Enrollment stays
	 * closed until generation has completed and the state is revalidated. */
	struct bt_le_oob generated = {0};
	int err;
#if defined(CONFIG_BT_EXT_ADV)
	if (adv) {
		/* ncs-v3.4.1 refreshes expired advertiser RPAs only on active
		 * sets. Exporting OOB while this set is stopped can return its old
		 * address, which then changes on first start. Keep the set active
		 * throughout preparation and the open window; pairing stays gated
		 * by explicit ARM plus the selected peer's OOB authentication. */
		err = wait_for_oob_advertiser(adv);
		if (!err) {
			err = bt_le_ext_adv_oob_get_local(adv, &generated);
		}
		if (!err) {
			struct bt_le_ext_adv_info info;
			err = bt_le_ext_adv_get_info(adv, &info);
			if (!err && (info.ext_adv_state != BT_LE_EXT_ADV_STATE_ENABLED ||
				     !address_equal(info.addr, &generated.addr))) {
				err = -ESTALE;
			}
		}
	} else
#else
	ARG_UNUSED(adv);
#endif
	{
		err = bt_le_oob_get_local(BT_ID_DEFAULT, &generated);
	}
	k_mutex_lock(&pair_lock, K_FOREVER);
	oob_preparing = false;
	if (!err && (storage_fault || owner_present || active_conn || window_open)) {
		err = -EPERM;
	}
	if (!err) {
		local_oob = generated;
		local_valid = true;
		window_open = true;
		window_deadline = k_uptime_get() + ENROLLMENT_MS;
	} else {
		wipe_oob_locked();
	}
	k_mutex_unlock(&pair_lock);
	wipe(&generated, sizeof(generated));
	return err;
}

static bool set_peer_line(const char *line)
{
	unsigned int type = 0;
	char address_hex[13] = {0}, random_hex[33] = {0}, confirm_hex[33] = {0};
	char extra = 0;
	struct bt_le_oob candidate = {0};
	bool result = false;
	if (sscanf(line, "SP PEER %2x %12s %32s %32s %c", &type, address_hex,
		   random_hex, confirm_hex, &extra) != 4 || type > BT_ADDR_LE_RANDOM ||
	    strlen(address_hex) != 12 || strlen(random_hex) != 32 || strlen(confirm_hex) != 32 ||
	    hex2bin(address_hex, 12, candidate.addr.a.val, 6) != 6 ||
	    hex2bin(random_hex, 32, candidate.le_sc_data.r, 16) != 16 ||
	    hex2bin(confirm_hex, 32, candidate.le_sc_data.c, 16) != 16) {
		goto out;
	}
	candidate.addr.type = (uint8_t)type;
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (live_window_locked() && local_valid && !armed && !active_conn &&
	    !address_equal(&candidate.addr, &local_oob.addr)) {
		remote_oob = candidate;
		remote_valid = true;
		result = true;
	}
	k_mutex_unlock(&pair_lock);
out:
	wipe(&candidate, sizeof(candidate));
	wipe(address_hex, sizeof(address_hex));
	wipe(random_hex, sizeof(random_hex));
	wipe(confirm_hex, sizeof(confirm_hex));
	return result;
}

static void export_local_usb_only(void)
{
	char address_hex[13], random_hex[33], confirm_hex[33];
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (!live_window_locked() || !local_valid || armed || active_conn) {
		k_mutex_unlock(&pair_lock);
		printk("SP ERR LOCAL_UNAVAILABLE\n");
		return;
	}
	bin2hex(local_oob.addr.a.val, 6, address_hex, sizeof(address_hex));
	bin2hex(local_oob.le_sc_data.r, 16, random_hex, sizeof(random_hex));
	bin2hex(local_oob.le_sc_data.c, 16, confirm_hex, sizeof(confirm_hex));
	/* Exactly one trusted-console write. Never replace with LOG_* or a trace
	 * backend; Register-NU54Pair.ps1 consumes it without printing or saving. */
	printk("SP OOB %u %s %s %s\n", local_oob.addr.type, address_hex, random_hex, confirm_hex);
	wipe(address_hex, sizeof(address_hex));
	wipe(random_hex, sizeof(random_hex));
	wipe(confirm_hex, sizeof(confirm_hex));
	k_mutex_unlock(&pair_lock);
}

static int erase_owner_usb_only(void)
{
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (!initialized || active_conn || oob_preparing) {
		k_mutex_unlock(&pair_lock);
		return -EBUSY;
	}
	close_window_locked();
	/* Keep policy closed while bt_unpair updates the controller's resolving
	 * list. It can wait for Bluetooth RX, so do not hold pair_lock here. */
	storage_fault = true;
	k_mutex_unlock(&pair_lock);
	/* Leave metadata locked if either operation fails. A crash between these
	 * two writes is fail-closed on reboot, not an automatic pairing reset. */
	int err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
	if (!err) {
		err = settings_delete(OWNER_SETTING);
	}
	k_mutex_lock(&pair_lock, K_FOREVER);
	if (!err) {
		owner_present = false;
		storage_fault = false;
		memset(&owner_address, 0, sizeof(owner_address));
	} else {
		storage_fault = true;
	}
	k_mutex_unlock(&pair_lock);
	return err;
}

bool secure_pair_handle_line(const char *line)
{
	if (!line || strncmp(line, "SP ", 3)) {
		return false;
	}
	secure_pair_poll();
	if (!strcmp(line, "SP STATUS")) {
		k_mutex_lock(&pair_lock, K_FOREVER);
		struct bond_summary summary = bonds_locked();
		bool authorized = active_conn && secure_pair_is_authorized(active_conn);
		printk("SP STATUS owner=%u bonds=%u window=%u armed=%u fault=%u authorized=%u "
		       "pair_attempts=%u auth_failures=%u oob_rejections=%u\n",
		       owner_present, summary.count, live_window_locked(), armed, storage_fault,
		       authorized, pairing_attempts, authentication_failures, oob_rejections);
		k_mutex_unlock(&pair_lock);
	} else if (!strcmp(line, "SP OPEN")) {
		printk(open_enrollment() ? "SP ERR OPEN_DENIED\n" : "SP OK OPEN\n");
	} else if (!strcmp(line, "SP LOCAL")) {
		export_local_usb_only();
	} else if (!strncmp(line, "SP PEER ", 8)) {
		printk(set_peer_line(line) ? "SP OK PEER\n" : "SP ERR PEER_REJECTED\n");
	} else if (!strcmp(line, "SP ARM")) {
		k_mutex_lock(&pair_lock, K_FOREVER);
		bool valid = live_window_locked() && local_valid && remote_valid && !active_conn;
		if (valid) {
			armed = true;
			bt_set_bondable(true);
			bt_le_oob_set_sc_flag(true);
		}
		k_mutex_unlock(&pair_lock);
		printk(valid ? "SP OK ARM\n" : "SP ERR ARM_DENIED\n");
	} else if (!strcmp(line, "SP CLOSE")) {
		struct bt_conn *abort = NULL;
		k_mutex_lock(&pair_lock, K_FOREVER);
		close_window_locked();
		if (active_conn && !owner_present) {
			abort = bt_conn_ref(active_conn);
		}
		k_mutex_unlock(&pair_lock);
		if (abort) {
			disconnect_denied(abort);
			bt_conn_unref(abort);
		}
		printk("SP OK CLOSE\n");
	} else if (!strcmp(line, "SP DISCONNECT")) {
		struct bt_conn *abort = NULL;
		k_mutex_lock(&pair_lock, K_FOREVER);
		close_window_locked();
		if (active_conn) {
			abort = bt_conn_ref(active_conn);
		}
		k_mutex_unlock(&pair_lock);
		if (abort) {
			(void)bt_conn_disconnect(abort, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			bt_conn_unref(abort);
		}
		printk("SP OK DISCONNECT\n");
	} else if (!strcmp(line, "SP ERASE CONFIRM")) {
		printk(erase_owner_usb_only() ? "SP ERR ERASE_FAILED\n" : "SP OK ERASE\n");
	} else {
		/* Never echo malformed input: it may contain secret OOB fields. */
		printk("SP ERR COMMAND\n");
	}
	return true;
}
