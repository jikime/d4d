/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NU54_SECURE_PAIR_H_
#define NU54_SECURE_PAIR_H_

#include <stdbool.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call AFTER successful bt_enable(NULL) and settings_load(), before advertising.
 * This module owns the global authentication callbacks and requests L4 from its
 * own connection callback. The application may register its own conn callbacks.
 * Missing/inconsistent ownership settings fail closed but keep USB repair usable.
 */
int secure_pair_init(void);
bool secure_pair_has_owner(void);
bool secure_pair_window_open(void);
/* Advertiser-owner main loop must keep the connectable set enabled while this
 * is true, including SP OPEN preparation before SP ARM. This is ONLY an RF
 * readiness request: unarmed connections/pairing remain unauthorized. */
bool secure_pair_oob_advertising_needed(void);
bool secure_pair_ready_to_connect(void);
bool secure_pair_peer_allowed(const bt_addr_le_t *address);
bool secure_pair_get_peer(bt_addr_le_t *out);
bool secure_pair_is_authorized(struct bt_conn *conn);

/* Set the CONNECTABLE advertising set, not the separate recovery beacon set.
 * Omit this call on the central/legacy advertiser. Set before SP OPEN.
 */
void secure_pair_set_oob_advertiser(struct bt_le_ext_adv *adv);

/* Call regularly from a normal thread (not ISR). Handles enrollment expiry.
 * Do not authorize CS merely on security_changed: first enrollment also needs
 * pairing_complete, persisted owner metadata and an actual bond.
 */
void secure_pair_poll(void);

/* TRUSTED USB/UART INPUT ONLY. Never call from BLE/GATT/radio or echo this line.
 * SP LOCAL emits secret OOB material directly to the trusted USB console ONLY.
 * Caller must wipe its mutable input buffer immediately after this returns.
 * SP ERASE CONFIRM is the sole owner/bond deletion command; never run it on boot.
 */
bool secure_pair_handle_line(const char *line);

/* Compatibility aliases for the main loops developed in parallel. */
static inline bool secure_pair_can_connect(const bt_addr_le_t *addr)
{
	return secure_pair_peer_allowed(addr);
}
static inline bool secure_pair_window_armed(void)
{
	return secure_pair_window_open() && secure_pair_ready_to_connect();
}
static inline bool secure_pair_console_line(const char *line)
{
	return secure_pair_handle_line(line);
}

#ifdef __cplusplus
}
#endif
#endif
