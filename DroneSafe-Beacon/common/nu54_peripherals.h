/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NU54_PERIPHERALS_H
#define NU54_PERIPHERALS_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>

enum nu54_role { NU54_ROLE_DRONE, NU54_ROLE_SEARCHER };

/* Thread-context initialization. Uses the NU54-specific P1.02/P1.03 bus only.
 * No PMIC/charger writes, persistent pad changes, or address-wide bus scan. */
int nu54_peripherals_init(enum nu54_role role);
const struct device *nu54_qwiic_bus(void);
int nu54_qwiic_prepare(void);
/* One-shot SW1 press after 40 ms stable, active-low. Poll at least every 20 ms. */
bool nu54_button_pressed(void);
/* LED calls are nonblocking and safe from application/BT callbacks after init.
 * An independent 20 ms work item services the indication, including during
 * bounded sensor initialization. Pulse indicates a scheduled advertisement,
 * not proof that a receiver received it. */
void nu54_leds_service(bool armed, bool recovery);
void nu54_leds_impact(void);
void nu54_leds_advert_pulse(void);
uint8_t nu54_led_mask(void);
uint8_t nu54_range_led_count(float metres);

/* SSD1306 functions perform I2C and must NOT run from ISR/BT RX callbacks.
 * A valid previous number is retained during MEASURING only. Actual failure
 * clears the number to '- m'. Values are rounded integer metres; LED rules
 * use the supplied estimate (the searcher supplies its walking-filter output).
 * No claim of range accuracy or physical pixel check. */
void nu54_ui_measuring(void);
/* A missed distance is not a lost BLE connection. Show age, never fake freshness. */
void nu54_ui_range_wait(uint32_t age_ms);
void nu54_ui_range(float metres);
void nu54_ui_no_signal(const char *reason);
void nu54_ui_status(const char *status);
bool nu54_ui_ready(void);
uint32_t nu54_ui_io_failures(void);

#endif
