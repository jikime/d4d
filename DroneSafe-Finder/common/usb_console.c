/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_console.h"
#include "secure_pair.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN), "Reliable USB command RX needs UART IRQ");

/* Dedicated IRQ-buffered input avoids the shell, input echo, and command histories.
 * The NU54's onboard USB probe exposes this target UART as the trusted VCOM.
 * Outgoing printk MUST remain wired UART-only (not BLE or network logging).
 */
static const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static void (*application_command)(const char *);
static atomic_t started;
static atomic_t rx_overflow;
RING_BUF_DECLARE(console_rx, 512);
K_SEM_DEFINE(console_rx_ready, 0, 1);
K_THREAD_STACK_DEFINE(console_stack, 3072);
static struct k_thread console_thread;

static void wipe_line(char *line, size_t length)
{
	volatile char *p = line;
	while (length--) {
		*p++ = 0;
	}
}

static void console_rx_irq(const struct device *dev, void *context)
{
	ARG_UNUSED(context);
	if (!uart_irq_update(dev)) {
		return;
	}
	if (uart_err_check(dev) > 0) {
		atomic_set(&rx_overflow, 1);
	}
	while (uart_irq_rx_ready(dev)) {
		uint8_t bytes[32];
		int received = uart_fifo_read(dev, bytes, sizeof(bytes));
		if (received <= 0) {
			break;
		}
		if (ring_buf_put(&console_rx, bytes, (uint32_t)received) != (uint32_t)received) {
			atomic_set(&rx_overflow, 1);
		}
		wipe_line((char *)bytes, sizeof(bytes));
		k_sem_give(&console_rx_ready);
	}
}

static bool pop_byte(uint8_t *value)
{
	uint8_t *claimed;
	if (ring_buf_get_claim(&console_rx, &claimed, 1) != 1) {
		return false;
	}
	*value = *claimed;
	*claimed = 0; /* Retire OOB bytes before this slot is returned to the ISR. */
	(void)ring_buf_get_finish(&console_rx, 1);
	return true;
}

static void console_main(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	char line[192] = {0};
	size_t used = 0;
	bool overflow = false;
	for (;;) {
		(void)k_sem_take(&console_rx_ready, K_MSEC(20));
		if (atomic_cas(&rx_overflow, 1, 0)) {
			wipe_line(line, sizeof(line));
			used = 0;
			overflow = true;
		}
		unsigned char ch;
		while (pop_byte(&ch)) {
			if (atomic_cas(&rx_overflow, 1, 0)) {
				wipe_line(line, sizeof(line));
				used = 0;
				overflow = true;
			}
			if (ch == '\r' || ch == '\n') {
				if (overflow) {
					printk("SP ERR LINE_TOO_LONG\n");
				} else if (used) {
					line[used] = '\0';
					if (!secure_pair_handle_line(line) && application_command) {
						application_command(line);
					}
				}
				wipe_line(line, sizeof(line));
				used = 0;
				overflow = false;
			} else if (ch >= 0x20 && ch <= 0x7e && !overflow) {
				if (used < sizeof(line) - 1) {
					line[used++] = (char)ch;
				} else {
					wipe_line(line, sizeof(line));
					overflow = true;
				}
			} else {
				/* Reject control-character injection rather than rewriting a
				 * malformed command into an accepted registration command. */
				overflow = true;
			}
		}
		secure_pair_poll();
	}
}

int usb_console_start(void (*app_command)(const char *line))
{
	if (!device_is_ready(console)) {
		return -ENODEV;
	}
	if (!atomic_cas(&started, 0, 1)) {
		return -EALREADY;
	}
	application_command = app_command;
	int err = uart_irq_callback_user_data_set(console, console_rx_irq, NULL);
	if (err) {
		atomic_set(&started, 0);
		return err;
	}
	k_thread_create(&console_thread, console_stack, K_THREAD_STACK_SIZEOF(console_stack),
			console_main, NULL, NULL, NULL, 7, 0, K_NO_WAIT);
	k_thread_name_set(&console_thread, "secure_usb_console");
	uart_irq_rx_enable(console);
	return 0;
}
