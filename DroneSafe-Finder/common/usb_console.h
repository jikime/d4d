/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NU54_USB_CONSOLE_H_
#define NU54_USB_CONSOLE_H_

/* Start the dedicated, non-echoing UART-IRQ / console-parser thread.
 * Requires CONFIG_UART_INTERRUPT_DRIVEN=y; owns this UART's RX callback.
 * SP commands are consumed by
 * secure_pair first. The optional app handler must only enqueue/set atomic flags;
 * it runs on this thread, not on the main Bluetooth state-machine thread.
 * Use the wired NU54 VCOM console only. Do not bridge this input over radio.
 */
int usb_console_start(void (*app_command)(const char *line));

#endif
