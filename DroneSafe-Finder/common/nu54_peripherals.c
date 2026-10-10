/* SPDX-License-Identifier: Apache-2.0
 * NU54V DK peripherals. Pin mapping from NUCODE NU54DK_Zephyr_DTS.
 * Written for this application; small 5x7 bitmap glyphs are conventional.
 */
#include "nu54_peripherals.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <hal/nrf_nfct.h>

LOG_MODULE_REGISTER(nu54_peripherals, LOG_LEVEL_INF);
static const struct device *const qwiic = DEVICE_DT_GET(DT_NODELABEL(i2c22));
static const struct gpio_dt_spec leds[] = {
    { DEVICE_DT_GET(DT_NODELABEL(gpio2)), 9, GPIO_ACTIVE_HIGH },
    { DEVICE_DT_GET(DT_NODELABEL(gpio1)), 10, GPIO_ACTIVE_HIGH },
    { DEVICE_DT_GET(DT_NODELABEL(gpio2)), 7, GPIO_ACTIVE_HIGH },
    { DEVICE_DT_GET(DT_NODELABEL(gpio1)), 14, GPIO_ACTIVE_HIGH },
};
static const struct gpio_dt_spec button = {
    DEVICE_DT_GET(DT_NODELABEL(gpio1)), 13, GPIO_ACTIVE_LOW | GPIO_PULL_UP
};
static enum nu54_role device_role;
static struct k_spinlock led_lock;
static bool led_armed, led_recovery, peripherals_started;
static int64_t alert_started = -1, pulse_until;
static uint8_t range_mask, current_mask;
static bool last_button, button_latched;
static int64_t button_changed;

static void led_tick(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(led_work, led_tick);

static void write_mask(uint8_t mask)
{
    for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
        (void)gpio_pin_set_dt(&leds[i], (mask >> i) & 1U);
    }
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    current_mask = mask;
    k_spin_unlock(&led_lock, key);
}

static void led_tick(struct k_work *work)
{
    ARG_UNUSED(work);
    const int64_t now = k_uptime_get();
    uint8_t mask = 0;
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    if (device_role == NU54_ROLE_SEARCHER) {
        mask = range_mask;
    } else if (alert_started >= 0 && now - alert_started < 900) {
        mask = (((now - alert_started) / 150) % 2 == 0) ? 15U : 0U;
    } else if (led_recovery) {
        mask = now < pulse_until ? 1U : 0U;
    } else if (led_armed) {
        mask = now % 200 < 150 ? (1U << ((now / 200) % 4)) : 0U;
    }
    k_spin_unlock(&led_lock, key);
    write_mask(mask);
    (void)k_work_reschedule(&led_work, K_MSEC(20));
}

void nu54_leds_service(bool armed, bool recovery)
{
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    led_armed = armed;
    led_recovery = recovery;
    if (!recovery) {
        alert_started = -1;
        pulse_until = 0;
    }
    k_spin_unlock(&led_lock, key);
}

void nu54_leds_impact(void)
{
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    led_armed = false;
    led_recovery = true;
    alert_started = k_uptime_get();
    k_spin_unlock(&led_lock, key);
}

void nu54_leds_advert_pulse(void)
{
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    if (led_recovery) pulse_until = k_uptime_get() + 80;
    k_spin_unlock(&led_lock, key);
}

uint8_t nu54_led_mask(void)
{
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    const uint8_t result = current_mask;
    k_spin_unlock(&led_lock, key);
    return result;
}

uint8_t nu54_range_led_count(float metres)
{
    if (!isfinite(metres) || metres < 0.0f || metres > 10.0f) return 0;
    if (metres > 5.0f) return 1;
    if (metres > 3.0f) return 2;
    if (metres > 1.0f) return 3;
    return 4;
}

static void set_range_leds(float metres)
{
    const uint8_t count = nu54_range_led_count(metres);
    k_spinlock_key_t key = k_spin_lock(&led_lock);
    range_mask = (1U << count) - 1U;
    k_spin_unlock(&led_lock, key);
}

bool nu54_button_pressed(void)
{
    if (!peripherals_started) return false;
    const int read = gpio_pin_get_dt(&button);
    if (read < 0) return false;
    const bool active = read != 0;
    const int64_t now = k_uptime_get();
    if (active != last_button) {
        last_button = active;
        button_changed = now;
    }
    if (!active) button_latched = false;
    if (active && !button_latched && now - button_changed >= 40) {
        button_latched = true;
        return true;
    }
    return false;
}

const struct device *nu54_qwiic_bus(void) { return qwiic; }

int nu54_qwiic_prepare(void)
{
    /* Runtime pad ownership, not a persistent UICR / debug protection write. */
#if defined(NRF_NFCT) && defined(NFCT_PADCONFIG_ENABLE_Msk)
    NRF_NFCT->PADCONFIG = NFCT_PADCONFIG_ENABLE_Disabled << NFCT_PADCONFIG_ENABLE_Pos;
#endif
    if (!device_is_ready(qwiic)) return -ENODEV;
    int err = i2c_configure(qwiic, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
    if (err) return err;
    /* A firmware reset during an IMU transfer can leave SDA low. The driver's
     * bounded bus-clear emits clocks/STOP only; it does not configure 0x6a. */
    err = i2c_recover_bus(qwiic);
    if (err && err != -ENOSYS && err != -ENOTSUP) {
        LOG_WRN("QWIIC recovery error %d", err);
        return err;
    }
    return 0;
}

/* Explicit SSD1306 128x64 I2C, no Arduino display dependency. */
K_MUTEX_DEFINE(display_lock);
static uint8_t framebuffer[1024];
static bool display_ready, last_range_valid;
static float last_range;
static uint8_t display_address;
static uint32_t display_errors;
static const char glyph_order[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-/? m";
static const uint8_t glyphs[][5] = {
    {0x3e,0x51,0x49,0x45,0x3e}, {0,0x42,0x7f,0x40,0},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3c,0x4a,0x49,0x49,0x30}, {1,0x71,9,5,3},
    {0x36,0x49,0x49,0x49,0x36}, {6,0x49,0x49,0x29,0x1e},
    {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36},
    {0x3e,0x41,0x41,0x41,0x22}, {0x7f,0x41,0x41,0x22,0x1c},
    {0x7f,0x49,0x49,0x49,0x41}, {0x7f,9,9,9,1},
    {0x3e,0x41,0x49,0x49,0x7a}, {0x7f,8,8,8,0x7f},
    {0,0x41,0x7f,0x41,0}, {0x20,0x40,0x41,0x3f,1},
    {0x7f,8,0x14,0x22,0x41}, {0x7f,0x40,0x40,0x40,0x40},
    {0x7f,2,0x0c,2,0x7f}, {0x7f,4,8,0x10,0x7f},
    {0x3e,0x41,0x41,0x41,0x3e}, {0x7f,9,9,9,6},
    {0x3e,0x41,0x51,0x21,0x5e}, {0x7f,9,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {1,1,0x7f,1,1},
    {0x3f,0x40,0x40,0x40,0x3f}, {0x1f,0x20,0x40,0x20,0x1f},
    {0x3f,0x40,0x38,0x40,0x3f}, {0x63,0x14,8,0x14,0x63},
    {7,8,0x70,8,7}, {0x61,0x51,0x49,0x45,0x43},
    {8,8,8,8,8}, {0x20,0x10,8,4,2}, {2,1,0x51,9,6},
    {0,0,0,0,0}, {0x7c,4,0x18,4,0x78},
};
BUILD_ASSERT(sizeof(glyph_order) - 1 == ARRAY_SIZE(glyphs));

static void pixel(unsigned x, unsigned y)
{
    if (x < 128 && y < 64) framebuffer[(y / 8) * 128 + x] |= 1U << (y % 8);
}

static void draw_text(unsigned x, unsigned y, const char *text, unsigned scale)
{
    for (; text && *text && x + 5 * scale <= 128; ++text, x += 6 * scale) {
        char ch = *text;
        if (ch >= 'a' && ch <= 'z' && ch != 'm') ch -= 'a' - 'A';
        const char *found = strchr(glyph_order, ch);
        unsigned index = found ? (unsigned)(found - glyph_order) : 38U;
        for (unsigned col = 0; col < 5; ++col) {
            for (unsigned row = 0; row < 7; ++row) {
                if (!(glyphs[index][col] & (1U << row))) continue;
                for (unsigned dx = 0; dx < scale; ++dx)
                    for (unsigned dy = 0; dy < scale; ++dy)
                        pixel(x + col * scale + dx, y + row * scale + dy);
            }
        }
    }
}

static int commands(const uint8_t *data, size_t length)
{
    uint8_t packet[32];
    if (length > sizeof(packet) - 1) return -EINVAL;
    packet[0] = 0;
    memcpy(packet + 1, data, length);
    return i2c_write(qwiic, packet, length + 1, display_address);
}

static void flush_display(void)
{
    if (!display_ready) return;
    const uint8_t position[] = {0x21,0,127,0x22,0,7};
    int err = commands(position, sizeof(position));
    uint8_t packet[33];
    packet[0] = 0x40;
    for (size_t offset = 0; !err && offset < sizeof(framebuffer); offset += 32) {
        memcpy(packet + 1, framebuffer + offset, 32);
        err = i2c_write(qwiic, packet, sizeof(packet), display_address);
    }
    if (err) {
        ++display_errors;
        LOG_WRN("OLED frame I2C failure %d (count %u)", err, display_errors);
    }
}

static bool valid_range(float metres)
{
    return isfinite(metres) && metres >= 0.0f && metres <= 9999.0f;
}

static void render(float metres, const char *status, const char *detail)
{
    memset(framebuffer, 0, sizeof(framebuffer));
    draw_text(0, 0, "NU54 CS / UNCAL", 1);
    for (unsigned x = 0; x < 128; ++x) pixel(x, 11);
    char number[8] = "-";
    if (valid_range(metres)) snprintf(number, sizeof(number), "%u", (unsigned)floorf(metres + 0.5f));
    draw_text(0, 18, number, 3);
    draw_text(112, 25, "m", 2);
    draw_text(0, 46, status, 1);
    draw_text(0, 56, detail, 1);
    flush_display();
}

void nu54_ui_measuring(void)
{
    k_mutex_lock(&display_lock, K_FOREVER);
    render(last_range_valid ? last_range : NAN,
           last_range_valid ? "LAST / MEASURING" : "MEASURING",
           "WAITING NEW RESULT");
    /* Do not present a historical value as current proximity on the LEDs. */
    set_range_leds(NAN);
    k_mutex_unlock(&display_lock);
}

void nu54_ui_range_wait(uint32_t age_ms)
{
    k_mutex_lock(&display_lock, K_FOREVER);
    char detail[24];
    if (last_range_valid) snprintf(detail, sizeof(detail), "AGE %uS / LINK OK", age_ms / 1000);
    else snprintf(detail, sizeof(detail), "WAITING NEW RESULT");
    render(last_range_valid ? last_range : NAN,
           last_range_valid ? "LAST / RETRYING" : "LINK OK / WAIT RANGE", detail);
    /* Keep LEDs only briefly; a stale estimate is not current proximity. */
    set_range_leds(last_range_valid && age_ms < 3000 ? last_range : NAN);
    k_mutex_unlock(&display_lock);
}

void nu54_ui_range(float metres)
{
    if (!valid_range(metres)) { nu54_ui_no_signal("INVALID RANGE"); return; }
    k_mutex_lock(&display_lock, K_FOREVER);
    last_range = metres;
    last_range_valid = true;
    render(metres, "FILTERED ESTIMATE", "UNCALIBRATED");
    set_range_leds(metres);
    k_mutex_unlock(&display_lock);
}

void nu54_ui_no_signal(const char *reason)
{
    k_mutex_lock(&display_lock, K_FOREVER);
    last_range_valid = false;
    render(NAN, "NO SIGNAL / NO RANGE", reason);
    set_range_leds(NAN);
    k_mutex_unlock(&display_lock);
}

void nu54_ui_status(const char *status)
{
    k_mutex_lock(&display_lock, K_FOREVER);
    last_range_valid = false;
    render(NAN, status, "NO CURRENT RANGE");
    set_range_leds(NAN);
    k_mutex_unlock(&display_lock);
}

bool nu54_ui_ready(void) { return display_ready; }
uint32_t nu54_ui_io_failures(void) { return display_errors; }

static int display_init(void)
{
    /* A command-control byte without payload is a non-configuring ACK probe. */
    const uint8_t control = 0;
    const bool at3c = i2c_write(qwiic, &control, 1, 0x3c) == 0;
    const bool at3d = i2c_write(qwiic, &control, 1, 0x3d) == 0;
    if (at3c == at3d) {
        LOG_ERR("OLED expected one responder at 0x3c/0x3d, got %d/%d", at3c, at3d);
        return -ENODEV;
    }
    display_address = at3c ? 0x3c : 0x3d;
    const uint8_t init[] = {0xae,0xd5,0x80,0xa8,0x3f,0xd3,0,0x40,0x8d,0x14,
                           0x20,0,0xa1,0xc8,0xda,0x12,0x81,0x7f,0xd9,0xf1,
                           0xdb,0x40,0xa4,0xa6,0x2e,0xaf};
    int err = commands(init, sizeof(init));
    if (err) return err;
    display_ready = true;
    nu54_ui_status("WAITING FOR DRONE");
    LOG_INF("OLED SSD1306 128x64 initialized at 0x%02x; pixels need visual check", display_address);
    return 0;
}

int nu54_peripherals_init(enum nu54_role role)
{
    device_role = role;
    for (size_t i = 0; i < ARRAY_SIZE(leds); ++i) {
        if (!gpio_is_ready_dt(&leds[i])) return -ENODEV;
        int err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
        if (err) return err;
    }
    if (!gpio_is_ready_dt(&button)) return -ENODEV;
    int err = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (err) return err;
    peripherals_started = true;
    (void)k_work_reschedule(&led_work, K_NO_WAIT);
    err = nu54_qwiic_prepare();
    if (err) return err;
    return role == NU54_ROLE_SEARCHER ? display_init() : 0;
}
