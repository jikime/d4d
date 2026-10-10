/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NU54_IMU_H
#define NU54_IMU_H
#include <stdbool.h>
#include <stdint.h>

#define NU54_IMPACT_THRESHOLD_G 3.0f
#define NU54_IMU_INTERVAL_US 8000U
struct nu54_imu_snapshot {
    float ax_g, ay_g, az_g, magnitude_g, peak_g;
    uint32_t samples, invalid, resets, sequence_gaps, age_ms, part;
    uint8_t address;
    bool ready, fresh, armed;
    const char *failure;
};
/* All IMU APIs are single-threaded. Call from one application thread; no
 * SH2 reentrancy, ISR or Bluetooth callback use. begin/stop may block for
 * bounded sensor transactions; poll normally returns after a packet. */
bool nu54_imu_begin(void);
void nu54_imu_poll(void);
bool nu54_imu_take_impact(void);
/* Disable reports + request sleep; transport success is not proof of current. */
void nu54_imu_stop(void);
struct nu54_imu_snapshot nu54_imu_snapshot(void);
#endif
