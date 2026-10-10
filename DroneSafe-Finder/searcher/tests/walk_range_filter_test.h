/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WALK_RANGE_FILTER_TEST_H
#define WALK_RANGE_FILTER_TEST_H

/* Deterministic, pure-C self-test. No hardware, settings or radio access.
 * Returns zero on success, otherwise the failing assertion's C source line.
 * All tested filter instances are local and never alter the live UI filter. */
int walk_range_selftest(void);

#endif
