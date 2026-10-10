# Third-party notices

This repository preserves mixed, file-specific licenses. It does not grant a new blanket MIT license or replace any upstream notice. Consult the original texts and source headers before redistribution.

## Nordic Semiconductor

The drone and searcher applications contain code derived from Nordic's Channel Sounding RAS reflector and initiator samples. Files marked `LicenseRef-Nordic-5-Clause` retain that license, including the Nordic IC use restriction. The original text is in [LICENSE-Nordic.txt](LICENSE-Nordic.txt). Nordic nRF Connect SDK v3.4.1 and its toolchain are external dependencies and are not bundled here.

## NUCODE / Zephyr board definition

`board_root/boards/nucode/nu54dk` comes from [NUCODE NU54DK_Zephyr_DTS](https://github.com/Nucode01/NU54DK_Zephyr_DTS), revision `fe65f2f0880bd05b32e562d9bf1ee59142b4f4d3`. The board extension retains its [LICENSE](board_root/LICENSE), [NOTICE](board_root/NOTICE), and [Apache-2.0 text](board_root/LICENSES/Apache-2.0.txt). See the NOTICE for the underlying Nordic/Zephyr attribution.

## SH2 IMU driver

`common/vendor/sh2` was taken from the SH2 driver included with SparkFun BNO08x Cortex Based IMU 1.0.6. Its origins include CEVA/Hillcrest SH2 and the acknowledgments in the bundled notices:

- [SparkFun BNO08x library](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library)
- [CEVA SH2](https://github.com/ceva-dsp/sh2)
- [Preserved LICENSE.md](common/vendor/sh2/LICENSE.md)
- [Preserved NOTICE.txt](common/vendor/sh2/NOTICE.txt)

Existing local driver modifications are preserved: a bounded two-second product query and release of the previous SHTP session before reopening. Their source comments remain in place. Individual file copyright notices and terms still apply.

## Other source

Files with `SPDX-License-Identifier: Apache-2.0` retain that designation. Scripts and documentation without a license header are not assigned a new license by this notice. Manufacturer names identify compatibility and source provenance, not endorsement.
