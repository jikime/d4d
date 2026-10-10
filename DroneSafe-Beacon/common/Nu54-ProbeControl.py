#!/usr/bin/env python3
"""Exact-probe reset/halt or state check; never flash, erase, recover or unlock.

Used only by Upload-NU54Secure.ps1. prepare is the API equivalent of the
verified pyOCD command `reset halt system`, followed by a HALTED assertion.
check-running does not reset/resume; it checks the ordinary nrf_ocd reset.
--check-dependencies imports libraries but never enumerates or opens probes.
"""
import argparse
import logging
from pathlib import Path
import re
import sys
import time
import traceback

ROLES = ("drone", "searcher")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=ROLES, required=True)
    parser.add_argument("--uid", required=True, help="Complete CMSIS-DAP UID for the physically identified role")
    parser.add_argument("--mode", choices=("prepare", "check-running"), default="prepare")
    parser.add_argument("--frequency", type=int, required=True)
    parser.add_argument("--check-dependencies", action="store_true")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Fa-f0-9]{8,128}", args.uid):
        parser.error("--uid must be the complete hexadecimal probe UID, not a prefix")
    if not 100000 <= args.frequency <= 4000000:
        parser.error("Unsupported SWD frequency")
    import pyocd
    from intelhex import IntelHex
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target
    from pyocd.probe.cmsis_dap_probe import CMSISDAPProbe
    if pyocd.__version__ != "0.42.0":
        raise RuntimeError("Only reviewed pyOCD 0.42.0 is supported")
    if args.check_dependencies:
        print("DEPENDENCIES_VALID pyocd=0.42.0 intelhex=available hardware_access=0")
        return 0
    logging.basicConfig(level=logging.ERROR)

    class NoUserScriptSession(Session):
        def _load_user_script(self):
            return None  # --no-config alone does not suppress pyocd_user.py.

    uid = args.uid.lower()
    probes = ConnectHelper.get_all_connected_probes(
        blocking=False, unique_id=uid, print_wait_message=False)
    exact = [p for p in probes if p.unique_id.lower() == uid]
    if len(exact) != 1 or not isinstance(exact[0], CMSISDAPProbe):
        raise RuntimeError("Exact role CMSIS-DAP probe is not uniquely available")
    options = {
        "target_override": "nrf54l", "auto_unlock": False,
        "connect_mode": "attach", "resume_on_disconnect": False,
        "no_config": True, "project_dir": str(Path(__file__).resolve().parent),
        "frequency": args.frequency, "dap_protocol": "swd",
        "cmsis_dap.prefer_v1": False, "cache.enable_memory": False,
        "cmsis_dap.limit_packets": True, "cmsis_dap.deferred_transfers": False,
        "cache.enable_register": False, "cache.read_code_from_elf": False,
        "pack.debug_sequences.enable": False,
    }
    session = NoUserScriptSession(exact[0], auto_open=False, options=options)
    try:
        print(f"PROBE_CONTROL_STAGE role={args.role} stage=attach", flush=True)
        session.open()
        if session.probe.unique_id.lower() != uid:
            raise RuntimeError("Probe identity changed")
        target = session.target
        if args.mode == "prepare":
            print(f"PROBE_CONTROL_STAGE role={args.role} stage=reset_halt", flush=True)
            target.reset_and_halt(reset_type=Target.ResetType.SYSTEM)
            print(f"PROBE_CONTROL_STAGE role={args.role} stage=halt_check", flush=True)
            if target.get_state() != Target.State.HALTED:
                raise RuntimeError("System reset did not leave the CPU halted")
            marker = f"RESET_HALT_CONFIRMED role={args.role} state=HALTED"
        else:
            deadline = time.monotonic() + 3.0
            while True:
                state = target.get_state()
                if state in (Target.State.RUNNING, Target.State.SLEEPING):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("CPU running/sleeping state not confirmed")
                time.sleep(0.05)
            marker = f"CPU_STARTED_CONFIRMED role={args.role} state={state.name} boot_health_unchecked=1"
    finally:
        print(f"PROBE_CONTROL_STAGE role={args.role} stage=close", flush=True)
        session.close()  # Never resume automatically on success or failure.
    print(marker, flush=True)  # Emit success only after successful probe release.
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("PROBE_CONTROL_FAILED interrupted=1", file=sys.stderr, flush=True)
        sys.exit(130)
    except Exception as exc:
        for frame in traceback.extract_tb(exc.__traceback__):
            print(f"PROBE_TRACE file={Path(frame.filename).name} line={frame.lineno} function={frame.name}", file=sys.stderr, flush=True)
        print(f"PROBE_CONTROL_FAILED success=0 type={type(exc).__name__}",
              file=sys.stderr, flush=True)
        sys.exit(1)
