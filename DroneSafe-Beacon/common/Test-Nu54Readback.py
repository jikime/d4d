#!/usr/bin/env python3
"""Compare NU54 application HEX ranges using installed pyOCD 0.42.0.

Run ONLY after the uploader has released the selected probe. No programming,
erase, recovery, reset, UART, flash algorithm, or ownership operation is used.
Debug attach itself enables debug access; --halt briefly interrupts execution
and restores a previously running/sleeping core in finally (best effort).
No raw memory, firmware bytes, settings, keys, or readback files are emitted.

Offline plan (does not enumerate probes):
  python -B Test-Nu54Readback.py --role searcher --uid FULL_UID --hex APP.hex --dry-run
Small hardware check, only after the uploader has finished:
  python -B Test-Nu54Readback.py --role searcher --uid FULL_UID --hex APP.hex --prefix-bytes 1024
Full comparison: omit --prefix-bytes. A prefix success is NOT full verification.
"""
from __future__ import annotations

import argparse
import hashlib
import logging
from pathlib import Path
import re
import sys
import time

from intelhex import IntelHex

ROLES = ("drone", "searcher")
# This application's generated DTS starts settings/NVS at 0x174000. Do not
# silently widen this range if another build layout changes.
APP_END_EXCLUSIVE = 0x174000
SUPPORTED_PYOCD = "0.42.0"


class VerificationFailure(RuntimeError):
    pass


def make_plan(path: Path, prefix: int | None):
    if path.suffix.lower() != ".hex":
        raise VerificationFailure("Intel HEX input required")
    image = IntelHex(str(path))
    segments = image.segments()
    if not segments:
        raise VerificationFailure("Empty HEX image")
    plan = []
    for start, end in segments:
        if start < 0 or end <= start or end > APP_END_EXCLUSIVE:
            raise VerificationFailure("HEX range outside approved application area")
        read_end = min(end, start + prefix) if prefix is not None else end
        data = bytes(image.tobinarray(start=start, end=read_end - 1))
        plan.append((start, read_end, data))
    return segments, plan


def compare_hardware(args, plan) -> None:
    # Import only on the explicitly hardware-using path. --dry-run never calls
    # a probe enumeration API, and no user pyOCD config/scripts are accepted.
    import pyocd
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target
    from pyocd.probe.cmsis_dap_probe import CMSISDAPProbe

    if pyocd.__version__ != SUPPORTED_PYOCD:
        raise VerificationFailure("Unaudited pyOCD version; review before use")

    class NoUserScriptSession(Session):
        def _load_user_script(self):
            # no_config alone does not suppress pyocd_user.py in this version.
            return None

    uid = args.uid.lower()
    probes = ConnectHelper.get_all_connected_probes(
        blocking=False, unique_id=uid, print_wait_message=False)
    exact = [p for p in probes if p.unique_id.lower() == uid]
    if len(exact) != 1 or not isinstance(exact[0], CMSISDAPProbe):
        raise VerificationFailure("Exact role CMSIS-DAP probe not uniquely available")
    options = {
        "target_override": "nrf54l",
        "auto_unlock": False,  # pyOCD default True can mass-erase a locked part.
        "connect_mode": "attach",
        "resume_on_disconnect": False,
        "no_config": True,
        "project_dir": str(Path(__file__).resolve().parent),
        "frequency": max(1000000, args.frequency), # Discovery needs >=1 MHz on these bridges.
        "dap_protocol": "swd",
        "cmsis_dap.prefer_v1": False,
        "cmsis_dap.limit_packets": True,
        "cmsis_dap.deferred_transfers": False,
        "cache.enable_memory": False,
        "cache.enable_register": False,
        "cache.read_code_from_elf": False,
        "pack.debug_sequences.enable": False,
    }
    session = NoUserScriptSession(exact[0], auto_open=False, options=options)
    target = None
    restore_running = False
    cleanup_failed = False
    checked = 0
    total = sum(end - start for start, end, _ in plan)
    started = time.monotonic()
    last_progress = started
    try:
        session.open()
        target = session.target
        if session.probe.unique_id.lower() != uid:
            raise VerificationFailure("Probe identity changed")
        session.probe.set_clock(args.frequency) # Bulk readback uses conservative 100 kHz.
        initial = target.get_state()
        print(f"ATTACHED role={args.role} target=nrf54l state={initial.name}", flush=True)
        if initial not in (Target.State.RUNNING, Target.State.SLEEPING, Target.State.HALTED):
            raise VerificationFailure("Unexpected CPU state; no reset/recovery attempted")
        if args.halt and initial in (Target.State.RUNNING, Target.State.SLEEPING):
            restore_running = True
            target.halt()
            halt_deadline = time.monotonic() + 2.0
            while target.get_state() != Target.State.HALTED:
                if time.monotonic() > halt_deadline:
                    raise VerificationFailure("Halt confirmation timed out")
                time.sleep(0.01)
        # Use the raw MEM-AP interface, not ELF/memory-cache or flash APIs.
        ap = target.aps[0]
        for index, (start, end, expected) in enumerate(plan, 1):
            observed_hash = hashlib.sha256()
            offset = 0
            while start + offset < end:
                now = time.monotonic()
                if now - started > args.max_seconds:
                    raise VerificationFailure("Readback deadline exceeded between transfers")
                count = min(args.chunk_bytes, end - start - offset)
                actual = bytes(ap.read_memory_block8(start + offset, count))
                if len(actual) != count:
                    raise VerificationFailure("Short block read")
                wanted = expected[offset:offset + count]
                if actual != wanted:
                    mismatch = next(i for i, (a, b) in enumerate(zip(actual, wanted)) if a != b)
                    raise VerificationFailure(f"Mismatch at address 0x{start + offset + mismatch:08x}; bytes hidden")
                observed_hash.update(actual)
                offset += count
                checked += count
                now = time.monotonic()
                if now - last_progress >= 2.0:
                    print(f"PROGRESS checked={checked} total={total} seconds={now-started:.1f}", flush=True)
                    last_progress = now
            print(f"SEGMENT_MATCH index={index} start=0x{start:08x} bytes={end-start} "
                  f"sha256={observed_hash.hexdigest()}", flush=True)
    finally:
        if restore_running and target is not None:
            try:
                if target.get_state() == Target.State.HALTED:
                    target.resume()
                current = target.get_state()
                if current not in (Target.State.RUNNING, Target.State.SLEEPING):
                    cleanup_failed = True
                print(f"RESTORE previous_running=1 current_state={current.name}", flush=True)
            except Exception:
                cleanup_failed = True
                print("RESTORE_FAILED no_reset_attempted=1", flush=True)
        try:
            session.close()
        except Exception:
            cleanup_failed = True
            print("PROBE_CLOSE_FAILED", flush=True)
        if cleanup_failed:
            raise VerificationFailure("CPU/probe cleanup not confirmed")
    if checked != total:
        raise VerificationFailure("Not all planned bytes compared")
    mode = "PREFIX_READBACK_MATCH" if args.prefix_bytes is not None else "FULL_READBACK_MATCH"
    elapsed = time.monotonic() - started
    print(f"{mode} role={args.role} bytes={checked} seconds={elapsed:.2f} "
          f"full_verification={int(args.prefix_bytes is None)}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--role", choices=ROLES, required=True)
    parser.add_argument("--uid", required=True, help="Complete CMSIS-DAP UID for the physically identified role")
    parser.add_argument("--hex", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true", help="Validate image/plan only; no probe enumeration")
    parser.add_argument("--prefix-bytes", type=int, help="Check only N initial bytes per segment; not full verification")
    parser.add_argument("--chunk-bytes", type=int, default=64, help="Read block size, 4..32768 and multiple of 4; default 64")
    parser.add_argument("--frequency", type=int, default=100000, help="SWD Hz, 100000..4000000; default 100000")
    parser.add_argument("--max-seconds", type=float, default=180.0, help="Deadline checked between block transfers")
    parser.add_argument("--halt", action="store_true", help="Temporarily halt if originally running; disrupts radio")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Fa-f0-9]{8,128}", args.uid):
        parser.error("--uid must be the complete hexadecimal probe UID, not a prefix")
    if args.prefix_bytes is not None and args.prefix_bytes < 1:
        parser.error("--prefix-bytes must be positive")
    if not 4 <= args.chunk_bytes <= 32768 or args.chunk_bytes % 4:
        parser.error("--chunk-bytes must be a multiple of 4 in 4..32768")
    if not 100000 <= args.frequency <= 4000000 or args.max_seconds <= 0:
        parser.error("Invalid frequency/deadline")
    logging.basicConfig(level=logging.ERROR)
    path = args.hex.resolve(strict=True)
    segments, plan = make_plan(path, args.prefix_bytes)
    print(f"PLAN role={args.role} exact_uid={args.uid.lower()} image_sha256="
          f"{hashlib.sha256(path.read_bytes()).hexdigest()} segments={len(segments)} "
          f"planned_bytes={sum(b-a for a,b,_ in plan)} full={int(args.prefix_bytes is None)}", flush=True)
    for index, (start, end, _) in enumerate(plan, 1):
        print(f"RANGE index={index} start=0x{start:08x} end_exclusive=0x{end:08x}", flush=True)
    if args.dry_run:
        print("OFFLINE_PLAN_VALID hardware_access=0 verification_performed=0", flush=True)
        return 0
    compare_hardware(args, plan)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("READBACK_INTERRUPTED success=0", file=sys.stderr, flush=True)
        sys.exit(130)
    except Exception as exc:
        # Do not dump exception internals from transport/HEX libraries or bytes.
        message = str(exc) if isinstance(exc, VerificationFailure) else type(exc).__name__
        print(f"READBACK_FAILED success=0 reason={message}", file=sys.stderr, flush=True)
        sys.exit(1)
