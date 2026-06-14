"""Probe BBox Duo per-element gain/phase through setBficConfig().

TLKCore v2.4.9 documents Duo element control through setBficConfig(), not the
legacy BBox setChannelGainPhase() API.
"""

import argparse
import os
import sys
from pathlib import Path

_root = Path(__file__).absolute().parent
_lib_path = _root / "lib"
if _lib_path.exists():
    sys.path.insert(0, os.path.abspath(_lib_path))

from tlkcore import CellRFMode, DevInterface, POLARIZATION_TYPE, RetCode, TLKCoreService  # noqa: E402


DEFAULT_SN = "BDA-2550009-2800"
DEFAULT_WORK_ROOT = str(_root.parent.parent)


def check(ret, action):
    code = getattr(ret, "RetCode", RetCode.OK)
    data = getattr(ret, "RetData", ret)
    msg = getattr(ret, "RetMsg", "")
    print(f"{action}: code={code} msg={msg} data={data}")
    if code != RetCode.OK:
        raise RuntimeError(f"{action} failed: {code} {msg}")
    return data


def build_bfic_config(mode_key, pol_key, element_gain_db, phase_deg):
    if len(element_gain_db) != len(phase_deg):
        raise ValueError("gain and phase arrays must have the same length")
    if len(element_gain_db) == 0 or len(element_gain_db) % 4 != 0:
        raise ValueError("element count must be a non-empty multiple of 4")

    branch = {}
    for offset in range(0, len(element_gain_db), 4):
        ic_id = offset // 4 + 1
        branch[str(ic_id)] = {
            "enable": [1, 1, 1, 1],
            "com_gain_db": 0.0,
            "ele_gain_db": [float(v) for v in element_gain_db[offset : offset + 4]],
            "phase_deg": [int(v) % 360 for v in phase_deg[offset : offset + 4]],
        }
    return {mode_key: {pol_key: branch}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=DEFAULT_SN)
    parser.add_argument("--root", default=DEFAULT_WORK_ROOT)
    parser.add_argument("--mode", choices=("tx", "rx"), default="tx")
    parser.add_argument("--elements", type=int, default=64)
    parser.add_argument("--gain-db", type=float, default=0.0)
    parser.add_argument("--phase-deg", type=int, default=0)
    args = parser.parse_args()

    mode = CellRFMode.TX if args.mode == "tx" else CellRFMode.RX
    mode_key = args.mode
    pol_key = "pol_1"
    polar = POLARIZATION_TYPE.POL_1

    gains = [args.gain_db] * args.elements
    phases = [args.phase_deg] * args.elements
    config = build_bfic_config(mode_key, pol_key, gains, phases)

    svc = TLKCoreService(working_root=args.root)
    check(svc.scanDevices(interface=DevInterface.ALL), "scanDevices")
    scan = svc.getScanInfo().RetData
    print("scan info:", scan)
    if args.sn not in scan:
        raise RuntimeError(f"{args.sn} not found. Scanned devices: {list(scan.keys())}")

    address, dev_type, _in_dfu = scan[args.sn]
    check(svc.initDev(args.sn, address, int(dev_type), False), "initDev")
    try:
        check(svc.setRFMode(args.sn, mode), f"setRFMode({mode.name})")
        check(svc.setUdGain(args.sn, polar, mode, 20.0), "setUdGain")
        check(svc.setBficConfig(args.sn, config), "setBficConfig")
    finally:
        svc.DeInitDev(args.sn)


if __name__ == "__main__":
    main()
