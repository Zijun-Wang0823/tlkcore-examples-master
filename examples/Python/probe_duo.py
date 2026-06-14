"""Minimal BBox Duo probe for TLKCore v2.4.9.

Close the official TLKCore GUI before running this script. The GUI middleware
keeps TCP port 5025 open and the device resets Python commands from a second
client.
"""

import argparse
import os
import sys
from pathlib import Path

_root = Path(__file__).absolute().parent
_lib_path = _root / "lib"
if _lib_path.exists():
    sys.path.insert(0, os.path.abspath(_lib_path))

from tlkcore import (  # noqa: E402
    CellRFMode,
    DevInterface,
    POLARIZATION_TYPE,
    RetCode,
    TLKCoreService,
    ThetaPhiAngle,
)


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=DEFAULT_SN)
    parser.add_argument("--root", default=DEFAULT_WORK_ROOT)
    parser.add_argument("--rf-khz", type=int, default=28_000_000)
    parser.add_argument("--if-khz", type=int, default=5_200_000)
    parser.add_argument("--mode", choices=("tx", "rx"), default="tx")
    parser.add_argument("--gain-db", type=float, default=20.0)
    args = parser.parse_args()

    lo_khz = args.rf_khz - args.if_khz
    mode = CellRFMode.TX if args.mode == "tx" else CellRFMode.RX
    polar = POLARIZATION_TYPE.POL_1

    svc = TLKCoreService(working_root=args.root)
    check(svc.scanDevices(interface=DevInterface.ALL), "scanDevices")
    scan = svc.getScanInfo().RetData
    print("scan info:", scan)
    if args.sn not in scan:
        raise RuntimeError(f"{args.sn} not found. Scanned devices: {list(scan.keys())}")

    address, dev_type, _in_dfu = scan[args.sn]
    check(svc.initDev(args.sn, address, int(dev_type), False), "initDev")
    try:
        check(svc.querySN(args.sn), "querySN")
        check(svc.queryFWVer(args.sn), "queryFWVer")
        check(svc.queryHWVer(args.sn), "queryHWVer")
        check(svc.getSysStatus(args.sn), "getSysStatus")
        check(svc.setRFFreq(args.sn, args.rf_khz), "setRFFreq")
        check(svc.setLoFreq(args.sn, lo_khz), "setLoFreq")
        check(svc.setRefSource(args.sn, 0), "setRefSource(INTERNAL)")
        check(svc.setRFMode(args.sn, mode), f"setRFMode({mode.name})")
        check(svc.setUdGain(args.sn, polar, mode, args.gain_db), "setUdGain")
        check(
            svc.setBeamAngle(
                args.sn,
                polar,
                mode,
                ThetaPhiAngle(theta=0, phi=0),
                args.gain_db,
            ),
            "setBeamAngle(theta=0, phi=0)",
        )
        check(svc.getRFFreq(args.sn), "getRFFreq")
        check(svc.getLoFreq(args.sn), "getLoFreq")
        check(svc.getRefSource(args.sn), "getRefSource")
        check(svc.getRFMode(args.sn), "getRFMode")
    finally:
        svc.DeInitDev(args.sn)


if __name__ == "__main__":
    main()
