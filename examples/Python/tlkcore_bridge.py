# tlkcore_bridge.py
"""MATLAB-friendly bridge for TLKCore v2.4.9 BBox Duo control."""

import json
import math
import os
import statistics
import sys
import threading
import time
from pathlib import Path

_root = Path(__file__).absolute().parent
_repo_root = _root.parent.parent
_lib_path = _root / "lib"
if _lib_path.exists():
    sys.path.insert(0, os.path.abspath(_lib_path))

from tlkcore import (  # noqa: E402
    AzElAngle,
    CellRFMode,
    DevInterface,
    POLARIZATION_TYPE,
    RetCode,
    TLKCoreService,
    ThetaPhiAngle,
)

DEFAULT_RF_FREQ_KHZ = 28_000_000
DEFAULT_IF_FREQ_KHZ = 5_200_000
DEFAULT_LO_FREQ_KHZ = DEFAULT_RF_FREQ_KHZ - DEFAULT_IF_FREQ_KHZ
DEFAULT_REF_SOURCE = 0
DEFAULT_UD_GAIN_DB = 20.0
DEFAULT_ELEMENTS_PER_POLARIZATION = 64
DEFAULT_COMMAND_RETRIES = 3
DEFAULT_RETRY_DELAY_S = 0.5

_service = None
_work_root = str(_repo_root)
_service_lock = threading.RLock()
_bfic_state = {}
_scan_info = {}

_switch_thread = None
_switch_stop = threading.Event()
_switch_state = {
    "running": False,
    "sn": None,
    "tx_ms": 0.0,
    "rx_ms": 0.0,
    "cycles": 0,
    "completed": 0,
    "last_rf_mode": None,
    "last_error": "",
}


def _require_service():
    if _service is None:
        raise RuntimeError("Service not initialized. Call init([...]) first.")
    return _service


def _ret_data(ret):
    return ret.RetData if hasattr(ret, "RetData") else ret


def _sn_from_action(action):
    if "(" not in action or ")" not in action:
        return None
    return action.split("(", 1)[1].split(")", 1)[0]


def _port_owner_hint(sn):
    if not sn or sn not in _scan_info:
        return ""
    address = _scan_info[sn][0]
    try:
        import psutil
    except Exception:
        return f" Close TLKCore GUI/web-tlk-local-middleware and retry; {address}:5025 may be busy."

    owners = []
    try:
        for conn in psutil.net_connections(kind="tcp"):
            raddr = getattr(conn, "raddr", None)
            if not raddr or len(raddr) < 2:
                continue
            if raddr.ip == address and raddr.port == 5025:
                name = f"pid={conn.pid}"
                try:
                    name = f"{psutil.Process(conn.pid).name()}(pid={conn.pid})"
                except Exception:
                    pass
                owners.append(f"{name} {conn.laddr.ip}:{conn.laddr.port}->{raddr.ip}:{raddr.port} {conn.status}")
    except Exception:
        owners = []

    if owners:
        return " Close TLKCore GUI/web-tlk-local-middleware and retry; device control port is owned by: " + "; ".join(owners)
    return f" Close TLKCore GUI/web-tlk-local-middleware and retry; {address}:5025 was reset."


def _check_ret(ret, action):
    code = getattr(ret, "RetCode", RetCode.OK)
    if code != RetCode.OK:
        msg = getattr(ret, "RetMsg", "")
        if code == RetCode.ERROR_SEND_CMD_TIMEOUT or "10054" in str(msg):
            msg = str(msg) + _port_owner_hint(_sn_from_action(action))
        raise RuntimeError(f"{action} failed: {code} {msg}".strip())
    return _ret_data(ret)


def _retry_call(action, func, *args, retries=DEFAULT_COMMAND_RETRIES, **kwargs):
    last_error = None
    for attempt in range(1, int(retries) + 1):
        try:
            ret = func(*args, **kwargs)
            return _check_ret(ret, action)
        except Exception as exc:
            last_error = exc
            if attempt >= int(retries):
                break
            time.sleep(DEFAULT_RETRY_DELAY_S)
    raise last_error


def _to_sn_list(sn_list):
    if isinstance(sn_list, str):
        return [sn_list]
    return [str(sn) for sn in list(sn_list)]


def _to_float_list(values):
    if values is None:
        return None
    if isinstance(values, (int, float)):
        return [float(values)]
    if hasattr(values, "tolist"):
        values = values.tolist()
    return [float(v) for v in list(values)]


def _to_int_list(values):
    if values is None:
        return None
    if isinstance(values, (int, float)):
        return [int(values)]
    if hasattr(values, "tolist"):
        values = values.tolist()
    return [int(v) for v in list(values)]


def _rf_mode(value):
    text = str(value).strip().lower()
    if text in ("tx", "0", "cellrfmode.tx"):
        return CellRFMode.TX
    if text in ("rx", "1", "cellrfmode.rx"):
        return CellRFMode.RX
    if text in ("standby", "-1", "cellrfmode.standby"):
        return CellRFMode.STANDBY
    if isinstance(value, CellRFMode):
        return value
    raise ValueError("rf_mode must be 'tx', 'rx', or 'standby'")


def _rf_key(value):
    mode = _rf_mode(value)
    if mode is CellRFMode.TX:
        return "tx"
    if mode is CellRFMode.RX:
        return "rx"
    raise ValueError("BFIC config only supports TX or RX")


def _polar(value):
    text = str(value).strip().lower().replace("-", "_")
    mapping = {
        "0": POLARIZATION_TYPE.POL_1,
        "1": POLARIZATION_TYPE.POL_2,
        "2": POLARIZATION_TYPE.POL_H,
        "3": POLARIZATION_TYPE.POL_V,
        "4": POLARIZATION_TYPE.POL_RC,
        "5": POLARIZATION_TYPE.POL_LC,
        "pol1": POLARIZATION_TYPE.POL_1,
        "pol_1": POLARIZATION_TYPE.POL_1,
        "pol2": POLARIZATION_TYPE.POL_2,
        "pol_2": POLARIZATION_TYPE.POL_2,
        "h": POLARIZATION_TYPE.POL_H,
        "horizon": POLARIZATION_TYPE.POL_H,
        "horizontal": POLARIZATION_TYPE.POL_H,
        "pol_h": POLARIZATION_TYPE.POL_H,
        "v": POLARIZATION_TYPE.POL_V,
        "vertical": POLARIZATION_TYPE.POL_V,
        "pol_v": POLARIZATION_TYPE.POL_V,
        "rc": POLARIZATION_TYPE.POL_RC,
        "rhcp": POLARIZATION_TYPE.POL_RC,
        "pol_rc": POLARIZATION_TYPE.POL_RC,
        "lc": POLARIZATION_TYPE.POL_LC,
        "lhcp": POLARIZATION_TYPE.POL_LC,
        "pol_lc": POLARIZATION_TYPE.POL_LC,
    }
    if isinstance(value, POLARIZATION_TYPE):
        return value
    if text in mapping:
        return mapping[text]
    raise ValueError("polarization must be pol_1, pol_2, h, v, rc, or lc")


def _bfic_polar_key(value):
    polar = _polar(value)
    if polar is POLARIZATION_TYPE.POL_1:
        return "pol_1"
    if polar is POLARIZATION_TYPE.POL_2:
        return "pol_2"
    raise ValueError("BFIC element config uses physical polarizations only: pol_1 or pol_2")


def _default_ic_start(polarization, element_count):
    if _bfic_polar_key(polarization) == "pol_1":
        return 1
    return int(math.ceil(float(element_count) / 4.0)) + 1


def _build_bfic_branch(
    gains_db,
    phases_deg,
    enables=None,
    common_gain_db=0.0,
    polarization="pol_1",
    ic_start=None,
):
    gains = _to_float_list(gains_db)
    phases = _to_int_list(phases_deg)
    if gains is None or phases is None:
        raise ValueError("gains_db and phases_deg are required")
    if len(gains) != len(phases):
        raise ValueError("gains_db and phases_deg must have the same length")
    if len(gains) == 0 or len(gains) % 4 != 0:
        raise ValueError("Element vectors must be non-empty and a multiple of 4")

    enables = [1] * len(gains) if enables is None else _to_int_list(enables)
    if len(enables) != len(gains):
        raise ValueError("enables must have the same length as gains_db")

    if ic_start is None:
        ic_start = _default_ic_start(polarization, len(gains))
    ic_start = int(ic_start)

    branch = {}
    for offset in range(0, len(gains), 4):
        ic_index = ic_start + offset // 4
        branch[str(ic_index)] = {
            "enable": [int(bool(x)) for x in enables[offset : offset + 4]],
            "com_gain_db": float(common_gain_db),
            "ele_gain_db": gains[offset : offset + 4],
            "phase_deg": [int(x) % 360 for x in phases[offset : offset + 4]],
        }
    return branch


def _set_bfic_config(sn, config):
    with _service_lock:
        ret = _require_service().setBficConfig(sn, config)
    _check_ret(ret, "setBficConfig")
    return json.dumps(config)


def init(
    sn_list,
    rf_freq_khz=DEFAULT_RF_FREQ_KHZ,
    if_freq_khz=DEFAULT_IF_FREQ_KHZ,
    ref_source=DEFAULT_REF_SOURCE,
    ud_gain_db=DEFAULT_UD_GAIN_DB,
):
    """Scan and initialize Duo devices.

    Frequencies are in kHz. For the current X410 setup:
    RF=28 GHz, IF=5.2 GHz, LO=22.8 GHz.
    """
    global _service, _scan_info
    sns = _to_sn_list(sn_list)
    rf_freq_khz = int(rf_freq_khz)
    if_freq_khz = int(if_freq_khz)
    lo_freq_khz = rf_freq_khz - if_freq_khz

    with _service_lock:
        if _service is not None:
            shutdown(sns)
        _service = TLKCoreService(working_root=_work_root)
        _check_ret(_service.scanDevices(interface=DevInterface.ALL), "scanDevices")
        scan = _service.getScanInfo().RetData
        _scan_info = dict(scan)
        for sn in sns:
            if sn not in scan:
                raise RuntimeError(f"{sn} not found. Scanned devices: {list(scan.keys())}")
            address, dev_type, _in_dfu = scan[sn]
            _retry_call(f"initDev({sn})", _service.initDev, sn, address, int(dev_type), False)
            time.sleep(0.2)
            _retry_call(f"setRFFreq({sn})", _service.setRFFreq, sn, rf_freq_khz)
            _retry_call(f"setLoFreq({sn})", _service.setLoFreq, sn, lo_freq_khz)
            _retry_call(f"setRefSource({sn})", _service.setRefSource, sn, int(ref_source))
            set_ud_gain(sn, "tx", "pol_1", ud_gain_db)
            set_ud_gain(sn, "rx", "pol_1", ud_gain_db)
    return {sn: scan[sn][0] for sn in sns}


def configure_duo(sn, rf_freq_khz, if_freq_khz, ref_source=DEFAULT_REF_SOURCE):
    rf_freq_khz = int(rf_freq_khz)
    if_freq_khz = int(if_freq_khz)
    lo_freq_khz = rf_freq_khz - if_freq_khz
    with _service_lock:
        svc = _require_service()
        _retry_call(f"setRFFreq({sn})", svc.setRFFreq, sn, rf_freq_khz)
        _retry_call(f"setLoFreq({sn})", svc.setLoFreq, sn, lo_freq_khz)
        _retry_call(f"setRefSource({sn})", svc.setRefSource, sn, int(ref_source))
    return int(lo_freq_khz)


def get_duo_status_json(sn):
    with _service_lock:
        svc = _require_service()
        status = {
            "rf_freq_khz": _check_ret(svc.getRFFreq(sn), f"getRFFreq({sn})"),
            "lo_freq_khz": _check_ret(svc.getLoFreq(sn), f"getLoFreq({sn})"),
            "lo_status": _check_ret(svc.getLoStatus(sn), f"getLoStatus({sn})"),
            "ref_source": _check_ret(svc.getRefSource(sn), f"getRefSource({sn})"),
            "rf_mode": _check_ret(svc.getRFMode(sn), f"getRFMode({sn})"),
        }
    return json.dumps(status, default=str)


def set_rf_mode(sn, rf_mode):
    mode = _rf_mode(rf_mode)
    with _service_lock:
        _retry_call(f"setRFMode({sn})", _require_service().setRFMode, sn, mode)
    return mode.name


def get_rf_mode(sn):
    with _service_lock:
        ret = _require_service().getRFMode(sn)
    return json.dumps(_check_ret(ret, f"getRFMode({sn})"), default=str)


def set_tx(sn):
    return set_rf_mode(sn, "tx")


def set_rx(sn):
    return set_rf_mode(sn, "rx")


def set_trx(sn, trx):
    """Compatibility wrapper: 1=TX, 2=RX, 0/-1=STANDBY."""
    trx = int(trx)
    if trx == 1:
        return set_tx(sn)
    if trx == 2:
        return set_rx(sn)
    if trx in (0, -1):
        return set_rf_mode(sn, "standby")
    raise ValueError("Duo v2.4.9 supports 1=TX, 2=RX, or 0/-1=STANDBY")


def get_trx(sn):
    return get_rf_mode(sn)


def set_ud_gain(sn, rf_mode, polarization="pol_1", gain_db=DEFAULT_UD_GAIN_DB):
    mode = _rf_mode(rf_mode)
    polar = _polar(polarization)
    with _service_lock:
        _retry_call(f"setUdGain({sn})", _require_service().setUdGain, sn, polar, mode, float(gain_db))
    return float(gain_db)


def set_tx_gain(sn, gain_db=DEFAULT_UD_GAIN_DB, polarization="pol_1"):
    return set_ud_gain(sn, "tx", polarization, gain_db)


def set_rx_gain(sn, gain_db=DEFAULT_UD_GAIN_DB, polarization="pol_1"):
    return set_ud_gain(sn, "rx", polarization, gain_db)


def set_tx_att(sn, att_db):
    """Compatibility wrapper for old scripts.

    Old API used attenuation where 0 meant maximum gain. Duo v2.4.9 uses
    setUdGain(), so this maps attenuation to gain = 30 - attenuation.
    """
    return set_tx_gain(sn, max(0.0, 30.0 - float(att_db)), "pol_1")


def set_rx_att(sn, att_db):
    return set_rx_gain(sn, max(0.0, 30.0 - float(att_db)), "pol_1")


def set_beam(sn, theta, phi, polarization="pol_1", rf_mode="tx", gain_db=0.0):
    angle = ThetaPhiAngle(theta=int(theta), phi=int(phi))
    with _service_lock:
        _retry_call(
            f"setBeamAngle({sn})",
            _require_service().setBeamAngle,
            sn,
            _polar(polarization),
            _rf_mode(rf_mode),
            angle,
            float(gain_db),
        )
    return True


def set_azel_beam(sn, azimuth, elevation, polarization="pol_1", rf_mode="tx", gain_db=0.0):
    angle = AzElAngle(azimuth=float(azimuth), elevation=float(elevation))
    with _service_lock:
        _retry_call(
            f"setBeamAngle({sn})",
            _require_service().setBeamAngle,
            sn,
            _polar(polarization),
            _rf_mode(rf_mode),
            angle,
            float(gain_db),
        )
    return True


def set_all_element_gain_phase(
    sn,
    gains_db,
    phases_deg,
    polarization="pol_1",
    rf_mode="tx",
    enables=None,
    common_gain_db=0.0,
    elements_per_polarization=DEFAULT_ELEMENTS_PER_POLARIZATION,
    ic_start=None,
):
    """Set per-element BFIC gain and phase for one physical polarization.

    gains_db/phases_deg are per-element vectors. For an 8x8 Duo polarization,
    pass 64 values. Values are grouped as 4 elements per BFIC IC.
    """
    pol_key = _bfic_polar_key(polarization)
    mode_key = _rf_key(rf_mode)
    gains = _to_float_list(gains_db)
    phases = _to_int_list(phases_deg)
    if elements_per_polarization is not None and int(elements_per_polarization) != len(gains):
        raise ValueError(
            f"Expected {int(elements_per_polarization)} elements for {pol_key}, got {len(gains)}"
        )
    branch = _build_bfic_branch(
        gains,
        phases,
        enables=enables,
        common_gain_db=common_gain_db,
        polarization=polarization,
        ic_start=ic_start,
    )
    _bfic_state.setdefault(sn, {}).setdefault(mode_key, {})[pol_key] = branch
    return _set_bfic_config(sn, {mode_key: {pol_key: branch}})


def set_element_gain_phase(
    sn,
    element,
    gain_db,
    phase_deg,
    polarization="pol_1",
    rf_mode="tx",
    enable=True,
    common_gain_db=0.0,
    elements_per_polarization=DEFAULT_ELEMENTS_PER_POLARIZATION,
    ic_start=None,
):
    """Set one element while preserving this bridge's cached state."""
    element = int(element)
    total = int(elements_per_polarization)
    if element < 1 or element > total:
        raise ValueError(f"element must be in 1..{total}")

    mode_key = _rf_key(rf_mode)
    pol_key = _bfic_polar_key(polarization)
    state = _bfic_state.setdefault(sn, {}).setdefault(mode_key, {}).setdefault(pol_key, None)
    if state is None:
        gains = [0.0] * total
        phases = [0] * total
        enables = [1] * total
    else:
        gains, phases, enables = _flatten_bfic_branch(state)
        if len(gains) != total:
            gains = (gains + [0.0] * total)[:total]
            phases = (phases + [0] * total)[:total]
            enables = (enables + [1] * total)[:total]

    idx = element - 1
    gains[idx] = float(gain_db)
    phases[idx] = int(phase_deg) % 360
    enables[idx] = 1 if bool(enable) else 0

    return set_all_element_gain_phase(
        sn,
        gains,
        phases,
        polarization=polarization,
        rf_mode=rf_mode,
        enables=enables,
        common_gain_db=common_gain_db,
        elements_per_polarization=total,
        ic_start=ic_start,
    )


def _flatten_bfic_branch(branch):
    gains = []
    phases = []
    enables = []
    for key in sorted(branch, key=lambda x: int(x)):
        item = branch[key]
        gains.extend(item["ele_gain_db"])
        phases.extend(item["phase_deg"])
        enables.extend(item["enable"])
    return gains, phases, enables


def shutdown(sn_list):
    """Call before MATLAB exits to release TLKCore device instances."""
    global _service
    stop_monostatic_switch()
    sns = _to_sn_list(sn_list)
    with _service_lock:
        if _service is None:
            return
        for sn in sns:
            try:
                _service.DeInitDev(sn)
            except Exception:
                pass
        _service = None


def start_monostatic_switch(sn, tx_ms=500, rx_ms=500, cycles=0):
    """Start a background TX/RX switching loop and return immediately."""
    global _switch_thread
    _require_service()
    stop_monostatic_switch()

    tx_ms = float(tx_ms)
    rx_ms = float(rx_ms)
    cycles = int(cycles)
    if tx_ms <= 0 or rx_ms <= 0:
        raise ValueError("tx_ms and rx_ms must be positive")

    _switch_stop.clear()
    _switch_state.update(
        {
            "running": True,
            "sn": sn,
            "tx_ms": tx_ms,
            "rx_ms": rx_ms,
            "cycles": cycles,
            "completed": 0,
            "last_rf_mode": None,
            "last_error": "",
        }
    )

    _switch_thread = threading.Thread(
        target=_switch_loop,
        args=(sn, tx_ms, rx_ms, cycles),
        daemon=True,
    )
    _switch_thread.start()
    return True


def stop_monostatic_switch():
    """Stop the background TX/RX switching loop."""
    global _switch_thread
    if _switch_thread is None:
        _switch_state["running"] = False
        return False
    _switch_stop.set()
    _switch_thread.join(timeout=2.0)
    alive = _switch_thread.is_alive()
    if not alive:
        _switch_thread = None
        _switch_state["running"] = False
    return not alive


def get_monostatic_switch_status_json():
    return json.dumps(dict(_switch_state))


def _switch_loop(sn, tx_ms, rx_ms, cycles):
    completed = 0
    try:
        while not _switch_stop.is_set() and (cycles == 0 or completed < cycles):
            set_tx(sn)
            _switch_state["last_rf_mode"] = "TX"
            if _switch_stop.wait(tx_ms / 1000.0):
                break

            set_rx(sn)
            _switch_state["last_rf_mode"] = "RX"
            if _switch_stop.wait(rx_ms / 1000.0):
                break

            completed += 1
            _switch_state["completed"] = completed
    except Exception as e:
        _switch_state["last_error"] = str(e)
    finally:
        _switch_state["running"] = False


def monostatic(sn, tx_ms=500, rx_ms=500, cycles=0):
    tx_ms = float(tx_ms)
    rx_ms = float(rx_ms)
    cycles = int(cycles)
    completed = 0
    try:
        while cycles == 0 or completed < cycles:
            set_tx(sn)
            time.sleep(tx_ms / 1000.0)
            set_rx(sn)
            time.sleep(rx_ms / 1000.0)
            if cycles > 0:
                completed += 1
    except KeyboardInterrupt:
        pass
    return int(completed)


def measure_switch_delay(sn, trials=5, timeout_ms=200):
    """Measure host/API RF-mode switching delay in milliseconds."""
    _require_service()
    trials = int(trials)
    timeout_ms = float(timeout_ms)
    delays = []

    for _ in range(trials):
        try:
            set_tx(sn)
        except Exception:
            pass
        time.sleep(0.02)

        try:
            t0 = time.time()
            set_rx(sn)
            start = time.time()
            while True:
                mode_json = json.loads(get_rf_mode(sn))
                if str(mode_json).upper().find("RX") >= 0:
                    break
                if (time.time() - start) * 1000.0 > timeout_ms:
                    break
                time.sleep(0.005)
            delays.append((time.time() - t0) * 1000.0)
        except Exception:
            delays.append(timeout_ms)
        time.sleep(0.05)

    med = float(statistics.median(delays)) if delays else float(timeout_ms)
    print(f"measure_switch_delay: trials={trials}, median_ms={med}, delays_ms={delays}")
    return med


def recommend_intervals(frameLenSamp, cap_frames, Fs, switch_delay_ms, safety_ms=2.0):
    frameLenSamp = float(frameLenSamp)
    cap_frames = int(cap_frames)
    Fs = float(Fs)
    switch_delay_ms = float(switch_delay_ms)
    safety_ms = float(safety_ms)

    cap_samples = cap_frames * frameLenSamp
    cap_time_ms = cap_samples / Fs * 1000.0
    tx_ms = math.ceil(cap_time_ms + switch_delay_ms + safety_ms)
    rx_ms = math.ceil(cap_time_ms + switch_delay_ms + safety_ms)

    print(
        "recommend_intervals: "
        f"cap_time_ms={cap_time_ms:.3f}, switch_delay_ms={switch_delay_ms:.3f}, "
        f"tx_ms={tx_ms}, rx_ms={rx_ms}"
    )
    return (tx_ms, rx_ms, cap_time_ms)
