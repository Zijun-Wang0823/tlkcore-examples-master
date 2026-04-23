# tlkcore_bridge.py
import os, sys
from pathlib import Path
_root = Path(__file__).absolute().parent
sys.path.insert(0, os.path.abspath(os.path.join(_root, "lib/")))

from tlkcore import TLKCoreService, DevInterface, RetCode

_service = None
_work_root = r"C:\BBox 8x8 Duo\tlkcore-examples-master"

def init(sn_list):
    """扫描并 initDev，返回 {sn: addr}"""
    global _service
    _service = TLKCoreService(working_root=_work_root)
    _service.scanDevices(interface=DevInterface.ALL)
    scan = _service.getScanInfo().RetData
    for sn in sn_list:
        if sn not in scan:
            raise RuntimeError(f"{sn} not found")
        _service.initDev(sn)
        # Duo 还要设 RF/LO/参考源
        _service.setRfFreq(sn, 28000000)
        _service.setLoFreq(sn, 22800000)
        _service.setRefSource(sn, 0)         # INTERNAL
    return {sn: scan[sn][0] for sn in sn_list}

def set_tx(sn):
    return _service.setTRx(sn, 1).RetData    # 1=TX

def set_rx(sn):
    return _service.setTRx(sn, 2).RetData    # 2=RX

def set_beam(sn, theta, phi):
    """以太网切波束 —— 阻塞直到 BBox ACK"""
    ret = _service.setBeam(sn, theta=int(theta), phi=int(phi))
    return int(ret.RetData) if hasattr(ret,"RetData") else int(ret)

def shutdown(sn_list):
    """MATLAB 结束前调用，避免 TLKCore 析构时噪音"""
    global _service
    if _service is None:
        return
    for sn in sn_list:
        try:
            _service.DeInitDev(sn)
        except Exception:
            pass
    _service = None