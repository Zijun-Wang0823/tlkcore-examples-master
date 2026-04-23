# probe_duo.py
import os, sys
from pathlib import Path
root_path = Path(__file__).absolute().parent
sys.path.insert(0, os.path.abspath(os.path.join(root_path, "lib/")))

from tlkcore import TLKCoreService, DevInterface, RetCode

service = TLKCoreService(working_root=r"C:\BBox 8x8 Duo\tlkcore-examples-master")
service.scanDevices(interface=DevInterface.ALL)
scan = service.getScanInfo().RetData
sn = next(iter(scan))                       # 拿扫到的第一台
service.initDev(sn)

print("SN:", sn)
print([m for m in dir(service) if any(k in m for k in
       ["Beam", "Parallel", "AAKit", "Fast"])])