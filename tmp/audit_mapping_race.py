"""Deterministically overlap the real mapper with the simulation motion prior."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import math, queue, threading, json
from unittest.mock import patch
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint, VelocityCommand
from mapping_runtime import MappingRuntime

e=NavigationEngine(OccupancyGrid(100,100,.04)); e.set_auto(True)
points=[]
for degree in range(0,360,2):
    a=math.radians(degree)
    d=.7/max(abs(math.sin(a)),abs(math.cos(a)))
    points.append(ScanPoint(a,d,is_echo=True))
out=queue.Queue(); runtime=MappingRuntime(e,on_result=out.put)
entered=threading.Event(); release=threading.Event()
original=NavigationEngine.process_scan
def paused(self,*args,**kwargs):
    result=original(self,*args,**kwargs)
    entered.set()
    if not release.wait(10): raise TimeoutError('audit release')
    return result
try:
    for seq in range(4):
        runtime.submit('s',seq,points)
        result=out.get(timeout=10)
        if result.error: raise result.error
    before=e.pose.y
    with patch.object(NavigationEngine,'process_scan',paused):
        runtime.submit('s',4,points)
        assert entered.wait(10)
        # Same lock and mutation used by the simulation command branch.
        with runtime.lock: e.predict_motion(VelocityCommand(forward_mps=.1,duration_s=1))
        predicted=e.pose.y
        release.set(); result=out.get(timeout=10)
        if result.error: raise result.error
    data={'before_y':before,'predicted_y':predicted,'committed_y':e.pose.y,
          'motion_pending_after_commit':e._motion_since_last_scan,
          'prediction_lost':abs(e.pose.y-predicted)>.05}
    assert data['prediction_lost']
    print(json.dumps(data,indent=2))
finally:
    release.set(); runtime.stop()
