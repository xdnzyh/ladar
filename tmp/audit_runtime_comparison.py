"""Synthetic MappingRuntime integration check, without UI/serial transport."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import json, math, queue
from navigation_core import HiddenWorld
from runtime_config import resolve_runtime_config, build_navigation_engine
from mapping_runtime import MappingRuntime
from virtual_hardware import HardwareSimulation
from scan_acquisition import scan_points_from_polar
from motion_safety import MotionSafetyGuard

base=json.loads(Path('navigation_config.json').read_text(encoding='utf-8'))
runs=[]
for source in ('simulation','hardware'):
    config=resolve_runtime_config(source,'navigation',base)
    config['simulation_profile']='IDEAL'
    if source=='hardware':
        config['simulation_min_range_m']=config['min_range_m']
        config['simulation_max_range_m']=config['max_range_m']
    world=HiddenWorld(map_path=Path('simulation_map.json'))
    sim=HardwareSimulation(world,config)
    engine=build_navigation_engine(config); engine.set_auto(True)
    out=queue.Queue(); runtime=MappingRuntime(engine,min_range_m=engine.min_range_m,on_result=out.put)
    guard=MotionSafetyGuard(config)
    history=[]; moves=0; safety_stop=None
    try:
        while sim.time<60:
            sweeps=sim.advance(.05)
            if guard.command is not None:
                for packet,estimate,timestamp in sim.last_safety_observations:
                    safety_stop=guard.observe(packet,estimate,timestamp)
                    if safety_stop: break
                safety_stop=safety_stop or guard.poll(sim.time)
                if safety_stop: sim.stop(); break
                if sim.time>=sim.resume_at: guard.clear()
            for sequence,polar,period in sweeps:
                if sim.time<sim.resume_at: continue
                runtime.submit('audit',sequence,scan_points_from_polar(polar))
                result=out.get(timeout=30)
                if result.error: raise result.error
                command=result.command
                history.append({'t':round(sim.time,2),'state':engine.state,'detail':engine.detail,
                    'updates':engine.grid.update_count,'pose':[engine.pose.x,engine.pose.y]})
                if not command.stopped:
                    moves+=1; guard.start(command,sim.time); sim.execute(command)
                    # Hardware app invalidates the wall pair before movement.
                    if source=='hardware':
                        runtime.invalidate(); out.get(timeout=5)
                    # Ideal command prior only: this does not emulate DONE protocol.
                    engine.predict_motion(command)
        runs.append({'source':source,'synthetic_only':True,'seconds':sim.time,'moves':moves,
            'safety_stop':safety_stop,'state':engine.state,'detail':engine.detail,
            'goal_error_m':math.dist((world.pose.x,world.pose.y),world.finish),
            'collisions':sim.chassis.collisions,'mapping_attempts':engine.mapping_attempts,
            'rejected':engine.rejected_scans,'history':history})
        print(json.dumps({k:v for k,v in runs[-1].items() if k!='history'},ensure_ascii=False),flush=True)
    finally: runtime.stop()
Path('tmp/audit_runtime_comparison.json').write_text(json.dumps(runs,ensure_ascii=False,indent=2),encoding='utf-8')
