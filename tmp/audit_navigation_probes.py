"""Read-only, deterministic review probes; no serial ports or robot motion."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json
import math
import queue
from types import SimpleNamespace
from unittest.mock import Mock
from navigation_core import NavigationEngine, OccupancyGrid, ScanPoint, Pose2D, VelocityCommand
from runtime_config import resolve_runtime_config, build_navigation_engine
from chassis_controller import ChassisMotionAdapter

def run():
    config = json.loads(Path('navigation_config.json').read_text(encoding='utf-8'))
    hardware = resolve_runtime_config('hardware', 'navigation', config)
    simulation = resolve_runtime_config('simulation', 'navigation', config)
    h, s = build_navigation_engine(hardware), build_navigation_engine(simulation)
    results = {'mode_comparison': {name: {
        'range_m': [e.min_range_m, e.max_range_m],
        'map_size_m': [e.grid.width*e.grid.resolution_m, e.grid.height*e.grid.resolution_m],
        'front_edge_m': e.grid.cell_to_world(e.grid.origin_col, 0)[1],
        'assumed_clear_m': e.unobserved_clear_range_m,
        'prefer_forward': e.prefer_forward_exploration,
        'enabled_modes': [k for k,v in e.translation_capabilities.items() if v['enabled']],
    } for name,e in [('hardware',h),('simulation',s)]}}
    adapter = ChassisMotionAdapter(hardware)
    results['unvalidated_distance_control'] = {'readiness': adapter.readiness(),
        'automatic_request': repr(adapter.request_for_command(VelocityCommand(forward_mps=.1,duration_s=1),automatic=True))}
    points = []
    for angle, distance in [(0,.25),(math.pi/2,.25),(-math.pi/2,1.0),(math.pi,1.0)]:
        points.extend(ScanPoint(angle+offset,distance,is_echo=True) for offset in [-.02,0,.02])
    e=NavigationEngine(); e.match_score=.9; e.latest_scan=points; e._last_scan_had_translation=True
    results['terminal_with_left_open_1m'] = e._terminal_geometry_confirmed()
    e=NavigationEngine(); e.set_auto(True)
    e.predict_motion(VelocityCommand(forward_mps=.1,duration_s=1))
    before=e._translation_since_last_scan
    e.process_scan([])
    e.match_score=.9; e.latest_scan=points
    # The next valid stationary scan records this pending flag before planning.
    e._last_scan_had_translation=e._translation_since_last_scan
    results['rejected_scan_consumes_translation']={'before':before,'after':e._translation_since_last_scan,
        'subsequent_terminal_can_start': e._terminal_geometry_confirmed()}
    g=OccupancyGrid(120,120,.02); wall=g.world_to_cell(0,.4); g._add(*wall,6)
    for _ in range(50): g.update_scan(Pose2D(),[ScanPoint(0,1,is_echo=False)],1,add_only=True)
    results['removed_wall_after_50_clear_scans']={'log_odds':g.value(*wall),'state':g.state(*wall)}
    revision=g._revision; updates=g.update_count
    for _ in range(10): g.update_scan(Pose2D(),[ScanPoint(0,1,is_echo=False)],1,add_only=True)
    results['accepted_scan_without_map_revision']={'revision_delta':g._revision-revision,'update_count_delta':g.update_count-updates}
    from navigation_app import NavigationApp
    from chassis_controller import ChassisState
    from mapping_runtime import MappingRequest, MappingResult
    app=NavigationApp.__new__(NavigationApp)
    app.mapping_generation=0; app.mapping_results=queue.Queue(); app.navigator=e
    app._post_motion_map_revision=100; app._post_motion_scan_count=5
    app.scan_collect_after=10; app.sync=SimpleNamespace(session='s')
    app.chassis_controller=SimpleNamespace(state=ChassisState.WAITING_SCAN,pending=None,mark_scan_ready=Mock(return_value=True))
    snapshot=SimpleNamespace(grid=g,map_version=100,completed_scans=6)
    request=MappingRequest(0,'s',20,11,12,'navigation',0,())
    app.mapping_results.put(MappingResult(request,snapshot,VelocityCommand()))
    app._handle_mapping_results()
    results['ui_valid_scan_same_revision']={'mark_scan_ready_called':app.chassis_controller.mark_scan_ready.called}
    print(json.dumps(results,indent=2,ensure_ascii=False))

if __name__=='__main__': run()
