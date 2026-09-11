"""Offline geometry diagnostics from a real navigation capture; no serial I/O."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import json, math
from navigation_core import ScanPoint, Pose2D, VelocityCommand
from runtime_config import build_navigation_engine
from mapping_policy import prepare_mapping_points, complete_open_scan, prepare_free_space_points

rows = [json.loads(x) for x in Path(sys.argv[1]).read_text(encoding='utf-8').splitlines()]
engine = build_navigation_engine(next(r['config'] for r in rows if r['kind']=='config'))
for row in rows:
    if row['kind'] != 'mapping' or not row.get('points'):
        continue
    engine.pose = Pose2D(**row['pose'])
    raw = [ScanPoint(**p) for p in row['points']]
    fitted = prepare_mapping_points(raw, engine.max_range_m, engine.min_range_m, engine.grid.resolution_m)
    engine.latest_scan = list(complete_open_scan(raw, engine.unobserved_clear_range_m,
                                               engine.max_range_m, engine.min_range_m, engine.grid.resolution_m))
    engine.grid.update_scan(engine._sensor_pose(), fitted.points + tuple(engine.latest_scan),
                            engine.max_range_m, add_only=True)
print('Reconstructed geometry, not an exact runtime snapshot')
print('pose',engine.pose,'nearest',min(p.distance_m for p in raw))
print('gaps',[(round(w,2),round(math.degrees(a),1)) for w,a,d in engine._unexplored_gap_candidates()])
for f,r in ((1,0),(0,1),(0,-1),(-1,0),(1,1),(-1,1)):
    norm=math.hypot(f,r)
    cmd=VelocityCommand(.1*f/norm,.1*r/norm,duration_s=.5,recovery_translation=True)
    print(f,r,engine._translation_guard(cmd,allow_backtracking=True)[1])
