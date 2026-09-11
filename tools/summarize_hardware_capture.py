import json
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

folder = Path(sys.argv[1])
rows = [json.loads(x) for x in (folder/'events.jsonl').read_text(encoding='utf-8').splitlines()]
start = next(r for r in rows if r['kind']=='test_start')
end = next(r for r in rows if r['kind']=='test_end')
maps = [r for r in rows if r['kind']=='mapping' and r.get('points')]
actions = [r['command'] for r in rows if r['kind']=='planned_action']
states = Counter(r['state'] for r in rows if r['kind']=='status')
summary = dict(duration_s=end['elapsed'], end_reason=end['reason'],
               closed=any(r['kind']=='capture_closed' for r in rows),
               mapping_scans=len(maps), max_occupied_cells=max((r['occupied'] or 0 for r in maps),default=0),
               first_occupied_scan=next((i+1 for i,r in enumerate(maps) if r['occupied']),None),
               planned_actions=len(actions), rotations=sum(bool(c['yaw_rps']) for c in actions),
               translations=sum(bool(c['forward_mps'] or c['right_mps']) for c in actions),
               recoveries=sum(bool(c.get('recovery_translation')) for c in actions),
               safety_stops=sum(r['kind']=='safety_stop' for r in rows),
               status_samples=dict(states), mapping_errors=[r['error'] for r in rows if r['kind']=='mapping' and r['error']])
(folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
serial = folder.parent/'chassis_serial.jsonl'
if serial.exists():
    selected=[]
    for line in serial.read_text(encoding='utf-8').splitlines():
        r=json.loads(line)
        stamp=datetime.fromisoformat(r['wall_time']).timestamp()
        if rows[0]['at']-1 <= stamp <= rows[-1]['at']+1:
            selected.append(line)
    (folder/'chassis_serial.jsonl').write_text('\n'.join(selected)+'\n',encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
