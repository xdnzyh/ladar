import pathlib, json, collections, statistics, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False
root = pathlib.Path('tmp')
summaries = []
for folder in sorted(root.glob('radar_diagnostic_*')):
    rows = [json.loads(line) for line in (folder/'raw.jsonl').read_text(encoding='utf-8').splitlines()]
    summary = dict(folder=str(folder))
    lines = {}
    for source in ['measurement','rotation']:
        lines[source] = ''.join(bytes.fromhex(r['hex']).decode('ascii','backslashreplace') for r in rows if r['kind']=='rx' and r['source']==source).splitlines()
        (folder/(source+'_lines.txt')).write_text('\n'.join(lines[source]),encoding='utf-8')
        summary[source+'_status'] = [s for s in lines[source] if s.startswith('STATUS')]
        summary[source+'_errors'] = [s for s in lines[source] if s.startswith(('ERROR','ERR '))]
    frames=[]
    for line in lines['measurement']:
        parts=line.split()
        if len(parts)==6 and parts[0]=='PIX':
            try: frames.append(tuple(map(int, parts[2:])))
            except ValueError: pass
    if frames:
        summary['pixels'] = len(frames)
        summary['sequence_span'] = frames[-1][0]-frames[0][0]+1
        summary['rx_hz'] = (len(frames)-1)/((frames[-1][1]-frames[0][1])*1e-6)
        summary['outside_calibration'] = sum(not 814<=f[3]<=1203 for f in frames)
    for kind in ['sync_sweep','sync_local_sweep']:
        scans=[r for r in rows if r['kind']==kind]
        summary[kind+'_counts']=[r['count'] for r in scans]
    summary['foreign_rotation']=[s for s in lines['rotation'] if s.startswith(('PIX','BURST'))]
    summary['errors']=[r['value'] for r in rows if r['kind']=='sync_error']
    qualities=[r for r in rows if r['kind']=='closed_quality']
    if qualities:
        chosen=min(qualities,key=lambda r:r['counts']['kept'])
        seq=chosen['anchor'][2]-1
        kept=next((r['points'] for r in rows if r['kind'] in ('sync_sweep','sync_local_sweep') and r['sequence']==seq),[])
        fig,axes=plt.subplots(1,2,figsize=(10,4.8))
        for ax,points,title in zip(axes,[chosen['display'],kept],['原始标定有效回波','通过建图质量筛选的回波']):
            ax.scatter([p['distance_m']*math.sin(p['angle_rad'])*100 for p in points],[p['distance_m']*math.cos(p['angle_rad'])*100 for p in points],s=14)
            ax.scatter([0],[0],marker='^',c='red',s=45)
            ax.set(xlim=(-55,55),ylim=(-55,55),xlabel='右侧 / cm',ylabel='前方 / cm',title=f'{title}：{len(points)} 点')
            ax.set_aspect('equal'); ax.grid(alpha=.3)
        c=chosen['counts']
        fig.suptitle(f'同一真实闭合圈 #{seq}：边界剔除 {c["boundary"]}，角度误差剔除 {c["timing_position"]}\n原始回波仍带角度不确定度，不是真值地图')
        fig.tight_layout()
        fig.savefig(folder/'scan_comparison.png',dpi=150)
        plt.close(fig)
        summary['example_quality'] = chosen['counts']
        summary['example_sequence'] = seq
    summaries.append(summary)
(root/'radar_diagnostic_summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2),encoding='utf-8')
for s in summaries:
    print(s['folder'], 'formal',len(s['sync_sweep_counts']), 'counts',s['sync_sweep_counts'], 'local',s['sync_local_sweep_counts'])
    print('rate',s.get('rx_hz'),'outside',s.get('outside_calibration'),'quality',s.get('example_quality'))
    print('stop',s['measurement_status'][-1:] ,s['rotation_status'][-1:])
