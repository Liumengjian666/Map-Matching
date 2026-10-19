#!/usr/bin/python3
"""Archive small evidence and descriptive anomaly intervals, without rerun/GT."""
import csv,json,shutil
from pathlib import Path
from prepare_inputs import ROOT,sha
from analyze_run import HERE,read,write_csv,delta,pose


def main():
    r=ROOT/'formal';rows=read(HERE/'frame_results.csv')
    descriptions={
        'translation_step_limited':lambda x:x['translation_limited']=='1',
        'rotation_step_limited':lambda x:x['rotation_limited']=='1',
        'iteration_limit_reached':lambda x:int(x['iterations'])>=40,
        'majority_used_points_beyond_8m':lambda x:float(x['used_overlap_8m'])<.5,
        'under_half_used_points_within_0p5m':lambda x:float(x['used_overlap_0p5m'])<.5}
    intervals=[]
    for name,predicate in descriptions.items():
        groups=[]
        for x in rows:
            if not x['iterations'] or not x['used_overlap_8m']:continue
            if predicate(x):
                if not groups or int(x['scan_index'])!=int(groups[-1][-1]['scan_index'])+1:groups.append([])
                groups[-1].append(x)
        for group in groups:
            intervals.append(dict(diagnostic=name,first_scan=group[0]['scan_index'],last_scan=group[-1]['scan_index'],count=len(group),
                start_ns=group[0]['header_ns'],end_ns=group[-1]['end_ns'],
                note='descriptive existing diagnostic only; not GT error or new online trigger'))
    write_csv(HERE/'diagnostic_intervals.csv',intervals)
    peaks={key:max(rows,key=lambda x:float(x[key])) for key in ('fitness','raw_correction_m','raw_correction_deg')}
    (HERE/'peak_cases.json').write_text(json.dumps(peaks,indent=2)+'\n')
    old=read(HERE.parent/'p61_r7_geode_continuous_baseline/ndt_diagnostics.csv');new=read(r/'ndt_diagnostics.csv')[:20]
    parity=[]
    for i,(a,b) in enumerate(zip(old,new)):
        d={p:delta(pose(a,p),pose(b,p)) for p in ('initial_guess','raw_ndt','final_used')}
        parity.append(dict(scan_index=2986+i,source_hash_equal=int(a['cloud_hash']==b['cloud_hash']),
            iterations_equal=int(a['ndt_iterations']==b['ndt_iterations']),fitness_abs_difference=abs(float(a['ndt_fitness'])-float(b['ndt_fitness'])),
            initial_translation_difference_m=d['initial_guess'][0],initial_rotation_difference_deg=d['initial_guess'][1],
            raw_translation_difference_m=d['raw_ndt'][0],raw_rotation_difference_deg=d['raw_ndt'][1],
            used_translation_difference_m=d['final_used'][0],used_rotation_difference_deg=d['final_used'][1]))
    write_csv(HERE/'r7_first20_parity.csv',parity)
    # Full raw traces stay inside ROOT; CSV cell strings are retained verbatim.
    for name in ('live_trajectories.csv','frame_terminals.csv','deskew.csv','fusion.csv','oosm.csv','geometry.csv'):
        with (r/name).open(newline='') as f:source=list(csv.reader(f))
        with (HERE/name).open('x',newline='') as f:csv.writer(f,lineterminator='\n').writerows(source)
    for name in ('rviz_live_early','rviz_live_middle','rviz_live_late','rviz_full_return_finished'):
        for suffix in ('.png','.json'):shutil.copyfile(r/(name+suffix),HERE/(name+suffix))
    shutil.copyfile(ROOT/'input/scans.csv',HERE/'scan_manifest.csv')
    log=(r/'roslaunch.log').read_text()
    (HERE/'startup_log_excerpt.txt').write_text('\n'.join(x for x in log.splitlines() if any(k in x for k in ('actual target_cells_', 'map load +', 'explicit causal initial state', 'started: mode=CONTROL')))+'\n')
    provenance={str(p.relative_to(HERE)):sha(p) for p in HERE.iterdir() if p.is_file() and p.suffix in ('.csv','.json','.png')}
    (HERE/'archive_hashes.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('archived; no new input, align, GT, or changes to raw outputs')


if __name__=='__main__':main()
