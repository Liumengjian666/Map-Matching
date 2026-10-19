#!/usr/bin/python3
"""Archive small LF-normalized CSVs; retain raw identities separately."""
import csv
import hashlib
import json
from pathlib import Path

ROOT=Path('/home/jian/livox_ws/p61_r7_geode_runs')
HERE=Path(__file__).resolve().parent


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    names={'ndt_diagnostics.csv':'ndt_diagnostics.csv','deskew.csv':'deskew_diagnostics.csv',
        'fusion.csv':'fusion_diagnostics.csv','oosm.csv':'oosm_statistics.csv',
        'live_trajectories.csv':'localization_trajectories.csv','ekf_lineage.csv':'ekf_lineage.csv',
        'geometry.csv':'geometry_diagnostics.csv','ndt_performance.csv':'runtime_per_frame.csv',
        'memory.csv':'memory_timeseries.csv','publisher_receipts.csv':'publisher_receipts_raw_normalized.csv'}
    provenance={}
    for raw,name in names.items():
        source=ROOT/'formal'/raw; dest=HERE/name
        with source.open(newline='') as f: rows=list(csv.reader(f))
        with dest.open('x',newline='') as f:
            writer=csv.writer(f,lineterminator='\n');writer.writerows(rows)
        with dest.open(newline='') as f: assert list(csv.reader(f))==rows
        provenance[name]={'raw_path':str(source),'raw_sha256':sha(source),
            'archived_sha256':sha(dest),'normalization':'CSV line endings/serialization only; cell strings unchanged'}
    for source,name in [(ROOT/'input/manifest.json','input_manifest.json'),
        (ROOT/'input/initialization.yaml','initialization_config.yaml'),
        (ROOT/'run_identity.json','run_identity.json'),(ROOT/'output_freeze.json','output_freeze.json'),
        (ROOT/'visualization_resources.json','visualization_resources.json'),
        (ROOT/'formal/connection_readiness.json','connection_readiness.json')]:
        with (HERE/name).open('xb') as f:f.write(source.read_bytes())
    (HERE/'archive_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    log=(ROOT/'formal/roslaunch.log').read_text()
    (HERE/'runtime_log_excerpt.txt').write_text('\n'.join(line for line in log.splitlines()
        if any(key in line for key in ('[GEODE]','explicit causal initial state','[DogPriorMap NDT] conv=')))+'\n')
    result=json.loads((HERE/'results.json').read_text())
    runtime={key:result[key] for key in ('durations','resources','replay','visualization_post_map_publish_resources','realtime_validation')}
    runtime['cpu_tick_hz']=100
    (HERE/'runtime_statistics.json').write_text(json.dumps(runtime,indent=2)+'\n')
    print('archived small outputs; original raw files unchanged')


if __name__=='__main__':main()
