#!/usr/bin/env python3
"""Independent numerical/time audit of the frozen run (no GT access)."""
import csv
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/jian/livox_ws/p61_r5_geode_runs')


def main():
    with (ROOT/'formal_mapping/mapping_trajectory.csv').open() as stream:r=list(csv.DictReader(stream))
    with (ROOT/'formal_mapping/replay.csv').open() as stream:replay=list(csv.DictReader(stream))
    assert len(r)==len(replay)==2441
    imu_support=[];se3=[];chain=[];extrinsic=[]
    last_rotation=None
    for i,x in enumerate(r):
        matrices=[]
        for prefix in ['T_M_I','T_I_L','T_M_L']:
            t=np.array([float(x[f'{prefix}_{a}{b}']) for a in range(4) for b in range(4)]).reshape(4,4)
            assert np.isfinite(t).all()
            se3.append(float(np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))))
            assert abs(np.linalg.det(t[:3,:3])-1)<1e-4
            matrices.append(t)
        mi,il,ml=matrices;chain.append(float(np.max(np.abs(mi@il-ml))))
        if x['stage']=='MAPPED':
            if last_rotation is None:last_rotation=il
            extrinsic.append(float(np.max(np.abs(il-last_rotation))))
            # Actual carry sample + measured tail availability bracket the deskew interval.
            carried=float(r[i-1]['imu_last_sec'])
            beginning=float(x['scan_start_sec']);end=float(x['scan_end_sec'])
            imu_support.append(carried<=beginning and float(x['imu_received_until_sec'])>=end and int(x['imu_count'])>0)
    assert max(se3)<1e-4 and max(chain)<1e-10 and all(imu_support)
    mapped=[x for x in r if x['stage']=='MAPPED']
    times=np.array([float(x['processing_ms']) for x in mapped])
    latency=np.array([float(x['publisher_to_ack_ms']) for x in replay])
    out=dict(consumed_scans=len(r),mapped_updates=len(mapped),
        max_so3_orthogonality_error=max(se3),max_transform_chain_error=max(chain),
        max_mapped_extrinsic_change=max(extrinsic),
        measured_imu_bracketed_mapped_scans=sum(imu_support),
        imu_samples_delivered=sum(int(x['imu_published']) for x in replay),
        map_count_busy_sentinel_frames=sum(int(x['map_points'])==-1 for x in r),
        map_count_busy_meaning='author validnum() returns -1 when background rebuild owns its mutex, not a negative map size',
        mapping_processing_ms=dict(mean=float(times.mean()),p95=float(np.percentile(times,95)),maximum=float(times.max())),
        publisher_to_ack_ms=dict(mean=float(latency.mean()),p95=float(np.percentile(latency,95)),maximum=float(latency.max())),
        first_mapped_scan_sequence=int(mapped[0]['sequence']),last_mapped_scan_sequence=int(mapped[-1]['sequence']),
        first_mapped_start_sec=float(mapped[0]['scan_start_sec']),last_state_end_sec=float(mapped[-1]['scan_end_sec']),
        gap_to_frozen_test_start_sec=1706671587.829840123-float(mapped[-1]['scan_end_sec']),
        peak_rss_mib=json.loads((ROOT/'formal_mapping/mapper_resources.json').read_text())['peak_rss_kib']/1024,
        legal_mapped_se3=True,gt_read=False)
    (HERE/'numerical_audit.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))


if __name__=='__main__':main()
