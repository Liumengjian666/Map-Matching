"""Measured ROS stages/resources; deliberately no online-vision cost claim."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import rosbag


def rows(path):
    with Path(path).open(newline='') as f:return list(csv.DictReader(f))


def stats(values):
    x=np.array(values,dtype=float)
    if not len(x) or not np.isfinite(x).all():raise ValueError('MISSING_OR_NONFINITE_TIMING')
    return dict(count=len(x),mean=float(x.mean()),p95=float(np.percentile(x,95)),max=float(x.max()))


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result={};timings=[];resource_rows=[]
    for name in ('control_short','visual_short','visual_full'):
        root=args.root/name;receipt=json.loads((root/'execution.json').read_text())
        ndt=rows(root/'ndt_diagnostics.csv');deskew=rows(root/'deskew.csv')
        d={float(r['reference_stamp']):float(r['deskew_processing_ms']) for r in deskew if r['status']=='PUBLISHED'}
        align=[]
        with rosbag.Bag(str(root/'outputs.bag')) as bag:
            for _,msg,_ in bag.read_messages(topics=['/dog_livo/diagnostics']):
                for status in msg.status:
                    values={v.key:v.value for v in status.values}
                    if 'align_time_ms' in values:align.append(float(values['align_time_ms']))
        if len(align)!=len(ndt):raise ValueError('INCOMPLETE_NDT_DIAGNOSTIC_TIMINGS')
        callbacks=[(float(r['ndt_odom_publish_wall_sec'])-float(r['wall_time']))*1000 for r in ndt]
        deskew_times=[d[float(r['lidar_header_stamp'])] for r in ndt]
        sums=[a+b for a,b in zip(callbacks,deskew_times)]
        stages={label:stats(v) for label,v in [('NDT_align_ms',align),('NDT_callback_to_publish_ms',callbacks),
                 ('Deskew_ms',deskew_times),('Deskew_plus_NDT_callback_ms',sums)]}
        for label,s in stages.items():timings.append(dict(run=name,stage=label,**s))
        for process,r in receipt['resources'].items():
            resource_rows.append(dict(run=name,process=process,peak_rss_bytes=r['peak_rss_bytes'],
                peak_rss_MiB=r['peak_rss_bytes']/1048576,process_cpu_sec=r['cpu_sec'],
                replay_orchestration_wall_sec=receipt['wall_sec']))
        result[name]=dict(stages=stages,resources=receipt['resources'],wall_sec=receipt['wall_sec'],
            node_RSS_peak_sum_upper_bound_MiB=sum(r['peak_rss_bytes'] for r in receipt['resources'].values())/1048576)
    args.output.mkdir(parents=True,exist_ok=True)
    for name,data in [('runtime_summary.csv',timings),('resource_summary.csv',resource_rows)]:
        with (args.output/name).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]),lineterminator='\n');w.writeheader();w.writerows(data)
    out=dict(runs=result,scope='NDT+deskew stages and two localization processes only; no online camera frontend',
             EKF_time_scope='cumulative process CPU from /proc; per-callback EKF latency not measured',
             RSS_scope='VmHWM; sum of independent peaks is an upper bound and double-counts shared pages; not unique whole-system RAM',
             visual_interface_separate_timer='NOT_MEASURED; included in NDT callback stage')
    (args.output/'runtime_metrics.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))


if __name__=='__main__':main()
