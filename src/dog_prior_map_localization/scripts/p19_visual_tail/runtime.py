"""Measured single-replay stages, not an online-vision performance claim."""
import argparse
import json
import sys
from pathlib import Path
import rosbag
from compare import rows, write_csv
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p17_visual'))
from summarize_runtime import stats


def summarize(run, output):
    receipt=json.loads((run/'execution.json').read_text())
    ndt=rows(run/'ndt_diagnostics.csv');deskew=rows(run/'deskew.csv')
    lookup={float(r['reference_stamp']):float(r['deskew_processing_ms']) for r in deskew if r['status']=='PUBLISHED'}
    align=[];bag_counts={}
    with rosbag.Bag(str(run/'outputs.bag')) as bag:
        for topic,msg,_ in bag.read_messages():
            bag_counts[topic]=bag_counts.get(topic,0)+1
            if topic=='/dog_livo/diagnostics':
                for status in msg.status:
                    values={v.key:v.value for v in status.values}
                    if 'align_time_ms' in values:align.append(float(values['align_time_ms']))
    expected={'/dog_livo/ndt_odom':396,'/dog_livo/odom_corrected':396,'/dog_livo/odom_high_rate':8004}
    if len(align)!=396 or any(bag_counts.get(k)!=v for k,v in expected.items()):raise ValueError('RECORDED_OUTPUT_INCOMPLETE')
    callback=[(float(r['ndt_odom_publish_wall_sec'])-float(r['wall_time']))*1000 for r in ndt]
    dw=[lookup[float(r['lidar_header_stamp'])] for r in ndt]
    result={name:stats(data) for name,data in [('NDT_align_ms',align),('Deskew_ms',dw),
        ('NDT_callback_ms',callback),('Deskew_plus_NDT_callback_ms',[a+b for a,b in zip(callback,dw)])]}
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'runtime_summary.csv',[dict(stage=k,**v) for k,v in result.items()])
    write_csv(output/'resource_summary.csv',[dict(process=k,**v,peak_MiB=v['peak_rss_bytes']/1048576) for k,v in receipt['resources'].items()])
    out=dict(stages=result,resources=receipt['resources'],wall_sec=receipt['wall_sec'],recorded_bag_counts=bag_counts,
        scope='Two node VmHWM and cumulative CPU; no per-EKF-callback latency, camera IO/rectification/online extraction',
        RSS_note='Independent peaks; their sum double-counts shared pages and is not whole-system unique memory')
    (output/'runtime_metrics.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    summarize(**vars(p.parse_args()))
