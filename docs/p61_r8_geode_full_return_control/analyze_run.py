#!/usr/bin/python3
"""Summarize the one frozen actual run. Never read a reference trajectory."""
import csv,json,math,os,shutil
from collections import Counter
from decimal import Decimal
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_inputs import ROOT,REPO,sha

HERE=Path(__file__).resolve().parent


def native_seconds(ns):
    # Avoid converting ~1.7e18 integer ns to double before division.
    return float(Decimal(ns)/Decimal(10**9))


def read(path):
    if not path.exists():return []
    with path.open() as f:return list(csv.DictReader(f))


def stats(values):
    a=np.asarray([float(v) for v in values if math.isfinite(float(v))])
    return dict(count=len(a),mean=float(a.mean()),p95=float(np.percentile(a,95)),max=float(a.max())) if len(a) else dict(count=0)


def pose(row,prefix=None):
    names=[prefix+'_'+s for s in ('tx','ty','tz','qx','qy','qz','qw')] if prefix else ['px','py','pz','qx','qy','qz','qw']
    return [float(row[k]) for k in names]


def delta(a,b):
    return float(np.linalg.norm(np.array(a[:3])-b[:3])),float(np.rad2deg((Rotation.from_quat(a[3:]).inv()*Rotation.from_quat(b[3:])).magnitude()))


def write_csv(path,rows):
    if not rows:return
    with path.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)


def main():
    r=ROOT/'formal';inp=ROOT/'input'
    # After input has stopped and ROS processes shut down: freeze before analysis.
    frozen={str(p.relative_to(ROOT)):dict(size=p.stat().st_size,sha256=sha(p)) for p in sorted(r.rglob('*')) if p.is_file()}
    (HERE/'output_freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    identity=json.loads((ROOT/'preflight_identity.json').read_text())
    m=json.loads((inp/'manifest.json').read_text());summary=json.loads((r/'replay_summary.json').read_text())
    assets={p:sha(p)==digest for p,digest in m['inputs'].items()}
    assert all(assets.values())
    assert all(sha(p)==digest for p,digest in identity['binaries'].items())
    assert sha(REPO/'src/dog_prior_map_localization/config/geode_r7_baseline.yaml')==identity['algorithm_config_sha256']
    scans=read(inp/'scans.csv');ndt=read(r/'ndt_diagnostics.csv');deskew=read(r/'deskew.csv')
    fusion=read(r/'fusion.csv');oosm=read(r/'oosm.csv');lineage=read(r/'ekf_lineage.csv')
    performance=read(r/'ndt_performance.csv');live=read(r/'live_trajectories.csv');memory=read(r/'memory.csv')
    terminals=read(r/'frame_terminals.csv');geo=read(r/'geometry.csv')
    streams={k:[x for x in live if x['stream']==k] for k in ('HIGH_RATE','PREDICTION','NDT_USED','CORRECTED')}
    pred={int(x['stamp_ns']):x for x in streams['PREDICTION']};used={int(x['stamp_ns']):x for x in streams['NDT_USED']}
    fused={int(x['measurement_ns']):x for x in fusion}
    corrections=[x for x in lineage if x['event_type']=='NDT_CORRECTION_APPLIED']
    callbacks=[x for x in lineage if x['event_type']=='NDT_CALLBACK_ENTER']
    replayed=[x for x in lineage if x['event_type']=='OOSM_REPLAY_COMPLETE']
    def near(rows,key,ns):
        hits=[x for x in rows if abs(float(x[key])-ns/1e9)<1e-6]
        return hits[0] if len(hits)==1 else None
    def correction_at(ns):return near(corrections,'ndt_measurement_stamp',ns)
    rows=[];time_errors=[];evidence_violations=[];matched_durations=[]
    scan_map={int(s['scan_index']):s for s in scans}
    for ordinal,t in enumerate(terminals):
        s=scan_map[int(t['scan_index'])];end=int(s['end_ns'])
        n=near(ndt,'lidar_header_stamp',end);d=near(deskew,'scan_end_stamp',end)
        o=near(oosm,'ndt_stamp',end);f=fused.get(end);a=correction_at(end)
        cb=near(callbacks,'ndt_measurement_stamp',end);rp=near(replayed,'ndt_measurement_stamp',end)
        p=pred.get(end);u=used.get(end)
        chits=[x for x in streams['CORRECTED'] if abs(int(x['stamp_ns'])-int(t['imu_watermark_ns'] or 0))<1000]
        c=chits[0] if len(chits)==1 else None
        evidence_complete=all(x is not None for x in (n,d,o,f,a,p,u,c,cb,rp))
        if evidence_complete:
            evidence_complete=(d['status']=='PUBLISHED' and d['post_scan_end_imu_used']=='0'
                and n['ndt_has_converged']=='1' and o['oosm_result']=='APPLIED' and f['update_applied']=='1'
                and float(a['state_stamp'])==float(a['ndt_measurement_stamp'])==native_seconds(end)
                and float(rp['state_stamp'])==float(cb['state_stamp'])==float(f['final_now_sec'])
                and abs(int(c['stamp_ns'])-int(t['imu_watermark_ns']))<1000
                and float(c['receive_monotonic'])>float(p['receive_monotonic'])
                and int(cb['ndt_update_count'])==int(cb['state_revision'])==ordinal
                and int(rp['state_revision'])==ordinal+1
                and int(cb['event_index'])<int(a['event_index'])<int(rp['event_index']))
        if t['terminal']=='APPLIED' and not evidence_complete:evidence_violations.append(int(t['scan_index']))
        perf=next((x for x in performance if int(x['stamp_ns'])==end),None)
        geometry=next((x for x in geo if int(x['scan_ns'])==end),None)
        if d and perf and f:matched_durations.append(float(d['deskew_processing_ms'])+float(perf['localization_time_ms'])+float(f['update_ms']))
        initial=pose(n,'initial_guess') if n else None;raw=pose(n,'raw_ndt') if n else None;final=pose(n,'final_used') if n else None
        if p and initial:assert delta(initial,pose(p))[0]<1e-8 and delta(initial,pose(p))[1]<1e-6
        if a:
            time_errors.append(float(abs(Decimal.from_float(float(a['state_stamp']))*10**9-Decimal(end))))
            assert float(a['state_stamp'])==float(a['ndt_measurement_stamp'])
        rows.append(dict(scan_index=s['scan_index'],header_ns=s['stamp_ns'],end_ns=end,source_raw_sha256=s['serialized_sha256'],
            source_hash=n['cloud_hash'] if n else '',source_points=n['cloud_size_after_filter'] if n else '',evidence_complete=int(evidence_complete),
            deskew_status=d['status'] if d else '',deskew_max_imu_source_stamp=d['max_imu_source_stamp'] if d else '',
            deskew_post_end_imu=d['post_scan_end_imu_used'] if d else '',imu_history_first=d['history_first_stamp'] if d else '',
            imu_history_last=d['history_last_stamp'] if d else '',max_imu_gap_sec=d['max_state_gap_sec'] if d else '',
            deskew_reference_sec=d['reference_stamp'] if d else '',ndt_measurement_ns=end,
            ndt_align_calls=int(n is not None and math.isfinite(float(n['ndt_align_complete_wall_sec']))),
            ndt_converged=n['ndt_has_converged'] if n else '',iterations=n['ndt_iterations'] if n else '',fitness=n['ndt_fitness'] if n else '',
            used_nn_median_m=geometry['used_nn_median_m'] if geometry else '',used_overlap_0p5m=geometry['used_overlap_0p5m'] if geometry else '',
            used_overlap_8m=geometry['used_overlap_8m'] if geometry else '',
            initial_pose=json.dumps(initial),raw_ndt_pose=json.dumps(raw),used_ndt_pose=json.dumps(final),
            initial_guess_source=n['initial_guess_source'] if n else '',prior_ndt_update_count=cb['ndt_update_count'] if cb else '',
            state_revision_before_update=cb['state_revision'] if cb else '',state_revision_after_replay=rp['state_revision'] if rp else '',
            raw_correction_m=n['raw_delta_from_guess_translation'] if n else '',raw_correction_deg=n['raw_delta_from_guess_rotation_deg'] if n else '',
            translation_limited=n['translation_limited'] if n else '',rotation_limited=n['rotation_limited'] if n else '',
            oosm_status=o['oosm_result'] if o else '',rollback_sec=o['rollback_stamp'] if o else '',
            actual_correction_sec=a['state_stamp'] if a else '',replay_imu_count=o['replay_imu_count'] if o else '',
            replay_end_sec=f['final_now_sec'] if f else '',current_watermark_ns=t['imu_watermark_ns'],
            corrected_publish_ns=c['stamp_ns'] if c else '',ekf_final_pose=json.dumps(pose(c) if c else None),
            update_applied=f['update_applied'] if f else '',terminal=t['terminal'],fatal=t['fatal'],ack_ms=t['ack_ms']))
    write_csv(HERE/'frame_results.csv',rows)
    write_csv(HERE/'anomalies.csv',[x for x in rows if x['terminal']!='APPLIED' or x['translation_limited']=='1' or x['rotation_limited']=='1'])
    assert all(x['coupled_mode']=='CONTROL' and int(x['coupled_jet_calls'])==int(x['coupled_value_calls'])==int(x['coupled_feedback_applied'])==0 for x in ndt)
    assert all(x['post_scan_end_imu_used']=='0' for x in deskew)
    jumps=[];turns=[];increments=[]
    for a,b in zip(streams['CORRECTED'],streams['CORRECTED'][1:]):
        dt,dr=delta(pose(a),pose(b));jumps.append(dt);turns.append(dr)
        increments.append(dict(from_ns=a['stamp_ns'],to_ns=b['stamp_ns'],translation_m=dt,rotation_deg=dr))
    write_csv(HERE/'trajectory_increments.csv',increments)
    resources={}
    for name in sorted({x['process'] for x in memory}):
        v=[x for x in memory if x['process']==name]
        resources[name]=dict(peak_rss_mib=max(int(x['rss_kib']) for x in v)/1024,vmhwm_mib=max(int(x['hwm_kib']) for x in v)/1024,
            samples=len(v),cpu_seconds=((int(v[-1]['utime_ticks'])+int(v[-1]['stime_ticks']))-(int(v[0]['utime_ticks'])+int(v[0]['stime_ticks'])))/os.sysconf('SC_CLK_TCK'))
    durations=dict(deskew_ms=stats(x['deskew_processing_ms'] for x in deskew),ndt_align_ms=stats(x['align_time_ms'] for x in performance),
        ndt_callback_ms=stats(x['localization_time_ms'] for x in performance),ekf_update_replay_ms=stats(x['update_ms'] for x in fusion),
        publisher_to_ack_ms=stats(x['ack_ms'] for x in terminals if x['ack_ms']))
    durations['steady_ndt_align_ms_excluding_first']=stats(x['align_time_ms'] for x in performance[1:])
    durations['sum_deskew_ndt_callback_fusion_ms']=stats(matched_durations)
    complete=summary['sent']==summary['completed_terminals']==len(terminals)==2441 and summary['failure'] is None
    failures=[x for x in terminals if x['terminal']!='APPLIED']
    # Conservative engineering classification: already-recorded 8m geometry
    # diagnostic shows a majority of used-pose points outside that distance.
    # This is not a new online gate, GT-error threshold, or certified wrong basin.
    poor_geometry=[x for x in geo if float(x['used_overlap_8m'])<.5]
    label='FULL_RETURN_CONTROL_COMPLETED_WITH_FAILURES' if failures or poor_geometry else 'FULL_RETURN_CONTROL_COMPLETED_STABLE_CANDIDATE'
    if not complete:
        text=summary['failure'] or ''
        label='FULL_RETURN_CONTROL_TIME_CONTRACT_BLOCKED' if any(k in text for k in ('TIME','HISTORY','REPLAY','FUTURE','DESKEW')) else 'FULL_RETURN_CONTROL_NUMERICAL_FAILURE' if 'NUMERICAL' in text else 'FULL_RETURN_CONTROL_RUNTIME_BLOCKED'
    if evidence_violations:label='FULL_RETURN_CONTROL_RUNTIME_BLOCKED'
    result=dict(TASK='P61-R8-GEODE-FULL-RETURN-CONTROL-BASELINE',FINAL_RESULT=label,
        START_SHA=identity['START_SHA'],ALGORITHM_CODE_SHA=identity['ALGORITHM_CODE_SHA'],
        CODE_SHA='1367e1d9587d1f90a47bf6f44405239a7b65e4f6',PUSH_EXECUTED=False,
        input_scans=2441,sent_scans=summary['sent'],terminal_scans=len(terminals),sensor_span_sec=(m['last_end_ns']-m['first_header_ns'])/1e9,
        imu_messages=summary['imu_sent'],deskew_published=sum(x['status']=='PUBLISHED' for x in deskew),
        ndt_align_calls=sum(x['ndt_align_calls'] for x in rows),ndt_converged=sum(x['ndt_has_converged']=='1' for x in ndt),
        ndt_not_converged=sum(x['ndt_has_converged']!='1' for x in ndt),oosm=dict(Counter(x['oosm_result'] for x in oosm)),
        correction_events=len(corrections),replay_imu_count=sum(int(x['replay_imu_count']) for x in oosm),
        imu_watermark_messages_match=all(x['replay_end_sec'] and abs(float(x['replay_end_sec'])-int(x['current_watermark_ns'])/1e9)<1e-6 for x in rows if x['terminal']=='APPLIED'),
        previous_correction_used_next_frame=all(int(x['prior_ndt_update_count'])==i for i,x in enumerate(rows) if x['terminal']=='APPLIED'),
        terminal_counts=dict(Counter(x['terminal'] for x in terminals)),stream_counts={k:len(v) for k,v in streams.items()},evidence_violations=evidence_violations,
        unique_source_hashes=len({x['cloud_hash'] for x in ndt}),all_assets_unchanged=all(assets.values()),
        map_points=321053,target_points=sorted({int(x['map_points']) for x in performance}),
        initial_T_M_L=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json').read_text())['initial_state']['T_M_L'],
        final_corrected_pose=pose(streams['CORRECTED'][-1]) if streams['CORRECTED'] else None,
        trajectory_length_m=sum(jumps),max_adjacent_translation_m=max(jumps) if jumps else None,max_adjacent_rotation_deg=max(turns) if turns else None,
        max_raw_correction_m=stats(x['raw_delta_from_guess_translation'] for x in ndt),max_raw_correction_deg=stats(x['raw_delta_from_guess_rotation_deg'] for x in ndt),
        translation_limited=sum(x['translation_limited']=='1' for x in ndt),rotation_limited=sum(x['rotation_limited']=='1' for x in ndt),
        iteration_limit_frames=sum(int(x['ndt_iterations'])>=40 for x in ndt),fitness=stats(x['ndt_fitness'] for x in ndt),
        majority_points_beyond_recorded_8m_diagnostic=len(poor_geometry),
        classification_note='WITH_FAILURES includes directly observed severe estimated-pose geometric mismatch; not a GT-certified localization error or nonconvergence claim',
        geometry_gate=dict(Counter(x['gate_status'] for x in geo)),used_overlap_0p5m=stats(x['used_overlap_0p5m'] for x in geo),
        maximum_native_double_stamp_representation_error_ns=max(time_errors) if time_errors else None,
        gt_read=False,accuracy_evaluated=False,coupled_jet_calls=0,coupled_value_calls=0,extra_align_calls=0,candidate_feedback=0,
        covariance_policy='R7 fixed 15D heuristic restart prior; not R6 full 23D posterior',
        durations=durations,resources=resources,replay=summary,realtime_validation='NOT_REALTIME_VALIDATED',
        hessian='not enabled in frozen R7 runtime; no extra derivatives computed',
        lock_loss='No independent correctness reference; terminal failures, corrections and geometry diagnostics reported separately')
    (HERE/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    (HERE/'runtime_statistics.json').write_text(json.dumps(dict(durations=durations,resources=resources,replay=summary),indent=2)+'\n')
    for source,name in [(inp/'manifest.json','input_manifest.json'),(ROOT/'preflight_identity.json','run_identity.json')]:shutil.copyfile(source,HERE/name)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,8))
    for name in ('PREDICTION','NDT_USED','CORRECTED'):
        a=np.array([pose(x)[:3] for x in streams[name]])
        if len(a):axes[0].plot(a[:,0],a[:,1],label=name,linewidth=1)
    axes[0].axis('equal');axes[0].legend();axes[0].set(xlabel='map X / m',ylabel='map Y / m',title='Actual return trajectories; no GT accuracy claim')
    axes[1].plot([(int(x['end_ns'])-m['first_header_ns'])/1e9 for x in rows],[float(x['raw_correction_m']) if x['raw_correction_m'] else np.nan for x in rows],label='raw NDT correction / m (gaps = unavailable)')
    axes[1].legend();axes[1].set(xlabel='sensor time from first header / s')
    fig.tight_layout();fig.savefig(HERE/'trajectories_and_corrections.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,4))
    for name in sorted(resources):
        v=[x for x in memory if x['process']==name]
        # Plot downsampled measured RSS, while maxima above use every sample.
        v=v[::max(1,len(v)//2000)]
        ax.plot([float(x['wall_monotonic'])-float(memory[0]['wall_monotonic']) for x in v],
                [int(x['rss_kib'])/1024 for x in v],label=name)
    ax.set(xlabel='wall time from first memory sample / s',ylabel='process RSS / MiB',title='Separate processes; not summed physical memory')
    ax.legend();fig.tight_layout();fig.savefig(HERE/'process_memory.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in result.items() if k not in ('initial_T_M_L','resources')},indent=2))


if __name__=='__main__':main()
