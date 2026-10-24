#!/usr/bin/env python3
"""Prepare figures/tables from frozen outputs; no original reference or NDT read/run."""
import csv,json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parents[1];R=P.parents[1]
def load(p):
 with Path(p).open() as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dumpcsv(p,rs):
 with p.open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rs[0]));w.writeheader();w.writerows(rs)
A=R/'docs/p62_r23_real_imu_closure';C=R/'docs/p62_r24_geometry_oosm_freshness'
sA=json.loads((A/'EVALUATION_SUMMARY.json').read_text())['trajectories']['W1/C3/MULTISTART_C'];sB=json.loads((P/'EVALUATION_SUMMARY.json').read_text())['trajectories']['W1/C3/G_ONLY'];sC=json.loads((C/'EVALUATION_SUMMARY.json').read_text())['trajectories']['W1/C3/G']
def get_errors(folder,mode):return [r for r in load(folder/'REFERENCE_ERRORS.csv') if (r['window'],r['arm'],r['strategy'],r['estimate'])==('W1','C3',mode,'EKF_CAUSAL_OUTPUT')]
ea,eb,ec=get_errors(A,'MULTISTART_C'),get_errors(P,'G_ONLY'),get_errors(C,'G');assert len(ea)==len(eb)==len(ec)==60
# Verify the entire causal state, not only equal aggregate errors.
ca=[x for x in load(A/'CAUSAL_TRAJECTORIES.csv') if (x['window'],x['arm'],x['strategy'])==('W1','C3','MULTISTART_C')];cb=load(P/'CAUSAL_RESULTS.csv')
fields=[f'{s}{i}{j}' for s in ('ekf','tp') for i in range(4) for j in range(4)]
assert all(all(float(a[k])==float(b[k]) for k in fields) for a,b in zip(ca,cb))
metrics=[]
for label,summary,es in [('A_ORIGINAL_R23',sA,ea),('B_G_ONLY_R25',sB,eb),('C_C_PLUS_G_R24',sC,ec)]:
 e=summary['reference_relative_errors']['EKF_CAUSAL_OUTPUT'];t=e['translation_m'];r=e['rotation_deg'];low=[float(x['translation_m'])<=.5 and float(x['rotation_deg'])<=5 for x in es]
 first=next((int(es[i]['raw_ordinal']) for i in range(60) if all(low[i:])),None)
 metrics.append({'method':label,'samples':60,'translation_rmse_m':t['rmse'],'translation_p95_m':t['p95'],'translation_max_m':t['max'],'rotation_rmse_deg':r['rmse'],'rotation_p95_deg':r['p95'],'rotation_max_deg':r['max'],'ndt_attempts':summary['nominal_ndt_attempts'],'ndt_converged':summary['nominal_ndt_converged'],'map_observations':summary['map_observations_published'],'map_rejections':summary['map_observations_rejected'],'extra_full':summary['extra_full_ndt'],'high_error_frames':summary['ekf_high_error_frames'],'longest_no_map_frames':summary['longest_no_map_run'],'first_sustained_reference_low_error_raw':first,'callback_mean_ms':summary['ndt_callback_wall_ms']['mean'],'callback_p95_ms':summary['ndt_callback_wall_ms']['p95'],'callback_max_ms':summary['ndt_callback_wall_ms']['max'],'ndt_peak_rss_KiB':summary['ndt_node_peak_rss_KiB'],'ekf_peak_rss_KiB':summary['ekf_node_peak_rss_KiB'],'pair_peak_rss_KiB':summary['simultaneous_node_peak_rss_KiB'],'process_wall_s':summary['process_wall_seconds']})
dumpcsv(P/'ABLATION_TABLE.csv',metrics)
figs=P/'figures';figs.mkdir(exist_ok=True)
plotrows=[]
plt.rcParams.update({'font.size':10,'pdf.fonttype':42,'ps.fonttype':42})
fig,axes=plt.subplots(2,1,figsize=(7.2,6),sharex=True)
styles=[('A: original pre-gate',ea,'#777777','-',3,.65),('B: G only (zero restarts)',eb,'#E69F00','--',1.6,1),('C: C + G (8 extra calls)',ec,'#0072B2','-',1.8,1)]
start=int(ea[0]['scan_ns'])
for name,errors,col,ls,lw,alpha in styles:
 x=[(int(e['scan_ns'])-start)/1e9 for e in errors]
 for ax,key in zip(axes,['translation_m','rotation_deg']):ax.plot(x,[float(e[key]) for e in errors],color=col,ls=ls,lw=lw,alpha=alpha,label=name)
 for e,t in zip(errors,x):plotrows.append({'method':name,'raw_ordinal':e['raw_ordinal'],'scan_elapsed_s':t,'translation_m':e['translation_m'],'rotation_deg':e['rotation_deg']})
axes[0].set_ylabel('Reference-relative\ntranslation error (m)');axes[1].set_ylabel('Reference-relative\nrotation error (deg)');axes[1].set_xlabel('Elapsed scan-start time (s)')
for ax in axes:ax.grid(alpha=.25);ax.axvline((int(ec[2]['scan_ns'])-start)/1e9,color='#009E73',ls=':',lw=1)
axes[0].legend(loc='upper left',fontsize=8);axes[0].set_title('W1/C3: causal IMU–NDT–EKF ablation (60 scans)')
fig.tight_layout()
for ext in ['png','pdf','svg']:fig.savefig(figs/('ablation_errors.'+ext),dpi=180)
plt.close(fig);dumpcsv(figs/'ablation_errors.csv',plotrows)
# Actual quality trajectories, not inferred correctness.
rb=load(P/'formal_runs/g_only_w1_c3_01/r23_admission.csv');rc=load(C/'formal_runs/g_w1_c3_01/r23_admission.csv')
fig,ax=plt.subplots(figsize=(7.2,3.2));x=[(int(r['scan_ns'])-start)/1e9 for r in rb]
for records,key,label,col,ls in [(rb,'nominal_coverage_mean','G-only nominal','#E69F00','--'),(rc,'nominal_coverage_mean','C+G nominal','#777777','-'),(rc,'selected_coverage_mean','C+G accepted candidate','#0072B2','-')]:
 y=[float(r[key]) for r in records];ax.plot(x,y,label=label,color=col,ls=ls,lw=1.5)
ax.axhline(.30,color='#D55E00',ls=':',label='Frozen quality gate = 0.30');ax.set(xlabel='Elapsed scan-start time (s)',ylabel='Coverage Consistency Mean',ylim=(0,1.03));ax.grid(alpha=.25);ax.legend(fontsize=8);fig.tight_layout()
for ext in ['png','pdf','svg']:fig.savefig(figs/('coverage_admission.'+ext),dpi=180)
plt.close(fig)
fig,ax=plt.subplots(figsize=(5.5,3.4));ax.bar(['P44 global target','P45 active tiles'],[775308/1024,138200/1024],color=['#777777','#0072B2']);ax.set_ylabel('Recorded NDT process HWM (MiB)');ax.set_title('Historical matched 300-scan resource study');fig.tight_layout()
for ext in ['png','pdf','svg']:fig.savefig(figs/('local_map_memory.'+ext),dpi=180)
plt.close(fig)
# Compact numerical ablation/report.
table='|Method|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|Nominal/extra Full|Map accepted/rejected|High-error / longest no-map|First sustained low-error raw|Callback Mean/P95/Max ms|NDT / pair peak KiB|\n|---|---|---|---|---|---|---|---|---|\n'
for m in metrics:table+='|'+m['method']+'|'+'/'.join(f'{m[k]:.6f}' for k in ['translation_rmse_m','translation_p95_m','translation_max_m'])+'|'+'/'.join(f'{m[k]:.6f}' for k in ['rotation_rmse_deg','rotation_p95_deg','rotation_max_deg'])+f"|{m['ndt_attempts']}/{m['extra_full']}|{m['map_observations']}/{m['map_rejections']}|{m['high_error_frames']}/{m['longest_no_map_frames']}|{m['first_sustained_reference_low_error_raw'] or 'NONE'}|"+'/'.join(f'{m[k]:.3f}' for k in ['callback_mean_ms','callback_p95_ms','callback_max_ms'])+f"|{m['ndt_peak_rss_KiB']}/{m['pair_peak_rss_KiB']}|\n"
(P/'ALGORITHM_ABLATION.md').write_text('''# G-only causal ablation: MULTISTART_ADDITIONAL_CONTRIBUTION_OBSERVED_ON_DEVELOPMENT_CASE\n\nA is frozen R23 W1/C3 MULTISTART_C under the original pre-gate (search unreachable). B is a new independent G-only run with the same post-quality rule and zero search. C is frozen R24 G-mode, i.e. C+G, not rerun. All60 raw scans, real IMU/calibration/initialization/Deskew and map-selection parameters are identical. Processed points may differ once filter histories diverge, as part of the intended causal effect. Actual target set remains the same2297615-point FNV11658115094751181825.\n\n'''+table+'''\nB performed60 native Full alignments,60 converged,0 map admissions,0 extra calls; its entire EKF and TP matrix sequences equal A. Removing the pre-gate alone made the optimizer reachable, but did not yield a candidate passing frozen post-quality. This is not a claim that all rejected raw NDT poses are incorrect.\n\nC used60 nominal+8 extra calls; the -5deg seed at raw2603 yielded the first accepted extra candidate (Coverage .296356), after two rejection frames. raw2604 was rejected again; raw2605 onward nominal matching and fusion succeeded. Total57 observations,3 rejections. The first actual map update and first frame satisfying the existing .5m/5deg reference-relative criterion for the entire remaining approved window are raw2603. That recovery-time statistic is posthoc, not an online decision.\n\nBoth translation and rotation complete-trajectory RMSE engineering targets pass for C; neither passes for A/B. C still has2 high-error frames, maximum rotation11.245245deg. G enables access; fixed multistart bridges the initial post-quality gap in this one known development case. Neither component is established as generally sufficient, novel, or cross-dataset safe. No parameters/methods were revised after B.\n\nNative B outputs were sealed in0d4a56f55b6550a61d5c97fa6945d853cc8e49a5 before reference evaluation. Reference remains conditional-Base, map-anchored Benchmark, not independent mocap. Errors use actual EKF current-state times, not scan-start reference. Rejections are evaluated using real IMU outputs, not zero errors or fake measurements.\n\n![Complete errors](figures/ablation_errors.png)\nThe A/B curves coincide. X-axis is scan-start elapsed time, whereas each error uses its own actual EKF state time. Green line denotes first C+G map update, not an independent truth certification.\n\n![Coverage](figures/coverage_admission.png)\nMissing selected scores are NaN gaps, never filled with zero. Full NDT uses these points; the plotted quality is not independent holdout evidence.\n''')
resource='''# Resource comparison with measurement scopes preserved\n\n## Integrated ablation\n\n'''+table+'''\nB's process wall is13.308355s, C's historical wall9.655876s, A's8.051409s. These are single executions with different causal branches and initialization/cache scheduling costs, not repeated paired timing trials. C uses8 extra calls but subsequently tracks a different, easier branch; do not present the wall-clock difference as a guaranteed speedup. Callback includes in-node preprocessing, map and decision work; input-to-final-ack latency including external Deskew is in RUNTIME_MEMORY.csv. First-frame map/Gaussian construction is included in maximum; no10Hz claim.\n\nRSS is supervisor50ms sampled per-process and simultaneous two-node sum, may miss short peaks, and excludes publisher/Deskew/RViz. A does not enter the optimizer; its lower RSS is not evidence of equal completed work.\n\n## Historical local-map result (P45), separate experiment\n\nThe matched300-scan comparison recorded global target3142751 points versus active552716 points (21 tiles). NDT process HWM was775308KiB global versus138200KiB local:82.2% observed reduction. Translation RMSE .067985m to .061612m, but maximum increased .119675m to .216396m and rotation RMSE1.81802deg to1.87495deg. Steady align mean increased7.086ms to8.079ms (about14%). End-to-end input-to-final-ack wall33.82s exceeded29.90s sensor span. Live tile transition was not exercised; the remote replacement probe used a synthetic pose with a real scan, zero NDT. These recorded profiles are not identical-binary kernel microbenchmarks.\n\n![P45 process memory](figures/local_map_memory.png)\n\nR24/R25 W1/C3 active target has2297615 points and roughly360MiB NDT sampled RSS; this is a different spatial footprint. The135MiB P45 result is not a universal whole-system memory limit. No new dog runtime, throughput or memory result exists inR25.\n'''
(P/'RESOURCE_COMPARISON.md').write_text(resource)
print('A/B EKF+TP pose identity PASS; ablation tables and PNG/PDF/SVG figures generated')
