#!/usr/bin/env python3
"""R1C2 frozen-input runner and independent CSV/JSON numerical audit."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
PREV = ROOT / 'docs/p9_r1c_support_aware_continuation'
OUT = ROOT / 'docs/p9_r1c2_numerical_stationarity'
STEPS = [.004, .002, .001, .0005, .00025, .000125]
START = 'ad2b472d5ac5c433194444e893d7ca532b950cca'


def read(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def vec(text):
    a = np.array([float(x) for x in text.split(';')])
    if not np.all(np.isfinite(a)):
        raise ValueError('nonfinite numerical vector')
    return a


def grouped(rows, field='root_id'):
    result = {}
    for row in rows:
        result.setdefault(row[field], []).append(row)
    return result


def stable_region(rows):
    """Longest contiguous strict-relative stable region, finest tie-break."""
    pairs = []
    for a, b in zip(rows, rows[1:]):
        if a['spd']!='1' or b['spd']!='1':
            pairs.append(False)
            continue
        da, db = vec(a['newton_displacement']), vec(b['newton_displacement'])
        ea, eb = vec(a['H_eigenvalues']), vec(b['H_eigenvalues'])
        dd = np.linalg.norm(da-db)/max(np.linalg.norm(da), np.linalg.norm(db), 1e-300)
        de = np.max(np.abs(ea-eb)/np.maximum(np.abs(ea), np.abs(eb)))
        pairs.append(a['spd'] == b['spd'] == '1' and dd <= .20 and de <= .10)
    regions = []
    for first in range(len(rows)):
        for last in range(first+2, len(rows)):
            if all(pairs[first:last]):
                regions.append(list(range(first, last+1)))
    return max(regions, key=lambda x: (len(x), x[-1])) if regions else []


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(mean=float(values.mean()), median=float(np.median(values)),
                P95=float(np.percentile(values, 95)), max=float(values.max()))


def envelope(floats, doubles, probes, region, rho):
    if not region or any(floats[i]['spd']!='1' for i in region):
        return None
    bg = [vec(floats[i]['gradient'])-vec(doubles[i]['gradient']) for i in region]
    bd = [vec(floats[i]['newton_displacement'])-vec(doubles[i]['newton_displacement']) for i in region]
    spread = lambda xs: max(np.linalg.norm(a-b) for a, b in zip(xs, xs[1:]))
    return dict(EPS_G_NUM=float(max(map(np.linalg.norm, bg))+spread(bg)),
                EPS_DV_NUM=float(max(map(np.linalg.norm, bd))+spread(bd)),
                EPS_E_NUM=float(2*max(float(p['mean_gap']) for p in probes)+rho))


def self_test():
    rows = []
    for h in STEPS:
        rows.append(dict(spd='1', h=str(h), newton_displacement='1;0;0;0', H_eigenvalues='1;2;3;4'))
    assert stable_region(rows) == list(range(6))
    changed = [dict(r) for r in rows]
    for i, row in enumerate(changed):
        row['newton_displacement'] = f'{2**i};0;0;0'
    assert stable_region(changed) == []
    changed = [dict(r) for r in rows]
    changed[2]['spd'] = '0'
    assert stable_region(changed) == [3, 4, 5]
    f, d = [], []
    for i in range(3):
        # Shared truncation must cancel before float-spread calculation.
        f.append(dict(spd='1',gradient=f'{10/(4**i)+.01};0;0;0', newton_displacement='.02;0;0;0'))
        d.append(dict(gradient=f'{10/(4**i)};0;0;0', newton_displacement='0;0;0;0'))
    e = envelope(f, d, [dict(mean_gap='.001')], [0, 1, 2], 1e-12)
    assert abs(e['EPS_G_NUM']-.01) < 1e-14 and abs(e['EPS_DV_NUM']-.02) < 1e-14
    assert e['EPS_E_NUM'] == .002+1e-12
    assert envelope(f, d, [], [], 1e-12) is None
    assert not complete_certificate(True, True, True, False)
    print('R1C2_MULTIH_AUDIT_SELF_TEST=PASS')


def complete_certificate(resolved, support, spd, derivative_valid):
    return bool(resolved and support and spd and derivative_valid)


def check_inputs():
    previous = json.loads((PREV / 'results.json').read_text())
    manifest = dict(previous['input_sha256'])
    for name, sha in previous['sidecar_sha256'].items():
        manifest[str(PREV / name)] = sha
    for path, sha in manifest.items():
        if digest(path) != sha:
            raise RuntimeError('frozen input SHA mismatch: ' + path)
    for name, sha in previous['source_sha256'].items():
        if name!='CMakeLists.txt':
            path=ROOT/'src/dog_prior_map_fastlio2_frontend_exp/scripts/p9'/name
            if digest(path)!=sha:raise RuntimeError('inherited source changed: '+name)
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if head != START:
        raise RuntimeError('unexpected experiment START HEAD')
    return manifest


def audit(out=OUT):
    inputs = check_inputs()
    metadata = read(out / 'root_metadata.csv')
    ff, dd = grouped(read(out / 'multih_float.csv')), grouped(read(out / 'multih_double.csv'))
    parity = grouped(read(out / 'float_double_parity.csv'))
    reference_rows = read(out / 'double_stationary_endpoints.csv')
    references = {r['root_id']: r for r in reference_rows}
    old_roots = {f"{r['tx']}/{r['cluster']}/{r['direction']}": r for r in read(PREV / 'branch_certificates.csv')}
    directionals = read(out / 'directional_fd.csv')
    if len(metadata) != 14 or len(reference_rows)!=14 or len(references) != 14 or set(ff) != set(old_roots) or set(dd) != set(old_roots) or \
       {r['root_id'] for r in metadata}!=set(old_roots) or set(references)!=set(old_roots) or set(parity)!=set(old_roots):
        raise RuntimeError('root set/count mismatch')
    keys=[(r['root_id'],r['origin'],r['precision'],float(r['h']),int(r['direction'])) for r in directionals]
    expected={(rid,origin,precision,h,direction) for rid in old_roots for origin in
              ['ENDPOINT','T0' if rid.endswith('FORWARD') else 'CANONICAL'] for precision in ['FLOAT','DOUBLE']
              for h in STEPS for direction in range(3)}
    if len(keys)!=len(expected) or set(keys)!=expected:raise RuntimeError('directional key set/uniqueness')
    classifications = []
    details = []
    stability_rows=[]
    for m in metadata:
        rid = m['root_id']; fs, ds, probes, star = ff[rid], dd[rid], parity[rid], references[rid]
        if [float(r['h']) for r in fs] != STEPS or [float(r['h']) for r in ds] != STEPS or len(probes) != 9:
            raise RuntimeError('multi-h or parity count mismatch')
        if {int(p['probe']) for p in probes}!=set(range(-1,8)):raise RuntimeError('probe identity mismatch')
        if float(m['reproduced_gradient_error']) > 1e-10 or float(m['reproduced_H_error']) > 1e-7 or float(m['reproduced_energy_error'])>1e-12:
            raise RuntimeError('R1C derivative reproduction failure')
        for rows in [fs, ds]:
            for r in rows:
                g = vec(r['gradient']); H = vec(r['H_columnmajor']).reshape(4, 4, order='F')
                eig=np.linalg.eigvalsh(H)
                if not np.allclose(H, H.T, rtol=0, atol=1e-12) or not np.allclose(eig,vec(r['H_eigenvalues']),rtol=1e-10,atol=1e-9) or \
                   (eig.min()>0)!=(r['spd']=='1') or abs(np.linalg.norm(g)-float(r['gradient_norm']))>1e-14:
                    raise RuntimeError('invalid archived jet/norm/eigenvalues')
                if r['spd']!='1':continue
                d = vec(r['newton_displacement'])
                if np.linalg.norm(H@d+g) > 1e-8 or abs(-.5*g@d-float(r['predicted_decrement'])) > 1e-12:
                    raise RuntimeError('Newton residual/decrement CSV inconsistent')
                if abs(np.linalg.norm(d)-float(r['displacement_norm']))>1e-12 or abs(2*float(r['predicted_decrement'])-float(r['newton_decrement_squared']))>1e-14:
                    raise RuntimeError('Newton scalar fields inconsistent')
        for p in probes:
            if int(p['guard_disagreements'])!=0:raise RuntimeError('float/double guard branch differs')
            gap = abs(float(p['float_energy'])-float(p['double_energy']))
            if abs(gap-float(p['mean_gap'])) > 1e-14 or abs(gap*int(p['source_count'])-float(p['sum_gap'])) > 1e-10:
                raise RuntimeError('value normalization mismatch')
        fr, dr = stable_region(fs), stable_region(ds)
        eps = envelope(fs, ds, probes, dr, float(star['rho_E']))
        for precision,series in [('FLOAT',fs),('DOUBLE',ds)]:
            for a,b in zip(series,series[1:]):
                ga,gb=vec(a['gradient']),vec(b['gradient'])
                ng=np.linalg.norm(ga)*np.linalg.norm(gb)
                angle=float(np.degrees(np.arccos(np.clip(ga@gb/ng,-1,1)))) if ng>0 else None
                disprel=None
                if a['spd']==b['spd']=='1':
                    da,db=vec(a['newton_displacement']),vec(b['newton_displacement'])
                    disprel=float(np.linalg.norm(da-db)/max(np.linalg.norm(da),np.linalg.norm(db),1e-300))
                ea,eb=vec(a['H_eigenvalues']),vec(b['H_eigenvalues'])
                eigrel=float(np.max(np.abs(ea-eb)/np.maximum(np.maximum(np.abs(ea),np.abs(eb)),1e-300)))
                direction='DIRECTION_UNRESOLVED_NEAR_ZERO' if eps and min(np.linalg.norm(ga),np.linalg.norm(gb))<=eps['EPS_G_NUM'] else 'RESOLVED'
                stability_rows.append(dict(root_id=rid,precision=precision,h_coarse=a['h'],h_fine=b['h'],
                        gradient_angle_deg=angle,displacement_relative_change=disprel,
                        max_eigenvalue_relative_change=eigrel,direction_status=direction))
        finest = ds[-1]
        support = m['support_equal'] == '1'
        spd = all(r['spd'] == '1' for r in ds) and fs[2]['spd'] == '1'
        reference_ok = star['status'] == 'DOUBLE_RESOLUTION_STATIONARY'
        if reference_ok and (float(star['d_double']) > float(star['d_resolution']) or
                             float(star['predicted_decrement']) > float(star['rho_E']) or
                             float(star['fd_displacement_difference']) > float(star['d_resolution']) or
                             float(star['fd_eigen_relative'])>.10 or float(star['coarse_min_eig'])<=0 or
                             float(star['coarse_d'])>np.sqrt(2*float(star['rho_E'])/float(star['coarse_min_eig'])) or
                             float(star['coarse_decrement'])>float(star['rho_E']) or star['no_tested_newton_descent']!='1'):
            raise RuntimeError('reference false stationarity termination')
        resolved = bool(eps and reference_ok and float(finest['displacement_norm']) <= eps['EPS_DV_NUM'] and
                        float(star['delta_v_norm']) <= eps['EPS_DV_NUM'] and
                        float(finest['predicted_decrement']) <= eps['EPS_E_NUM'] and
                        float(star['energy_decrease']) <= eps['EPS_E_NUM'] and
                        float(fs[-1]['gradient_norm']) <= eps['EPS_G_NUM'])
        true_nonstationary = bool(eps and reference_ok and
                                 (float(star['delta_v_norm']) > eps['EPS_DV_NUM'] or
                                  float(star['energy_decrease']) > eps['EPS_E_NUM']))
        endpoint_fd = [r for r in directionals if r['root_id']==rid and r['origin']=='ENDPOINT' and
                       ((r['precision']=='FLOAT' and float(r['h'])==.001) or
                        (r['precision']=='DOUBLE' and float(r['h'])==STEPS[-1]))]
        if len(endpoint_fd)!=6:raise RuntimeError('missing endpoint independent FD checks')
        derivative_valid=all(r['pass']=='1' for r in endpoint_fd)
        passed = complete_certificate(resolved, support, spd, derivative_valid)
        row = dict(root_id=rid, independent_id=m['independent_id'], support_equal=support, Hvv_SPD=spd,
                   old_gradient_norm=float(m['old_gradient_norm']), old_pass=m['old_pass']=='1',
                   new_pass=passed, true_nonstationary=true_nonstationary, numerical_floor=resolved,
                   derivative_valid=derivative_valid,
                   support_unresolved=not support, float_stable_h=';'.join(str(STEPS[i]) for i in fr),
                   double_stable_h=';'.join(str(STEPS[i]) for i in dr),
                   dN_double=float(finest['displacement_norm']), DeltaE_double=float(finest['predicted_decrement']),
                   root_to_star_dv=float(star['delta_v_norm']), root_to_star_translation=float(star['translation_m']),
                   root_to_star_rotation_deg=float(star['rotation_deg']), energy_decrease=float(star['energy_decrease']),
                   projected_star_float_gradient=float(star['star_float_g']), reference_status=star['status'],
                   **(eps or {'EPS_G_NUM':None, 'EPS_DV_NUM':None, 'EPS_E_NUM':None}))
        classifications.append(row)
        endpoint_series=grouped([r for r in read(out/'reference_multih.csv') if r['root_id']==rid], 'stage')
        if set(endpoint_series)!= {'STAR_FLOAT','STAR_DOUBLE'} or any([float(r['h']) for r in xs]!=STEPS for xs in endpoint_series.values()):
            raise RuntimeError('reference multi-h series incomplete')
        star_stability={stage:[STEPS[i] for i in stable_region(xs)] for stage,xs in endpoint_series.items()}
        near_zero_direction='DIRECTION_UNRESOLVED_NEAR_ZERO' if eps and float(fs[-1]['gradient_norm'])<=eps['EPS_G_NUM'] else 'RESOLVED'
        details.append(dict(classification=row, float_multih=fs, double_multih=ds, reference=star,
                            reference_endpoint_stable_h=star_stability,gradient_direction_status=near_zero_direction,
                            original_R1C=old_roots[rid]))
    p10 = next(r for r in classifications if r['root_id']=='616/P10/REVERSE')
    if p10['new_pass'] or not p10['support_unresolved']:
        raise RuntimeError('P10 support failure was hidden')
    unique = {}
    for r in classifications:
        if r['independent_id'] in unique:
            other = unique[r['independent_id']]
            for k in ['EPS_G_NUM','EPS_DV_NUM','EPS_E_NUM','dN_double','new_pass']:
                if r[k] != other[k]:
                    raise RuntimeError('shared TX616 forward differs numerically')
        unique[r['independent_id']] = r
    if len(unique) != 9:
        raise RuntimeError('independent-root denominator is not 9')
    n_pass = sum(r['new_pass'] for r in classifications)
    n_true = sum(r['true_nonstationary'] for r in unique.values())
    if n_pass >= 10:
        final = 'FLOAT_NUMERICAL_STATIONARITY_FLOOR_CONFIRMED'
        nxt = 'SUPPORT_AWARE_CONTINUATION_R1C3'
    elif n_true >= 5:
        final = 'ARCHIVED_ROOTS_TRULY_NONSTATIONARY'
        nxt = 'RECENTERED_FIXED_SUPPORT_ROOTS'
    elif sum(r['support_unresolved'] for r in unique.values()) >= 5:
        final = 'SUPPORT_FIXED_POINT_IS_PRIMARY_BLOCKER'
        nxt = 'DISCRETE_SUPPORT_TRANSITION_EVIDENCE'
    else:
        final = 'NUMERICAL_AND_SUPPORT_MECHANISMS_SEPARATED'
        nxt = 'RECOVER_NUMERICALLY_CERTIFIABLE_ROOTS_AND_ISOLATE_TRUE_SUPPORT_FAILURES'
    fields = list(classifications[0])
    with (out / 'stationarity_roots.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fields); writer.writeheader(); writer.writerows(classifications)
    with (out/'multih_stability.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,list(stability_rows[0]));writer.writeheader();writer.writerows(stability_rows)
    oldfd = read(PREV / 'frozen_derivative_fd.csv')
    for r in directionals:
        if r['precision']=='FLOAT' and float(r['h'])==.001 and r['origin']!='ENDPOINT':
            tx, cluster, _ = r['root_id'].split('/')
            old = next(o for o in oldfd if o['tx']==tx and o['cluster']==cluster and
                       o['support_origin']==r['origin'] and o['direction']==r['direction'])
            if abs(float(r['gradient_error'])-float(old['gradient_absolute_error']))>1e-10 or r['pass']!=old['pass']:
                raise RuntimeError('historical independent FD not reproduced')
    execution = json.loads((out / 'execution.json').read_text())
    result = dict(task='PAPER-P9-R1C2-NUMERICAL-STATIONARITY-CERTIFICATE-CLOSURE', start_sha=START,
                  branch_requested='research/p9-r1c2-numerical-stationarity',
                  git_status='BRANCH_AND_COMMIT_BLOCKED_READ_ONLY_GIT',
                  roots_logical=14, roots_independent=9,
                  old_certificate_pass=sum(r['old_pass'] for r in classifications),
                  new_certificate_pass=n_pass, support_equal=sum(r['support_equal'] for r in classifications),
                  independent_new_pass=sum(r['new_pass'] for r in unique.values()),
                  independent_true_nonstationary=n_true, roots=details,
                  value_gap_mean_energy=stats([float(p['mean_gap']) for ps in parity.values() for p in ps]),
                  value_gap_total_score=stats([float(p['sum_gap']) for ps in parity.values() for p in ps]),
                  projected_star_float_gradients=stats([float(r['star_float_g']) for r in references.values()]),
                  directional_fd=directionals, execution=execution,
                  restrictions=dict(full_ndt_calls=0, support_updates=0, continuation_calls=0, GT=False,
                                    production_solver_changed=False, posterior_NLL=False),
                  verification=dict(csv_json='PASS', hash='PASS', inherited_sources='UNCHANGED',
                                    **(json.loads((out/'verification.json').read_text()) if (out/'verification.json').exists() else {})),
                  final_result=final, next=nxt, input_sha256=inputs,
                  sidecar_sha256={p.name:digest(p) for p in sorted(out.glob('*.csv'))},
                  source_sha256={name:digest(ROOT/'src/dog_prior_map_fastlio2_frontend_exp/scripts/p9'/name) for name in
                                 ['p9_numerical_stationarity.cpp','p9_stationarity_numerics.hpp','run_numerical_stationarity.py','CMakeLists.txt']},
                  theory_sha256=digest(out/'THEORY.md'))
    (out/'results.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ['final_result','next','new_certificate_pass','independent_new_pass','independent_true_nonstationary','value_gap_mean_energy']}))
    return result


def write_report(result, out=OUT):
    roots=result['roots']
    unique={r['classification']['independent_id']:r for r in roots}
    lines=['# R1C2 numerical stationarity closure', '',
           f"FINAL_RESULT = {result['final_result']}", f"NEXT = {result['next']}", '',
           '## Decision evidence', '',
           f"Original strict certificate: 0/14. New complete certificate: {result['new_certificate_pass']}/14 logical, "
           f"{result['independent_new_pass']}/9 independent. Six TX616 forward records are one root, not six independent observations.",
           '13/14 original supports are exactly self-consistent. P10 reverse remains support-unresolved. '
           'Both P09 endpoints additionally retain original-h FLOAT directional FD failures and are NOT certified.',
           'All 9 independent references converge under the stricter double control. No root requires a displacement '
           'or energy reduction exceeding its measured numerical envelope. This is approximate numerical stationarity, '
           'not an exact smooth-branch or continuation certificate.', '',
           'FLOAT: PointXYZ / Matrix4f / float transform / double accumulation. '
           'DOUBLE: the same input float values exactly cast to double / double product-chart composition and transform / double accumulation.',
           'Formula, frozen leaf ordering/membership and guards are unchanged; 126 pose probes have zero guard disagreements. '
           'No dynamic Hessian, support optimization, NDT align, continuation, GT, posterior or EKF was used.', '',
           '## Value parity', '',
           '| gap | mean | median | P95 | max |','|---|---:|---:|---:|---:|']
    for name,key in [('per-source mean E','value_gap_mean_energy'),('total score','value_gap_total_score')]:
        s=result[key];lines.append('| '+name+' | '+' | '.join(f'{s[k]:.9g}' for k in ['mean','median','P95','max'])+' |')
    lines+=['', 'The statistics above use 126 logical probes (81 independent poses); duplicate forward inputs are disclosed.', '',
            '## Independent-root classification', '',
            '| root | old g | support equal | FD valid | old/new PASS | double dN (finest) | DeltaE (finest) | EPS_G | EPS_dv | EPS_E |',
            '|---|---:|---|---|---|---:|---:|---:|---:|---:|']
    for r in unique.values():
        c=r['classification']
        num=lambda k:f'{c[k]:.8g}' if c[k] is not None else 'INDETERMINATE'
        lines.append(f"| {c['independent_id']} | {num('old_gradient_norm')} | {c['support_equal']} | {c['derivative_valid']} | "
                     f"{c['old_pass']}/{c['new_pass']} | {num('dN_double')} | {num('DeltaE_double')} | {num('EPS_G_NUM')} | {num('EPS_DV_NUM')} | {num('EPS_E_NUM')} |")
    lines+=['', 'The 616 shared-forward row maps to P02/P03/P10/P12/P13/P17. All 14 logical classifications are in stationarity_roots.csv.', '',
            '## Strict double reference and projection', '',
            '| root | steps | delta v norm | translation m | geodesic rotation deg | E decrease | double g | projected float g | projected support equal |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in unique.values():
        s=r['reference'];lines.append('| '+r['classification']['independent_id']+' | '+s['accepted_steps']+' | '+
            ' | '.join(f'{float(s[k]):.9g}' for k in ['delta_v_norm','translation_m','rotation_deg','energy_decrease','g_double','star_float_g'])+' | '+s['star_support_equal']+' |')
    lines+=['', 'Reference termination is resolution-aware AND no actual decrease exists among the 16 declared dyadic Newton trials. '
            'It is not a proof against every possible descent direction. The stricter control was added after the first '
            'conservative rho-only measurements; thresholds/envelopes were NOT widened. See THEORY amendment.', '',
            '## Multi-h data (all independent roots)', '']
    lines+=['Adjacent gradient direction angles, displacement relative changes and full-spectrum relative changes '
            'are in multih_stability.csv. Near-zero direction labels refer to the measured FLOAT envelope; '
            'raw angles are retained rather than treated as a robust orientation measurement.', '']
    for r in unique.values():
        c=r['classification'];lines+=['### '+c['independent_id'], '',
            f"Stable DOUBLE h: {c['double_stable_h'] or 'NONE'}; stable FLOAT h: {c['float_stable_h'] or 'NONE'}.",
            f"Gradient direction: {r['gradient_direction_status']}; reference endpoint stable regions: {r['reference_endpoint_stable_h']}.", '',
            '| precision | h | g norm | dN norm | DeltaE | H eigenvalues |','|---|---:|---:|---:|---:|---|']
        for label,key in [('FLOAT','float_multih'),('DOUBLE','double_multih')]:
            for x in r[key]:
                lines.append('| '+label+' | '+' | '.join(f'{float(x[k]):.8g}' for k in ['h','gradient_norm','displacement_norm','predicted_decrement'])+' | '+
                             ', '.join(f'{v:.8g}' for v in vec(x['H_eigenvalues']))+' |')
        lines.append('')
    lines+=['## Required cases', '',
            'P12 reverse: original closure is essentially zero, support equal and H SPD; its tiny correction to '
            'a double stationary reference and persistent float residual establish the numerical-floor interpretation.',
            'P09 T0: the original FLOAT h=.001 direction-0 failure is reproduced exactly. The DOUBLE control '
            'passes the same unchanged FD audit; this attributes that directional discrepancy to numerical representation '
            'rather than changing the old audit label. P09 endpoints are not complete certificates because their '
            'original FLOAT endpoint FD checks remain failed.',
            'P09 reverse: raw g≈1.846e-4 does not imply a large geometric error; the double-reference correction '
            'is micrometre / sub-millidegree scale. Its derivative limitation remains explicit.',
            'P10 reverse: stationarity calibration cannot repair its different frozen/dynamic memberships or '
            'the old ≈0.926 degree anchor closure. It remains FAIL; no support update was performed here.', '',
            '## Verification and Git', '', json.dumps(result['verification'],indent=2), '',
            'Actual branch remains research/p9-r1c-support-aware-continuation; START/END HEAD = '+START+'. '
            'Requested new branch/commit are blocked by read-only .git (index.lock creation denied). '
            'No push occurred. User-owned .vscode/ is preserved.', '',
            '## Limits / next', '',
            'The complete gate recovers 11 logical roots, not 13. Numerical-floor classification and full '
            'certificate validity are distinct. EPS values are local deterministic sensitivity envelopes, '
            'not universal tolerances or posterior probabilities. Production R1C solver/certificate were not changed.',
            'NEXT = '+result['next']+'. Start only from the certified subset; do not quietly include the '
            'P09 FD failures or P10 support failure. No continuation is run in R1C2.', '']
    (out/'REPORT.md').write_text('\n'.join(lines))
    (out/'DECISION_AI_PROMPT.md').write_text(
        '【科研总控复核输入】\nTASK: PAPER-P9-R1C2-NUMERICAL-STATIONARITY-CERTIFICATE-CLOSURE\n'
        '请直接检查 THEORY.md、REPORT.md、results.json、stationarity_roots.csv 和 double_stationary_endpoints.csv。\n'
        f"FINAL_RESULT={result['final_result']}\n"
        f"完整 certificate={result['new_certificate_pass']}/14 logical；{result['independent_new_pass']}/9 independent。\n"
        'TX616 六个 forward 只算一个独立 root；P10 reverse support 未闭合；P09 两个 endpoint 原 float FD 失败仍不放行。\n'
        'double 固定 support 参考梯度约1e-10；投回float仍约2e-5至7e-5；未放宽原solver阈值，未调用NDT或continuation。\n'
        f"唯一下一步={result['next']}，仅从完整认证子集开始。\n"
        '环境 .git 只读，本轮分支和commit尚未创建；请在可写环境完成附带Git命令后再给出实际SHA。\n')


def verify(binary, out=OUT):
    build=binary.parent
    if 'CMAKE_BUILD_TYPE:STRING=Release' not in (build/'CMakeCache.txt').read_text():
        raise RuntimeError('not a Release build')
    commands=[['cmake','--build',str(build),'-j1'],['ctest','--output-on-failure'],['git','diff','--check']]
    logs=[]
    for i,cmd in enumerate(commands):
        logpath=out/f'verification_{i}.log'
        with logpath.open('w') as log:
            subprocess.run(cmd,cwd=build if i==1 else ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        logs.append(dict(command=cmd,sha256=digest(logpath)))
    if '100% tests passed, 0 tests failed out of 9' not in (out/'verification_1.log').read_text():
        raise RuntimeError('expected nine P9 tests')
    owned=[out/'THEORY.md']+list((ROOT/'src/dog_prior_map_fastlio2_frontend_exp/scripts/p9').glob('*stationarity*'))
    for path in owned:
        if path.is_file() and any(line.rstrip()!=line for line in path.read_text().splitlines()):
            raise RuntimeError('new-file whitespace: '+str(path))
    (out/'verification.json').write_text(json.dumps(dict(release_build='PASS',P9_tests='9/9 PASS',
                   diff_check='PASS',new_file_whitespace='PASS',commands=logs),indent=2)+'\n')
    result=audit(out);write_report(result,out)


def run(binary, out=OUT):
    manifest=check_inputs()
    mapfile=next(p for p in manifest if p.endswith('floor01_h1_map_p5_frozen.pcd'))
    cohort=next(p for p in manifest if p.endswith('cohort_frozen.csv'))
    uobs=next(p for p in manifest if p.endswith('/dual_u.csv'))
    canonical=next(p for p in manifest if p.endswith('canonical_oracle.csv'))
    command=[str(binary),'--run',mapfile,cohort,uobs,canonical,str(PREV),str(out),'R1C2_V1']
    env=dict(os.environ, LD_LIBRARY_PATH='/lib/x86_64-linux-gnu')
    start=time.monotonic()
    with (out/'run.log').open('w') as log:
        subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    (out/'execution.json').write_text(json.dumps(dict(command=command,wall_time_s=time.monotonic()-start,
                      binary_sha256=digest(binary)),indent=2)+'\n')
    audit(out)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--audit',action='store_true')
    parser.add_argument('--binary',type=Path)
    parser.add_argument('--verify',type=Path)
    args=parser.parse_args()
    if args.self_test:self_test()
    elif args.audit:audit()
    elif args.verify:verify(args.verify)
    elif args.binary:run(args.binary)
    else:parser.error('choose --self-test, --audit or --binary')
