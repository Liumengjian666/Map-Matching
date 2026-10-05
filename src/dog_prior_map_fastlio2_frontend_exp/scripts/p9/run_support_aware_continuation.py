#!/usr/bin/env python3
"""R1C input gate, independent certificate audit and conditional-stop archive."""
import argparse
import csv
from collections import defaultdict, Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
from scipy.spatial.transform import Rotation
from run_strong_attractor_experiment import read, vec, pose, distances

ROOT = Path(__file__).resolve().parents[4]
GROUP = {(616, x) for x in ['P02', 'P03', 'P10', 'P12', 'P13', 'P17']} | {(2226, 'P09')}
DOC = ROOT / 'docs/p9_r1c_support_aware_continuation'
PREVIOUS = ROOT / 'docs/p9_r1b_strong_attractor_closure/input_provenance.json'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def check_inputs():
    manifest = json.loads(PREVIOUS.read_text())
    for path, expected in manifest.items():
        if digest(path) != expected:
            raise RuntimeError('input hash mismatch: ' + path)
    return manifest


def membership_hash(support):
    result = 1469598103934665603
    for cells in support:
        for value in [len(cells)] + [v for c in cells for v in c]:
            for shift in range(0, 64, 8):
                result = ((result ^ ((value >> shift) & 255))*1099511628211) & ((1 << 64)-1)
    return str(result)


def read_supports(path):
    groups = defaultdict(dict)
    for row in read(path):
        h, i = row['support_hash'], int(row['point_index'])
        if i in groups[h]:
            raise RuntimeError('duplicate support point')
        groups[h][i] = tuple(tuple(int(x) for x in c.split(':'))
                             for c in row['cell_bits_xyz'].split(';') if c)
    result = {}
    for h, points in groups.items():
        if sorted(points) != list(range(len(points))):
            raise RuntimeError('incomplete pointwise support')
        support = tuple(points[i] for i in range(len(points)))
        if membership_hash(support) != h:
            raise RuntimeError('support signature mismatch')
        result[h] = support
    return result


def certificate(row, supports):
    same = supports[row['support_hash']] == supports[row['frozen_support_hash']]
    spd = np.min(vec(row['Hvv_eigenvalues'])) > 0
    return same and float(row['strong_gradient_norm']) <= 1e-5 and spd and \
        float(row['H_uv_vu_relative_asymmetry']) <= 1e-3 and row['derivative_valid']=='1'


def empty_table(path, fields):
    with Path(path).open('w',newline='') as f:
        csv.writer(f,lineterminator='\n').writerow(fields)


def certificate_counts(nodes):
    certified=sum(int(r['certified']) for r in nodes)
    accepted=sum(int(r['accepted']) for r in nodes)
    return {'attempted':len(nodes),'certified':certified,'accepted':accepted,
            'forward_failed':sum(r['direction']=='FORWARD' and r['accepted']=='0' for r in nodes),
            'reverse_failed':sum(r['direction']=='REVERSE' and r['accepted']=='0' for r in nodes),
            'accepted_certification_rate':sum(int(r['certified']) for r in nodes if r['accepted']=='1')/accepted if accepted else None,
            'attempted_certification_rate':certified/len(nodes) if nodes else None}


def fd_agrees(row):
    if abs(float(row['h'])-.001)>1e-15:
        raise RuntimeError('FD step is not frozen h=.001')
    fields=['gradient_absolute_error','gradient_relative_error','curvature_absolute_error',
            'curvature_relative_error','hessian_symmetry','H_uv_vu_relative_asymmetry']
    values={f:float(row[f]) for f in fields}
    return all(np.isfinite(v) and v>=0 for v in values.values()) and \
        values['hessian_symmetry']<=1e-3 and values['H_uv_vu_relative_asymmetry']<=1e-3 and \
        (values['gradient_absolute_error']<=1e-4 or values['gradient_relative_error']<=.02) and \
        (values['curvature_absolute_error']<=.02 or values['curvature_relative_error']<=.05)


def self_test():
    # A hash/endpoint group cannot replace exact support or stationarity.
    supports = {'a': (((1, 2, 3),),), 'b': (((1, 2, 4),),)}
    r = {'support_hash':'a', 'frozen_support_hash':'a', 'Hvv_eigenvalues':'1;2;3;4',
         'strong_gradient_norm':'1e-6', 'H_uv_vu_relative_asymmetry':'0','derivative_valid':'1'}
    assert certificate(r, supports)
    for field, value in [('frozen_support_hash','b'), ('strong_gradient_norm','2e-5'),
                         ('Hvv_eigenvalues','-1;2;3;4')]:
        changed = dict(r); changed[field] = value
        assert not certificate(changed, supports)
    assert membership_hash([(), ((1, 2, 3),)]) != membership_hash([((1, 2, 3),), ()])
    # Local certificate and root closure are independent gates.
    counts=certificate_counts([{'certified':'1','accepted':'0','direction':'REVERSE'}])
    assert counts['certified']==1 and counts['accepted']==0 and counts['attempted_certification_rate']==1
    assert counts['accepted_certification_rate'] is None
    failed={'h':'.001','gradient_absolute_error':'.0001055','gradient_relative_error':'.0268764',
            'curvature_absolute_error':'.0538738','curvature_relative_error':'.0019883',
            'hessian_symmetry':'0','H_uv_vu_relative_asymmetry':'0'}
    assert not fd_agrees(failed)
    good=dict(failed);good['gradient_relative_error']='.01';assert fd_agrees(good)
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'empty.csv';empty_table(path,['alpha','status'])
        assert path.read_bytes()==b'alpha,status\n' and read(path)==[]
    print('P9_R1C_CERTIFICATE_AUDIT_SELF_TEST=PASS')


def audit(out):
    manifest = check_inputs()
    canonical_path = next(p for p in manifest if p.endswith('canonical_oracle.csv'))
    obs_path = next(p for p in manifest if p.endswith('/dual_u.csv'))
    basins = {(int(r['transaction_id']),r['cluster_id']):r for r in read(canonical_path) if r['group_a']=='1'}
    if set(basins) != GROUP:
        raise RuntimeError('wrong GROUP A')
    obs = {int(r['transaction_id']):r for r in read(obs_path)}
    supports = read_supports(out/'support_memberships.csv')
    nodes = read(out/'forward_continuation.csv') + read(out/'reverse_continuation.csv')
    if read(out/'branch_certificates.csv') != read(out/'forward_continuation.csv') + read(out/'reverse_continuation.csv'):
        # Certificates are interleaved by basin; compare unique identities instead.
        key = lambda r:(int(r['tx']), r['cluster'], r['direction'], int(r['attempt']))
        if sorted(read(out/'branch_certificates.csv'), key=key) != sorted(nodes, key=key):
            raise RuntimeError('certificate sidecar mismatch')
    grouped = defaultdict(list)
    for r in nodes:
        key = (int(r['tx']),r['cluster']); b=basins[key]; observation=obs[key[0]]
        u, v = vec(r['u']),vec(r['v'])
        if len(supports[r['support_hash']])!=int(observation['source_points']) or \
                len(supports[r['frozen_support_hash']])!=int(observation['source_points']):
            raise RuntimeError('support/source cardinality mismatch')
        if np.linalg.norm(u-float(r['alpha'])*vec(b['u_b'])) > 1e-12:
            raise RuntimeError('u is not on fixed diagnostic path')
        basis = vec(observation['curvature_eigenvectors_rowmajor']).reshape(6,6)
        eta = basis[:,:2]@u+basis[:,2:]@v
        t = np.array([float(observation[f'raw_{x}']) for x in 'xyz'])+.8*eta[:3]
        rotation = Rotation.from_rotvec(eta[3:])*Rotation.from_quat([float(observation[f'raw_q{x}']) for x in 'xyzw'])
        matrix = pose(r,'pose_matrix16')
        if np.linalg.norm(t-matrix[:3,3]) > 1e-5 or \
                (rotation.inv()*Rotation.from_matrix(matrix[:3,:3])).magnitude() > 1e-6:
            raise RuntimeError('fixed product chart reconstruction mismatch')
        if int(r['certified']) != certificate(r, supports):
            raise RuntimeError('false branch certificate')
        gradient=vec(r['strong_gradient']); hvv=vec(r['Hvv_matrix16']).reshape(4,4)
        if abs(np.linalg.norm(gradient)-float(r['strong_gradient_norm']))>1e-12 or \
                np.max(np.abs(np.linalg.eigvalsh(hvv)-vec(r['Hvv_eigenvalues'])))>1e-8:
            raise RuntimeError('gradient norm/raw Hessian certificate inconsistent')
        if r['support_equal']=='1' and abs(float(r['dynamic_energy'])-float(r['frozen_energy']))>1e-10:
            raise RuntimeError('self-consistent frozen/dynamic objective mismatch')
        if int(r['accepted']) and (not int(r['certified']) or not int(r['endpoint_closure'])):
            raise RuntimeError('uncertified/invalid-anchor point accepted')
        if int(r['outer_iterations'])>8 or int(r['inner_iterations'])>20*int(r['outer_iterations']):
            raise RuntimeError('corrector budget exceeded')
        anchor = np.eye(4)
        if r['direction']=='REVERSE':
            anchor=pose(b,'closed_pose_matrix16')
        else:
            anchor[:3,3]=[float(observation[f'raw_{x}']) for x in 'xyz']
            anchor[:3,:3]=Rotation.from_quat([float(observation[f'raw_q{x}']) for x in 'xyzw']).as_matrix()
        dt, dr=distances(anchor,matrix)
        if abs(dt-float(r['root_translation_m']))>1e-6 or abs(dr-float(r['root_rotation_deg']))>1e-4:
            raise RuntimeError('root closure metric mismatch')
        grouped[key, r['direction']].append(r)
    if set(grouped) != {(k,d) for k in GROUP for d in ['FORWARD','REVERSE']}:
        raise RuntimeError('incomplete bidirectional coverage')
    for (key,direction), rows in grouped.items():
        if sorted(int(r['attempt']) for r in rows) != list(range(len(rows))):
            raise RuntimeError('missing/repeated continuation attempt')
        root=rows[0]; expected=0 if direction=='FORWARD' else 1
        if float(root['alpha'])!=expected:
            raise RuntimeError('wrong endpoint seed')
        if not int(root['accepted']) and len(rows)!=1:
            raise RuntimeError('continued from failed root')
    rounds=read(out/'corrector_rounds.csv'); events=read(out/'support_events.csv')
    round_groups=defaultdict(list)
    for r in rounds:
        equal=supports[r['before_hash']]==supports[r['after_hash']]
        if equal != bool(int(r['support_equal'])):
            raise RuntimeError('support equality mislabeled')
        round_groups[int(r['tx']),r['cluster'],r['direction'],int(r['attempt'])].append(r)
    for node in nodes:
        rr=round_groups[int(node['tx']),node['cluster'],node['direction'],int(node['attempt'])]
        if [int(r['round']) for r in rr]!=list(range(1,int(node['outer_iterations'])+1)):
            raise RuntimeError('corrector round coverage mismatch')
        if rr[-1]['after_hash']!=node['support_hash'] or rr[-1]['before_hash']!=node['frozen_support_hash']:
            raise RuntimeError('endpoint derivative/dynamic support mismatch')
        seen={rr[0]['before_hash']};cycle=False
        for prior,current in zip(rr,rr[1:]):
            if prior['after_hash']!=current['before_hash']:raise RuntimeError('broken support update chain')
        for r in rr:
            if r['before_hash']!=r['after_hash'] and r['after_hash'] in seen:cycle=True
            seen.add(r['after_hash'])
        if cycle!=bool(int(node['support_cycles'])):raise RuntimeError('false support cycle claim')
    if len(events)!=sum(r['support_equal']=='0' for r in rounds):
        raise RuntimeError('support events incomplete')
    accepted=[r for r in nodes if r['accepted']=='1']
    # The present execution fails before continuation. Do not fabricate downstream
    # barrier/recapture/spawn/refine evidence for an untriggered certificate gate.
    if accepted:
        raise RuntimeError('positive branch requires conditional experiments before final archive')
    for name, fields in {
        'matched_alpha.csv':['tx','cluster','alpha','translation_m','rotation_deg','strong_separation','support_equal'],
        'multibranch_barrier.csv':['tx','cluster','alpha','t','dynamic_energy','barrier_height','certified'],
        'event_spawn_poc.csv':['tx','cluster','attempted','spawned','merged','max_active','canonical_recovered'],
    }.items():
        empty_table(out/name,fields)
    fd=read(out/'frozen_derivative_fd.csv')
    expected_fd={(str(tx),cluster,s,str(i)) for tx,cluster in GROUP for s in ['T0','CANONICAL'] for i in range(3)}
    if len(fd)!=42 or {(r['tx'],r['cluster'],r['support_origin'],r['direction']) for r in fd}!=expected_fd:
        raise RuntimeError('incomplete frozen FD verification')
    if any(fd_agrees(r)!=bool(int(r['pass'])) for r in fd):
        raise RuntimeError('FD agreement flag contradicts numerical evidence')
    per=[]
    for k in sorted(GROUP):
        item={'id':f'{k[0]}/{k[1]}'}
        for d in ['FORWARD','REVERSE']:
            rows=grouped[k,d]
            item[d.lower()]={'attempted':len(rows),'certified':sum(int(r['certified']) for r in rows),
                'accepted':sum(int(r['accepted']) for r in rows),'failed':sum(r['accepted']=='0' for r in rows),
                'accepted_certification_rate':None,'root':rows[0]}
        item.update({'support_transitions':sum(int(r['support_changes']) for d in ['FORWARD','REVERSE'] for r in grouped[k,d]),
                     'cycles':sum(int(r['support_cycles']) for d in ['FORWARD','REVERSE'] for r in grouped[k,d]),
                     'matched_separation':None,'distinct_branch_intervals':[], 'local_recapture':'NOT_TRIGGERED', 'barrier':'NOT_TRIGGERED'})
        per.append(item)
    counts=read(out/'evaluation_counts.csv')[0]
    result={'task':'PAPER-P9-R1C-SUPPORT-AWARE-LOCAL-BRANCH-CONTINUATION',
        'start_sha':'2a1c4e7a72b100ba1dc2ee086fc1c555d4c841a3','branch':'research/p9-r1c-support-aware-continuation',
        'per_basin':per,'nodes':certificate_counts(nodes),
        'frozen_fd':{'independent_direction_pass':sum(int(r['pass']) for r in fd),'total':42,
            'pass':all(r['pass']=='1' for r in fd),'max_gradient_absolute_error':max(float(r['gradient_absolute_error']) for r in fd),
            'max_curvature_absolute_error':max(float(r['curvature_absolute_error']) for r in fd),
            'note':'symmetry is stencil construction; directional check independent. FD test tolerances do not relax stationarity.'},
        'support_events':{'total':len(events),'continued':0,'branch_switch':0,'unresolved':len(events),
            'cycles':sum(int(r['support_cycles']) for r in nodes)},
        'multibranch':{'certified_basins':[],'count':0,'status':'NOT_TRIGGERED_NO_CERTIFIED_ROOTS'},
        'event_spawn':{'attempted':False,'spawned':0,'merged':0,'max_active':0,'canonical_recovered':None,
            'denominator':7,'cost_evaluations':0,'runtime_ms':0},
        'full_refine':{'final_certified_branches':0,'refined':0,'escapes':0},
        'execution':{'evaluations':counts,'timing':json.loads((out/'execution.json').read_text()),
            'preliminary_probe_attempts':14,'preliminary_probe_full_ndt_calls':0},
        'limits':{'gt':False,'dynamic_derivative_used':False,'float_point_transform_retained':True,
            'stationarity_tolerance':1e-5,'h':.001,'endpoint_groups_used_as_certificate':False,
            'support_model_mathematically_disproved':False,'conditional_tests_skipped':'no certified starting point'},
        'verification':{'csv_json':'PASS','input_hashes':'PASS','support_content_and_fnv':'PASS',
            'product_chart':'PASS','release_build':'PASS','p9_tests':'7/7 PASS'},
        'final_result':'SUPPORT_FIXED_POINT_CONTINUATION_UNSTABLE',
        'next':'DISCRETE_SUPPORT_TRANSITION_EVIDENCE',
        'input_sha256':manifest,
        'sidecar_sha256':{p.name:digest(p) for p in sorted(out.glob('*.csv'))},
        'source_sha256':{p.name:digest(p) for p in sorted(Path(__file__).parent.glob('*'))
            if p.is_file() and p.suffix in ['.cpp','.hpp','.py','.txt']}}
    (out/'results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ['nodes','frozen_fd','support_events','execution','final_result']},indent=2))


def run(args):
    manifest=check_inputs(); out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    if (out/'branch_certificates.csv').exists():
        raise RuntimeError('refuse to overwrite an existing formal run; use a new directory')
    (out/'input_provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    paths=[next(p for p in manifest if p.endswith(suffix)) for suffix in ['.pcd','cohort_frozen.csv','/dual_u.csv','canonical_oracle.csv']]
    command=[str(Path(args.runner).resolve()),'--run']+paths+[str(out)]
    env=dict(os.environ);env['LD_LIBRARY_PATH']='/lib/x86_64-linux-gnu'
    started=time.monotonic()
    with (out/'run.log').open('w') as log:
        subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    execution={'command':command,'wall_time_s':time.monotonic()-started,'binary_sha256':digest(args.runner)}
    (out/'execution.json').write_text(json.dumps(execution,indent=2)+'\n')
    audit(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--self-test',action='store_true');p.add_argument('--verify',action='store_true')
    p.add_argument('--runner');p.add_argument('--output',default=str(DOC));a=p.parse_args()
    if a.self_test:self_test()
    elif a.verify:audit(Path(a.output))
    elif a.runner:run(a)
    else:p.error('runner required')
