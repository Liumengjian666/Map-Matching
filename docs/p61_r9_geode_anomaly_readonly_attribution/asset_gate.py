#!/usr/bin/python3
"""Read-only identity gate. Never imports ROS or executes a frozen binary."""
import argparse,hashlib,json,subprocess,time
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
R8=REPO/'docs/p61_r8_geode_full_return_control'
RUN=Path('/home/jian/livox_ws/p61_r8_geode_runs')
BASE='eba814000a06a2b110acf3835247cd6cffe9db6b'
ORIGINAL=Path('/home/jian/geode/Shield_tunnel1_gamma/Shield_tunnel1_gamma.bag')
MAP=Path('/home/jian/livox_ws/p61_r5_geode_runs/formal_mapping/frozen_map.pcd')
BAG_SHA='162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091'
MAP_SHA='4a279d822fd6090d3aa3fc0cec989b4f6fde5c9a43df476a29dba4494f6821b4'
MANIFEST_SHA='4444883c441f0c3c08650bf629c05812d34b1a069c7102d565053a47b976c76d'
CONFIG_SHA='9d2e32ebf5bd89230d46b7af7b5d0cf9115cbe72d600634899337eab09eba0d3'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def map_xyz(path):
    with Path(path).open('rb') as f:
        header={}
        while True:
            line=f.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key,*values=line.split();header[key]=values
                if key=='DATA':break
        assert header['DATA']==['binary']
        assert header['FIELDS']==['x','y','z','intensity','normal_x','normal_y','normal_z','curvature']
        assert header['SIZE']==['4']*8 and header['TYPE']==['F']*8 and header['COUNT']==['1']*8
        count=int(header['POINTS'][0]);data=np.frombuffer(f.read(),dtype='<f4')
        assert data.size==count*8
        xyz=data.reshape(-1,8)[:,:3].astype(float)
        assert count==321053 and np.isfinite(xyz).all()
        return xyz,header


def verify(output):
    records=[]
    def check(name,path,expected,source,size=None):
        path=Path(path);actual=sha(path)
        row=dict(asset=name,path=str(path.resolve()),size_bytes=path.stat().st_size,
                 expected_sha256=expected,sha256=actual,frozen_source=source,
                 status='PASS' if len(expected)==64 and actual==expected and (size is None or path.stat().st_size==size) else 'FAIL')
        records.append(row)
        if row['status']!='PASS':raise RuntimeError('ASSET_IDENTITY_CONTRACT_BLOCKED: '+str(path))
    try:
        check('original_bag',ORIGINAL,BAG_SHA,'corrected R9 task + R8 input manifest')
        check('archived_input_manifest',R8/'input_manifest.json',MANIFEST_SHA,'R8 task literal')
        m=json.loads((R8/'input_manifest.json').read_text())
        assert m['inputs'][str(ORIGINAL)]==BAG_SHA
        assert m['bag_sha256']=='a138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331'
        check('prepared_input_bag',RUN/'input/full_return.bag',m['bag_sha256'],'R8 manifest bag_sha256')
        check('runtime_input_manifest',RUN/'input/manifest.json',MANIFEST_SHA,'R8 input manifest identity')
        for p,d in m['inputs'].items():check('declared_input',p,d,'R8 manifest inputs')
        assert m['scan_count']==2441 and (m['first_index'],m['last_index'])==(2986,5426) and m['gt_read'] is False
        assert [(int(x['scan_index']),x['admitted']) for x in m['boundary']]==[(2985,False),(2986,True),(5426,True),(5427,False)]
        assert m['first_header_ns']==1706671587900364160 and m['last_end_ns']==1706671831999706132
        for path,key in [('scans.csv','scans_sha256'),('imu.csv','imu_sha256'),('initialization.yaml','config_sha256')]:
            check('runtime_input_'+path,RUN/'input'/path,m[key],'R8 input manifest '+key)
        check('R9_R6_initialization',REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json','9a5532ce3b7162e92b6a711c509d9a8e52d28eff81f6515fb3618067582b733d','R9 task')
        check('R9_R6_covariance',REPO/'docs/p61_r6_geode_causal_init_bridge/initial_covariance.csv','55ac110fd02620c494e916fdb575bd514b734245b26d09764fc7de1116670f39','R9 task')
        check('R9_R2_ledger',REPO/'docs/p61_r2_geode_real_data_run/scan_ledger.csv','21cba76ec059016700b5ac3f0a0851f769b338ee4550e19108910d3efd3902fb','R9 task')
        config='src/dog_prior_map_localization/config/geode_r7_baseline.yaml'
        for ws in (REPO,Path('/home/jian/livox_ws/p61_r7_geode_baseline_ws'),Path('/home/jian/livox_ws/p61_r8_geode_full_return_ws')):
            check('numeric_config',ws/config,CONFIG_SHA,'R8 runtime identity')
        identity=json.loads((R8/'run_identity.json').read_text())
        for p,d in identity['binaries'].items():check('runtime_binary_only_hash',p,d,'R8 run identity, not executed')
        for name,v in json.loads((R8/'output_freeze.json').read_text()).items():
            check('frozen_raw_output',RUN/name,v['sha256'],'R8 output_freeze.json',v['size'])
        for name,d in json.loads((R8/'archive_hashes.json').read_text()).items():
            check('archived_R8_output',R8/name,d,'R8 archive_hashes.json')
        # Also pin all R8 tracked archival files to the enclosing frozen commit.
        names=subprocess.check_output(['git','ls-tree','-r','--name-only',BASE,'--','docs/p61_r8_geode_full_return_control'],cwd=REPO,text=True).splitlines()
        for name in names:
            blob=subprocess.check_output(['git','show',BASE+':'+name],cwd=REPO)
            check('R8_git_blob',REPO/name,hashlib.sha256(blob).hexdigest(),'R8 archive commit '+BASE)
        xyz,header=map_xyz(MAP)
        summary=dict(status='PASS',map_points=len(xyz),map_readable=True,boundary_contract='PASS',assets=records)
    except Exception as e:
        summary=dict(status='FAIL',failure=repr(e),assets=records)
    summary.update(method='streaming hashlib.sha256, 1MiB blocks; no binary execution',unix_time=time.time())
    with Path(output).open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
    if summary['status']!='PASS':raise RuntimeError(summary['failure'])
    print('PASS',len(records),'asset comparisons; map321053; binaries not executed',flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output');args=parser.parse_args();verify(args.output)
