"""Bind committed research source, actual executable, inputs and native runtime."""
import hashlib
import json
from pathlib import Path
import subprocess
from prepare import HERE,WS,sha,save


def main():
    if (HERE/'FORMAL_STARTED.json').exists():raise RuntimeError('Formal stage already attempted')
    head=subprocess.check_output(['git','-C',str(WS),'rev-parse','HEAD'],text=True).strip()
    prereg=json.loads((HERE/'TEST_FRAME_PREREGISTRATION.json').read_text())
    data=Path(prereg['data_root']);binary=data/'build/pilot'
    dependencies=[p for p in HERE.rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.hpp')]
    dependencies += [HERE/'tools/CMakeLists.txt',HERE.parent/'inspect_inputs.py']
    for folder in ['p62_r6_dog_shadow_prototype','p62_r8_structural_admission',
                   'p62_r9_coverage_aware_recovery_v2','p62_r10_feasibility_event_recovery',
                   'p62_r11_cause_aware_recovery','p62_r12_feasibility_preserving_pool',
                   'p62_r13_training_guided_multihypothesis']:
        dependencies.extend(sorted((WS/'docs'/folder/'tools').glob('*.hpp')))
    files=[]
    for p in sorted(set(dependencies)):
        actual=sha(p);rel=str(p.relative_to(WS))
        committed=subprocess.check_output(['git','-C',str(WS),'show',f'{head}:{rel}'])
        if hashlib.sha256(committed).hexdigest()!=actual:raise RuntimeError('Uncommitted source/dependency: '+rel)
        files.append({'path':str(p),'sha256':actual})
    for p in [Path('/usr/include/pcl-1.10/pcl/registration/impl/ndt.hpp'),
              Path('/usr/include/pcl-1.10/pcl/registration/ndt.h'),
              Path('/usr/include/pcl-1.10/pcl/pcl_config.h'),
              Path('/lib/x86_64-linux-gnu/libusb-1.0.so.0'),
              *[Path('/lib/x86_64-linux-gnu')/f'libpcl_{n}.so.1.10' for n in ['common','registration','io','filters','kdtree']]]:
        files.append({'path':str(p),'sha256':sha(p)})
    save('EXECUTION_FREEZE.json',{'code_sha':head,'binary_path':str(binary),'binary_sha256':sha(binary),
        'source_files':files,'preregistration_sha256':sha(HERE/'TEST_FRAME_PREREGISTRATION.json'),
        'scans_csv_sha256':sha(HERE/'SCANS.csv'),'anchor_matrix_sha256':sha(HERE/'ANCHOR_MATRIX.txt'),
        'map_sha256':json.loads((HERE/'PREFLIGHT_PASS.json').read_text())['map_sha256'],
        'native_runtime_env':{'LD_PRELOAD':'/lib/x86_64-linux-gnu/libusb-1.0.so.0','LD_BIND_NOW':'1'},
        'pcl_version':'1.10.0','NDT_parameters':{'resolution':.8,'step_size':.08,'epsilon':.001,'max_iterations':40,'outlier_ratio':.55},
        'formal_frames':20,'full_calls':20,'split_calls':20,'max_extra_calls':20,
        'reference_pose_values_available_to_native':False,
        'coordinate_assumption':'PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION'})
    print(json.dumps({'code_sha':head,'binary_sha256':sha(binary),'source_files':len(files)},indent=2))


if __name__=='__main__':main()
