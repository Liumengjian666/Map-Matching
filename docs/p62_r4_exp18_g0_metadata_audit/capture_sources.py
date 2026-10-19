"""Bounded primary-source snapshots; no sensor downloads or runtime dependency install."""
import hashlib
import importlib.metadata
import json
import subprocess
import sys
import urllib.request
from pathlib import Path
from assets import DOC,guard

DRIVER='f574bb8a680e215a6ade9bf35e0898932b2a2234'
FAQ='e4aacf7c111ab832a0bdffe3f069255454068540'
REV='e62017f907007fdc5ab8c721842e4ae7359d7f49'
SOURCE_PATHS=[
 ('hesai_point_types.h','https://raw.githubusercontent.com/HesaiTechnology/HesaiLidar_General_ROS/'+DRIVER+'/src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include/pandarGeneral/point_types.h'),
 ('hesai_internal.cc','https://raw.githubusercontent.com/HesaiTechnology/HesaiLidar_General_ROS/'+DRIVER+'/src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral_internal.cc'),
 ('hilti_readme.md','https://raw.githubusercontent.com/Hilti-Research/hilti-slam-challenge-2022/'+FAQ+'/README.md'),
 ('dataset_readme.md','https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/'+REV+'/README.md')]


def row(path,source):
    data=path.read_bytes()
    return {'path':str(path),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'source':source}


def main():
    directory=DOC/'sources';directory.mkdir(exist_ok=False);rows=[]
    for name,url in SOURCE_PATHS:
        guard();path=directory/name
        try:
            with urllib.request.urlopen(url,timeout=20) as response:
                data=response.read(2*1024**2+1)
                if response.status!=200 or len(data)>2*1024**2:raise ValueError('SOURCE_RESPONSE_LIMIT')
            guard();path.write_bytes(data);rows.append(row(path,url))
        except Exception as error:rows.append({'source':url,'status':'UNAVAILABLE','error':repr(error)})
    repo='/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/sources/phasma_description'
    commit='79002026e9b229cf9b4e95a403eaec4f00acfd79'
    for name in ('phasma.urdf.xacro','phasma.urdf','calibrations/20220322_calibration.yaml'):
        guard();data=subprocess.check_output(['git','-C',repo,'show',commit+':urdf/'+name],timeout=15)
        path=directory/Path(name).name;path.write_bytes(data)
        rows.append(row(path,'https://github.com/Hilti-Research/phasma_description/blob/'+commit+'/urdf/'+name))
    parser=Path('/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/parser')
    modules=[row(p,'Existing R2 isolated target; no R4 installation') for p in sorted(parser.rglob('*'))
             if p.is_file() and '__pycache__' not in p.parts]
    sys.path.insert(0,str(parser));import rosbag,numpy,yaml,pyquaternion
    environment={'python':sys.version,'executable':sys.executable,'rosbag_path':rosbag.__file__,
        'numpy_version':numpy.__version__,'pyquaternion_version':importlib.metadata.version('pyquaternion'),
        'yaml_version':yaml.__version__,'pye57_version':'0.4.5','pye57_files':modules,
        'new_packages_installed':False,'new_venv_created':False,
        'original_wheel_file':'NOT_RETAINED_IN_EXISTING_R2_TARGET; installed-file SHA256 recorded instead',
        'driver_snapshot_commit':DRIVER,'actual_recording_driver_commit':'NOT_CERTIFIED',
        'sources':rows,'high_level_pye57_constructor_used':False}
    guard();(DOC/'parser_environment.json').write_text(json.dumps(environment,indent=2)+'\n')
    print(json.dumps({'source_snapshots':len(rows),'installed_parser_files_hashed':len(modules)}))


if __name__=='__main__':main()
