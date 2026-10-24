#!/usr/bin/env python3
"""Build documents only; no ROS/registration/data pipeline is imported."""
from pathlib import Path
import os,subprocess,shutil,json,hashlib
D=Path(__file__).resolve().parents[1]
engine=Path(os.environ.get('P62_R27_TECTONIC','/home/jian/livox_ws/p62_r27_typesetting_assets/tectonic'))
records=[]
for name in ['PAPER_SUBMISSION_DRAFT','SUPPLEMENTARY_MATERIAL']:
 cmd=[str(engine),'-k','--keep-logs','--outdir','build',name+'.tex']
 with (D/'build'/(name+'_build.log')).open('w') as log:
  result=subprocess.run(cmd,cwd=D,stdout=log,stderr=subprocess.STDOUT)
 records.append({'command':cmd,'cwd':str(D),'exit_code':result.returncode})
 if result.returncode:raise SystemExit('Document build failed; inspect '+name+'_build.log')
 target='PAPER_SUBMISSION_PDF.pdf' if name=='PAPER_SUBMISSION_DRAFT' else name+'.pdf'
 shutil.copyfile(D/'build'/(name+'.pdf'),D/target)
 if name=='PAPER_SUBMISSION_DRAFT':shutil.copyfile(D/'build'/(name+'.bbl'),D/'REFERENCES_COMPILED.bbl')
(D/'BUILD_RECEIPT.json').write_text(json.dumps({'engine':'Tectonic0.15.0','engine_sha256':hashlib.sha256(engine.read_bytes()).hexdigest(),'commands':records,'new_native_calls':0},indent=2)+'\n')
print('Typesetting complete')
