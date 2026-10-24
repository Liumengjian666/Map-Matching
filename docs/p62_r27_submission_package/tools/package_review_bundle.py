#!/usr/bin/env python3
"""Package document sources and their per-file hashes; never submit/upload."""
from pathlib import Path
import subprocess,hashlib,zipfile,json
D=Path(__file__).resolve().parents[1];R=D.parents[1]
name='P62_R27_ADVISOR_REVIEW_PACKAGE.zip';excluded={name,'checksums.sha256'}
files=[]
for p in sorted(D.rglob('*')):
 if not p.is_file() or p.name in excluded:continue
 rel=p.relative_to(D)
 if rel.parts[0]=='build' and p.name not in ['PAPER_SUBMISSION_DRAFT_build.log','SUPPLEMENTARY_MATERIAL_build.log','package_validation.log']:continue
 files.append(p)
manifest='\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(D)) for p in files)+'\n'
with zipfile.ZipFile(D/name,'w',compression=zipfile.ZIP_DEFLATED) as z:
 for p in files:z.write(p,p.relative_to(D))
 z.writestr('MANIFEST.sha256',manifest)
with zipfile.ZipFile(D/name) as z:
 assert z.testzip() is None
 for line in z.read('MANIFEST.sha256').decode().splitlines():
  h,rel=line.split('  ',1);assert hashlib.sha256(z.read(rel)).hexdigest()==h
all_files=files+[D/name]
(D/'checksums.sha256').write_text('\n'.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(D)) for p in all_files)+'\n')
print('bundle',len(files),'files plus manifest; bytes',(D/name).stat().st_size,'checksummed',len(all_files))
