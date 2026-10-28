"""Archive sealed run byte-exactly (gzip only); never delete or overwrite originals."""
import gzip,hashlib,json,shutil
from pathlib import Path
A=Path(__file__).resolve().parents[1];D=Path('/home/jian/livox_ws/p63_r10_imu_full_hard_data/full_01');O=A/'raw/full_01';O.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'ONLINE_SEAL.json').read_text());assert all(sha(D/p)==v for p,v in seal['files'].items())
manifest=[]
for p in sorted(D.iterdir()):
    if not p.is_file():continue
    out=O/(p.name+'.gz' if p.suffix in ['.csv','.jsonl','.log'] else p.name)
    data=p.read_bytes()
    if out.suffix=='.gz':out.write_bytes(gzip.compress(data,compresslevel=9,mtime=0));assert gzip.decompress(out.read_bytes())==data
    else:out.write_bytes(data)
    manifest.append({'source':str(p),'source_sha256':sha(p),'archive':str(out.relative_to(A)),'archive_sha256':sha(out),'bytes':len(data)})
(A/'RAW_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('PASS',len(manifest),'raw artifacts, original files retained, gzip content verified')
