#!/usr/bin/env python3
"""Read ZIP metadata.yaml entries only; neither point clouds nor IMU payloads are downloaded."""
import hashlib,json,struct,zlib,time
from pathlib import Path
import requests,yaml
P=Path(__file__).resolve().parents[1];receipt=json.loads((P/'sources/OUTDOOR_ASSET_METADATA.json').read_text())
results=[]
for item in receipt['selected_files']:
 tailpath=P/'sources'/(item['name']+'.directory_tail.bin')
 if not tailpath.exists():continue
 data=tailpath.read_bytes();start=item['bytes']-len(data);end=data.rfind(b'PK\x05\x06');fields=struct.unpack_from('<4s4H2IH',data,end);pos=fields[6]-start
 for _ in range(fields[4]):
  v=struct.unpack_from('<4s6H3I5H2I',data,pos)
  name=data[pos+46:pos+46+v[10]].decode();pos+=46+v[10]+v[11]+v[12]
  if not name.endswith('metadata.yaml'):continue
  offset=v[16];extra_read=0
  if offset>=start:
   local=data[offset-start:]
  else:
   limit=8192
   response=requests.get(item['url'],headers={'Range':f'bytes={offset}-{offset+limit-1}','Accept-Encoding':'identity'},stream=True,timeout=(15,30))
   if response.status_code!=206:response.close();raise RuntimeError('refuse non-range response')
   local=bytearray()
   for b in response.iter_content(2048):
    local.extend(b)
    if len(local)>limit:response.close();raise RuntimeError('metadata range cap')
   response.close();extra_read=len(local)
  h=struct.unpack_from('<4s5H3I2H',local,0);assert h[0]==b'PK\x03\x04'
  begin=30+h[9]+h[10];payload=local[begin:begin+v[8]];assert len(payload)==v[8]
  raw=zlib.decompress(payload,-15) if v[4]==8 else bytes(payload)
  assert len(raw)==v[9] and zlib.crc32(raw)==v[7]
  dest=P/'sources'/(item['name']+'.metadata.yaml');dest.write_bytes(raw)
  meta=yaml.safe_load(raw)['rosbag2_bagfile_information']
  results.append({'archive':item['name'],'metadata_entry':name,'sha256':hashlib.sha256(raw).hexdigest(),'crc_verified':True,'additional_range_bytes':extra_read,'metadata':meta})
(P/'sources/BAG_METADATA_ONLY.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
