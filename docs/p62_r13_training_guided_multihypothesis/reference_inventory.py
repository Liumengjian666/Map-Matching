#!/usr/bin/env python3
"""One bounded name/metadata inventory, no reference fitting or data-set audit."""
import json
import os
from pathlib import Path
import re
import time

DOC=Path(__file__).resolve().parent
ROOTS=[Path('/home/jian/rosbag/loop2'),Path('/home/jian/livox_ws/dog_visual_loc_ws')]
PATTERN=re.compile(r'ground[_ -]?truth|mocap|vicon|optitrack|rtk|gnss|gps|survey|qualisys|真值|测量',re.I)

def main():
    target=DOC/'LOCAL_REFERENCE_INVENTORY.json'
    if target.exists():raise RuntimeError('one bounded inventory only')
    found=[];visited=0;start=time.monotonic();truncated=False
    for root in ROOTS:
        for directory,dirs,files in os.walk(root,followlinks=False):
            relative=Path(directory).relative_to(root)
            dirs[:]=sorted(d for d in dirs if d not in ['.git','build','devel','install','node_modules'] and len(relative.parts)<4)
            for name in sorted(files):
                visited+=1;p=Path(directory)/name
                if PATTERN.search(str(p.relative_to(root))):
                    try:found.append(dict(path=str(p),bytes=p.stat().st_size,suffix=p.suffix,content_read=False))
                    except OSError as e:found.append(dict(path=str(p),error=str(e),content_read=False))
            if time.monotonic()-start>30 or visited>20000:truncated=True;break
        if truncated:break
    with target.open('x') as f:json.dump(dict(roots=list(map(str,ROOTS)),max_depth=4,seconds_limit=30,file_limit=20000,
        files_considered=visited,candidates=found,truncated=truncated,method='one metadata/name inventory; no symlink following; no fitting or GT data parsing'),f,indent=2,ensure_ascii=False);f.write('\n')
    print(json.dumps(dict(files_considered=visited,candidates=found,truncated=truncated),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
