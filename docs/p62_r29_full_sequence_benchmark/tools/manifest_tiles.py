#!/usr/bin/env python3
from pathlib import Path
import csv,hashlib,json
D=Path('/home/jian/rosbag/public_datasets/koide_p62_r29')
for role in ['easy','hard']:
 p=D/('map_indoor_'+role+'_tiles');rows=list(csv.DictReader((p/'tile_table.csv').open()));tiles=[]
 for r in rows:
  t={k:float(r[k]) for k in ['x_min','x_max','y_min','y_max','z_min','z_max']}
  t.update(tile_id=r['tile_id'],tile_path=r['path'],tile_x=int(r['tile_x']),tile_y=int(r['tile_y']),point_count=int(r['point_count']),sha256=hashlib.sha256(Path(r['path']).read_bytes()).hexdigest());tiles.append(t)
 d={'schema_version':1,'tile_size_m':20.,'grid_origin_x':0.,'grid_origin_y':0.,'global_target_point_count':sum(t['point_count'] for t in tiles),'preprocessing':{'filter':'PCL VoxelGrid XYZ','passes':2,'leaf_xyz_m':[.15,.15,.15]},'tiles':tiles,'source_map':str(D/('map_indoor_'+role+'.ply'))}
 (p/'tile_manifest.json').write_text(json.dumps(d,indent=2)+'\n');print(role,len(tiles),d['global_target_point_count'])
