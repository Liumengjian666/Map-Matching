from pathlib import Path
import csv
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
PREV=HERE.parent/'p64_r3_q1_observation_contribution'
sys.path.insert(0,str(PREV))
from freeze import PAIRS,digest,read
from contribution_selection import fill_budget,salience_order,voxel_interleave
import geometry as g

HIST=Path('/home/jian/p64_r3_q1_ws/run')
R2=Path('/home/jian/p64_r2_q2_ws')
FRAMES=('H307','H515','H309','O164')
BUDGETS=(20,50,80)
FIELDS=[f'T{i}{j}' for i in range(4) for j in range(4)]

def output(path, rows):
    fields=list(dict.fromkeys(key for row in rows for key in row))
    with path.open('x') as stream:
        writer=csv.DictWriter(stream,fields,lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)

def packet(frame, path=HIST):
    with np.load(path/f'{frame}.npz') as archive:
        return {k:archive[k] for k in archive.files}

def archived_pair(frame):
    r=read(PREV/'fixed_candidate_ranking.csv')
    return next(x for x in r if x['frame']==frame and x['pair']==PAIRS[frame][0][0] and x['method']=='GEO0' and x['budget_percent']=='50')

def mask(frame):
    with np.load(HIST/'blind/shared_masks.npz') as masks:
        return masks[archived_pair(frame)['mask_id']]

def pcd(path, xyz, value):
    # value 0 unknown, 1 green, 2 red, 3 blue.
    rgb=np.array([0x888888,0x2ab470,0xf45543,0x3487de],dtype='<u4')
    vals=rgb[value].view('<f4')
    with path.open('wb') as stream:
        stream.write((f'VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                      f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n').encode())
        stream.write(np.column_stack((xyz,vals)).astype('<f4').tobytes())
