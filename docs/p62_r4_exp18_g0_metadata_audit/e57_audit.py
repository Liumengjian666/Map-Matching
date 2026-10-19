#!/usr/bin/python3
"""Read E57 using low-level read-only ImageFile; 100k-point bounded buffers."""
import csv
import json
import resource
import signal
import sys
import time
sys.dont_write_bytecode=True
sys.path.insert(0,'/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data/parser')
import numpy as np
from pye57 import libe57
from assets import ASSETS,DOC,DEADLINE,guard,diagnostic
from audit_math import quaternion_wxyz_matrix

CAPACITY=100000


def metadata(node):
    if isinstance(node,(libe57.StructureNode,libe57.VectorNode)):
        return {child.elementName():metadata(child) for child in node}
    if isinstance(node,libe57.CompressedVectorNode):
        return {'compressed_vector_count':node.childCount(),
                'prototype':metadata(libe57.StructureNode(node.prototype()))}
    if hasattr(node,'value'):return node.value()
    return {'type':str(type(node)),'note':'NONSCALAR_NOT_DECODED_AS_METADATA'}


def main():
    if not (DOC/'asset_identity_before.json').exists():raise RuntimeError('IDENTITY_GATE_REQUIRED')
    guard();started=time.monotonic()
    # Avoid pye57.E57.__init__: its exception path may remove its input.
    image=libe57.ImageFile(str(ASSETS[1][0]),'r')
    rows=[];failure=None;progress={'completed':False,'decoded_points':0}
    try:
        root=image.root();root_meta=metadata(root)
        guard();(DOC/'e57_structure.json').write_text(json.dumps(root_meta,indent=2)+'\n')
        scans=root['data3D']
        for i in range(scans.childCount()):
            guard();scan=scans[i];points=scan['points'];count=points.childCount()
            proto=libe57.StructureNode(points.prototype());fields=[n.elementName() for n in proto]
            if not all(f in fields for f in ('cartesianX','cartesianY','cartesianZ')):
                raise RuntimeError('UNSUPPORTED_COORDINATE_FIELDS')
            pose=metadata(scan['pose']) if scan.isDefined('pose') else None
            q=[pose['rotation'][k] for k in ('w','x','y','z')] if pose else [1,0,0,0]
            translation=np.array([pose['translation'][k] for k in ('x','y','z')]) if pose else np.zeros(3)
            if translation.shape!=(3,) or not np.isfinite(translation).all():
                raise RuntimeError('INVALID_SCAN_TRANSLATION')
            rotation=quaternion_wxyz_matrix(q)
            buffers=libe57.VectorSourceDestBuffer();arrays={}
            for field in fields:
                arrays[field]=np.empty(CAPACITY,dtype=np.float64)
                buffers.append(libe57.SourceDestBuffer(image,field,arrays[field],CAPACITY,True,True))
            reader=points.reader(buffers)
            decoded=0;nonfinite_xyz=0;nonfinite_fields=0;flagged_invalid=0
            low=np.full(3,np.inf);high=np.full(3,-np.inf)
            global_low=np.full(3,np.inf);global_high=np.full(3,-np.inf)
            invalid_field='cartesianInvalidState' if 'cartesianInvalidState' in fields else None
            try:
                while True:
                    guard();n=reader.read()
                    if not n:break
                    if n<0 or n>CAPACITY:raise RuntimeError('UNBOUNDED_READER_RETURN')
                    decoded+=n
                    progress={'completed':False,'scan_index':i,'decoded_points':decoded,'declared_points':count}
                    xyz=np.column_stack([arrays[f][:n] for f in ('cartesianX','cartesianY','cartesianZ')])
                    finite=np.isfinite(xyz).all(axis=1)
                    nonfinite_xyz+=int(np.count_nonzero(~finite))
                    nonfinite_fields+=sum(int(np.count_nonzero(~np.isfinite(a[:n]))) for a in arrays.values())
                    valid=finite.copy()
                    if invalid_field:
                        invalid=arrays[invalid_field][:n]!=0
                        flagged_invalid+=int(np.count_nonzero(invalid));valid &= ~invalid
                    if valid.any():
                        xyz=xyz[valid];low=np.minimum(low,xyz.min(axis=0));high=np.maximum(high,xyz.max(axis=0))
                        transformed=xyz@rotation.T+translation
                        global_low=np.minimum(global_low,transformed.min(axis=0))
                        global_high=np.maximum(global_high,transformed.max(axis=0))
                    if decoded%10000000==0:diagnostic(DOC/'e57_progress.json',progress)
            finally:reader.close()
            if decoded!=count:raise RuntimeError('DECODED_COUNT_MISMATCH')
            rows.append({'scan_index':i,'name':metadata(scan['name']) if scan.isDefined('name') else 'NA',
                'declared_points':count,'decoded_points':decoded,'decode_status':'PASS',
                'point_fields':';'.join(fields),'buffer_capacity_points':CAPACITY,
                'buffer_bytes':sum(a.nbytes for a in arrays.values()),'pose_present':bool(pose),
                'pose_quaternion_order':'wxyz','qw':q[0],'qx':q[1],'qy':q[2],'qz':q[3],
                'tx_m':float(translation[0]),'ty_m':float(translation[1]),'tz_m':float(translation[2]),
                'quaternion_norm':float(np.linalg.norm(q)),'rotation_determinant':float(np.linalg.det(rotation)),
                'nonfinite_xyz_points':nonfinite_xyz,'nonfinite_numeric_fields':nonfinite_fields,
                'invalid_state_field':invalid_field or 'NOT_PRESENT',
                'flagged_invalid_points':flagged_invalid if invalid_field else 'NA',
                **{'local_'+axis+'_min':float(low[j]) if np.isfinite(low[j]) else 'NA' for j,axis in enumerate('xyz')},
                **{'local_'+axis+'_max':float(high[j]) if np.isfinite(high[j]) else 'NA' for j,axis in enumerate('xyz')},
                **{'root_'+axis+'_min':float(global_low[j]) if np.isfinite(global_low[j]) else 'NA' for j,axis in enumerate('xyz')},
                **{'root_'+axis+'_max':float(global_high[j]) if np.isfinite(global_high[j]) else 'NA' for j,axis in enumerate('xyz')}})
    except BaseException as error:
        failure=repr(error)
        raise
    finally:
        image.close()
        if rows:
            try:
                guard()
                with (DOC/'e57_scan_metadata.csv').open('x',newline='') as handle:
                    writer=csv.DictWriter(handle,list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
            except BaseException as receipt_error:
                if failure is None:failure='RECEIPT_FAILED:'+repr(receipt_error)
        summary={'completed':failure is None,'error':failure,'scans_decoded':len(rows),
            'decoded_points':sum(r['decoded_points'] for r in rows),'wall_s':time.monotonic()-started,
            'peak_RSS_KiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            'capacity_points':CAPACITY,'whole_scan_arrays_used':False,'pointcloud_exported':False,
            'last_partial_progress':progress,
            'map_frame_definition':'E57 root frame after applying each stored scan pose; relation to GT world NOT assumed'}
        diagnostic(DOC/'e57_summary.json',summary)


def timeout(signum,frame):raise TimeoutError('TASK_DEADLINE')


if __name__=='__main__':
    signal.signal(signal.SIGALRM,timeout)
    signal.setitimer(signal.ITIMER_REAL,max(.001,DEADLINE-60-time.time()))
    main()
