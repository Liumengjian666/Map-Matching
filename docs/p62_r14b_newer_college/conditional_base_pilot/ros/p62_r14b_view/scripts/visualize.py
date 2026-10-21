#!/usr/bin/env python3
"""Display sealed outputs only: no optimizer, predictor update or feedback."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parents[3]


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def xyz(path):
    with Path(path).open('rb') as f:
        header={}
        while True:
            line=f.readline(4097)
            if not line or len(line)>4096:raise ValueError('Invalid PCD header')
            key,*value=line.decode('ascii').strip().split()
            if key.startswith('#'):continue
            header[key]=value
            if key=='DATA':break
        if header['DATA']!=['binary']:raise ValueError('Only frozen binary PCD')
        sizes=list(map(int,header['SIZE']));counts=list(map(int,header['COUNT']))
        step=sum(s*c for s,c in zip(sizes,counts));n=int(header['POINTS'][0])
        payload=f.read(n*step);tail=f.read()
        if len(payload)!=n*step or any(tail):raise ValueError('Invalid payload/suffix')
        offsets=np.cumsum([0]+[s*c for s,c in zip(sizes,counts)])
        fields=header['FIELDS'];indices=[fields.index(k) for k in ('x','y','z')]
        if any(sizes[i]!=4 or header['TYPE'][i]!='F' or counts[i]!=1 for i in indices):
            raise ValueError('Invalid XYZ schema')
        data=np.ndarray(n,dtype=np.dtype({'names':['x','y','z'],
            'formats':['<f4']*3,'offsets':[int(offsets[i]) for i in indices],'itemsize':step}),buffer=payload)
        points=np.column_stack([data[k] for k in ('x','y','z')])
        return points[np.isfinite(points).all(axis=1)]


def pose(row,prefix):
    return np.array([[float(row[f'{prefix}{i}{j}']) for j in range(4)] for i in range(4)])


def inputs():
    prereg=json.loads((HERE/'TEST_FRAME_PREREGISTRATION.json').read_text())
    seal=json.loads((HERE/'NATIVE_OUTPUT_FREEZE.json').read_text())
    if not (HERE/'REFERENCE_SUMMARY.json').exists():raise RuntimeError('Evaluation not completed')
    for r in seal['files']:
        if sha(HERE/Path(r['path']).name)!=r['sha256']:raise RuntimeError('Archive native output changed')
    with (HERE/'FINAL_OBSERVATIONS.csv').open() as f:final=list(csv.DictReader(f))
    selected={r['stamp_ns'] for r in prereg['frames'][1:]};reference={}
    source=prereg['source_inputs'][0]
    if sha(source['path'])!=source['sha256']:raise RuntimeError('Reference changed')
    with Path(source['path']).open() as f:
        reader=csv.reader(f);next(reader)
        for row in reader:
            ns=int(row[0])*10**9+int(row[1])
            if ns not in selected:continue
            t=np.eye(4);t[:3,3]=list(map(float,row[2:5]));t[:3,:3]=Rotation.from_quat(list(map(float,row[5:9]))).as_matrix()
            reference[ns]=t@np.array(prereg['T_B_L'])
    admission=json.loads((HERE/'PREFLIGHT_PASS.json').read_text())
    if sha(admission['map_path'])!=admission['map_sha256']:raise RuntimeError('Map changed')
    return prereg,final,reference,xyz(admission['map_path'])


def main():
    prereg,final,reference,map_points=inputs()
    import rospy
    from std_msgs.msg import Header
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    rospy.init_node('r14b_sealed_view')
    def header(ns):return Header(stamp=rospy.Time(ns//10**9,ns%10**9),frame_id='leica_map')
    def stamped(t,ns):
        p=PoseStamped();p.header=header(ns);p.pose.position.x,p.pose.position.y,p.pose.position.z=t[:3,3]
        q=Rotation.from_matrix(t[:3,:3]).as_quat()
        p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=q
        return p
    pubs={name:rospy.Publisher('/r14b/'+name,RosPath,queue_size=1,latch=True)
          for name in ('full_path','split_path','proposed_path','reference_path')}
    paths={name:RosPath() for name in pubs}
    map_pub=rospy.Publisher('/r14b/map',PointCloud2,queue_size=1,latch=True)
    scan_pub=rospy.Publisher('/r14b/scan_at_full',PointCloud2,queue_size=1,latch=True)
    map_pub.publish(point_cloud2.create_cloud_xyz32(header(prereg['frames'][0]['stamp_ns']),map_points))
    for i,r in enumerate(final,1):
        if rospy.is_shutdown():return
        ns=int(r['stamp_ns']);frame=prereg['frames'][i]
        if sha(frame['path'])!=frame['sha256']:raise RuntimeError('Source changed')
        points=xyz(frame['path']);full=pose(r,'full_')
        world=points@full[:3,:3].T+full[:3,3]
        scan_pub.publish(point_cloud2.create_cloud_xyz32(header(ns),world))
        mapping={'full_path':full,'split_path':pose(r,'split_'),'reference_path':reference[ns]}
        if int(r['proposed_observation_valid']):mapping['proposed_path']=pose(r,'accepted_')
        for name,t in mapping.items():paths[name].poses.append(stamped(t,ns))
        for name,p in paths.items():p.header=header(ns);pubs[name].publish(p)
        rospy.sleep(.1)
    rospy.loginfo('R14B_VIEW_COMPLETE frames=20 no_NDT no_feedback conditional_Base_reference')
    if rospy.get_param('~keep_alive',True):rospy.spin()


if __name__=='__main__':main()
