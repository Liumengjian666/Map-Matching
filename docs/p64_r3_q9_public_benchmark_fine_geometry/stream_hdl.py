"""Deliver the same map and causal scan/IMU inputs to the unmodified HDL nodelet."""
import csv
import sys
import threading
import time
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header
import rospy
import numpy as np
from scipy.spatial.transform import Rotation

from paired import read_pcd


def stamp(ns):
    return rospy.Time(*divmod(ns,10**9))


def xyzmsg(path, frame, ns):
    data = np.asarray(read_pcd(path),dtype="<f4")
    data=np.column_stack((data,np.zeros(len(data),dtype="<f4")))
    fields=[PointField(key,i*4,PointField.FLOAT32,1) for i,key in enumerate(("x","y","z","intensity"))]
    return PointCloud2(header=Header(stamp=stamp(ns),frame_id=frame),
          height=1,width=len(data),fields=fields,is_bigendian=False,point_step=16,
          row_step=16*len(data),data=data.tobytes(),is_dense=True)


def wait_until(predicate, sec):
    stop=time.monotonic()+sec
    while time.monotonic()<stop and not rospy.is_shutdown():
        if predicate():
            return True
        time.sleep(.02)
    return False


def main(folder,output):
    rospy.init_node("p64_q9_replay")
    map_pub=rospy.Publisher("/globalmap",PointCloud2,queue_size=1,latch=True)
    scanner=rospy.Publisher("/velodyne_points",PointCloud2,queue_size=1)
    imu=rospy.Publisher("/gpsimu_driver/imu_data",Imu,queue_size=256)
    initial=rospy.Publisher("/initialpose",PoseWithCovarianceStamped,queue_size=1)
    observed={}
    lock=threading.Lock()

    def response(msg):
        with lock:
            observed[msg.header.stamp.to_nsec()]=msg

    rospy.Subscriber("/odom",Odometry,response,queue_size=100)
    if not wait_until(lambda:all(x.get_num_connections()>0 for x in (map_pub,scanner,imu,initial)),15):
        raise TimeoutError("Official nodelet topic connections not ready")
    scans=[]
    for line in (folder/"scans.tsv").read_text().splitlines():
        ns,name=line.split()
        scans.append((int(ns),name))
    values=[]
    for line in (folder/"imu.tsv").read_text().splitlines():
        tick,*six=line.split()
        values.append((int(tick),tuple(map(float,six))))
    map_pub.publish(xyzmsg(folder/"map.pcd","map",scans[0][0]))
    time.sleep(2)
    initial_values=np.loadtxt(folder/"anchor.tsv")
    x,y,z,yaw=initial_values[:4]
    qx,qy,qz,qw=(initial_values[4:] if initial_values.size==8 else Rotation.from_euler("z",yaw,degrees=True).as_quat())
    start=PoseWithCovarianceStamped(header=Header(stamp=stamp(scans[0][0]),frame_id="map"))
    p,q=start.pose.pose.position,start.pose.pose.orientation
    p.x,p.y,p.z=x,y,z
    q.x,q.y,q.z,q.w=qx,qy,qz,qw
    initial.publish(start)
    time.sleep(.5)
    idx=0
    with output.open("x") as f:
        w=csv.writer(f)
        w.writerow(("frame","stamp_ns","x","y","z","qx","qy","qz","qw","input_to_output_s","imu_count","status"))
        for frame,(ns,name) in enumerate(scans):
            begin=time.perf_counter()
            prev=idx
            while idx<len(values) and values[idx][0]<=ns:
                ns_imu,pose=values[idx]
                m=Imu(header=Header(stamp=stamp(ns_imu),frame_id="base_link"))
                a,g=m.linear_acceleration,m.angular_velocity
                a.x,a.y,a.z,g.x,g.y,g.z=pose
                imu.publish(m)
                idx+=1
            time.sleep(.02)
            scanner.publish(xyzmsg(folder/name,"base_link",ns))
            good=wait_until(lambda:ns in observed,4)
            if good:
                with lock:
                    r=observed.pop(ns).pose.pose
                x,y,z=r.position.x,r.position.y,r.position.z
                qx,qy,qz,qw=r.orientation.x,r.orientation.y,r.orientation.z,r.orientation.w
            else:
                x=y=z=qx=qy=qz=qw=""
            w.writerow((frame,ns,x,y,z,qx,qy,qz,qw,time.perf_counter()-begin,idx-prev,"POSE" if good else "NO_OUTPUT"))
            f.flush()
            if not good:
                raise TimeoutError(f"No official localization output for frame {frame}")


if __name__=="__main__":
    main(Path(sys.argv[1]),Path(sys.argv[2]))
