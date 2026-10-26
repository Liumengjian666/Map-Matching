#!/usr/bin/env python3
"""Fail closed before executing the unchanged author library through its read-only observer."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
import rospy

def main():
 rospy.init_node('p63_baseline_gate',anonymous=False)
 file=Path(rospy.get_param('~sensor_contract'))
 if not file.is_file():raise RuntimeError('SENSOR_CONTRACT_BLOCKED: provide a device/map-qualified LiDAR-IMU contract; no placeholder accepted')
 c=json.loads(file.read_text())
 smoke = c.get('status')=='HISTORICAL_IDENTITY_SMOKE_ONLY' and rospy.get_param('~allow_historical_identity_smoke',False)
 if not smoke and (c.get('status')!='QUALIFIED' or not c.get('calibration_evidence')):
  raise RuntimeError('SENSOR_CONTRACT_BLOCKED: calibration provenance is not qualified')
 if rospy.get_param('/lio/relocation/update_map',True) or rospy.get_param('/lio/map/save_map',True):
  raise RuntimeError('MAP_FREEZE_CONFIG_INVALID')
 map_file=Path(c['map_path']);assert hashlib.sha256(map_file.read_bytes()).hexdigest()==c['map_sha256']
 ext=rospy.get_param('/lio/extrinsic/lidar_imu');assert ext==c['T_imu_lidar_translation_plus_column_major_rotation']
 if smoke:assert ext==[0.,0.,0.,1.,0.,0.,0.,1.,0.,0.,0.,1.]
 exe=Path(c['observer_binary']);assert hashlib.sha256(exe.read_bytes()).hexdigest()==c['observer_binary_sha256']
 # No new filter or predictor: replace this process with the author executable.
 os.execv(str(exe),[str(exe),'__name:=p63_original_relocation'])
if __name__=='__main__':
 try:main()
 except Exception as e:
  print(str(e),file=sys.stderr);sys.exit(2)
