#!/usr/bin/env python3
"""Capture only this run's RViz X11 window; do not capture unrelated desktop windows."""
import subprocess,re,argparse
from PyQt5.QtWidgets import QApplication
p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
text=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
for line in text.splitlines():
 if 'rviz' not in line.lower():continue
 m=re.search(r'0x[0-9a-fA-F]+',line)
 if not m:continue
 prop=subprocess.run(['xprop','-id',m[0],'_NET_WM_PID'],capture_output=True,text=True).stdout
 if not re.search(r'=\s*'+str(a.pid)+r'\s*$',prop):continue
 app=QApplication([]);image=app.primaryScreen().grabWindow(int(m[0],16))
 if image.isNull() or not image.save(a.output):raise RuntimeError('RViz image capture failed')
 break
else:raise RuntimeError('owned RViz window not found')
