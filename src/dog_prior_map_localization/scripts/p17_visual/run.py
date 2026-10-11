#!/usr/bin/env python3
"""Isolated real ROS replay; frozen P10 parameters, no GT accepted."""
import argparse
import csv
import json
import os
import signal
import socket
import subprocess
import time
import xmlrpc.client
from pathlib import Path
import psutil
from prepare import sha


def stop(p):
    if p is None or p.poll() is not None: return
    os.killpg(p.pid, signal.SIGINT)
    try: p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=10)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--visual',type=Path,required=True)
    p.add_argument('--enable',action='store_true')
    p.add_argument('--duration',type=float,default=0)
    p.add_argument('--geometry',action='store_true')
    p.add_argument('--port',type=int,default=11387)
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    with socket.socket() as s: s.bind(('127.0.0.1',args.port))
    bag=args.repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag'
    if sha(bag) != '7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e':
        raise ValueError('INPUT_HASH_MISMATCH')
    env=dict(os.environ,ROS_MASTER_URI=f'http://127.0.0.1:{args.port}',ROS_IP='127.0.0.1',
             ROS_LOG_DIR=str(args.output/'roslogs'))
    processes=[]; streams=[]; resources={}; commands=[]; tick=time.monotonic(); completed=False
    def spawn(command,name):
        commands.append(command); f=(args.output/(name+'.log')).open('w');streams.append(f)
        child=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(child); return child
    try:
        master=spawn(['roscore','-p',str(args.port)],'master')
        api=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        for _ in range(60):
            try:
                if api.getPid('/p17_runner')[0]==1: break
            except (OSError,xmlrpc.client.Error): pass
            if master.poll() is not None: raise RuntimeError('MASTER_EXITED')
            time.sleep(.25)
        else: raise RuntimeError('MASTER_TIMEOUT')
        subprocess.run(['rosparam','set','/use_sim_time','true'],env=env,check=True)
        command=['roslaunch','dog_prior_map_localization','dog_prior_map_localization_split.launch',
          'rviz:=false','publish_path:=false','coupled_mode:=CONTROL','camera_enable:=false',
          f'dataset_config:={args.repo}/src/dog_prior_map_localization/config/p10_corridor01_prospective_causal.yaml',
          'scan_reference_time:=start','oosm_fixed_lag_enable:=true','oosm_fixed_lag_sec:=0.15',
          'diagnostic_determinism_enable:=true',
          f'visual_initial_guess_enable:={str(args.enable).lower()}',
          f'visual_initial_guess_csv:={args.visual}',
          f'ndt_diagnostics_csv_path:={args.output}/ndt_diagnostics.csv',
          f'ekf_prediction_diagnostics_csv_path:={args.output}/ekf_lineage.csv',
          f'oosm_csv_path:={args.output}/oosm.csv',
          f'oosm_fixed_lag_csv_path:={args.output}/oosm_fixed_lag.csv',
          f'imu_deskew_csv_path:={args.output}/deskew.csv',
          f'runtime_csv_path:={args.output}/ekf_runtime.csv']
        if args.geometry: command += [f'visual_geometry_csv:={args.output}/geometry.csv']
        launch=spawn(command,'launch')
        for _ in range(120):
            nodes=set(subprocess.check_output(['rosnode','list'],env=env,text=True).splitlines())
            if {'/dog_prior_map_ndt','/dog_prior_map_ekf'}.issubset(nodes):
                uri=api.lookupNode('/p17_runner','/dog_prior_map_ndt')[2]
                subscriptions=xmlrpc.client.ServerProxy(uri).getSubscriptions('/p17_runner')[2]
                if {'/imu/data','/p10/corridor01/deskewed_points'}.issubset({s[0] for s in subscriptions}): break
            if launch.poll() is not None: raise RuntimeError('LOCALIZATION_EXITED')
            time.sleep(.25)
        else: raise RuntimeError('NODES_TIMEOUT')
        binaries={}; node_processes=[]
        for child in psutil.Process(launch.pid).children(recursive=True):
            if child.name().startswith('dog_prior_map_'):
                real=Path(child.exe()).resolve()
                if args.repo.resolve() not in real.parents: raise RuntimeError('WRONG_RUNTIME_WORKSPACE')
                binaries[child.name()]={'path':str(real),'sha256':sha(real)}
                node_processes.append(child)
        if len(binaries)!=2: raise RuntimeError('RUNTIME_BINARY_COUNT')
        subprocess.run(['rosparam','dump',str(args.output/'rosparams.yaml')],env=env,check=True)
        record=spawn(['rosbag','record','-O',str(args.output/'outputs.bag'),
            '/dog_livo/ndt_odom','/dog_livo/odom_corrected','/dog_livo/odom_high_rate',
            '/dog_livo/diagnostics'],'record')
        time.sleep(1)
        command=['rosbag','play','--clock','--rate','0.25','--delay','2','--quiet']
        if args.duration: command += ['--duration',str(args.duration)]
        command += [str(bag)]
        playback=spawn(command,'playback')
        with (args.output/'resources.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['wall_s','pid','name','rss_bytes','hwm_bytes','cpu_sec'])
            while playback.poll() is None:
                if any(not c.is_running() or c.status()==psutil.STATUS_ZOMBIE for c in node_processes):
                    raise RuntimeError('LOCALIZATION_NODE_DIED')
                for child in psutil.Process(launch.pid).children(recursive=True):
                    try:
                        name=child.name()
                        if not name.startswith('dog_prior_map_'): continue
                        status=Path(f'/proc/{child.pid}/status').read_text()
                        hwm=next(int(x.split()[1])*1024 for x in status.splitlines() if x.startswith('VmHWM:'))
                        rss=child.memory_info().rss;cpu=sum(child.cpu_times()[:2])
                        r=resources.setdefault(name,{'peak_rss_bytes':0,'cpu_sec':0})
                        r['peak_rss_bytes']=max(r['peak_rss_bytes'],rss,hwm);r['cpu_sec']=max(r['cpu_sec'],cpu)
                        w.writerow([time.monotonic()-tick,child.pid,name,rss,hwm,cpu])
                    except (psutil.Error,OSError,StopIteration): pass
                f.flush()
                if launch.poll() is not None: raise RuntimeError('NODES_EXITED_DURING_PLAYBACK')
                time.sleep(.25)
        if playback.returncode: raise RuntimeError('PLAYBACK_FAILED')
        time.sleep(2)
        if any(not c.is_running() or c.status()==psutil.STATUS_ZOMBIE for c in node_processes):
            raise RuntimeError('LOCALIZATION_NODE_DIED')
        with (args.output/'ndt_diagnostics.csv').open() as f: ndt_rows=list(csv.DictReader(f))
        if not ndt_rows or (not args.duration and len(ndt_rows)!=396):
            raise RuntimeError('NDT_FRAME_COUNT_MISMATCH')
        completed=True
    finally:
        for child in reversed(processes): stop(child)
        for stream in streams: stream.close()
        receipt=dict(commands=commands,resources=resources,wall_sec=time.monotonic()-tick,
            enable=args.enable,completed=completed,GT_LOADED=False,input_sha256=sha(bag),visual_sha256=sha(args.visual))
        if 'binaries' in locals(): receipt['binaries']=binaries
        (args.output/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt),flush=True)


if __name__=='__main__': main()
