"""Bounded Release/test receipts; never starts any real NDT replay."""
import os
import argparse
from pathlib import Path
import subprocess
from run_budgeted import csv_write
from run_admission import ARCHIVE


def main(revision):
    directory=ARCHIVE/("validation" if revision==0 else "validation_improvement_1")
    directory.mkdir(exist_ok=False)
    env=dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1")
    commands=[("release_build",Path("/tmp/p10_r2_build.CdJ4jf"),["cmake","--build",".","--target",
        "p10_r2_replay","p10_r4_admission_test","p10_r3_temporal_test","p10_r2_shadow_test",
        "current_frame_ndt_test","p7_replay_io_test","-j2"]),
        ("p7_p10_tests",Path("/tmp/p10_r2_build.CdJ4jf"),["ctest","--output-on-failure"]),
        ("p9_tests",Path("/tmp/p9_r4_release.Eirto1"),["ctest","--output-on-failure"]),
        ("p10_r1_tests",Path("/tmp/p10_coupled_build.jxTN7w"),["ctest","--output-on-failure"])]
    receipts=[]
    for name,cwd,command in commands:
        result=subprocess.run(command,cwd=cwd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (directory/(name+".log")).write_text(result.stdout)
        receipts.append(dict(check=name,cwd=str(cwd),command=" ".join(command),returncode=result.returncode))
        print(name,result.returncode,flush=True)
        if result.returncode:
            csv_write(directory/"failed_checks.csv",receipts)
            raise RuntimeError("verification failed; preserved log "+name)
    csv_write(directory/"checks.csv",receipts)


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--revision",type=int,choices=(0,1),default=0)
    main(parser.parse_args().revision)
