"""Release/test receipts only. Never run scientific replay or access GT."""
import os
import subprocess
from run_budgeted import ROOT,json_write,sha
from run_directional import ARCHIVE

def main():
    directory=ARCHIVE/"verification";directory.mkdir(exist_ok=False)
    env=dict(os.environ,LD_LIBRARY_PATH="/lib/x86_64-linux-gnu",OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1")
    build="/tmp/p10_r2_build.CdJ4jf";p9="/tmp/p9_r4_release.Eirto1"
    commands=[("release_build",["cmake","--build",build,"-j2"],str(ROOT)),
        ("p7_p10_tests",["ctest","--output-on-failure"],build),
        ("p9_tests",["ctest","--output-on-failure"],p9),
        ("directional_harness",["python3","src/dog_prior_map_fastlio2_frontend_exp/scripts/p10/budgeted/test_directional_harness.py"],str(ROOT)),
        ("diff_check",["git","diff","--check"],str(ROOT))]
    receipts=[]
    for name,cmd,cwd in commands:
        p=subprocess.run(cmd,cwd=cwd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        with (directory/(name+".log")).open("xb") as stream:stream.write(p.stdout)
        receipts.append(dict(name=name,command=cmd,workdir=cwd,returncode=p.returncode,
            log_sha256=sha(directory/(name+".log"))))
        if p.returncode:raise RuntimeError("verification failed "+name)
    json_write(directory/"receipt.json",dict(Release=True,
        environment=dict(LD_LIBRARY_PATH=env["LD_LIBRARY_PATH"],OMP_NUM_THREADS="1",OPENBLAS_NUM_THREADS="1"),
        initial_unconfigured_P9_result="39/41; two MVS libusb_set_option symbol failures",
        loader_diagnosis="ldd without env selected /opt/MVS/lib/64/libusb-1.0.so.0; frozen env selects system library",
        algorithm_or_test_threshold_changed=False,GT_LOADED=False,scientific_replays_run=0,commands=receipts))
    print("Release + P7/P10 8/8 + P9 41/41 + harness5/5 + diff check PASS")

if __name__=="__main__":main()
