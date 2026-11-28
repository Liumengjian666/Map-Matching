"""Run pinned Hilti's SE(3)-aligned evaluator as a separate evaluation category."""
import argparse
import importlib.util
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")


def run(module, trajectory, references, output):
    spec=importlib.util.spec_from_file_location("hilti_eval",module)
    evaluator=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(evaluator)
    output.mkdir()
    results={}
    for label,reference in zip(("dense","sparse"),references):
        metrics,is_dense,aligned,ref=evaluator.evaluate(str(trajectory),str(reference),str(output)+"/")
        results[label]=dict(scope="OFFICIAL_UMEYAMA_ALIGNED_NOT_FIXED_MAP",
                poses=ref.num_poses,statistics={str(k.value):float(v) for k,v in metrics.items()})
    (output/"official_aligned.json").write_text(json.dumps(results,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser()
    for n in ("module","trajectory","dense","sparse","output"):
        p.add_argument(n,type=Path)
    a=p.parse_args()
    run(a.module,a.trajectory,(a.dense,a.sparse),a.output)
