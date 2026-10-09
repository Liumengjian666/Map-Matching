"""Exact scientific-output comparison for the final allocation-only change."""
from run_budgeted import ARCHIVE, read, csv_write, json_write


def verify():
    comparisons=[]
    jobs=["integration_C"]+[f"two_{m}" for m in "ABC"]+["continuous_control"]+[f"continuous_{m}" for m in "ABC"]
    for job in jobs:
        tables=["frames.csv","candidates.csv","models.csv"]
        if job.startswith("continuous_"): tables += ["registration.csv","trajectory.csv"]
        for table in tables:
            original=read(ARCHIVE/"attempt_0"/job/table)
            final=read(ARCHIVE/"attempt_2"/job/table)
            if len(original)!=len(final): raise RuntimeError("row count changed")
            bad=[]; fields=[]
            for index,(a,b) in enumerate(zip(original,final)):
                fields=[key for key in a if not key.endswith("_ms")]
                for key in fields:
                    if a[key]!=b[key]: bad.append((index,key,a[key],b[key]))
            comparisons.append(dict(job=job,table=table,rows=len(original),non_timing_fields=len(fields),
                                    mismatch_count=len(bad),exact_parity=int(not bad)))
            if bad: raise RuntimeError("scientific values changed: "+repr((job,table,bad[:3])))
    csv_write(ARCHIVE/"value_buffer_exact_parity.csv",comparisons)
    json_write(ARCHIVE/"value_buffer_exact_parity.json",dict(PARITY="PASS",tables=len(comparisons),
               rows=sum(r["rows"] for r in comparisons),timing_excluded=True,resource_RSS_excluded=True))


if __name__=="__main__": verify()
