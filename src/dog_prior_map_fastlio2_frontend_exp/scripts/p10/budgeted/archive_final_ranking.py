"""Reproduce final frozen recommendation ordering using no GT/oracle inputs."""
import numpy as np
from evaluate_budgeted import matrix, distance
from run_budgeted import ARCHIVE, read, csv_write


def archive_ranking():
    output=[]
    for attempt in range(3):
        for directory in sorted((ARCHIVE/f"attempt_{attempt}").iterdir()):
            if not (directory/"frames.csv").is_file(): continue
            candidates=read(directory/"candidates.csv")
            for frame in read(directory/"frames.csv"):
                count=int(frame["source_count"]); energy=float(frame["nominal_energy"])
                if frame["shadow_status"]=="CONTROL_NO_SHADOW": continue
                nominal_score=-energy*count; nominal_merit=float(frame["nominal_merit"])
                prediction=matrix(frame["prediction_pose"])
                rows=[dict(candidate_id=-1,score_sum=nominal_score,merit=nominal_merit,admissible=True,reason="NOMINAL_FALLBACK",visit=-1)]
                refined=[r for r in candidates if r["transaction_id"]==frame["transaction_id"] and r["selected_for_refinement"]=="1"]
                refined.sort(key=lambda r:int(r["rank"]))
                for visit,row in enumerate(refined):
                    dt,dr=distance(matrix(row["refined_pose"]),prediction)
                    score=float(row["refined_score"])
                    merit=(-score/count)/max(1,abs(energy))+.05*((dt/2)**2+(dr/15)**2)
                    eligible=(row["refined_successful"]=="1" and row["refined_converged"]=="1" and
                        score>=nominal_score+2.747604276e-4 and merit<nominal_merit)
                    reason="ELIGIBLE" if eligible else "NOT_SUCCESSFUL" if row["refined_successful"]!="1" else "NO_RAW_SCORE_IMPROVEMENT" if score<nominal_score+2.747604276e-4 else "NO_MERIT_IMPROVEMENT"
                    rows.append(dict(candidate_id=int(row["candidate_id"]),score_sum=score,merit=merit,admissible=eligible,reason=reason,visit=visit))
                rows.sort(key=lambda r:(not r["admissible"],r["merit"],r["visit"]))
                if rows[0]["candidate_id"]!=int(frame["recommended_id"]): raise RuntimeError("final ranking disagrees with frozen builder")
                for rank,row in enumerate(rows):
                    output.append(dict(attempt=attempt,job=directory.name,transaction_id=frame["transaction_id"],final_rank=rank,
                        candidate_id=row["candidate_id"],raw_score=row["score_sum"],raw_advantage=row["score_sum"]-nominal_score,
                        merit=row["merit"],admissible=int(row["admissible"]),reason=row["reason"],recommended=int(rank==0)))
    csv_write(ARCHIVE/"final_candidate_ranking.csv",output)


if __name__=="__main__":archive_ranking()
