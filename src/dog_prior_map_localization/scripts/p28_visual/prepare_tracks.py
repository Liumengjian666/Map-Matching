#!/usr/bin/env python3
"""Real-image FAST/KLT tracks only. No PnP, depth, NDT pose or GT input.

Independent adaptation of the OpenVINS tracking pipeline. GPL source is not
copied. Existing project camera decoder/MEI calibration utilities are reused.
"""
import argparse
import csv
import hashlib
import json
import resource
import sys
import time
from pathlib import Path

import numpy as np
import rosbag

FRONT = Path(__file__).resolve().parents[3] / "dog_prior_map_fastlio2_frontend_exp/scripts"
sys.path.insert(0, str(FRONT))
from p4_i3_visual_frontend import load_calibration, rectify, cv2


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""): h.update(block)
    return h.hexdigest()


def detect(image, valid, occupied):
    mask = valid.copy()
    mask[:15] = mask[-15:] = 0
    mask[:, :15] = mask[:, -15:] = 0
    for point in occupied:
        cv2.circle(mask, tuple(np.rint(point).astype(int)), 10, 0, -1)
    detector = cv2.FastFeatureDetector_create(20, True)
    h, w = image.shape
    found = []
    for y in range(5):
        for x in range(5):
            y0, y1 = y*h//5, (y+1)*h//5
            x0, x1 = x*w//5, (x+1)*w//5
            candidates = sorted(detector.detect(image[y0:y1, x0:x1], mask[y0:y1, x0:x1]),
                                key=lambda k: (-k.response, k.pt[1], k.pt[0]))
            count = 0
            for keypoint in candidates:
                point = np.array(keypoint.pt) + [x0, y0]
                ix, iy = np.rint(point).astype(int)
                if not mask[iy, ix]: continue
                found.append(point.astype(np.float32))
                cv2.circle(mask, (ix, iy), 10, 0, -1)
                count += 1
                if count >= 8: break
    return found[:max(0, 200-len(occupied))]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--camera-bag", type=Path, required=True)
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    if sha(args.input) != "7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e":
        raise ValueError("INPUT_HASH_MISMATCH")
    calib = load_calibration(args.calibration, "corridor01")
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    with rosbag.Bag(str(args.input)) as bag:
        scans = [m.header.stamp.to_nsec() for _, m, _ in bag.read_messages(topics=["/p10/corridor01/raw_timed_points"])]
    camera_stat = args.camera_bag.stat()
    active = {}  # ID -> observations, at most five consecutive selected images.
    next_id = 1
    previous = None
    previous_ns = 0
    output = []
    ledger = []
    image_hash = hashlib.sha256()
    with rosbag.Bag(str(args.camera_bag)) as bag:
        camera = bag.read_messages(topics=["/camera_1/image_raw"])
        upcoming = next(camera, None)
        latest = None
        last_seen = 0
        for frame, scan in enumerate(scans, 1):
            while upcoming is not None and upcoming[1].header.stamp.to_nsec() <= scan:
                stamp = upcoming[1].header.stamp.to_nsec()
                if stamp <= last_seen: raise ValueError("NONMONOTONIC_CAMERA")
                latest = upcoming[1]; last_seen = stamp
                upcoming = next(camera, None)
            if latest is None: raise ValueError("NO_CAUSAL_CAMERA")
            image_ns = latest.header.stamp.to_nsec()
            if not 0 <= scan-image_ns <= 50000000 or image_ns <= previous_ns:
                # A repeated/old image is not a new sensor measurement. Keep the
                # original 50 ms gate; do not invent an image or reuse its noise.
                ledger.append(dict(frame_index=frame,scan_ns=scan,image_ns=image_ns,
                    active_tracks=len(active),completed_tracks=0,frontend_ms=0.0,
                    status="REPEATED_IMAGE" if image_ns<=previous_ns else "STALE_IMAGE"))
                continue
            tick = time.perf_counter()
            image = cv2.equalizeHist(rectify(latest, calib))
            image_hash.update(image_ns.to_bytes(8, "little")); image_hash.update(image.tobytes())
            closed = []
            occupied = []
            if previous is not None and active:
                ids = list(active)
                old = np.array([active[i][-1][1:] for i in ids], dtype=np.float32).reshape(-1,1,2)
                new, status, _ = cv2.calcOpticalFlowPyrLK(previous, image, old, None,
                    winSize=(15,15), maxLevel=5,
                    criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
                back, reverse_status, _ = cv2.calcOpticalFlowPyrLK(image, previous, new, None,
                    winSize=(15,15), maxLevel=5,
                    criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,0.01))
                xy = new.reshape(-1,2)
                good = (status.ravel()!=0) & (reverse_status.ravel()!=0)
                good &= np.linalg.norm(back.reshape(-1,2)-old.reshape(-1,2), axis=1) <= 1.0
                good &= np.isfinite(xy).all(axis=1)
                good &= (xy[:,0]>=15)&(xy[:,0]<image.shape[1]-15)&(xy[:,1]>=15)&(xy[:,1]<image.shape[0]-15)
                indices = np.flatnonzero(good)
                if len(indices) >= 8:
                    _, inliers = cv2.findFundamentalMat(old.reshape(-1,2)[indices],xy[indices],cv2.FM_RANSAC,1.0,0.999)
                    if inliers is None: good[indices] = False
                    else: good[indices] &= inliers.ravel()!=0
                for j, feature in enumerate(ids):
                    if good[j]:
                        ix, iy = np.rint(xy[j]).astype(int)
                        good[j] = calib["mask"][iy,ix] != 0
                    if good[j]:
                        active[feature].append((image_ns,float(xy[j,0]),float(xy[j,1])))
                        occupied.append(xy[j])
                    if not good[j] or len(active[feature]) >= 5:
                        track = active.pop(feature)
                        if len(track) >= 3: closed.append((feature,track))
            for point in detect(image,calib["mask"],occupied):
                active[next_id] = [(image_ns,float(point[0]),float(point[1]))]
                next_id += 1
            elapsed = (time.perf_counter()-tick)*1000
            # One frame marker even if no completed track: cloning is independent
            # of degeneracy or successful feature geometry, not a fake observation.
            output.append(dict(scan_ns=scan,image_ns=image_ns,feature_id=0,observation_ns=0,u=0,v=0))
            for feature, track in sorted(closed):
                for stamp,u,v in track:
                    output.append(dict(scan_ns=scan,image_ns=image_ns,feature_id=feature,observation_ns=stamp,u=u,v=v))
            ledger.append(dict(frame_index=frame,scan_ns=scan,image_ns=image_ns,
                               active_tracks=len(active),completed_tracks=len(closed),frontend_ms=elapsed,status="NEW_CAUSAL_IMAGE"))
            previous, previous_ns = image, image_ns
            if frame % 100 == 0: print("frames",frame,"completed",sum(x["completed_tracks"] for x in ledger),flush=True)
    for filename, data in (("tracks.csv",output),("frontend_summary.csv",ledger)):
        with (args.output/filename).open("w", newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(data[0]),lineterminator="\n")
            writer.writeheader(); writer.writerows(data)
    times = [x["frontend_ms"] for x in ledger]
    receipt = dict(method="OPENVINS_STYLE_FAST_KLT_REAL_PIXEL_TRACKS_NO_PNP",frames=len(ledger),
        completed_tracks=sum(x["completed_tracks"] for x in ledger),observations=sum(r["feature_id"]!=0 for r in output),
        wall_sec=time.perf_counter()-started,frontend_mean_ms=float(np.mean(times)),frontend_p95_ms=float(np.percentile(times,95)),
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,GT_LOADED=False,NDT_POSES_LOADED=False,
        camera_path=str(args.camera_bag),camera_size=camera_stat.st_size,camera_mtime_ns=camera_stat.st_mtime_ns,
        camera_full_hash_rechecked=False,selected_equalized_rectified_image_sha256=image_hash.hexdigest(),
        tracks_sha256=sha(args.output/"tracks.csv"),K_rect=calib["Krect"].tolist(),T_imu_camera=calib["T_imu_camera"].tolist(),
        calibration_sha256={f.name:sha(f) for f in args.calibration.glob("corridor01_*.yaml")},
        parameters=dict(max_tracks=200,max_observations=5,grid=5,fast_threshold=20,min_distance_px=10,
                        lk_window=15,lk_pyramid_levels=5,fb_max_px=1.0,ransac_max_px=1.0),
        time_contract="latest camera header <= scan, lag <=50ms; each track closes using no later image")
    (args.output/"extraction.json").write_text(json.dumps(receipt,indent=2)+"\n")
    print(json.dumps(receipt,indent=2))

if __name__ == "__main__": main()
