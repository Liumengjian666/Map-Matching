"""Bounded public-download probes. No credentials and no large payload writes."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import csv
import re
from html import unescape
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ASSETS = [
    ("Corridor02", "corridor02.zip", "1fbQIjza6zCVZ719VvXfNhAONDZflqGnf"),
    ("Corridor02", "corridor02.pcd", "1d5vv4kfrTyntZw-82fGeTyih3H7DWxg5"),
    ("Corridor02", "corridor02_gt.txt", "1hb3IVu1OqyPZA61kBZG1raGTBcTv_h3H"),
    ("Corridor02", "corridor02_extrinsics.yaml", "1FzepVzxan_9GjS0Rg_3f1LIjjwm2VZrR"),
    ("Corridor02", "corridor02_Intrinsics.yaml", "1gCjmOVcwhm55Rwosel79FMxG_WzvnV8g"),
    ("Corridor01", "SubT_MRS_Hawkins_Long_Corridor_RC.zip", "1aIIqPiE10nX3IhidpxKhc4Psq-AMER1X"),
    ("Cave01", "cave01.zip", "1cyHbmxmJQGuK5UCm_f7SXomr8Gd6GEww"),
    ("Cave02", "cave02.zip", "1HwUYboHbCh4_GfyvZYQn25KmA12yiuMR"),
]

ENDPOINTS = {
    "preview": "https://drive.google.com/file/d/{}/view?usp=sharing",
    "uc": "https://drive.google.com/uc?export=download&id={}",
    "confirmed": "https://drive.usercontent.google.com/download?export=download&confirm=t&id={}",
}


def inspect(job):
    sequence, filename, id_, method, raw = job
    result = dict(sequence=sequence, file=filename, method=method, url=ENDPOINTS[method].format(id_))
    request = Request(result["url"], headers={"Range": "bytes=0-65535", "User-Agent": "Mozilla/5.0"})
    try:
        try:
            response = urlopen(request, timeout=12)
        except HTTPError as error:
            response = error
        with response:
            data = response.read(65536)
            result.update(http=response.status, mime=response.headers.get("Content-Type"),
                          received=len(data), content_range=response.headers.get("Content-Range"),
                          length=response.headers.get("Content-Length"))
        if "html" in (result["mime"] or ""):
            page = data.decode("utf8", errors="replace")
            match = re.search(r"<title[^>]*>(.*?)</title>", page, re.I | re.S)
            result["title"] = unescape(match.group(1)).strip() if match else ""
            # Match rendered page text rather than Drive's generic script strings.
            visible = re.sub(r"<(script|style)\b.*?</\1>", "", page, flags=re.I | re.S)
            visible = " ".join(unescape(re.sub(r"<[^>]+>", " ", visible)).split())
            result["message"] = visible[:420]
        else:
            result["magic"] = data[:12].hex()
        (raw/f"{sequence}.{filename}.{method}.sample").write_bytes(data)
    except OSError as error:
        result["message"] = str(error)
    return result


def main(out, raw):
    raw.mkdir(parents=True, exist_ok=True)
    jobs = [(*asset, method, raw) for asset in ASSETS for method in ENDPOINTS]
    with ThreadPoolExecutor(max_workers=8) as pool:
        data = list(pool.map(inspect, jobs))
    with out.open("w") as stream:
        writer = csv.DictWriter(stream, list(dict.fromkeys(k for row in data for k in row)))
        writer.writeheader()
        writer.writerows(data)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("out", type=Path)
    parser.add_argument("raw", type=Path)
    args=parser.parse_args()
    main(args.out,args.raw)
