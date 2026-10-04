#!/usr/bin/env python3
"""Generate six fixed-camera RViz captures for the frozen TX666 geometry."""

import argparse
import copy
import ctypes
import json
import math
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import rospy
import yaml
from rviz.srv import SendFilePath
from std_msgs.msg import String


def apply_pose(matrix, point):
    point = np.asarray(point, dtype=np.float64)
    return matrix[:3, :3] @ point + matrix[:3, 3]


def cloud_display(name, topic, color, size_m, alpha):
    return {
        "Alpha": alpha,
        "Autocompute Intensity Bounds": True,
        "Autocompute Value Bounds": {"Max Value": 10, "Min Value": -10, "Value": True},
        "Axis": "Z",
        "Channel Name": "intensity",
        "Class": "rviz/PointCloud2",
        "Color": color,
        "Color Transformer": "FlatColor",
        "Decay Time": 0,
        "Enabled": True,
        "Invert Rainbow": False,
        "Max Color": "255; 255; 255",
        "Min Color": "0; 0; 0",
        "Name": name,
        "Position Transformer": "XYZ",
        "Queue Size": 1,
        "Selectable": False,
        "Size (Pixels)": 2,
        "Size (m)": size_m,
        "Style": "Points",
        "Topic": topic,
        "Unreliable": False,
        "Use Fixed Frame": True,
        "Use rainbow": False,
        "Value": True,
    }


def axes_display(name, reference_frame, length=2.5, radius=0.07):
    return {
        "Class": "rviz/Axes",
        "Enabled": True,
        "Length": length,
        "Name": name,
        "Radius": radius,
        "Reference Frame": reference_frame,
        "Value": True,
    }


def make_display_config(template, fixed_frame, map_topic, initial_topic,
                        terminal_topic, camera):
    config = copy.deepcopy(template)
    manager = config["Visualization Manager"]
    source_tf = next(display for display in manager["Displays"]
                     if display.get("Class") == "rviz/TF")
    tf_display = copy.deepcopy(source_tf)
    tf_display.update({
        # TF is still published and used for fixed-frame transforms. The
        # four explicit Axes displays below are the visual frame markers;
        # disabling duplicate TF glyphs avoids overlapping labels/axes.
        "Enabled": False,
        "Frame Timeout": 30,
        "Marker Alpha": 1,
        "Marker Scale": 1.0,
        "Show Arrows": False,
        "Show Axes": False,
        "Show Names": False,
        "Frames": {"All Enabled": True},
    })
    manager["Displays"] = [
        tf_display,
        axes_display("Axes: world origin", "world", 0.9, 0.035),
        axes_display("Axes: official IMU", "imu", 0.9, 0.035),
        axes_display("Axes: official LiDAR", "lidar", 0.9, 0.035),
        axes_display("Axes: NDT terminal", "tx666_ndt_lidar", 0.9, 0.035),
        cloud_display("Map (gray)", map_topic, "175; 180; 188", 0.035, 0.68),
        cloud_display("Official-initial scan (green)", initial_topic,
                      "45; 230; 80", 0.045, 1.0),
        cloud_display("NDT terminal scan (red)", terminal_topic,
                      "255; 55; 55", 0.045, 1.0),
    ]
    manager["Global Options"]["Fixed Frame"] = fixed_frame
    manager["Global Options"]["Background Color"] = "35; 38; 44"
    manager["Views"]["Current"] = {
        "Class": "rviz/Orbit",
        "Distance": camera["distance"],
        "Enable Stereo Rendering": {
            "Stereo Eye Separation": 0.06,
            "Stereo Focal Distance": 1,
            "Swap Stereo Eyes": False,
            "Value": False,
        },
        "Field of View": 0.7853981852531433,
        "Focal Point": {
            "X": float(camera["focus"][0]),
            "Y": float(camera["focus"][1]),
            "Z": float(camera["focus"][2]),
        },
        "Focal Shape Fixed Size": True,
        "Focal Shape Size": 0.05,
        "Invert Z Axis": False,
        "Name": "Current View",
        "Near Clip Distance": 0.01,
        "Pitch": camera["pitch"],
        "Target Frame": "<Fixed Frame>",
        "Yaw": camera["yaw"],
    }
    manager["Views"]["Saved"] = None
    return config


def x11_raise_and_focus(pid):
    tree = subprocess.run(["xwininfo", "-root", "-tree"], text=True,
                          stdout=subprocess.PIPE, check=True).stdout
    for match in re.finditer(r"(0x[0-9a-fA-F]+).*", tree):
        window_id = match.group(1)
        prop = subprocess.run(["xprop", "-id", window_id, "_NET_WM_PID"],
                              text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, check=False).stdout
        if not re.search(r"=\s*" + str(pid) + r"\s*$", prop):
            continue
        x11 = ctypes.CDLL("libX11.so.6")
        x11.XOpenDisplay.restype = ctypes.c_void_p
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError("cannot_open_x_display")
        wid = ctypes.c_ulong(int(window_id, 16))
        x11.XRaiseWindow(display, wid)
        x11.XSetInputFocus(display, wid, 2, ctypes.c_ulong(0))
        x11.XFlush(display)
        x11.XCloseDisplay(display)
        return window_id
    raise RuntimeError(f"rviz_x11_window_not_found_for_pid:{pid}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--visualization-manifest", required=True)
    parser.add_argument("--rviz-template", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rviz-node-name", default="corridor01_tx666_rviz")
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    config_dir = output_dir / "rviz_configs"
    screenshot_dir = output_dir / "screenshots"
    config_dir.mkdir(parents=True, exist_ok=True)
    screenshot_dir.mkdir(parents=True, exist_ok=True)

    manifest = json.loads(Path(args.visualization_manifest).read_text(encoding="utf-8"))
    template = yaml.safe_load(Path(args.rviz_template).read_text(encoding="utf-8"))
    T_normalized_world = np.asarray(manifest["T_NORMALIZED_WORLD"], dtype=np.float64)
    initial_world = np.asarray(
        manifest["T_WORLD_LIDAR_tx666_official_initial"], dtype=np.float64)
    terminal_world = np.asarray(manifest["T_WORLD_LIDAR_tx666_ndt_terminal"], dtype=np.float64)
    center_world = 0.5 * (initial_world[:3, 3] + terminal_world[:3, 3])
    center_world[2] = -0.45
    center_normalized = apply_pose(T_normalized_world, center_world)
    raw_axis = np.array([0.0, 1.0, 0.0])
    normalized_axis = T_normalized_world[:3, :3] @ raw_axis
    raw_yaw = math.atan2(raw_axis[1], raw_axis[0])
    normalized_yaw = math.atan2(normalized_axis[1], normalized_axis[0])

    views = [
        ("rviz_raw_top", "world", "/corridor01/raw_map",
         "/corridor01/tx666_official_raw", "/corridor01/tx666_ndt_raw",
         {"focus": center_world, "distance": 6.0, "pitch": 1.56, "yaw": 0.0}),
        ("rviz_raw_perspective", "world", "/corridor01/raw_map",
         "/corridor01/tx666_official_raw", "/corridor01/tx666_ndt_raw",
         {"focus": center_world, "distance": 8.0, "pitch": 0.78, "yaw": 0.55}),
        ("rviz_raw_corridor", "world", "/corridor01/raw_map",
         "/corridor01/tx666_official_raw", "/corridor01/tx666_ndt_raw",
         {"focus": center_world, "distance": 10.0, "pitch": 0.14,
          "yaw": raw_yaw + math.pi}),
        ("rviz_normalized_top", "normalized_map", "/corridor01/normalized_map",
         "/corridor01/tx666_official_normalized",
         "/corridor01/tx666_ndt_normalized",
         {"focus": center_normalized, "distance": 6.0, "pitch": 1.56, "yaw": 0.0}),
        ("rviz_normalized_perspective", "normalized_map", "/corridor01/normalized_map",
         "/corridor01/tx666_official_normalized",
         "/corridor01/tx666_ndt_normalized",
         {"focus": center_normalized, "distance": 8.0, "pitch": 0.78, "yaw": 0.55}),
        ("rviz_normalized_corridor", "normalized_map", "/corridor01/normalized_map",
         "/corridor01/tx666_official_normalized",
         "/corridor01/tx666_ndt_normalized",
         {"focus": center_normalized, "distance": 10.0, "pitch": 0.14,
          "yaw": normalized_yaw + math.pi}),
    ]

    config_paths = []
    for name, fixed_frame, map_topic, initial_topic, terminal_topic, camera in views:
        config = make_display_config(template, fixed_frame, map_topic,
                                     initial_topic, terminal_topic, camera)
        path = config_dir / f"{name}.rviz"
        path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        config_paths.append((name, path))

    rospy.init_node("corridor01_tx666_rviz_capture", anonymous=False)
    rviz_path = shutil.which("rviz")
    if not rviz_path:
        raise RuntimeError("rviz_executable_not_found")
    first_config = config_paths[0][1]
    log = (output_dir / "rviz_capture.log").open("w", encoding="utf-8")
    rviz_environment = os.environ.copy()
    rviz_environment["DISABLE_ROS1_EOL_WARNINGS"] = "1"
    rviz = subprocess.Popen(
        [rviz_path, "--display-config", str(first_config), "--fullscreen",
         f"__name:={args.rviz_node_name}"],
        stdout=log, stderr=subprocess.STDOUT, env=rviz_environment)
    service_name = f"/{args.rviz_node_name}/load_config_discarding_changes"
    try:
        rospy.wait_for_service(service_name, timeout=45)
        load_config = rospy.ServiceProxy(service_name, SendFilePath)
        time.sleep(args.settle_seconds)
        screenshots = []
        for name, config_path in config_paths:
            response = load_config(String(data=str(config_path)))
            if not response.success:
                raise RuntimeError(f"rviz_rejected_config:{config_path}")
            time.sleep(args.settle_seconds)
            window_id = x11_raise_and_focus(rviz.pid)
            image_path = screenshot_dir / f"{name}.png"
            subprocess.run(["gnome-screenshot", "--window", "--file", str(image_path)],
                           check=True)
            if not image_path.is_file() or image_path.stat().st_size < 100_000:
                raise RuntimeError(f"rviz_screenshot_missing_or_too_small:{image_path}")
            screenshots.append({"name": name, "path": str(image_path),
                                "bytes": image_path.stat().st_size,
                                "window_id": window_id})
            print(f"CAPTURED={image_path} bytes={image_path.stat().st_size}")
        record = {
            "screenshots": screenshots,
            "rviz_pid": rviz.pid,
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
            "fixed_frame_views": [
                {"name": name, "fixed_frame": frame}
                for (name, frame, *_rest) in views
            ],
        }
        (output_dir / "screenshot_manifest.json").write_text(
            json.dumps(record, indent=2) + "\n", encoding="utf-8")
    finally:
        rviz.terminate()
        try:
            rviz.wait(timeout=10)
        except subprocess.TimeoutExpired:
            rviz.kill()
            rviz.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    main()
