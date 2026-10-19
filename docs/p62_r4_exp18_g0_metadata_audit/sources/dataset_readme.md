---
license: cc-by-nc-sa-3.0
pretty_name: Hilti SLAM Challenge 2022
tags:
- robotics
- slam
- sensor-fusion
- lidar
- imu
- computer-vision
- construction
- ros
- localization
- mapping
---

# Hilti SLAM Challenge 2022

The **Hilti SLAM Challenge 2022** dataset is a multimodal robotics benchmark for evaluating simultaneous localization and mapping (SLAM) and sensor-fusion systems in challenging real-world environments.

The dataset was recorded with the handheld **Phasma** sensor platform at an active construction site in Schaan, Liechtenstein, and at the historic Sheldonian Theatre in Oxford, United Kingdom. The sequences include multi-level construction environments, staircases, long corridors, galleries, basements, attic spaces, outdoor areas, and long trajectories with revisits.

The sensor platform combines five synchronized global-shutter cameras, a Hesai PandarXT-32 LiDAR, and a Bosch BMI085 IMU.

For detailed sensor specifications, sequence descriptions, calibration information, and benchmark details, see the [official Hilti SLAM Challenge 2022 dataset page](https://hilti-challenge.com/dataset-2022). Questions and implementation discussions can be raised in the [challenge GitHub repository](https://github.com/Hilti-Research/hilti-slam-challenge-2022).

## Dataset contents

This repository provides approximately **350 GB** of data organized as follows:

```text
.
├── CAD/             # CAD models of the Phasma sensor platform
├── calibration/     # Sensor calibration files and calibration recordings
├── ground_truth/    # Sparse 3-DoF and selected dense 6-DoF reference trajectories
├── laser_scans/     # High-accuracy reference point clouds
└── rosbags/         # ROS 1 sensor recordings
```

The dataset contains recordings for 16 sequences.

The ROS recordings account for approximately **336 GB** of the repository. Most sequences are stored as a single `.bag` file. The long `exp23_the_sheldonian_slam` sequence is split into three parts:

```text
exp23_the_sheldonian_slam_part_0.bag
exp23_the_sheldonian_slam_part_1.bag
exp23_the_sheldonian_slam_part_2.bag
```

The ROS bags contain synchronized camera, LiDAR, and inertial measurements from the Phasma platform.

### Calibration

The `calibration/` directory contains approximately **7.1 GB** of calibration material, including:

```text
calibration/
├── calibration_files/
├── april_grid_2500x1500_7x12_15cm.yaml
├── board_calibration_sequence.bag
└── imu_noise_calibration.bag
```

The calibration sequence can be used to reproduce or inspect the camera and camera-IMU calibration, while the IMU recording is provided for inertial noise characterization.

### CAD models

The `CAD/` directory contains CAD representations of the Phasma sensor platform:

```text
CAD/
├── Phasma25.step
├── Phasma25.stl
└── Phasma25_GOM_high_res.stl
```

These files can be useful for inspecting sensor placement and coordinate-frame relationships.

### Reference laser scans

The `laser_scans/` directory contains high-accuracy reference point clouds for the two main recording environments:

```text
laser_scans/
├── construction_site_full.e57
└── sheldonian_full.e57
```

The point clouds are provided in the E57 format.

## Ground truth

Ground-truth data are provided as text files in the `ground_truth/` directory.

Sparse millimeter-accurate reference positions are available for the challenge sequences. These reference measurements were established using surveyed control points and are intended for high-accuracy trajectory evaluation.

Selected sequences additionally provide dense 6-DoF trajectories in the IMU frame:

```text
exp14_basement_2_imu.txt
exp16_attic_to_upper_gallery_2_imu.txt
exp18_corridor_lower_gallery_2_imu.txt
```

Corresponding sparse 3-DoF reference files are also available for these sequences.

Ground-truth filenames ending in `_imu` are expressed in the IMU frame. Users should inspect the calibration and transform information before comparing estimated trajectories expressed in other sensor frames.

## Links

* [Official dataset page](https://hilti-challenge.com/dataset-2022)
* [Challenge GitHub repository](https://github.com/Hilti-Research/hilti-slam-challenge-2022)
* [Hilti-Oxford Dataset paper](https://arxiv.org/abs/2208.09825)

## Citation

When using this dataset in academic work, please cite:

```bibtex
@article{zhang2023hilti,
  author  = {Lintong Zhang and Michael Helmberger and Lanke Frank Tarimo Fu and David Wisth and Marco Camurri and Davide Scaramuzza and Maurice Fallon},
  title   = {Hilti-Oxford Dataset: A Millimeter-Accurate Benchmark for Simultaneous Localization and Mapping},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {8},
  number  = {1},
  pages   = {408--415},
  year    = {2023},
  doi     = {10.1109/LRA.2022.3226077}
}
```

## License

The dataset is released under the **Creative Commons Attribution-NonCommercial-ShareAlike 3.0** license ([CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/)).

Use is limited to non-commercial purposes. Attribution is required, and adaptations must be distributed under the same or a compatible license. Consult the full license text and the official challenge page before redistribution or publication of derived data.
