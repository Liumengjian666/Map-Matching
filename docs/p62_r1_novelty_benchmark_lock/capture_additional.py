"""One finite metadata/source supplement; no large data download."""
from capture_sources import capture


if __name__=='__main__':
    base='https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/e62017f907007fdc5ab8c721842e4ae7359d7f49/'
    capture([
      ('hilti_exp18_6dof_schema.txt',base+'ground_truth/exp18_corridor_lower_gallery_2_imu.txt'),
      ('hilti_exp18_3dof_schema.txt',base+'ground_truth/exp18_corridor_lower_gallery_2_imu_3dof.txt'),
      ('hf_readme.md',base+'README.md'),
      ('superloc2025.pdf','https://arxiv.org/pdf/2412.02901v2'),
      ('large_errors2024.pdf','https://www.ee.bgu.ac.il/~francos/On_Minimizing_the_Probability_of_Large_Errors_in_Robust_Point_Cloud_Registration.pdf'),
      ('dali_mapping.cpp','https://raw.githubusercontent.com/DCSI2022/DALI_SLAM/main/DA_LIO/src/laserMapping.cpp'),
    ],'capture_additional.json')
