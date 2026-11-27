"""Materialize minimal instrumentation on pinned local research dependencies."""
import hashlib
from pathlib import Path
import sys

root=Path('/home/jian/rosbag/public_datasets/IILABS3D/nav_a_diff_omni_official_v1/gate2_051_runtime/src/hdl_localization')
out=Path(sys.argv[1])
out.mkdir(parents=True,exist_ok=False)
files={
    root/'include/hdl_localization/pose_estimator.hpp':out/'hdl_localization/pose_estimator.hpp',
    root/'src/hdl_localization/pose_estimator.cpp':out/'pose_estimator.cpp',
}
(out/'hdl_localization').mkdir()
for src,dst in files.items():
    s=src.read_text()
    print(hashlib.sha256(src.read_bytes()).hexdigest(),src)
    if src.suffix=='.hpp':
        s=s.replace('#include <memory>','#include <memory>\n#include <functional>')
        start='  /* getters */'
        s=s.replace(start,'''  using DecisionHook = std::function<bool(const Eigen::Matrix4f&, Eigen::Matrix4f&)>;
  DecisionHook decision_hook;
  Eigen::MatrixXf covariance() const;
  unsigned map_updates = 0;

'''+start)
    else:
        needle='  Eigen::Matrix4f trans = registration->getFinalTransformation();'
        assert s.count(needle)==1
        s=s.replace(needle,needle+'''
  // Invoke before the original correction, preserving its observation model.
  if (decision_hook && !decision_hook(init_guess, trans)) return aligned;
  last_correction_stamp = stamp;
''')
        s=s.replace('  last_correction_stamp = stamp;\n\n  Eigen::Matrix4f no_guess','  Eigen::Matrix4f no_guess')
        s=s.replace('  ukf->correct(observation);','  ukf->correct(observation);\n  ++map_updates;')
        s=s.replace('/* getters */','Eigen::MatrixXf PoseEstimator::covariance() const { return ukf->cov; }\n\n/* getters */')
    dst.write_text(s)
q2=Path(__file__).resolve().parent.parent/'p64_r2_q2_bounded_search_adaptation'
artifact=Path('/home/jian/p64_r2_q2_ws/run/build_final/scoring_profile.hpp')
expected='3'
profile=artifact.read_text()
assert 'parkMap()' in profile
(out/'scoring_profile.hpp').write_text(profile)
s=(q2/'search.cpp').read_text()
needle='    ++time_.probes;'
assert s.count(needle)==1
s=s.replace(needle,'''    if (budget && !budget()) throw std::runtime_error("search deadline");
'''+needle)
s=s.replace('class Grid {\n public:', 'class Grid {\n public:\n  std::function<bool()> budget;')
(out/'search_reuse.hpp').write_text(s)
print(hashlib.sha256(artifact.read_bytes()).hexdigest(),artifact)
