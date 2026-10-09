# CMake generated Testfile for
# Source directory: /tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2
# Build directory: /tmp/p9_corridor_robust_bootstrap_v2_build
#
# This file includes the relevant testing commands required for
# testing this directory and lists subdirectories to be tested as well.
add_test(robust_bootstrap_geometry_time_test "/tmp/p9_corridor_robust_bootstrap_v2_build/p9_bootstrap_v2" "--self-test")
set_tests_properties(robust_bootstrap_geometry_time_test PROPERTIES  _BACKTRACE_TRIPLES "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;12;add_test;/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;0;")
add_test(robust_bootstrap_preparation_test "/usr/bin/python3.8" "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/test_prepare.py")
set_tests_properties(robust_bootstrap_preparation_test PROPERTIES  _BACKTRACE_TRIPLES "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;13;add_test;/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;0;")
add_test(robust_bootstrap_motion_adapter_test "/usr/bin/python3.8" "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/test_motion.py")
set_tests_properties(robust_bootstrap_motion_adapter_test PROPERTIES  _BACKTRACE_TRIPLES "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;14;add_test;/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;0;")
add_test(inherited_preintegration_test "/usr/bin/python3.8" "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/../corridor_moving_init/test_preintegration.py")
set_tests_properties(inherited_preintegration_test PROPERTIES  _BACKTRACE_TRIPLES "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;15;add_test;/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;0;")
add_test(robust_bootstrap_one_shot_test "/usr/bin/python3.8" "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/test_one_shot.py")
set_tests_properties(robust_bootstrap_one_shot_test PROPERTIES  _BACKTRACE_TRIPLES "/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;16;add_test;/tmp/dog_loc_paper_r4_ws.Fq21k2/src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/corridor_robust_bootstrap_v2/CMakeLists.txt;0;")
