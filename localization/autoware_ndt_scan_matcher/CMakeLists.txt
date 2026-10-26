cmake_minimum_required(VERSION 3.14)
project(autoware_ndt_scan_matcher)

# cspell:ignore multigrid

find_package(autoware_cmake REQUIRED)
autoware_package()

# Compile flags for SIMD instructions
# Be careful to change these options, especially when `ndt_omp` implementation is used.
# All packages linked to `ndt_omp` should use the same SIMD instruction set.
# In case mismatched instruction set are used, program causes a crash at its initialization
# because of a misaligned access to the `Eigen` libraries' data structure.
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  # For x86_64 architecture, SIMD instruction set is fixed below versions,
  # because the `ndt_omp` is optimized to these versions.
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
else()
  # For other architecture, like arm64, compile flags are generally prepared by compiler
  # march=native is disabled as default for specific depending pcl libraries
  # or pre-building packages for other computers.
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()

find_package(OpenMP)

find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

ament_auto_add_library(multigrid_ndt_omp SHARED
  src/ndt_omp/multi_voxel_grid_covariance_omp.cpp
  src/ndt_omp/multigrid_ndt_omp.cpp
  src/ndt_omp/estimate_covariance.cpp
)
target_link_libraries(multigrid_ndt_omp ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(multigrid_ndt_omp OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

link_directories(${PCL_LIBRARY_DIRS})

# The core layer: the decision logic, with no dependency on rclcpp.
ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/initial_pose_intake.cpp
  src/initial_pose_search.cpp
  src/map_update_module.cpp
  src/ndt_scan_matcher_helper.cpp
  src/particle.cpp
  src/scan_match_alignment.cpp
  src/scan_match_covariance.cpp
  src/scan_match_evaluation.cpp
  src/scan_match_no_ground_score.cpp
  src/scan_match_pipeline.cpp
)

target_link_libraries(${PROJECT_NAME}_core ${PCL_LIBRARIES} multigrid_ndt_omp)

# The ROS layer. Keeps the ${PROJECT_NAME} name so that the plugin string, the executable name and
# every existing target_link_libraries stay as they are.
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hyper_parameters_loader.cpp
  src/ndt_scan_matcher_core.cpp
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core ${PCL_LIBRARIES} multigrid_ndt_omp)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ndt_scan_matcher::NDTScanMatcher"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    add_launch_test(
      test/test_ndt_scan_matcher_launch.py
      TIMEOUT "30"
    )

    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_ros REQUIRED)

    ament_add_ros_isolated_gtest(standard_sequence_for_initial_pose_estimation
      test/test_cases/standard_sequence_for_initial_pose_estimation.cpp
      TIMEOUT "300"
    )
    target_link_libraries(standard_sequence_for_initial_pose_estimation ${PROJECT_NAME})
    ament_target_dependencies(standard_sequence_for_initial_pose_estimation
      ament_index_cpp rcl_yaml_param_parser)

    ament_add_ros_isolated_gtest(once_initialize_at_out_of_map_then_initialize_correctly
      test/test_cases/once_initialize_at_out_of_map_then_initialize_correctly.cpp
      TIMEOUT "300"
    )
    target_link_libraries(once_initialize_at_out_of_map_then_initialize_correctly ${PROJECT_NAME})
    ament_target_dependencies(once_initialize_at_out_of_map_then_initialize_correctly
      ament_index_cpp rcl_yaml_param_parser)

    ament_add_ros_isolated_gtest(particles_num_less_than_publish_num
      test/test_cases/particles_num_less_than_publish_num.cpp
      TIMEOUT "300"
    )
    target_link_libraries(particles_num_less_than_publish_num ${PROJECT_NAME})
    ament_target_dependencies(particles_num_less_than_publish_num
      ament_index_cpp rcl_yaml_param_parser)

    # Characterization tests: they pin the node's current observable behaviour through
    # `/diagnostics`, its output topics and its services, so that extracting the decision logic into
    # a ROS-free core can be shown to preserve it.
    ament_add_ros_isolated_gtest(test_ndt_scan_matcher_characteristics
      test/test_ndt_scan_matcher_characteristics.cpp
      TIMEOUT "600"
    )
    target_link_libraries(test_ndt_scan_matcher_characteristics ${PROJECT_NAME})
    ament_target_dependencies(test_ndt_scan_matcher_characteristics
      ament_index_cpp rcl_yaml_param_parser)
  endif()

  # Fast, dependency-light unit tests for the pure covariance / oscillation math.
  ament_auto_add_gtest(test_estimate_covariance
    test/test_estimate_covariance.cpp
  )
  target_link_libraries(test_estimate_covariance multigrid_ndt_omp ${PCL_LIBRARIES})

  ament_auto_add_gtest(test_ndt_scan_matcher_helper
    test/test_ndt_scan_matcher_helper.cpp
  )
  target_link_libraries(test_ndt_scan_matcher_helper ${PROJECT_NAME}_core)

  # Unit tests for the ROS-free scan-match convergence decision. No node, clock, TF or publisher is
  # involved, so these run in milliseconds where the characterization suite needs a live node.
  ament_auto_add_gtest(test_scan_match_evaluation
    test/test_scan_match_evaluation.cpp
  )
  target_link_libraries(test_scan_match_evaluation ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Unit tests for the ROS-free alignment step, against a small synthetic map.
  ament_auto_add_gtest(test_scan_match_alignment
    test/test_scan_match_alignment.cpp
  )
  target_link_libraries(test_scan_match_alignment ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Unit tests for the ROS-free covariance estimation. The MULTI_NDT cases align against a small
  # synthetic map, so no node or map loader is needed.
  ament_auto_add_gtest(test_scan_match_covariance
    test/test_scan_match_covariance.cpp
  )
  target_link_libraries(test_scan_match_covariance ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Unit tests for the ROS-free no-ground rescoring, against a small synthetic map.
  ament_auto_add_gtest(test_scan_match_no_ground_score
    test/test_scan_match_no_ground_score.cpp
  )
  target_link_libraries(test_scan_match_no_ground_score ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Unit tests for the chained scan match. Runs against the same synthetic map, so no node, map
  # loader or ROS spin is needed.
  ament_auto_add_gtest(test_scan_match_pipeline
    test/test_scan_match_pipeline.cpp
  )
  target_link_libraries(test_scan_match_pipeline ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Unit tests for the ROS-free initial pose search, against the same synthetic map.
  ament_auto_add_gtest(test_initial_pose_search
    test/test_initial_pose_search.cpp
  )
  target_link_libraries(test_initial_pose_search ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Unit tests for the ROS-free initial pose intake gates. Pure decisions, so no map or NDT.
  ament_auto_add_gtest(test_initial_pose_intake
    test/test_initial_pose_intake.cpp
  )
  target_link_libraries(test_initial_pose_intake ${PROJECT_NAME}_core)

  # Unit tests for the map update module driven by an injected fake pcd loader (no ROS service).
  ament_auto_add_gtest(test_map_update_module
    test/test_map_update_module.cpp
  )
  target_link_libraries(test_map_update_module ${PROJECT_NAME}_core multigrid_ndt_omp ${PCL_LIBRARIES})

  # Compiling this is the test: the core's headers, with rclcpp off the include path. See the file
  # for what it covers. An OBJECT library because there is nothing to run -- success is the compile.
  add_library(test_ros_independence OBJECT test/test_ros_independence.cpp)
  target_include_directories(test_ros_independence PRIVATE include src)
  # SYSTEM so that PCL's own headers are not held to this package's -Wpedantic: PCL 1.12 uses
  # anonymous structs, which -Wpedantic rejects and autoware_package() only waives for Clang.
  target_include_directories(test_ros_independence SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  # Eigen and the message interface targets only. The message targets bring rosidl, not rclcpp;
  # adding a target that pulls rclcpp in would silently defeat this test.
  target_link_libraries(test_ros_independence PRIVATE
    Eigen3::Eigen
    ${autoware_map_msgs_TARGETS}
    ${builtin_interfaces_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  )
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
