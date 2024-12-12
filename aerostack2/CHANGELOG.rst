^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerostack2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2024-08-22)
------------------
* [as2_behaviors_path_planning, as2_map_server] Remove backward_ros dependency
* Contributors: Miguel Fernandez-Cortizas, pariaspe

1.1.1 (2024-08-20)
------------------
* [as2_keyboard_teleoperation] context is modified manually from generated config file
* [as2_msgs] add sensor msgs dependency
* [as2_python_api] License fix using ament copyright
* [as2_state_estimator] Fix gtest warning: publisher already registered
* [as2_usb_camera_interface] Fix gtest warning: publisher already registered
* Contributors: Rafael Perez-Segui, pariaspe, Miguel Fernandez-Cortizas, Javilinos

1.1.0 (2024-08-08)
------------------
* [aerostack2] Update package dependencies and CI with new aerostack2 packages
* [aerostack2] many platforms extracted from main repo
* [aerostack2] Generate all aerostack2 doxygen together
* [as2_gazebo_classic] EOL - removing from aerostack2
* [as2_platform_gazebo] Use sim time true by default in gazebo platform
* [as2_platform_gazebo] Migrate to ament_lint_auto and use as2_core param utils
* [as2_platform_multirotor_simulator] Enable control using odometry as state instead of ground truth
* [as2_platform_multirotor_simulator] Update platform to use world.yaml and rename some params
* [as2_platform_multirotor_simulator] New launch utils to declare parameters from a config file, update launchers at multirotor simulation
* [as2_platform_multirotor_simulator] Add basic fly gtest mock
* [as2_platform_multirotor_simulator] Add ament_cmake_gtest to package.xml
* [as2_platform_multirotor_simulator] Add as2_multirotor_simulator package
* [as2_behavior_tree] Use ament_lint
* [as2_behavior] ament format, passing all tests
* [as2_behaviors_motion] Not overwritting behavior config file if user config file is empty
* [as2_behaviors_motion] Update to ament lint auto and use as2_core launch param utils
* [as2_behaviors_motion] Fix warning about service client access
* [as2_behaviors_path_planning] New behavior for navigation with basic a_star plugin working
* [as2_behaviors_perception] as2::sensors::Camera change to load parameters from ROS 2 parameters
* [as2_behaviors_perception] New launch utils to declare parameters from a config file
* [as2_behaviors_perception] migrate deprecated api from cv::aruco::drawAxis to cv::drawFrameAxes
* [as2_behaviors_perception] Update launchers to Flake8
* [as2_behaviors_perception] List launch arguments from parameter config file
* [as2_behaviors_platform] as2_behaviors_platform passing ament tests
* [as2_behaviors_platform] Fix warning about service client access
* [as2_cli] improvement in as2_list and as2_test
* [as2_core] Extended params utils with new capabilities
* [as2_core] Make SensorData public to access getData and getTopicName
* [as2_core] Don't call ownSendCommand until new ref arrives after control mode change
* [as2_core] AS2 Sensors Camera read encoding from ROS param
* [as2_core] as2::sensors::Camera change to load parameters from ROS 2 parameters
* [as2_core] New launch utils to declare parameters from a config file
* [as2_core] Add sync methods call in platform mocks
* [as2_core] Add mock library. Add a mock for aerial platform
* [as2_core] Refactor sensors and add gimbal sensor
* [as2_core] Add launch params utils
* [as2_core] Add GroundTruth to handler ground truth publish
* [as2_core] Change as2_aerial_platform methods visibility
* [as2_core] adding as2_core tests
* [as2_realsense_interface] Fix timestamp
* [as2_realsense_interface] Update real sense camera sensor constructor and methods
* [as2_realsense_interface] node passes ament, pep257 and flake8 tests
* [as2_usb_camera_interface] Add camera calibration file param
* [as2_usb_camera_interface] as2::sensors::Camera change to load parameters from ROS 2 parameters
* [as2_usb_camera_interface] as2 usb interface should pass ament tests format
* [as2_map_server] New map server with 2d mapping plugin
* [as2_motion_controller] update controller launcher with launch utils and particular launchers for each plugin
* [as2_motion_controller] Update launch using as2_core launch params utils
* [as2_motion_controller] Fix library exports and add simple gtest
* [as2_motion_controller_manager] Publish speed limits when bypassing and control mode is Position
* [as2_motion_controller] Use ament_lint_auto
* [as2_motion_reference_handlers] Ament common tests compliance
* [as2_msgs] New action for path planning behavior
* [as2_msgs] move interfaces from as2_utilities to as2_msgs package
* [as2_msgs] Add lint tests
* [as2_python_api] Deactivating drone interface tests
* [as2_python_api] Return bool service response when using ServiceBoolHandler (arm, offboard and manual)
* [as2_python_api] Fix using spin rate as a rate and not as an time interval
* [as2_python_api] Added class argument spin rate
* [as2_python_api] New navigate to behavior and module
* [as2_python_api] Behavior Status updated after call methods correctly
* [as2_python_api] ament lint auto compliance
* [as2_python_api] Add goal accepted/rejected return when call behavior method
* [as2_python_api] Continue using Pydantic V1
* [as2_gazebo_assets] Improve crazyflie model: controller tuned and added IMU
* [as2_gazebo_assets] Added crazyflie viz MODEL
* [as2_gazebo_assets] Updated Crazyflie Gazebo model
* [as2_gazebo_assets] Add optical frames to cameras
* [as2_gazebo_assets] Fix model templates for sensor naming
* [as2_gazebo_assets] Support yaml files for gazebo world configuration
* [as2_gazebo_assets] Fix malformed bridges
* [as2_gazebo_assets] ament lint auto compliance
* [as2_gazebo_assets] Continue using Pydantic V1
* [as2_gazebo_assets] environment variable IGN_GAZEBO_RESOURCE_PATH changed to GZ_SIM_RESOURCE_PATH
* [as2_gazebo_assets] Add actuators bridge
* [as2_gazebo_assets] Add Gazebo Harmonic launcher by env variable GZ_VERSION
* [as2_gazebo_assets] Add actuators bridge and disable velocity controller with a param
* [as2_gazebo_assets] Fix gimbal_bridge on Gazebo Harmonic
* [as2_gazebo_assets] Gz harmonic support
* [as2_gazebo_assets] Sensor model name and sensor link name were swapped in camera bridges
* [as2_state_estimator] map frame generates at the drone initial position
* [as2_state_estimator] Change gazebo tf parameter name
* [as2_state_estimator] update state estimator with new launch utils and new launchers for each plugin
* [as2_state_estimator] Code refactor, adding individually CMakeLists.txt and config for each plugin
* [as2_state_estimator] Migrate to ament lint
* [as2_state_estimator] mocap_pose plugin changed to work with mocap4ros2
* [as2_alphanumeric_viewer] Battery status problem fixed
* [as2_alphanumeric_viewer] alphanumeric viewer passing every ament test
* [as2_keyboard_teleoperation] extend default behaviors from argument
* [as2_keyboard_teleoperation] ability to load modules into interface to control behaviors
* [as2_keyboard_teleoperation] drone_frequency param added
* [as2_keyboard_teleoperation] ament lint compliance
* [as2_keyboard_teleoperation] add ros 2 parameters with config file using as2 launch utils
* [as2_keyboard_teleoperation] launcher improved to load namespace with different dividers, config file and speed behavior
* [as2_rviz_plugins] add as2_rviz_plugins package
* [as2_visualization] add as2_visualization package
* [as2_external_object_tf_tf] fix mocap source data
* [as2_external_object_to_tf] default config file within the node, added description
* [as2_external_object_to_tf] removed interface generation dependencies from external object to tf
* [as2_external_object_to_tf] as2_utilities package created, external object to tf node added to aerostack2
* [as2_geozones] Create a utilities folder, as2_geozones package added
* Contributors: pariaspe, Rafael Perez-Segui, Javilinos, Miguel Fernandez-Cortizas, keikei014, Sakura286

1.0.9 (2024-03-25)
------------------
* [as2_platform_tello] Missing params and new file name
* [as2_behaviors_motion] Relative yaw in go_to fixed when frame is other than earth
* [as2_behaviors_perception] PointGimbalBehavior to use TF
* [as2_behaviors_perception] Point Gimbal behavior
* [as2_msgs] New point gimbal action msg
* [as2_core] get quaternion stamped included in tf utils
* [as2_core] quaternion convert wrapped in try and catch
* [as2_core] Add quaternion support for TF convert method
* [as2_python_api] added try except in deserialize method
* [as2_python_api] Added feedback to rtl module
* [as2_python_api] Add init files to missing subpkgs
* [as2_python_api] New RTL module
* [as2_python_api] Point Gimbal behavior client
* [as2_python_api] Add topic namespace with argument in mission interpreter
* [as2_gazebo_assets] added local frame to simulated gimbal msg
* [as2_gazebo_assets] Partial fix on Crazyflie Model
* [as2_gazebo_assets] Point gimbal working and Gimbal Inertial links enabling to fly
* [as2_gazebo_assets] Nested sdf model for gimbal and fixed TF tree
* [as2_state_estimator] latlon2local function fails in z coordinate, added earth_to_map_height parameter
* [as2_state_estimator] raw odometry plugin should have a set gps origin
* Contributors: Javier Melero, Javilinos, pariaspe, Rafael Perez-Segui, Rafael Pérez, Miguel Fernandez-Cortizas, cvar-developers, Mickey Li

1.0.8 (2024-02-12)
------------------
* [as2_core] added pybind11 dependency
* [as2_python_api] Jenkins test fail: searching for module first at modules folder and hook as2_modules_path fix
* Contributors: pariaspe, Rafael Pérez

1.0.7 (2024-02-04)
------------------
* [as2_platform_dji_osdk] Added libusb-dev dependency
* [as2_platform_gazebo] Remove ign from name
* [as2_cli] Clean old unused files
* [as2_core] Bug fixed, getPoseStamped function differs from timeout 0 and not 0
* [as2_core] Python as2_names bindings
* [as2_core] format tests according with ament_lint_common() packages
* [as2_python_api] Moving test files to test folder
* [as2_gazebo_assets] Gimbal model added
* [as2_gazebo_assets] Cleaning remaining ign, bridges use gz instead ign
* [as2_gazebo_assets] Renamed to as2_gazebo_assets
* [as2_gazebo_classic_assets] Fix pass drone namespace to run_sitl.sh
* Contributors: Miguel Fernandez-Cortizas, Javilinos, Rafael Pérez, pariaspe, pawanw17

1.0.6 (2023-12-12)
------------------
* [as2_python_api] Specifying subdirectories in setup.py
* [as2_motion_controller] Update PID controller dependence to v1.0
* [aerostack2] Takeoff and GoTo behaviors renaming
* [as2_platform_crazyflie] Fix wrong sensor name for multiranger deck
* [as2_core] Add node options to aerial platform
* [aerostack2] Sorted and updated list of dependencies in metapackage
* [as2_behaviors] Behaviors composable nodes
* [as2_motion_controller] Refactor CMakeLists.txt for creating a dynamic lib for each plugin
* [as2_motion_controller] Add trajectory reference to actuators commands
* [as2_msgs] Geofence messages removed from as2_msgs
* [as2_gazebo_classic_assets] Load custom world in gazebo 11
* Contributors: Rafael Pérez, Miguel Fernandez-Cortizas, pariaspe, RPS98, Javilinos, adri-mp, 

1.0.5 (2023-11-08)
------------------
* [as2_platform_crazyflie] Multi-ranger deck interface to laser_scan msg
* [as2_platform_dji_osdk] Add camera change source topic
* [as2_platform_dji_osdk] Fixes gps time subscription
* [as2_platform_tello] Add camera_freq param to platform config file
* [as2_platform_tello] Fixed tello camera
* [as2_core] Deal with low latency frames that are not earth
* [as2_motion_reference_handlers] Explicit namespace for motion reference handlers
* [as2_msgs] Improve MissionUpdate message
* [as2_python_api] Load modules from project path for mission interpreter
* [as2_python_api] Improve MissionUpdate message
* [as2_gazebo_classsic_assets] Add gazebo_ros_pkgs dependence
* [as2_gazebo_classsic_assets] Runs PX4 in the foreground if gzclient is disabled (HEADLESS)
* [as2_ign_gazebo_assets] Spawn objects from gz resource path
* [as2_ign_gazebo_assets] Hexrotor back to fly
* Contributors: Javilinos, pariaspe, RPS98, pawanw17, Miguel Fernandez-Cortizas, Rodrigo Da Silva

1.0.4 (2023-08-23)
------------------

1.0.3 (2023-08-22)
------------------

1.0.2 (2023-08-17)
------------------

1.0.1 (2023-04-25)
------------------
* Merge pull request `#223 <https://github.com/aerostack2/aerostack2/issues/223>`_ from aerostack2/200-unify-maintainer-in-packagexmls
  Maintainer unified to CVAR-UPM
* Maintainer unified to CVAR-UPM
* Contributors: Miguel Fernandez-Cortizas, pariaspe

1.0.0 (2023-03-18)
------------------

0.2.2 (2022-12-20)
------------------

0.2.1 (2022-12-19)
------------------
* Merge pull request `#33 <https://github.com/aerostack2/aerostack2/issues/33>`_ from aerostack2/behavior_tree
  Update behavior tree
* Update behavior tree
* Merge pull request `#15 <https://github.com/aerostack2/aerostack2/issues/15>`_ from aerostack2/pkg_dependencies
  Pkg dependencies
* Update pkg dependencies
* aerostack2 pkg added
* Contributors: Miguel Fernandez-Cortizas, RPS98, miferco97

0.2.0 (2022-07-22)
------------------
* basic_state_estimator first release
* usv_ignition_platform first release
* behaviour_trees first release
* basic_tf_tree_generator deprecated
* ros_ign deprecated
* [as2_msgs] GoToWaypoint action: new yaw_mode_flag to replace ignore_pose_yaw
* [as2_msgs] New msg MissionEvent
* [as2_core] Added mode2string utils
* [as2_core] Added frame utils
* [as2_core] Added launch parameters
* [as2_core] Odom refactorization
* [as2_core] New topic names
* [as2_core] Minor bug fixes
* [as2_core] Added addStaticTransform() method to as2::sensor
* [motion_reference_handlers] New postion motion handler
* [motion_reference_handlers] New hover motion handler
* [motion_reference_handlers] Multiple instances bug fixed
* [motion_reference_handlers] Added frame_id to handlers
* [motion_reference_handlers] Minor bugs fixed
* [controller_manager] New launchers with config files
* [controller_manager] Added hover support
* [controller_manager] Odom refatorization
* [controller_manager] Added bypass launch argument
* [controller_plugin_speed_controller] Robust yaw angle computation
* [controller_plugin_speed_controller] Added position control speed limit
* [controller_plugin_speed_controller] Added position control bypass limit
* [controller_plugin_speed_controller] Yaw control bug fixed
* [controller_plugin_speed_controller] Adapted to new launcher with config files
* [controller_plugin_speed_controller] Renamed to follow name convention
* [controller_plugin_speed_controller] Added hover support
* [controller_plugin_speed_controller] Added bypass to speed controller
* [controller_plugin_speed_controller] Speed limit changed to proportional limit
* [controller_plugin_speed_controller] Odom refactorization
* [trajectory_generator] Time evaluation in trajectory fixed
* [trajectory_generator] Yaw angle bug fixed
* [trajectory_generator] New launcher with config files
* [trajectory_generator] Odom refactorization
* [ignition_platform] Added dynamic multiple sensors support
* [ignition_platform] Added laser_scan sensor support
* [ignition_platform] Minor bug fixed
* [ignition_platform] New launcher with config files
* [ignition_platform] Added gps sensor
* [ignition_platform] Odom refactorization
* [ignition_platform] Added frame and TF to sensors
* [ignition_assets] Added crazyflie model (WIP)
* [ignition_assets] Added verbose mode to ign launcher
* [ignition_assets] Added hexrotor model
* [ignition_assets] Added hooks, cmake created
* [ignition_assets] New script to only drone spawning
* [ignition_assets] Added odometry plugin
* [ignition_assets] Odom, bat and comms can be selectable through jinja generator
* [ignition_assets] New semantic camera sensor
* [ignition_assets] New USV model
* [ignition_assets] new GPS sensor
* [ignition_assets] Minor fixes and improvements
* [ignition_assets] Added lidar sensor
* [python_interface] Added yaw_mode argument to follow_path
* [python_interface] Added gps go_to methods
* [python_interface] Fixed bug on python method overload
* [python_interface] Odom refactorization
* [python_interface] set_home changed into public method
* [as2_basic_behaviours] New launchers with config files
* [takeoff_behaviour] New launcher with config files
* [takeoff_behaviour] Odom refactorization
* [takeoff_plugins] Plugin renamed to follow name convention
* [takeoff_plugins] Added position takeoff plugin
* [takeoff_plugins] Added platform takeoff plugin
* [land_behaviour] Disarm after land bug fixed
* [land_behaviour] New launcher with config files
* [land_behaviour] Odom refactorization
* [land_plugins] Land goal condition imporved
* [land_plugins] Plugin renamed to follow name convention
* [land_plugins] Changed to hover when land is cancelled
* [land_plugins] Added platfotm land plugin
* [go_to_behaviour] Yaw angle computation fixed
* [go_to_behaviour] New launcher with config files
* [go_to_behaviour] Enable go_to with negative height
* [go_to_behaviour] Added launch argument for speed limit flag
* [go_to_behaviour] Odom refactorization
* [go_to_plugins] Yaw angle computation fixed
* [go_to_plugins] Added position go_to plugin
* [go_to_plugins] Fixed yaw_angle computation
* [go_to_plugins] Plugin renamed following name convention
* [go_to_plugins] Enable path facing go_to position
* [go_to_plugins] Hover after go_to
* [go_to_plugins] Added speed limit to plugins 
* [follow_path_behaviour] New launcher with config files
* [follow_path_behaviour] Odom refactorization
* [follow_path_plugins] Plugins renamed following name convention
* [follow_path_plugins] Improved goal condition in traj plugin

0.1.0 (2022-05-13)
------------------
* as2_msgs first release
* as2_core first release
* basic_tf_tree_generator first release
* actuator_command_handlers first release
* motion_reference_handlers first release
* controller_manager first release
* controller_plugin_speed_controller first release
* trajectory_generator first release
* ignition_platform first release
* ignition_assets first release
* python_interface first release
* as2_basic_behaviours first release
* takeoff_behaviour first release
* takeoff_plugins first release
* land_behaviour first release
* land_plugins first release
* go_to_behaviour first release
* go_to_plugins first release
* follow_path_behaviour first release
* follow_path_plugins first release
* ros_ign first release