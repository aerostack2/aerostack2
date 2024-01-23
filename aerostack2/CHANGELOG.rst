^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerostack2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
