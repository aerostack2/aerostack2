^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_behaviors_motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2023-12-12)
------------------
* Takeoff and GoTo behavior renaming
* Behavior composable nodes
* Contributors: Miguel Fernandez-Cortizas, pariaspe

1.0.5 (2023-11-08)
------------------
* tf timeout param added to go to function plus hotfix
* tf utils, behaviors and controller adapted to deal with low latency frames that are not earth
* timeout param added
* Contributors: Javier Melero, Javilinos, Miguel Fernandez-Cortizas, Rodrigo Da Silva

1.0.4 (2023-08-23)
------------------

1.0.3 (2023-08-22)
------------------
* Merge pull request `#317 <https://github.com/aerostack2/aerostack2/issues/317>`_ from aerostack2/316-tf2_geometry_msgs-dep
  Only use tf2_geometry_msgs library when needed
* only use library when needed
* Contributors: Miguel Fernandez-Cortizas, pariaspe

1.0.2 (2023-08-17)
------------------
* Merge pull request `#304 <https://github.com/aerostack2/aerostack2/issues/304>`_ from aerostack2/302-parametrize_tf_threshold_time
  302-Parametrize tf timeout in go to and follow reference behavior
* parametrize tf timeout in go to and follow reference behavior
* Merge pull request `#273 <https://github.com/aerostack2/aerostack2/issues/273>`_ from aerostack2/272-add_reference_facing_mode_to_follow_reference
  follow reference with reference facing added
* add follow reference with new yaw mode
* follow reference with reference facing added
* Merge pull request `#268 <https://github.com/aerostack2/aerostack2/issues/268>`_ from aerostack2/fix_follow_path_with_reference
  follow path frame bug fixed, frame id argument added to python interface
* follow path frame bug fixed, frame id argument added to python interface
* Merge pull request `#237 <https://github.com/aerostack2/aerostack2/issues/237>`_ from aerostack2/unify_platform_launchers
  Unify launchers
* Unify as2_behaviors_motion with config files
* Contributors: Javier Melero, Javilinos, Miguel Fernandez-Cortizas, RPS98, Rafael Pérez, pariaspe

1.0.1 (2023-04-25)
------------------
* Merge pull request `#223 <https://github.com/aerostack2/aerostack2/issues/223>`_ from aerostack2/200-unify-maintainer-in-packagexmls
  Maintainer unified to CVAR-UPM
* Maintainer unified to CVAR-UPM
* Merge pull request `#214 <https://github.com/aerostack2/aerostack2/issues/214>`_ from aerostack2/213-as2_behaviors_motion-add-land-with-trajectory
  [as2_behaviors_motion] Add land with trajectory
* Add land plugin trajectory
* Merge pull request `#199 <https://github.com/aerostack2/aerostack2/issues/199>`_ from aerostack2/196-as2_behaviors_motion-follow-reference-behavior
  [as2_behaviors_motion] Follow Reference Behavior Transforms Error
* redundant-missplaced warning removed
* Remove follow reference from motion behaviors launch
* Contributors: Javilinos, Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.0 (2023-03-18)
------------------
* Merge pull request `#195 <https://github.com/aerostack2/aerostack2/issues/195>`_ from aerostack2/add_follow_reference_behavior
  Follow reference typos fix
* typos fix
* Merge pull request `#193 <https://github.com/aerostack2/aerostack2/issues/193>`_ from aerostack2/add_follow_reference_behavior
  Add follow reference behavior
* clang test passes, format changes, new individual launcher
* feedback fixed
* removed gdb from compiling
* new behavior created and tested, modify tbd
* Merge pull request `#176 <https://github.com/aerostack2/aerostack2/issues/176>`_ from aerostack2/175-as2_behaviors_motion-land-behavior-not-loading-land-speed-condition-parameter
  Loading parameters in land behavior fixed
* fixes `#175 <https://github.com/aerostack2/aerostack2/issues/175>`_
* Merge pull request `#163 <https://github.com/aerostack2/aerostack2/issues/163>`_ from aerostack2/crazyflie_swarm_demo
  Crazyflie swarm demo updates
* Merge pull request `#160 <https://github.com/aerostack2/aerostack2/issues/160>`_ from aerostack2/trajectory_generation
  Change trajectory generator name
* Demo cf swarm updates
* Change trajectory generator name
* Revert "Change trayectory generator"
  This reverts commit fe17ea3f0a5ef1eb377e0779359bd75dc8c3212d.
* Change trayectory generator
* Merge branch 'main' into aruco_detect
* Merge pull request `#156 <https://github.com/aerostack2/aerostack2/issues/156>`_ from aerostack2/go_to
  [as2_behaviors_motion] Change goto to go_to
* Change goto to go_to
* Merge pull request `#152 <https://github.com/aerostack2/aerostack2/issues/152>`_ from aerostack2/151-as2_behaviors_motion-follow-path-launch
  Stripping whitespaces in launch file name
* stripping whitespaces in launch file name
* rename movement_behavior launcher into motion_behaviors_launcher.py
* Merge pull request `#125 <https://github.com/aerostack2/aerostack2/issues/125>`_ from aerostack2/as2_behaviors
  [as2_behaviors] Reorganize behaviors
* Behaviors motion clang test
* Reorganize behaviors
* Contributors: Javilinos, Miguel Fernandez-Cortizas, RPS98, Rafael Pérez, pariaspe

0.2.2 (2022-12-22)
------------------

0.2.1 (2022-12-19)
------------------
