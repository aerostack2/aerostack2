^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_gazebo_assets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2024-03-25)
------------------
* added local frame to simulated gimbal msg
* Partial fix on Crazyflie Model
* Point gimbal working and Gimbal Inertial links enabling to fly
* Nested sdf model for gimbal and fixed TF tree
* Contributors: Javier Melero, Javilinos, Mickey Li, Miguel Fernandez-Cortizas, Rafael Pérez, pariaspe

1.0.8 (2024-02-12)
------------------

1.0.7 (2024-02-04)
------------------
* Gimbal model added
* Cleaning remaining ign, bridges use gz instead ign
* [as2_gazebo_assets] Renamed to as2_gazebo_assets
* Contributors: Javilinos, Rafael Pérez, pariaspe

1.0.6 (2023-12-12)
------------------

1.0.5 (2023-11-08)
------------------
* Spawn objects from gz resource path
* Hexrotor back to fly
* Contributors: Javilinos, pariaspe

1.0.4 (2023-08-23)
------------------

1.0.3 (2023-08-22)
------------------

1.0.2 (2023-08-17)
------------------
* Merge pull request `#314 <https://github.com/aerostack2/aerostack2/issues/314>`_ from aerostack2/313-payload-bridges-missing
  Payload bridges fix
* fixes issue 313: payload bridges
* Merge pull request `#306 <https://github.com/aerostack2/aerostack2/issues/306>`_ from aerostack2/as2_viz
  Added drone_viz sdf
* added drone_viz sdf
* Merge pull request `#305 <https://github.com/aerostack2/aerostack2/issues/305>`_ from aerostack2/300-add_sim_time_to_gps_bridge
  use_sim_time now works in gps bridge
* use sim time now works in gps bridge
* added grass world
* Merge pull request `#295 <https://github.com/aerostack2/aerostack2/issues/295>`_ from aerostack2/290-add_gps_origin_from_json_file
  add gps origin from json file
* now assets can be loaded from other directory
* jinja always used, if it doesn't exist, sdf is used
* add gps origin from json file
* Merge pull request `#286 <https://github.com/aerostack2/aerostack2/issues/286>`_ from aerostack2/285-fix_python_list_type
  Changed typing list to List
* changed list to List
* Merge pull request `#278 <https://github.com/aerostack2/aerostack2/issues/278>`_ from aerostack2/229-as2_simulation_assets-earth-needs-to-be-alligned-in-enu-frame
  Change 0 deg reference from North to East
* Change 0 deg reference from North to East
* Merge pull request `#266 <https://github.com/aerostack2/aerostack2/issues/266>`_ from aerostack2/add_position_plugin_to_gates
  added position plugin to gates
* added position plugin to gates
* Clean gazebo on exit
* Merge pull request `#232 <https://github.com/aerostack2/aerostack2/issues/232>`_ from aerostack2/launch_gz_v2
  Gazebo launcher engine refactored
* Change assets simulation_config_file name
* default empty value for joints and bridges
* added pydantic dep
* models splitted in several files
* updated to new model
* model refactor
* Contributors: Javilinos, Miguel Fernandez-Cortizas, RPS98, Rafael Pérez, pariaspe, rdasilva01

1.0.1 (2023-04-25)
------------------
* Merge pull request `#223 <https://github.com/aerostack2/aerostack2/issues/223>`_ from aerostack2/200-unify-maintainer-in-packagexmls
  Maintainer unified to CVAR-UPM
* Maintainer unified to CVAR-UPM
* Merge pull request `#222 <https://github.com/aerostack2/aerostack2/issues/222>`_ from aerostack2/221-gazebo-gui-config-file
  [as2_ign_gazebo_assets] Added GUI config file
* gui config file and some documenting
* Merge pull request `#209 <https://github.com/aerostack2/aerostack2/issues/209>`_ from aerostack2/208-as2_simulation_assets-use-sim-time-default-parameter
  [ as2_simulation_assets] Use sim time default parameter
* Change use_sim_time default value
* Merge pull request `#203 <https://github.com/aerostack2/aerostack2/issues/203>`_ from aerostack2/simulation_assets_for_windmill_test
  [as2_ign_gazebo_assets] added use_sim_time parameter
* added use_sim_time parameter to ign_assets
* Merge pull request `#202 <https://github.com/aerostack2/aerostack2/issues/202>`_ from aerostack2/new-windmill
  [as2_ign_gazebo_assets] Added debug model for visualize in rviz
* added debug model for visualize in rviz
* Contributors: Javier Melero, Javilinos, Miguel Fernandez-Cortizas, RPS98, Rafael Pérez, pariaspe

1.0.0 (2023-03-18)
------------------
* Merge pull request `#174 <https://github.com/aerostack2/aerostack2/issues/174>`_ from aerostack2/as2_simulation_assets_for_windmill_project
  [as2_simulation_assets] Windmill updates
* ground truth bridge in cmakelist was missing
* fix bug tf broadcaster
* tf broadcaster renamed to object tf broadcaster
* last changes
* stop windmill gps reference frame from rotating when blades rotate aswell
* object tf bridge added, windmill object finished
* added controller for windmill in ros2
* grass patch with sky added
* joint controller bridge added, windmill joints controllable from ros2
* added velocity controller to windmill for box
* Merge remote-tracking branch 'origin' into as2_simulation_assets_for_windmill_project
* Merge pull request `#171 <https://github.com/aerostack2/aerostack2/issues/171>`_ from aerostack2/ign_empty_gps
  [as2_ign_gazebo_assets] Add GPS coordinates to empty world
* Add GPS coordinates to empty world
* azimuth bridge added
* added gps odometry for azimuth calculation, gps bridge for windmill
* Merge pull request `#145 <https://github.com/aerostack2/aerostack2/issues/145>`_ from aerostack2/add_objects_ignition_enhancement
  Windmill + load objects with bridges
* removed windmill world
* Merge pull request `#163 <https://github.com/aerostack2/aerostack2/issues/163>`_ from aerostack2/crazyflie_swarm_demo
  Crazyflie swarm demo updates
* Demo cf swarm updates
* changes and bug fix
* deleted comments
* structural changes
* fix, check for objects in json file
* added working windmill model, added feature to load object with bridges into the world from config file
* Merge pull request `#124 <https://github.com/aerostack2/aerostack2/issues/124>`_ from aerostack2/123-ground-truth-bridge-segmentation-fault
  Ground truth bridge segmentation fault fix
* move publishers before ign subscriber
* Merge pull request `#114 <https://github.com/aerostack2/aerostack2/issues/114>`_ from aerostack2/devel
  [all] Reduce packages and update names
* Update namespace names
* Rename ignition_assets to as2_ign_gazebo_assets
* Contributors: Javier Melero, Javilinos, Miguel Fernandez-Cortizas, RPS98, pariaspe

0.2.2 (2022-12-22)
------------------

0.2.1 (2022-12-19)
------------------
