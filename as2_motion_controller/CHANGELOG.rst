^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_motion_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2024-08-22)
------------------

1.1.1 (2024-08-20)
------------------

1.1.0 (2024-08-08)
------------------
* [as2_motion_controller] Update motion_controller_default.yaml typo
* [as2_motion_controller] update controller launcher with launch utils and particular launchers for each plugin
* [as2_motion_controller] Update launch using as2_core launch params utils
* [as2_motion_controller] Fix library exports and add simple gtest
* [as2_motion_controller] Update launchers to Flake8
* [as2_motion_controller_manager] Publish speed limits when bypassing and control mode is Position
* [as2_motion_controller] Use ament_lint_auto
* Contributors: Miguel Fernandez-Cortizas, Rafael Perez-Segui, pariaspe

1.0.9 (2024-03-25)
------------------

1.0.8 (2024-02-12)
------------------

1.0.7 (2024-02-04)
------------------

1.0.6 (2023-12-12)
------------------
* Update PID controller dependence to v1.0
* Refactor CMakeLists.txt for creating a dynamic lib for each plugin
* Add trajectory reference to actuators commands
* Contributors: Miguel Fernandez-Cortizas, RPS98

1.0.5 (2023-11-08)
------------------

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
* Merge pull request `#275 <https://github.com/aerostack2/aerostack2/issues/275>`_ from aerostack2/274-as2_motion_controller-parameter-changes-are-not-applied-properly
  [as2_motion_controller] Change updateParams input from std::string to rclcpp::Parameter
* Change updateParams input from std::string to rclcpp::Parameter
* Convert motion controller params to config file
* Contributors: Javilinos, Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.1 (2023-04-25)
------------------
* Merge pull request `#227 <https://github.com/aerostack2/aerostack2/issues/227>`_ from aerostack2/226-as2_motion_controller-failed-to-load-differential_flatness_controller
  [as2_motion_controller] Fix failed to load differential_flatness_controller
* Namespace and Plugin Name must be equals
* Merge pull request `#223 <https://github.com/aerostack2/aerostack2/issues/223>`_ from aerostack2/200-unify-maintainer-in-packagexmls
  Maintainer unified to CVAR-UPM
* Maintainer unified to CVAR-UPM
* Merge pull request `#210 <https://github.com/aerostack2/aerostack2/issues/210>`_ from aerostack2/207-as2_controller_manager-fill-info-message
  [as2_controller_manager] Fill info message
* Merge pull request `#212 <https://github.com/aerostack2/aerostack2/issues/212>`_ from aerostack2/211-nodes-should-use-aerostack2-node-create-timer-method-not-create-wall-timer
  Nodes use Aerostack2 Node Create Timer method, not create wall timer
* Change create wall timer to as2 create timer
* Fix freq parameters names
* Contributors: Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.0 (2023-03-18)
------------------
* Merge branch 'main' into aruco_detect
* Merge branch 'main' into state_estimator_plugins
* Merge pull request `#158 <https://github.com/aerostack2/aerostack2/issues/158>`_ from aerostack2/motion_controller
  [as2_controller] Change as2_controller to as2_motion_controller
* Change as2_controller to as2_motion_controller
* Contributors: Miguel Fernandez-Cortizas, RPS98

0.2.2 (2022-12-22)
------------------

0.2.1 (2022-12-19)
------------------