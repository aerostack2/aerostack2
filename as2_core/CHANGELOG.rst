^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2024-03-25)
------------------
* get quaternion stamped included in tf utils
* quaternion convert wrapped in try and catch
* Add quaternion support for TF convert method
* Contributors: Javilinos, Rafael Perez-Segui, Rafael Pérez, pariaspe

1.0.8 (2024-02-12)
------------------
* [as2_core] added pybind11 dep
* Contributors: Rafael Pérez, pariaspe

1.0.7 (2024-02-04)
------------------
* Bug fixed, getPoseStamped function differs from timeout 0 and not 0
* [as2_core] Python as2_names bindings
* [refactor] format tests according with ament_lint_common() packages
* [refactor] check rosdistro if galactic use tf2_\_.h in other cases use tf2_\_.hpp
* Contributors: Javilinos, Miguel Fernandez-Cortizas, pariaspe

1.0.6 (2023-12-12)
------------------
* Takeoff and GoTo behavior renaming
* Add node options to aerial platform
* Add trajectory reference to actuators commands
* Contributors: Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.5 (2023-11-08)
------------------
* Deal with low latency frames that are not earth
* Contributors: Javier Melero, Javilinos, Miguel Fernandez-Cortizas

1.0.4 (2023-08-23)
------------------

1.0.3 (2023-08-22)
------------------
* Merge pull request `#317 <https://github.com/aerostack2/aerostack2/issues/317>`_ from aerostack2/316-tf2_geometry_msgs-dep
  Only use tf2_geometry_msgs library when needed
* revert change in include library
* only use library when needed
* Contributors: Miguel Fernandez-Cortizas, pariaspe

1.0.2 (2023-08-17)
------------------
* Merge pull request `#304 <https://github.com/aerostack2/aerostack2/issues/304>`_ from aerostack2/302-parametrize_tf_threshold_time
  302-Parametrize tf timeout in go to and follow reference behavior
* parametrize tf timeout in go to and follow reference behavior
* Contributors: Javier Melero, Javilinos, RPS98, pariaspe

1.0.1 (2023-04-25)
------------------
* Merge pull request `#225 <https://github.com/aerostack2/aerostack2/issues/225>`_ from aerostack2/224-as2_core-sensor-class-do-not-support-sensor_measurements-topics-in-construction
  [as2_core] Fix sensor class do not support sensor_measurements/ topics in construction
* Remove hardcode name of sensor_measurements
* Fix sensor topic empty error
* Merge pull request `#223 <https://github.com/aerostack2/aerostack2/issues/223>`_ from aerostack2/200-unify-maintainer-in-packagexmls
  Maintainer unified to CVAR-UPM
* Maintainer unified to CVAR-UPM
* Merge pull request `#212 <https://github.com/aerostack2/aerostack2/issues/212>`_ from aerostack2/211-nodes-should-use-aerostack2-node-create-timer-method-not-create-wall-timer
  Nodes use Aerostack2 Node Create Timer method, not create wall timer
* Change create wall timer to as2 create timer
* Merge pull request `#198 <https://github.com/aerostack2/aerostack2/issues/198>`_ from aerostack2/197-as2_core-change-behaviour-to-behavior
  197 as2 core change behaviour to behavior
* Change behaviour to behavior
* Contributors: Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.0 (2023-03-18)
------------------

0.2.2 (2022-12-20)
------------------

0.2.1 (2022-12-19)
------------------
* Merge pull request `#30 <https://github.com/aerostack2/aerostack2/issues/30>`_ from aerostack2/add_test_to_ci
  test added to ci
* new gtest in as2_core
* Merge pull request `#18 <https://github.com/aerostack2/aerostack2/issues/18>`_ from aerostack2/fix_tests
  Fix tests
* as2 core formatted
* trajectory generator added to as2_core
* as2_core update
* reformat as2_core
* First commit
* Contributors: David Perez-Saura, Miguel, Rafael Pérez, miferco97
