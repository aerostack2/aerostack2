^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_state_estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2023-12-12)
------------------

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
* Merge pull request `#279 <https://github.com/aerostack2/aerostack2/issues/279>`_ from aerostack2/230-gps_support_for_ground_truth_plugin
  Add gps support for ground truth plugin
* Change origin param
* parameters added, compiling, add origin manually working
* gps support for ground truth plugin
* Contributors: Javilinos, Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.1 (2023-04-25)
------------------
* Merge pull request `#223 <https://github.com/aerostack2/aerostack2/issues/223>`_ from aerostack2/200-unify-maintainer-in-packagexmls
  Maintainer unified to CVAR-UPM
* Maintainer unified to CVAR-UPM
* Merge pull request `#212 <https://github.com/aerostack2/aerostack2/issues/212>`_ from aerostack2/211-nodes-should-use-aerostack2-node-create-timer-method-not-create-wall-timer
  Nodes use Aerostack2 Node Create Timer method, not create wall timer
* Change create wall timer to as2 create timer
* Contributors: Miguel Fernandez-Cortizas, RPS98, pariaspe

1.0.0 (2023-03-18)
------------------
* Merge branch 'main' into trajectory_generation
* Merge pull request `#159 <https://github.com/aerostack2/aerostack2/issues/159>`_ from aerostack2/state_estimator_plugins
  [as2_state_estimator] Change plugins names
* Clang format
* Chnage plugins names
* Merge pull request `#137 <https://github.com/aerostack2/aerostack2/issues/137>`_ from aerostack2/main
  Sync branch with main
* Merge pull request `#134 <https://github.com/aerostack2/aerostack2/issues/134>`_ from aerostack2/71-all-include-gps-translator-functions
  GPS behavior support
* fixing gps origin earth to map tf
* set and get origin srvs created on external_odom plugin
* Merge pull request `#114 <https://github.com/aerostack2/aerostack2/issues/114>`_ from aerostack2/devel
  [all] Reduce packages and update names
* Clang format fix
* State estimator launch config file bug fix
* Update namespace names
* Merge pull request `#119 <https://github.com/aerostack2/aerostack2/issues/119>`_ from aerostack2/as2_state_estimator_unify
  [as2_state_estimator] As2 state estimator unify
* Add license header
* Unify state estimators in one package
* Contributors: Miguel, Miguel Fernandez-Cortizas, RPS98, Rafael Pérez, pariaspe

0.2.2 (2022-12-22)
------------------
* license and versions updated to v0.2.0 and BSD-3
* CHANGELOG_GENERATED
* Contributors: Miguel

0.2.1 (2022-12-19)
------------------
* Merge pull request `#79 <https://github.com/aerostack2/aerostack2/issues/79>`_ from aerostack2/mocap_test
  [as2_state_estimator_plugin_mocap] mocap test added to gtest and alpha reduced to 0.1
* params added to test
* mocap test added to gtest and algha reduced to 0.1
* Merge pull request `#68 <https://github.com/aerostack2/aerostack2/issues/68>`_ from aerostack2/state_estimator_odom_config_file
  Add config file to state estimator external odom plugin with odom topic
* Add config file to state estimator external odom plugin with odom topic
* Merge pull request `#21 <https://github.com/aerostack2/aerostack2/issues/21>`_ from aerostack2/launcher_update
  Update launchers
* Update launchers
* Merge pull request `#12 <https://github.com/aerostack2/aerostack2/issues/12>`_ from aerostack2/as2_state_estimator_launch
  [as2_state_estimator] Update parameters config file
* Update parameters config file
* bug fixed
* bug fixed in ground truth plugin
* bug fixed in external odom plugin
* Remove basic_state_estimator
* Names fixed
* as2_state_estimator pluginized
* Merge pull request `#1 <https://github.com/aerostack2/aerostack2/issues/1>`_ from aerostack2/state_estimation_pluginized
  State estimation pluginized
* plugin base dependence added to package.xml
* state estimation folder structured
* basic_state_estimator external_odom uploaded
* basic_state_estimator ground_truth_plugin uploaded
* basic_state_estimator mocap_plugin uploaded
* basic_state_estimator removed
* as2_basic_state_estimator_plugin_base
* First commit
* Contributors: Miguel, Miguel Fernandez-Cortizas, RPS98, miferco97, pariaspe
