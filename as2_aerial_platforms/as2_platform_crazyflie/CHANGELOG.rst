^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package as2_platform_crazyflie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2023-12-12)
------------------
* Fix wrong sensor name for multiranger deck
* Contributors: Rafael Pérez, pariaspe

1.0.5 (2023-11-08)
------------------
* Multi-ranger deck interface to laser_scan msg
* Contributors: pariaspe

1.0.4 (2023-08-23)
------------------

1.0.3 (2023-08-22)
------------------

1.0.2 (2023-08-17)
------------------
* Merge pull request `#239 <https://github.com/aerostack2/aerostack2/issues/239>`_ from aerostack2/cf-namespace
  [as2_crazyflie_platform] Remove namespace from launch
* Fix aideck config file in launcher
* Remove namespace from launch
  Namespace was overriding each crazyflie namespace which is intended to be set by swarm_config_file
* Merge pull request `#237 <https://github.com/aerostack2/aerostack2/issues/237>`_ from aerostack2/unify_platform_launchers
  Unify launchers
* Fix crazyflie params file read
* Unify platform launchers
* Contributors: Javilinos, Miguel Fernandez-Cortizas, RPS98, Rafael Pérez, pariaspe

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

0.2.2 (2022-12-20)
------------------
* Merge pull request `#88 <https://github.com/aerostack2/aerostack2/issues/88>`_ from aerostack2/main
  update with main
* Merge pull request `#85 <https://github.com/aerostack2/aerostack2/issues/85>`_ from aerostack2/as2_movement_behaviors_rename
  rename movement_behavior to as2_movement_behaviors
* reformat
* Contributors: Miguel, Miguel Fernandez-Cortizas, Rafael Pérez

0.2.1 (2022-12-19)
------------------
* Merge pull request `#84 <https://github.com/aerostack2/aerostack2/issues/84>`_ from aerostack2/clang-test-fixing
  [CI] Clang format fixed
* clang format
* Merge pull request `#32 <https://github.com/aerostack2/aerostack2/issues/32>`_ from aerostack2/main
  update_branch_before_merging
* Merge pull request `#28 <https://github.com/aerostack2/aerostack2/issues/28>`_ from aerostack2/cf_launcher
  Update package and node name
* Update package and node name
* Merge pull request `#15 <https://github.com/aerostack2/aerostack2/issues/15>`_ from aerostack2/pkg_dependencies
  Pkg dependencies
* Update pkg dependencies
* px4 extended and ci actions
* as2 added to platforms
* Contributors: Miguel Fernandez-Cortizas, RPS98, miferco97, pariaspe
