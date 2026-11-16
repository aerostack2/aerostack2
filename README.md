<p align="center">
  <a href="https://aerostack2.org"><img src=".github/assets/banner.png" alt="Aerostack2: build and fly aerial robots on ROS 2" width="100%"></a>
</p>

<p align="center">
  <a href="https://github.com/aerostack2/aerostack2/actions/workflows/ci.yaml"><img src="https://github.com/aerostack2/aerostack2/actions/workflows/ci.yaml/badge.svg?branch=main" alt="CI"></a>
  <a href="https://app.codecov.io/gh/aerostack2/aerostack2"><img src="https://codecov.io/gh/aerostack2/aerostack2/graph/badge.svg" alt="Coverage"></a>
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble"></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white" alt="ROS 2 Jazzy"></a>
  <a href="https://pixi.sh"><img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json" alt="Pixi"></a>
  <a href="https://arxiv.org/abs/2303.18237"><img src="https://img.shields.io/badge/arXiv-2303.18237-b31b1b.svg" alt="arXiv"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-BSD_3--Clause-blue.svg" alt="License"></a>
</p>

<p align="center">
  <a href="https://aerostack2.org/docs">Documentation</a> ·
  <a href="https://aerostack2.org/docs/getting-started/quickstart">Quickstart</a> ·
  <a href="https://arxiv.org/abs/2303.18237">Paper</a> ·
  <a href="https://hub.docker.com/u/aerostack2">Docker Hub</a> ·
  <a href="CONTRIBUTING.md">Contributing</a>
</p>

Aerostack2 is an open source framework for developing multi-robot aerial systems, built on ROS 2.

- **ROS 2 native.** Every component is a ROS 2 node with standard topics, services and actions.
- **Modular.** Controllers, state estimators, behaviors and planners are plugins you can swap without touching the rest of the stack.
- **Platform independent.** The same mission runs in simulation and on real aircraft, so moving from sim to real is a configuration change.
- **Behavior based.** Missions are built from reusable behaviors, driven from Python or behavior trees.
- **Swarm ready.** Multi-drone namespacing and coordination are part of the design, not an add-on.

https://user-images.githubusercontent.com/35956525/231999883-e491aa08-2835-47a9-9c68-5b2936e8594e.mp4

## Installation

| Method | ROS 2 | Systems |
| ------ | ----- | ------- |
| apt | Humble | Ubuntu 22.04 |
| Source | Humble, Jazzy | Ubuntu 22.04, 24.04 |
| Docker | Humble, Jazzy (built locally) | Any host with Docker |
| [Pixi](https://pixi.sh) (preview) | Humble, Jazzy | Linux x86-64 and arm64, macOS on Apple silicon |

Binary packages:

```bash
sudo apt install ros-humble-aerostack2
```

Pixi, with no system-wide ROS 2 install:

```bash
git clone https://github.com/aerostack2/aerostack2.git && cd aerostack2
pixi install && pixi shell                   # Humble
pixi install -e jazzy && pixi shell -e jazzy # Jazzy
```

Docker:

```bash
docker run -it --rm aerostack2/humble
```

The [installation guide](https://aerostack2.org/docs/getting-started/installation) covers building from source, hardware platforms and the Jazzy image, and the [quickstart](https://aerostack2.org/docs/getting-started/quickstart) walks through a first flight.

## Repository layout

| Package | Description |
| ------- | ----------- |
| `aerostack2` | Metapackage that pulls in the whole framework |
| `as2_core` | Core node, platform and sensor base classes, shared utilities |
| `as2_msgs` | Messages, services and actions |
| `as2_aerial_platforms` | Platform interfaces for Gazebo and the multirotor simulator |
| `as2_behaviors` | Motion, perception, planning, payload and swarm behaviors |
| `as2_behavior_tree` | Behavior tree nodes for mission execution |
| `as2_motion_controller` | Motion controller manager and controller plugins |
| `as2_motion_reference_handlers` | Helpers to send position, speed and trajectory references |
| `as2_state_estimator` | State estimator manager and estimation plugins |
| `as2_map_server` | Map server and mapping plugins |
| `as2_python_api` | Python API to write missions |
| `as2_hardware_drivers` | Camera and sensor drivers |
| `as2_simulation_assets` | Gazebo models and worlds |
| `as2_user_interfaces` | Keyboard teleoperation, alphanumeric viewer and RViz tools |
| `as2_utilities` | Geozones, swarm communication and external object to TF |
| `as2_cli` | Command line tools |

Real platforms such as Crazyflie, DJI and Tello live in their own repositories, listed in [`more_platforms.repos`](as2_aerial_platforms/more_platforms.repos).

## Contributing

Contributions are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) and the [Code of Conduct](CODE_OF_CONDUCT.md), then open an [issue](https://github.com/aerostack2/aerostack2/issues) or a pull request.

## Citation

If you use Aerostack2 in academic work, please cite:

```bibtex
@article{fernandez2023aerostack2,
  title   = {Aerostack2: A Software Framework for Developing Multi-robot Aerial Systems},
  author  = {Fernandez-Cortizas, Miguel and Molina, Martin and Arias-Perez, Pedro and Perez-Segui, Rafael and Perez-Saura, David and Campoy, Pascual},
  journal = {arXiv preprint arXiv:2303.18237},
  year    = {2023}
}
```

## License

Aerostack2 is released under the [BSD 3-Clause License](LICENSE).
