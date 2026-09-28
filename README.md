[![arXiv](https://img.shields.io/badge/arXiv-2303.18237-b31b1b.svg)](https://arxiv.org/abs/2303.18237) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) [![Build for Ubuntu 22.04 and ROS humble](https://github.com/aerostack2/aerostack2/actions/workflows/build-humble.yaml/badge.svg)](https://github.com/aerostack2/aerostack2/actions/workflows/build-humble.yaml) [![codecov](https://github.com/aerostack2/aerostack2/actions/workflows/codecov_test.yaml/badge.svg)](https://github.com/aerostack2/aerostack2/actions/workflows/codecov_test.yaml)

# Aerostack2 — CORESENSE Inspection Testbed

Documentation: [https://aerostack2.github.io](https://aerostack2.github.io)

---

Funded by the European Union through the Horizon Europe programme under Grant Agreement No. 101070254 (CoreSense).

---

<!-- Replace the line below with: ![CORESENSE logo](docs/images/coresense_logo.png) -->
> 📷 *CORESENSE / EU logo — drop your image at `docs/images/coresense_logo.png` and update this line.*

---

**Aerostack2** is an open-source ROS 2 framework for developing autonomous multi-aerial-robot systems. This repository is the **CORESENSE WP7 fork**, extending Aerostack2 with a complete multi-drone inspection testbed including collective awareness, distributed task allocation, and collision avoidance.

🧩 **Modular**, with a plugin architecture that allows components to be swapped without affecting the rest of the system.  
🚁 **Platform-agnostic**, enabling easy Sim2Real deployment across different aerial vehicles.  
🤝 **Swarm-oriented**, with native support for multi-robot coordination and distributed behaviours.  
🔍 **Inspection-ready**, featuring distributed auction-based task allocation and pairwise collision avoidance.  
⚡ **ROS 2 native**, developed and tested on ROS 2 Humble (Ubuntu 22.04).

---

<!-- Replace the line below with: ![Demo](docs/images/coresense_demo.gif) -->
> 🎬 *Demo video / GIF — drop your file at `docs/images/coresense_demo.gif` and update this line.*

---

## 📦 CORESENSE Packages

The following packages were added to Aerostack2 as part of the CORESENSE WP7 inspection testbed (deliverable D7.4):

| Package | Description |
|---------|-------------|
| [`as2_ca`](as2_ca/) | Collective Awareness gateway — inter-agent messaging and shared situational awareness. |
| [`as2_behaviors/as2_auction_behavior`](as2_behaviors/as2_auction_behavior/) | Distributed task allocation behaviour. Implements a greedy-sequential plugin and a CBBA-based plugin for multi-drone auction-based mission assignment. |
| [`as2_behaviors/as2_behaviors_collision_avoidance`](as2_behaviors/as2_behaviors_collision_avoidance/) | Pairwise path-lock collision avoidance plugin for safe multi-drone operations. |
| [`as2_state_interface`](as2_state_interface/) | State interface layer bridging platform state with the Knowledge Base (KB). |
| [`as2_core/kb_interface`](as2_core/) | C++ adapter exposing the KB API to Aerostack2 components. |
| [`as2_python_api/kb_monitor`](as2_python_api/) | Python KB monitor for high-level mission supervision. |

New message types added to [`as2_msgs`](as2_msgs/): `AuctionItem`, `Bid`, `StartAuction`, `CAPathLockRequest/Grant/Release`, `InterAgentMessage`, `LocalGenericMessage`, `PoseStampedWithID`.

---

## 🚀 Getting Started

Full documentation and installation instructions are available at:

- **Documentation:** [https://aerostack2.github.io](https://aerostack2.github.io)
- **Installation:** [https://aerostack2.github.io/_00_getting_started/index.html#ubuntu-debian](https://aerostack2.github.io/_00_getting_started/index.html#ubuntu-debian)
- **Docker images:** [https://hub.docker.com/u/aerostack2](https://hub.docker.com/u/aerostack2)

---

## 👥 Maintainers

| Name | Organization | Role |
|------|--------------|------|
| Guillermo González-Peña Lenza | Universidad Politécnica de Madrid | CORESENSE WP7 Lead |
| Miguel Fernandez-Cortizas | Universidad Politécnica de Madrid | Aerostack2 Lead |
| Pedro Arias-Perez | Universidad Politécnica de Madrid | Core Developer |
| Rafael Perez-Segui | Universidad Politécnica de Madrid | Core Developer |
| David Perez-Saura | Universidad Politécnica de Madrid | Core Developer |
| Martin Molina | Universidad Politécnica de Madrid | Advisor |
| Pascual Campoy | Universidad Politécnica de Madrid | PI |

---

## 📄 Citation

If you use this work in an academic context, please cite:

```bibtex
@misc{fernandez2023aerostack2,
  title={Aerostack2: A software framework for developing multi-robot aerial systems},
  author={M. Fernandez-Cortizas and M. Molina and P. Arias-Perez and R. Perez-Segui
          and D. Perez-Saura and P. Campoy},
  year={2023},
  eprint={2303.18237},
  archivePrefix={arXiv}
}
```

---

<sub>This project has received funding from the European Union's Horizon Europe research and innovation programme under grant agreement No 101070254 (CoreSense). Views and opinions expressed are however those of the authors only and do not necessarily reflect those of the European Union or the European Research Executive Agency. Neither the European Union nor the granting authority can be held responsible for them.</sub>
