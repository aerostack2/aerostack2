# SwarmLink — Collective Awareness Middleware

> CoreSense | **Resilience Essential** | WP7 / TB2 (UPM)

## Overview

`swarmlink` is the Collective Awareness (CA) middleware layer for Aerostack2. It implements the *Communication Interface* and *Organisation Handling* subsystems of the CoreSense Collective Awareness cognitive module, enabling peer-to-peer modelet exchange between drone instances without a centralised broker.

The package exposes two artifacts:

- **CA Gateway Node** (`swarmlink::SwarmLink`) — one ROS 2 node per drone that owns all inter-agent traffic: peer discovery, inbound demultiplexing, and outbound forwarding.
- **CA Gateway Client** (`swarmlink::SwarmLinkClient`) — a lightweight library embedded in behaviors that provides a fleet-size-agnostic `register / forward` interface without knowledge of peer addresses or transport details.

## Architecture

### Message flow

```
Behavior          CA Client          CA Node          Remote Agent
   │                  │                  │                  │
   │─register(T, cb)──▶                  │                  │
   │                  │─subscribe(T)────▶│                  │
   │                  │                  │                  │
   │                  │                  │◀─[msg of type T]─│
   │                  │◀─dispatch(msg)───│                  │
   │◀─cb(msg)─────────│                  │                  │
   │                  │                  │                  │
   │─forward(T, msg)──▶                  │                  │
   │                  │─route(msg)──────▶│                  │
   │                  │                  │──[network]──────▶│
```

1. **register** — A behavior calls `register_module<T>()` with a type tag and a callback. The gateway creates a typed publisher for that tag; the client subscribes to it.
2. **receive** — Incoming peer messages arrive on `<ns>/gateway_in`, are demultiplexed by the gateway, and dispatched to the registered callback with the sender's agent ID.
3. **forward** — A behavior calls `forward_IA_msg()` with a typed message and a list of receiver namespaces. The gateway routes it to each peer's incoming topic.

### Peer discovery and failure detection

The gateway runs a 1 Hz timer that scans the ROS 2 topic graph for topics ending in `/gateway_in`. Newly discovered topics trigger publisher creation; publishers whose topic has no subscribers are pruned automatically — giving implicit failure detection without an explicit heartbeat.

### Comparison with raw ROS 2 DDS

| Dimension | CA Gateway | Raw ROS 2 DDS |
|---|---|---|
| Discovery overhead | Registry resolved once at send time | Graph queried per message type at runtime |
| Naming coupling | Typed registry; no topic names in behavior code | Per-message-type topic names hard-coded |
| Transport flexibility | Backend swappable (DDS, Zenoh, MQTT, …) | Tied to DDS |
| Fleet reconfiguration | Handled automatically by the discovery timer | Explicit publisher/subscriber rewiring required |

## Usage

### Embedding the client in a behavior

```cpp
#include "swarmlink/swarmlink_client.hpp"
#include "my_pkg/msg/my_modelet.hpp"

class MyBehavior : public as2_behavior::BehaviorServer<...>
{
  swarmlink::SwarmLinkClient ca_client_;

  MyBehavior() : ..., ca_client_(this)
  {
    ca_client_.register_module<my_pkg::msg::MyModelet>(
      "my_modelet",       // type tag — must match the sender's tag
      "my_behavior",      // module name (for logging)
      [this](const my_pkg::msg::MyModelet & msg, const std::string & sender) {
        handle_modelet(msg, sender);
      });
  }

  void send_to_peers()
  {
    my_pkg::msg::MyModelet out_msg;
    // ... fill out_msg ...
    auto peers = ca_client_.get_known_peers();
    ca_client_.forward_IA_msg(out_msg, "my_modelet", peers);
  }
};
```

### Launch

Include the gateway node in the drone's launch file:

```python
from launch_ros.actions import Node

swarmlink_node = Node(
    package='swarmlink',
    executable='swarmlink_node',
    namespace=drone_id,
    output='screen',
)
```

Or use the provided launch file:

```bash
ros2 launch swarmlink swarmlink_launch.py drone_id:=drone0
```

## Dependencies

- `rclcpp`, `as2_msgs`, `as2_core`

## License

BSD-3-Clause — Copyright 2026 Universidad Politécnica de Madrid
