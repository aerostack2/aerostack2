# as2_flight_governor

A declarative safety layer for Aerostack2. Every motion reference a behavior
sends is checked against a list of policies written in YAML, and is then
**allowed** unchanged, **clamped** to comply, or **vetoed** and replaced by a
hold, before it reaches the motion controller.

```
behaviors ──motion_reference/*──▶ flight_governor ──flight_governor/motion_reference/*──▶ controller ──▶ platform
                                        │
                                        ├── flight_governor/verdict   (one message per reference)
                                        └── alert_event               (when a veto starts)
```

Policies are data: one reviewable file says what the vehicle may never do,
independently of which behavior, mission or planner is flying it.

## Quick start

1. Write your policies (see [`config/policies.yaml`](config/policies.yaml)).
2. Launch the governor:

   ```bash
   ros2 launch as2_flight_governor as2_flight_governor_launch.py \
     namespace:=drone0 policies_file:=/path/to/policies.yaml
   ```

3. Launch the motion controller reading through the governor:

   ```bash
   ros2 launch as2_motion_controller controller_launch.py \
     namespace:=drone0 use_flight_governor:=true
   ```

Without `use_flight_governor:=true` the controller keeps reading
`motion_reference/*` directly, and the governor logs a warning every two
seconds that nothing is being governed.

## Policies

Policies are applied in file order. Each one sees the output of the previous
one, so clamps compose. The first veto stops evaluation. Coordinates are in
the policy frame (parameter `policy_frame`, by default the drone's odom frame),
in metres and seconds.

| type | fields | effect |
|------|--------|--------|
| `geofence_box` | `min`, `max` (`[x, y, z]`), `lookahead` (s, default 1.0), `action` (`clamp` / `veto`, default `clamp`) | Position targets are moved inside the box. Velocity components are limited so that the box cannot be crossed within `lookahead`. Outside the box, motion back towards it is still allowed. |
| `max_speed` | `horizontal`, `vertical` (m/s; omitted or negative = no limit) | Limits horizontal speed as a vector (direction kept) and vertical speed. |
| `keepout_sphere` | `center`, `radius`, `margin` (default 0), `lookahead` (default 1.0), `action` | Position targets inside `radius + margin` are projected onto the surface. The velocity component towards the centre is limited; tangential motion is kept, so the vehicle can slide around the obstacle. |
| `speed_zone` | `center`, `radius`, `max_speed` | Lower speed limit while the vehicle is inside the zone. |

The loader rejects unknown types, unknown or misspelt fields, missing fields,
duplicate names and invalid geometry. A bad file stops the node from starting:
it never runs with a policy silently missing.

## How references are interpreted

What a reference means depends on the controller's input control mode, which
the governor reads from `controller/info`:

| Input control mode | `pose` | `twist` | `trajectory` | Veto output |
|---|---|---|---|---|
| `POSITION` | position target: governed | per-axis speed limits: capped by `max_speed` and `speed_zone` | forwarded, `governed: false` | position hold, `hold_speed` limits |
| `SPEED` | yaw only: forwarded | velocity: governed | forwarded, `governed: false` | zero velocity |
| `TRAJECTORY` | yaw only: forwarded | forwarded, `governed: false` | each setpoint governed | single hold setpoint |
| `HOVER` | forwarded | forwarded | forwarded, `governed: false` | — |
| other modes | forwarded, `governed: false` | forwarded, `governed: false` | forwarded, `governed: false` | — |

Until the first `controller/info` message arrives, references are held back:
the controller ignores them in that state anyway.

`thrust` references are always forwarded ungoverned.

### Speed limits in POSITION mode

With the PID speed controller, a speed limit of `0` (or below) means "no
limit", and a single `0` disables the limits on **every** axis. The governor
therefore treats `0` as unlimited when checking, and never writes a `0` back
when it has limited any axis: an unlimited axis is written as 1000 m/s, and a
limited axis is at least 0.01 m/s.

## Vetoes

A reference is vetoed when a policy with `action: veto` is broken, or when the
governor cannot check it safely:

- no vehicle state yet, or state older than `state_timeout`;
- non-finite values in the reference or the state;
- the reference or state cannot be transformed into the policy frame.

With `latch_veto: true` (the default), the governor keeps sending the hold
until `flight_governor/reset` (`std_srvs/Trigger`) is called, even if later
references are fine. The hold position is fixed at the moment of the veto.

If a position hold is needed before any vehicle position has been received,
the governor sends **nothing** rather than invent a hold point, and reports
`sent: false`.

## Interfaces

Subscribed (relative to the drone namespace):
`motion_reference/{pose,twist,trajectory,thrust}`, `self_localization/pose`,
`controller/info`.

Published:
`flight_governor/motion_reference/{pose,twist,trajectory,thrust}`,
`flight_governor/verdict` (`as2_msgs/GovernorVerdict`), `alert_event`.

Service: `flight_governor/reset` (`std_srvs/Trigger`).

## Parameters

| name | default | description |
|------|---------|-------------|
| `policies_file` | (required) | YAML file with the policies |
| `policy_frame` | odom frame | frame the policies are written in |
| `state_timeout` | `0.5` | maximum age of the vehicle state (s) |
| `latch_veto` | `true` | keep vetoing until reset |
| `hold_speed` | `0.5` | per-axis speed limit (m/s) sent with a position hold |
| `veto_alert` | `0` | `as2_msgs/AlertEvent` value published when a veto starts (`0` info, `-2` emergency hover, `-3` emergency land, `1` force hover, `2` force land) |

## Limitations

- Velocity policies are based on the current position and a lookahead. They
  bound the reference, not the vehicle: controller tracking error and
  overshoot are not modelled.
- In `TRAJECTORY` mode, clamping a setpoint drops its feed-forward
  acceleration, because it no longer matches the clamped velocity.
- Low-level modes (attitude, body rates, acro) and `SPEED_IN_A_PLANE` are
  forwarded ungoverned and flagged with `governed: false`.
- This is a software safety layer. It does not replace the autopilot's own
  geofence and failsafes.

## Design

The core (`governor_core`, `policies`, `policy_loader`) has no ROS dependency
and never reads a clock: the node passes the current time in, so the same
inputs always give the same decision and the core is tested without ROS.
