from typing import Sequence

import numpy as np
import numpy.typing as npt

# What each entry of Filter.state is, in order
STATE_NAMES: tuple[str, ...]

class SourceFrame:
    EARTH: SourceFrame
    MAP: SourceFrame
    ODOM: SourceFrame
    BASE: SourceFrame

class Transform:
    # A position, and an orientation as a quaternion (x, y, z, w), the order ROS messages use
    def __init__(self, position: Sequence[float] = ...,
                 orientation: Sequence[float] = ...) -> None: ...
    @staticmethod
    def from_rpy(position: Sequence[float], roll: float, pitch: float,
                 yaw: float) -> Transform: ...
    @property
    def position(self) -> list[float]: ...
    # (x, y, z, w)
    @property
    def orientation(self) -> list[float]: ...
    # Roll, pitch and yaw in radians, each in [-pi, pi]
    @property
    def rpy(self) -> list[float]: ...
    # The 4x4 homogeneous matrix
    def matrix(self) -> npt.NDArray[np.float64]: ...
    def inverse(self) -> Transform: ...
    def __mul__(self, other: Transform) -> Transform: ...

class SourceConfig:
    # Used in log lines
    name: str
    # A correction is absorbed by odom->base (True) or moves map->odom (False)
    is_odometry: bool
    # Maximum rate at which measurements are fused, in Hz. 0 fuses every one
    update_rate_hz: float
    reject_repeated_positions: bool
    # Positions closer than this, in metres, count as the same one
    repeated_position_threshold: float
    # Width in standard deviations. 0 disables it
    innovation_gate: float
    # Seconds of uninterrupted rejection after which the next measurement is fused anyway
    innovation_gate_timeout: float
    # Scale the variances a measurement carries (True) or replace them (False) with the values
    # below; see generate_covariance_from_config and get_covariance_with_config
    use_message_covariance: bool
    position_values: list[float]
    orientation_values: list[float]
    linear_values: list[float]

    def __init__(self) -> None: ...

class Config:
    # Unset fields keep the defaults of the plugin's config/plugin_default.yaml
    initial_position_covariance: float
    initial_velocity_covariance: float
    initial_orientation_covariance: float
    initial_bias_acc_covariance: float
    initial_bias_gyro_covariance: float
    # Positive: the model subtracts it from the rotated specific force
    gravity: float
    accelerometer_noise_density: float
    gyroscope_noise_density: float
    accelerometer_random_walk: float
    gyroscope_random_walk: float
    max_update_latency_ms: float
    unobserved_variance: float
    # Weight of the newest raw map->odom in the published one, per tick. 1 disables smoothing
    map_odom_alpha: float
    # Until the drone first goes offboard, every tick corrects the state towards this pose, in the
    # map frame, with this variance on every component
    preflight_pose: Transform
    preflight_variance: float
    verbose: bool
    debug_verbose: bool

    def __init__(self) -> None: ...

class ImuSample:
    # Nanoseconds
    stamp: int
    # In the vehicle's frame
    linear_acceleration: list[float]
    angular_velocity: list[float]

    def __init__(self, stamp: int = ..., linear_acceleration: Sequence[float] = ...,
                 angular_velocity: Sequence[float] = ...) -> None: ...

class PoseSample:
    # Nanoseconds
    stamp: int
    frame: SourceFrame
    pose: Transform
    # 6x6 row-major. A non-positive variance marks a component the source does not measure
    covariance: list[float]

    def __init__(self, stamp: int = ..., frame: SourceFrame = ..., pose: Transform = ...,
                 covariance: Sequence[float] = ...) -> None: ...

class TwistSample:
    # Nanoseconds
    stamp: int
    frame: SourceFrame
    linear: list[float]
    # 6x6 row-major, as PoseSample's
    covariance: list[float]

    def __init__(self, stamp: int = ..., frame: SourceFrame = ...,
                 linear: Sequence[float] = ..., covariance: Sequence[float] = ...) -> None: ...

class TwistInBase:
    @property
    def linear(self) -> list[float]: ...
    @property
    def angular(self) -> list[float]: ...

class Outputs:
    # The tree to publish is earth_to_map, published_map_to_odom and odom_to_base. map_to_odom and
    # internal_twist_in_base are the same before the output smoothing
    earth_to_map: Transform
    map_to_odom: Transform
    published_map_to_odom: Transform
    odom_to_base: Transform
    twist_in_base: TwistInBase
    internal_twist_in_base: TwistInBase
    # The vehicle's pose in the earth frame, as the published tree composes it
    @property
    def earth_to_base(self) -> Transform: ...

class Filter:
    # Reads no clock: every time is an argument, in nanoseconds. It logs to the Python logger
    # "simple_ekf_core", configured as any other: logging.basicConfig(level=logging.INFO) shows
    # the INFO lines as well
    def __init__(self, config: Config = ...) -> None: ...
    def add_source(self, config: SourceConfig) -> int: ...
    # The configuration in use, after validation
    @property
    def config(self) -> Config: ...
    # A copy of the current outputs
    @property
    def outputs(self) -> Outputs: ...
    # Ordered as STATE_NAMES
    @property
    def state(self) -> npt.NDArray[np.float64]: ...
    # 15x15
    @property
    def state_covariance(self) -> npt.NDArray[np.float64]: ...
    def is_earth_to_map_set(self) -> bool: ...
    def mark_earth_to_map_set(self) -> None: ...
    def set_earth_to_map(self, earth_to_map: Transform) -> None: ...
    # Only a pose in the EARTH or the MAP frame can set it
    def set_earth_to_map_from_first_pose(self, pose: Transform, frame: SourceFrame) -> bool: ...
    def set_offboard(self, offboard: bool) -> None: ...
    def should_throttle_update(self, source: int, stamp: int) -> bool: ...
    def is_repeated_position(self, source: int, position: Sequence[float], now: int) -> bool: ...
    def on_imu(self, imu: ImuSample) -> None: ...
    def on_pose(self, source: int, pose: PoseSample, now: int) -> None: ...
    def on_twist(self, source: int, twist: TwistSample, now: int) -> None: ...
    # True if the pre-flight correction was applied
    def on_tick(self, now: int) -> bool: ...

def generate_covariance_from_config(config: SourceConfig) -> list[float]: ...
def get_covariance_with_config(covariance: Sequence[float],
                               config: SourceConfig) -> list[float]: ...
def get_linear_covariance_with_config(covariance: Sequence[float],
                                      config: SourceConfig) -> list[float]: ...
