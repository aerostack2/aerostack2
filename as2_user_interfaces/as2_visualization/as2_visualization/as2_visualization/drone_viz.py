import inspect
import importlib
from typing import Callable
from rclpy.executors import MultiThreadedExecutor
from as2_visualization.rviz_adapter import RvizAdapter
import as2_visualization.rviz_adapter as ra
from visualization_msgs.msg import Marker, MarkerArray
from as2_visualization.viz_params import (
    CustomAdapterParams,
    PresetAdapterParams,
)


class AdapterBuilder:

    def build_preset(
        self, name: str, preset_type: str, in_topic: str, out_topic: str
    ) -> RvizAdapter:
        members = inspect.getmembers(
            ra, lambda x: inspect.isclass(x) and (issubclass(x, RvizAdapter))
        )
        for name, obj in members:
            if obj.__name__ == preset_type:
                return obj(name, in_topic, out_topic)

        raise ValueError(f"Preset type {preset_type} not found")

    def build_custom(
        self,
        name: str,
        adapter_func,
        in_topic: str,
        in_msg_type: str,
        out_topic: str,
        out_msg_type: str,
    ) -> RvizAdapter:
        """
        MSG type is to be specified with <package_name>/<msg_name>
        """
        in_msg = None
        out_msg = None
        # Exctact module name from in_msg_type
        package_name = in_msg_type.split("/")[0] + ".msg"
        module_name = in_msg_type.split("/")[1]
        # Import module
        mod_obj = importlib.import_module(package_name)
        # Get message type
        in_msg = getattr(mod_obj, module_name)

        # Since only Marker or MarkerArray can be output types, check it directly
        if out_msg_type == "visualization_msgs/Marker":
            out_msg = Marker
        elif out_msg_type == "visualization_msgs/MarkerArray":
            out_msg = MarkerArray
        else:
            raise ValueError(
                "Output message type must be either visualization_msgs/Marker or visualization_msgs/MarkerArray"
            )
        return RvizAdapter(name, adapter_func, in_topic, in_msg, out_topic, out_msg)


class DroneViz:

    def __init__(self, name: str):
        self.name = name
        self.custom_adapters: list[CustomAdapterParams] = []
        self.preset_adapters: list[PresetAdapterParams] = []

    def add_preset_adapter(
        self, name: str, preset_type: str, in_topic: str, out_topic: str
    ):
        self.preset_adapters.append(
            PresetAdapterParams(name, preset_type, in_topic, out_topic)
        )

    def add_custom_adapter(
        self,
        name: str,
        adapter: Callable,
        in_topic: str,
        in_msg_type: str,
        out_topic: str,
        out_msg_type: str,
    ):
        self.custom_adapters.append(
            CustomAdapterParams(
                name, adapter, in_topic, in_msg_type, out_topic, out_msg_type
            )
        )

    def generate_executor(self, builder: AdapterBuilder) -> MultiThreadedExecutor:
        executor = MultiThreadedExecutor()
        for preset in self.preset_adapters:
            executor.add_node(
                builder.build_preset(
                    preset.name, preset.preset_type, preset.in_topic, preset.out_topic
                )
            )
        for custom in self.custom_adapters:
            executor.add_node(
                builder.build_custom(
                    custom.name,
                    custom.adapter,
                    custom.in_topic,
                    custom.in_msg_type_name,
                    custom.out_topic,
                    custom.out_msg_type_name,
                )
            )
        return executor

    def to_yml(self):
        pass
