from dataclasses import dataclass
from typing import Callable


@dataclass
class TopicParams:
    depth: int
    durability: str
    filtersize: int
    history: str
    reliability: str
    value: str
    history: str
    namespaces: list[str]

    def __post_init__(self): ...

    # Check if str values for depth, durability... are valid


@dataclass
class AdapterParams:
    name: str
    in_topic: str
    out_topic: str
    sub_cfg: TopicParams
    pub_cfg: TopicParams

    def to_yml(self, drone_id: str):
        custom_yml = {}
        custom_yml["Enabled"] = True
        custom_yml["Name"] = "Marker"
        custom_yml["Namespaces"] = {}
        for ns in self.pub_cfg.namespaces:
            custom_yml["Namespaces"][ns] = True
        custom_yml["Topic"] = {}
        custom_yml["Topic"]["Depth"] = self.pub_cfg.depth
        custom_yml["Topic"]["Durability Policy"] = self.pub_cfg.durability
        custom_yml["Topic"]["Filter size"] = self.pub_cfg.filtersize
        custom_yml["Topic"]["History Policy"] = self.pub_cfg.history
        custom_yml["Topic"]["Reliability Policy"] = self.pub_cfg.reliability
        custom_yml["Topic"]["Value"] = "/viz/" + drone_id + "/" + self.name
        custom_yml["Value"] = True

        return custom_yml


@dataclass
class PresetAdapterParams(AdapterParams):
    preset_type: str

    def to_yml(self, drone_id):
        preset_yml = super().to_yml(drone_id)
        preset_yml["Class"] = "rviz_default_plugins/Marker"

        return preset_yml


@dataclass
class CustomAdapterParams(AdapterParams):
    adapter: Callable
    in_msg_type_name: str
    out_msg_type_name: str

    def to_yml(self, drone_id: str):
        custom_yml = super().to_yml(drone_id)
        if self.out_msg_type_name == "Marker":
            custom_yml["Class"] = "rviz_default_plugins/Marker"
        elif self.out_msg_type_name == "MarkerArray":
            custom_yml["Class"] = "rviz_default_plugins/MarkerArray"

        return custom_yml
