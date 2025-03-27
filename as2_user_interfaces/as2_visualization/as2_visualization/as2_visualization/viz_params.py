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


@dataclass
class PresetAdapterParams:
    name: str
    preset_type: str
    in_topic: str
    out_topic: str
    sub_cfg: TopicParams
    pub_cfg: TopicParams


@dataclass
class CustomAdapterParams:
    name: str
    adapter: Callable
    in_topic: str
    in_topic: str
    in_msg_type_name: str
    out_topic: str
    out_msg_type_name: str
    sub_cfg: TopicParams
    pub_cfg: TopicParams
