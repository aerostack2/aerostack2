from dataclasses import dataclass
from typing import Callable

@dataclass
class PresetAdapterParams:
    name : str
    preset_type : str
    in_topic : str
    out_topic : str


@dataclass
class CustomAdapterParams:
    name : str
    adapter : Callable
    in_topic : str
    in_topic : str
    in_msg_type_name : str
    out_topic : str
    out_msg_type_name : str

HISTORY_POLICIES = ['keep_last, keep_all']
RELIABILITY_POLICIES = ['best_effort, reliable']
DURABILITY_POLICIES = ['volatile, transient_local, transient, persistent']

@dataclass
class TopicParams:
    depth : int
    durability : str
    filtersize : int
    history : str
    reliability : str
    value : str
    history : str
    namespaces : list[str]

    def __post_init__(self):
        if self.history not in HISTORY_POLICIES:
            raise ValueError(f"{self.history} history policy not available {HISTORY_POLICIES}")
        if self.reliability not in RELIABILITY_POLICIES:
            raise ValueError(f"{self.reliability} reliability policy not available {RELIABILITY_POLICIES}")
        if self.durability not in DURABILITY_POLICIES:
            raise ValueError(f"{self.durability} durability policy not available {DURABILITY_POLICIES}")