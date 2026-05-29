"""drone_bringup.launch.py — 드론 부팅 시 필수 노드 자동 실행."""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'jongmin lee'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# ---------------------------------------------------------------------------
# 설정 파일 로더
# ---------------------------------------------------------------------------

def _load_config(config_path: str) -> dict:
    """drone_bringup.yaml 을 읽어 dict 로 반환한다."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f) or {}


def _get(cfg: dict, *keys, default=None):
    """중첩 dict 에서 키를 순서대로 탐색한다."""
    for key in keys:
        if not isinstance(cfg, dict):
            return default
        cfg = cfg.get(key, default)
    return cfg


# ---------------------------------------------------------------------------
# 각 Phase 런치 빌더
# ---------------------------------------------------------------------------

def _make_platform(ns: str, use_sim_time: str, cfg: dict) -> IncludeLaunchDescription:
    """Phase 1-1: platform 노드."""
    pkg = _get(cfg, 'platform', 'pkg', default='as2_platform_mavlink')
    pkg_dir = get_package_share_directory(pkg)
    launch_file = os.path.join(pkg_dir, 'launch', f'{pkg}_launch.py')

    args = {
        'namespace': ns,
        'use_sim_time': use_sim_time,
    }

    # 사용자 지정 platform_config_file 이 있으면 전달
    custom_cfg = _get(cfg, 'platform', 'platform_config_file')
    if custom_cfg:
        args['platform_config_file'] = custom_cfg

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=args.items(),
    )


def _make_state_estimator(ns: str, use_sim_time: str, cfg: dict) -> IncludeLaunchDescription:
    """Phase 1-2: state_estimator 노드."""
    pkg_dir = get_package_share_directory('as2_state_estimator')
    launch_file = os.path.join(pkg_dir, 'launch', 'state_estimator_launch.py')

    plugin = _get(cfg, 'state_estimator', 'plugin_name', default='ground_truth')
    args = {
        'namespace': ns,
        'use_sim_time': use_sim_time,
        'plugin_name': plugin,
    }

    custom_cfg = _get(cfg, 'state_estimator', 'config_file')
    if custom_cfg:
        args['config_file'] = custom_cfg

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=args.items(),
    )


def _make_controller(ns: str, use_sim_time: str, cfg: dict) -> IncludeLaunchDescription:
    """Phase 1-3: controller_manager 노드."""
    pkg_dir = get_package_share_directory('as2_motion_controller')
    launch_file = os.path.join(pkg_dir, 'launch', 'controller_launch.py')

    plugin = _get(cfg, 'controller', 'plugin_name', default='pid_speed_controller')
    args = {
        'namespace': ns,
        'use_sim_time': use_sim_time,
        'plugin_name': plugin,
    }

    custom_cfg = _get(cfg, 'controller', 'config_file')
    if custom_cfg:
        args['config_file'] = custom_cfg

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=args.items(),
    )


def _make_behaviors(ns: str, use_sim_time: str, cfg: dict) -> IncludeLaunchDescription:
    """Phase 2: composable motion behaviors (Takeoff/Land/GoTo/FollowPath)."""
    pkg_dir = get_package_share_directory('as2_behaviors_motion')
    launch_file = os.path.join(
        pkg_dir, 'launch', 'composable_motion_behaviors.launch.py')

    b_cfg = _get(cfg, 'behaviors', default={})
    args = {
        'namespace': ns,
        'use_sim_time': use_sim_time,
    }

    # 플러그인 명시 시에만 전달 (빈 문자열은 각 behavior 기본값 사용)
    for key in ('takeoff_plugin_name', 'land_plugin_name',
                'go_to_plugin_name', 'follow_path_plugin_name'):
        val = b_cfg.get(key, '')
        if val:
            args[key] = val

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=args.items(),
    )


# ---------------------------------------------------------------------------
# 메인 런치 생성 (OpaqueFunction — 런타임에 config 파일 경로 확정)
# ---------------------------------------------------------------------------

def _generate_nodes(context: LaunchContext):
    config_path = LaunchConfiguration('config').perform(context)
    cfg = _load_config(config_path)

    ns = LaunchConfiguration('namespace').perform(context) or _get(
        cfg, 'namespace', default='drone0')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) or str(
        _get(cfg, 'use_sim_time', default=False)).lower()

    delays = _get(cfg, 'startup_delay', default={})
    d_se = float(delays.get('state_estimator', 3.0))
    d_ctrl = float(delays.get('controller', 5.0))
    d_bhv = float(delays.get('behaviors', 7.0))

    actions = [
        # Phase 1-1: platform — 즉시 시작
        LogInfo(msg=f'[Bringup] Phase 1-1: platform (namespace={ns})'),
        _make_platform(ns, use_sim_time, cfg),

        # Phase 1-2: state_estimator — d_se 초 후
        TimerAction(
            period=d_se,
            actions=[
                LogInfo(msg=f'[Bringup] Phase 1-2: state_estimator '
                            f'(plugin={_get(cfg, "state_estimator", "plugin_name", default="ground_truth")})'),
                _make_state_estimator(ns, use_sim_time, cfg),
            ],
        ),

        # Phase 1-3: controller_manager — d_ctrl 초 후
        TimerAction(
            period=d_ctrl,
            actions=[
                LogInfo(msg=f'[Bringup] Phase 1-3: controller_manager '
                            f'(plugin={_get(cfg, "controller", "plugin_name", default="pid_speed_controller")})'),
                _make_controller(ns, use_sim_time, cfg),
            ],
        ),

        # Phase 2: behaviors — d_bhv 초 후
        TimerAction(
            period=d_bhv,
            actions=[
                LogInfo(msg='[Bringup] Phase 2: motion behaviors (Takeoff/Land/GoTo/FollowPath)'),
                _make_behaviors(ns, use_sim_time, cfg),
            ],
        ),
    ]

    return actions


def generate_launch_description():
    """드론 부팅 런치 진입점."""
    default_config = os.path.join(
        get_package_share_directory('as2_drone_bringup'),
        'config', 'drone_bringup.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='drone_bringup.yaml 경로. 미지정 시 패키지 기본값 사용.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='드론 네임스페이스. 미지정 시 config 파일 값 사용.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='',
            description='시뮬레이션 시간 사용 여부. 미지정 시 config 파일 값 사용.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS2 로그 레벨 (debug, info, warn, error).',
        ),
        OpaqueFunction(function=_generate_nodes),
    ])
