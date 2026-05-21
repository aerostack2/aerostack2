#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
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
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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
r"""
Manual orchestration script for the polynomial trajectory generator behavior.

Sends a configurable zigzag path goal and exercises the full behavior
lifecycle (start_on_paused, modify, pause/resume, stop) by reacting to the
action feedback. Not an automated CI test — invoked by hand against a
running behavior:

    ros2 launch as2_behaviors_trajectory_generation \
        generate_polynomial_trajectory_behavior_launch.py namespace:=drone0

    python3 behavior_test.py

Tweak the configuration block at the top of this file to choose the path
shape, the YawMode, and which lifecycle triggers to exercise.

Requires the aerostack2 environment to be sourced (uses
``as2_python_api.behavior_actions.behavior_handler.BehaviorHandler``).
"""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import sys
import threading
import time
from typing import Optional

from as2_msgs.action import GeneratePolynomialTrajectory
from as2_msgs.msg import BehaviorStatus, PoseStampedWithID, YawMode
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


# ───────────────────────────── Configuration ────────────────────────────────
# Connection
NAMESPACE = 'drone0'
FRAME_ID = 'earth'

# Goal: motion params.
# YAW_MODE accepts: KEEP_YAW | PATH_FACING | FIXED_YAW | YAW_FROM_TOPIC |
# YAW_FROM_ORIENTATION | YAW_TO_FRAME | FACE_REFERENCE.
MAX_SPEED = 1.0                  # m/s
YAW_MODE = YawMode.KEEP_YAW
YAW_ANGLE = 0.0                  # rad, only used when YAW_MODE == FIXED_YAW
# START_ON_PAUSED True: behavior plans the trajectory and stays PAUSED
# until the first resume().
START_ON_PAUSED = False

# Zigzag path (initial goal)
ZZ_N_SEGMENTS = 10                # number of segments along +X
ZZ_SEGMENT_LEN = 2.0             # m per segment in X
ZZ_AMPLITUDE = 2.0               # m peak-to-peak in Y
ZZ_ALTITUDE = 1.0                # m, constant Z
ZZ_X0 = 0.0                      # m, X start
ZZ_Y0 = 0.0                      # m, Y center (zigzag oscillates around this)
ZZ_ID_PREFIX = 'wp'

# Lifecycle triggers (set to None to disable)
# "after N waypoints" means: trigger when feedback.next_waypoint_id matches
# the id of path[N], i.e. the first N waypoints have been consumed.
MODIFY_AFTER_N_WAYPOINTS: Optional[int] = None
PAUSE_AFTER_N_WAYPOINTS: Optional[int] = None
RESUME_AFTER_SECONDS: float = 5.0   # only honored if PAUSE_AFTER_N_WAYPOINTS != None
STOP_AFTER_N_WAYPOINTS: Optional[int] = None

# Auto-resume of the initial start_on_paused (set to None to leave the
# resume to a manual ros2 service call).
RESUME_INITIAL_PAUSE_AFTER_S: Optional[float] = 5.0

# Modify path
#
# The behavior's on_modify merges the new path into the pending list:
#   - waypoints whose IDs match a pending one have their pose UPDATED;
#   - waypoints with new IDs are APPENDED at the end of the pending list.
# It does NOT replace the pending list. Therefore, to actually deform the
# trajectory that the drone is currently executing you must reuse the
# pending IDs.
#
# MOD_OVERRIDE_PENDING == True (recommended): the modify path reuses the
#     original pending IDs (path[MODIFY_AFTER_N_WAYPOINTS..end]) so the
#     remaining trajectory is reshaped in place. The geometry below
#     (segment length / amplitude / altitude) is applied to all of them
#     starting from the trigger waypoint position.
# MOD_OVERRIDE_PENDING == False: the modify path uses fresh IDs
#     (MOD_ID_PREFIX) so it is appended after the original path. The drone
#     finishes the original zigzag first and only then visits the new
#     waypoints. Use this to test the append branch of on_modify.
MOD_OVERRIDE_PENDING = True
MOD_SEGMENT_LEN = 2.0
MOD_AMPLITUDE = 2.0             # peak-to-peak in Y
MOD_ALTITUDE = 1.0               # m
MOD_N_SEGMENTS = 4               # only used when MOD_OVERRIDE_PENDING is False
MOD_ID_PREFIX = 'mod'            # only used when MOD_OVERRIDE_PENDING is False

# Fail-safe / timing
SERVER_TIMEOUT_S = 5.0
LOOP_PERIOD_S = 0.05
GLOBAL_TIMEOUT_S = 120.0         # hard cap on the orchestration loop
# ───────────────────────────────────────────────────────────────────────────


def build_zigzag_path(
    n_segments: int,
    segment_length: float,
    amplitude: float,
    altitude: float,
    frame_id: str,
    id_prefix: str,
    x0: float = 0.0,
    y0: float = 0.0,
) -> list:
    """
    Build a zigzag path along +X alternating Y between ±amplitude/2.

    Returns a list of PoseStampedWithID waypoints with unique string IDs of
    the form ``f"{id_prefix}_{i:03d}"``. The first waypoint is at
    ``(x0, y0, altitude)`` (no Y offset) so the drone always starts with a
    forward-only segment.
    """
    half = amplitude / 2.0
    path = []
    for i in range(n_segments + 1):
        wp = PoseStampedWithID()
        wp.id = f'{id_prefix}_{i:03d}'
        wp.pose.header.frame_id = frame_id
        wp.pose.pose.position.x = x0 + i * segment_length
        if i == 0:
            wp.pose.pose.position.y = y0
        else:
            wp.pose.pose.position.y = y0 + (half if (i % 2) == 1 else -half)
        wp.pose.pose.position.z = altitude
        wp.pose.pose.orientation.w = 1.0
        path.append(wp)
    return path


def build_goal(
    path: list,
    max_speed: float,
    yaw_mode: int,
    yaw_angle: float,
    start_on_paused: bool,
    stamp,
) -> GeneratePolynomialTrajectory.Goal:
    """Assemble a GeneratePolynomialTrajectory goal from path + motion params."""
    goal = GeneratePolynomialTrajectory.Goal()
    goal.stamp = stamp
    goal.max_speed = float(max_speed)
    goal.start_on_paused = bool(start_on_paused)

    yaw = YawMode()
    yaw.mode = int(yaw_mode)
    yaw.angle = float(yaw_angle)
    goal.yaw = yaw

    goal.path = list(path)
    return goal


def trigger_id(path: list, n: Optional[int]) -> Optional[str]:
    """Return the id of path[n] if n is in range, else None."""
    if n is None or n < 0 or n >= len(path):
        return None
    return path[n].id


class TrajectoryGeneratorTester(Node):
    """Drives the polynomial trajectory behavior through a configurable scenario."""

    def __init__(self, namespace: str):
        """Create the node and the BehaviorHandler bound to the behavior."""
        super().__init__('trajectory_generator_tester', namespace=namespace)
        BehaviorHandler.TIMEOUT = SERVER_TIMEOUT_S
        self._handler = BehaviorHandler(
            self, GeneratePolynomialTrajectory, 'TrajectoryGeneratorBehavior')

    def destroy(self) -> None:
        """Release behavior clients before destroying the node."""
        try:
            self._handler.destroy()
        finally:
            self.destroy_node()

    def run(self) -> bool:
        """Execute the configured scenario. Returns True on overall success."""
        log = self.get_logger()

        path = build_zigzag_path(
            n_segments=ZZ_N_SEGMENTS,
            segment_length=ZZ_SEGMENT_LEN,
            amplitude=ZZ_AMPLITUDE,
            altitude=ZZ_ALTITUDE,
            frame_id=FRAME_ID,
            id_prefix=ZZ_ID_PREFIX,
            x0=ZZ_X0,
            y0=ZZ_Y0,
        )
        log.info(f'Zigzag path: {len(path)} waypoints, ids '
                 f'[{path[0].id}..{path[-1].id}]')

        modify_id = trigger_id(path, MODIFY_AFTER_N_WAYPOINTS)
        pause_id = trigger_id(path, PAUSE_AFTER_N_WAYPOINTS)
        stop_id = trigger_id(path, STOP_AFTER_N_WAYPOINTS)
        log.info(f'Triggers: modify@{modify_id} pause@{pause_id} '
                 f'(resume after {RESUME_AFTER_SECONDS}s) stop@{stop_id}')

        goal = build_goal(
            path=path,
            max_speed=MAX_SPEED,
            yaw_mode=YAW_MODE,
            yaw_angle=YAW_ANGLE,
            start_on_paused=START_ON_PAUSED,
            stamp=self.get_clock().now().to_msg(),
        )

        try:
            self._handler.start(goal, wait_result=False)
        except BehaviorHandler.GoalRejected as err:
            log.error(f'Goal rejected: {err}')
            return False
        log.info('Goal accepted, entering orchestration loop')

        # Build the modify goal lazily.
        #
        # When MOD_OVERRIDE_PENDING is True we reshape the pending portion of
        # the original path: we generate a zigzag with as many points as
        # pending waypoints, anchored at the trigger waypoint position, and
        # rename the IDs so they match the original pending IDs. on_modify
        # will then update each pending waypoint pose in place.
        #
        # When False we generate an independent zigzag with fresh IDs that
        # the behavior will append after the original path.
        def make_modify_goal() -> GeneratePolynomialTrajectory.Goal:
            if MOD_OVERRIDE_PENDING and MODIFY_AFTER_N_WAYPOINTS is not None:
                trigger_idx = MODIFY_AFTER_N_WAYPOINTS
                pending = path[trigger_idx:]
                anchor = pending[0].pose.pose.position
                mod_path = build_zigzag_path(
                    n_segments=len(pending) - 1,
                    segment_length=MOD_SEGMENT_LEN,
                    amplitude=MOD_AMPLITUDE,
                    altitude=MOD_ALTITUDE,
                    frame_id=FRAME_ID,
                    id_prefix='__tmp',
                    x0=anchor.x,
                    y0=anchor.y,
                )
                for new_wp, orig_wp in zip(mod_path, pending):
                    new_wp.id = orig_wp.id
            else:
                anchor = path[-1].pose.pose.position
                mod_path = build_zigzag_path(
                    n_segments=MOD_N_SEGMENTS,
                    segment_length=MOD_SEGMENT_LEN,
                    amplitude=MOD_AMPLITUDE,
                    altitude=MOD_ALTITUDE,
                    frame_id=FRAME_ID,
                    id_prefix=MOD_ID_PREFIX,
                    x0=anchor.x,
                    y0=anchor.y,
                )
            return build_goal(
                path=mod_path,
                max_speed=MAX_SPEED,
                yaw_mode=YAW_MODE,
                yaw_angle=YAW_ANGLE,
                start_on_paused=False,
                stamp=self.get_clock().now().to_msg(),
            )

        # Latches and timing
        modify_done = modify_id is None
        pause_done = pause_id is None
        resume_done = pause_id is None
        stop_done = stop_id is None
        initial_resume_done = (
            not START_ON_PAUSED or RESUME_INITIAL_PAUSE_AFTER_S is None)

        t_start = time.monotonic()
        t_resume_at: Optional[float] = None
        t_initial_resume_at: Optional[float] = (
            t_start + RESUME_INITIAL_PAUSE_AFTER_S
            if not initial_resume_done else None)

        last_seen_id = ''

        while True:
            now = time.monotonic()
            if now - t_start > GLOBAL_TIMEOUT_S:
                log.error(f'Global timeout ({GLOBAL_TIMEOUT_S}s) hit, stopping')
                self._handler.stop()
                return False

            status = self._handler.status
            if status == BehaviorStatus.IDLE:
                # Behavior finished (success, abort, or stopped externally).
                break

            # Auto-resume the initial start_on_paused once the elapsed time has passed.
            if (not initial_resume_done
                    and t_initial_resume_at is not None
                    and now >= t_initial_resume_at):
                log.info('Auto-resuming initial start_on_paused')
                self._handler.resume(wait_result=False)
                initial_resume_done = True

            fb = self._handler.feedback
            if fb is not None and fb.next_waypoint_id != last_seen_id:
                log.info(f'feedback: next={fb.next_waypoint_id} '
                         f'remaining={fb.remaining_waypoints} '
                         f'status={status}')
                last_seen_id = fb.next_waypoint_id

            if fb is not None:
                if not modify_done and fb.next_waypoint_id == modify_id:
                    log.info(f'Triggering modify at {fb.next_waypoint_id}')
                    accepted = self._handler.modify(make_modify_goal())
                    log.info(f'Modify accepted={accepted}')
                    modify_done = True

                if not pause_done and fb.next_waypoint_id == pause_id:
                    log.info(f'Triggering pause at {fb.next_waypoint_id}')
                    ok = self._handler.pause()
                    log.info(f'Pause success={ok}')
                    pause_done = True
                    t_resume_at = time.monotonic() + RESUME_AFTER_SECONDS

                if not stop_done and fb.next_waypoint_id == stop_id:
                    log.info(f'Triggering stop at {fb.next_waypoint_id}')
                    ok = self._handler.stop()
                    log.info(f'Stop success={ok}')
                    stop_done = True
                    return bool(ok)

            if (pause_done and not resume_done
                    and t_resume_at is not None
                    and time.monotonic() >= t_resume_at):
                log.info(f'Triggering resume after {RESUME_AFTER_SECONDS}s')
                ok = self._handler.resume(wait_result=False)
                log.info(f'Resume success={ok}')
                resume_done = True

            time.sleep(LOOP_PERIOD_S)

        # Behavior went IDLE on its own (completed or failed). Pull the result.
        try:
            success = self._handler.wait_to_result()
        except BehaviorHandler.ResultUnknown as err:
            log.error(f'Result unknown: {err}')
            return False
        log.info(f'Trajectory generator result: success={success}')
        return bool(success)


def main():
    """Spin the node in a background thread and run the orchestration scenario."""
    rclpy.init(args=sys.argv)
    try:
        node = TrajectoryGeneratorTester(NAMESPACE)
    except BehaviorHandler.BehaviorNotAvailable as err:
        print(f'Behavior not available: {err}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ok = False
    try:
        ok = node.run()
    finally:
        executor.shutdown()
        node.destroy()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
