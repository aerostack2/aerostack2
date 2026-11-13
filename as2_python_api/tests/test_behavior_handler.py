"""Tests for behavior result lifecycle with mocked ROS clients."""

# Copyright 2025 Universidad Politécnica de Madrid
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

from types import SimpleNamespace
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus
from as2_msgs.msg import BehaviorStatus
from as2_python_api.behavior_actions import behavior_handler
import pytest


@pytest.fixture
def handler(monkeypatch):
    """Create a handler without contacting ROS services."""
    monkeypatch.setattr(behavior_handler, 'ActionClient', MagicMock())
    monkeypatch.setattr(behavior_handler, 'get_sendgoal_action_msg', MagicMock())
    return behavior_handler.BehaviorHandler(MagicMock(), MagicMock(), 'test_behavior')


def complete_goal(handler, status):
    """Configure an immediately completed goal."""
    result = SimpleNamespace(status=status, result=object())
    goal = MagicMock(accepted=True)
    goal.get_result_async.return_value.result.return_value = result
    client = handler._BehaviorHandler__action_client
    client.send_goal_async.return_value.result.return_value = goal
    return result


def test_result_before_goal(handler):
    """An unset result has unknown status and raises the documented exception."""
    assert handler.result_status == GoalStatus.STATUS_UNKNOWN
    with pytest.raises(handler.ResultUnknown):
        _ = handler.result


def test_wait_before_goal(handler):
    """Waiting without a goal returns false."""
    assert handler.wait_to_result() is False


@pytest.mark.parametrize('status', [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED])
def test_start_clears_previous_result(handler, status):
    """A new asynchronous goal does not expose the previous result."""
    complete_goal(handler, status)
    handler.start(object())
    assert handler.result_status == status
    handler.start(object(), wait_result=False)
    assert handler.result_status == GoalStatus.STATUS_UNKNOWN
    with pytest.raises(handler.ResultUnknown):
        _ = handler.result


@pytest.mark.parametrize('status', [
    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
])
def test_completed_goal(handler, status):
    """Completed goals retain their status, result and success semantics."""
    result = complete_goal(handler, status)
    assert handler.start(object()) is (status == GoalStatus.STATUS_SUCCEEDED)
    assert handler.result_status == status
    if status == GoalStatus.STATUS_ABORTED:
        with pytest.raises(handler.ResultUnknown):
            _ = handler.result
    else:
        assert handler.result is result.result


def test_rejected_goal(handler):
    """Rejected goals continue raising GoalRejected."""
    complete_goal(handler, GoalStatus.STATUS_SUCCEEDED)
    client = handler._BehaviorHandler__action_client
    client.send_goal_async.return_value.result.return_value.accepted = False
    with pytest.raises(handler.GoalRejected):
        handler.start(object())


def test_resume_without_owned_goal(handler):
    """A paused behavior started elsewhere has no local result to wait for."""
    handler._BehaviorHandler__status = BehaviorStatus.PAUSED
    assert handler.resume() is False
