"""Mission Stack test."""

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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import unittest

from as2_python_api.mission_interpreter.mission import MissionItem
from as2_python_api.mission_interpreter.mission_stack import MissionStack


class TestMissionStack(unittest.TestCase):
    """Mission Stack test case."""

    def setUp(self):
        """Set up test fixtures."""
        plan = []
        plan.append(MissionItem(behavior='dummy', args={'arg1': 1.0, 'arg2': 1.0}))
        plan.append(MissionItem(behavior='dummy', args={'arg1': 2.0, 'arg2': 2.0}))
        plan.append(MissionItem(behavior='dummy', args={'arg1': 3.0, 'arg2': 3.0}))

        self.stack = MissionStack(mission_stack=plan)

    def test_mission_stack(self):
        """Test mission stack."""
        self.assertEqual(self.stack.current, None)
        self.assertEqual(len(self.stack.pending), 3)
        self.assertListEqual(self.stack.done, [])
        self.assertEqual(self.stack.last_done, None)

    def test_mission_stack_next(self):
        """Test mission stack next item."""
        item = self.stack.next_item()
        self.assertEqual(item.behavior, 'dummy')
        self.assertDictEqual(item.args, {'arg1': 1.0, 'arg2': 1.0})

        self.assertEqual(len(self.stack.pending), 2)
        self.assertEqual(self.stack.done, [])
        self.assertEqual(self.stack.last_done, None)

        new_item = self.stack.next_item()
        self.assertEqual(new_item.behavior, 'dummy')
        self.assertDictEqual(new_item.args, {'arg1': 2.0, 'arg2': 2.0})

        self.assertEqual(len(self.stack.pending), 1)
        self.assertEqual(len(self.stack.done), 1)
        self.assertEqual(self.stack.last_done, item)

    def test_mission_stack_previous(self):
        """Test mission stack previous item."""
        null_item = self.stack.previous_item()
        self.assertEqual(null_item, None)
        self.assertEqual(len(self.stack.pending), 3)
        self.assertListEqual(self.stack.done, [])
        self.assertEqual(self.stack.last_done, None)

        item = self.stack.next_item()
        new_item = self.stack.next_item()
        self.assertEqual(len(self.stack.pending), 1)
        self.assertEqual(self.stack.current, new_item)
        self.assertEqual(len(self.stack.done), 1)

        prev = self.stack.previous_item()
        self.assertEqual(len(self.stack.pending), 2)
        self.assertEqual(prev, item)
        self.assertEqual(len(self.stack.done), 0)

    def test_mission_stack_add(self):
        """Test mission stack add."""
        self.stack.add(MissionItem(behavior='dummy', args={'arg1': 4.0, 'arg2': 4.0}))
        self.assertEqual(len(self.stack.pending), 4)

        item5 = MissionItem(behavior='dummy', args={'arg1': 5.0, 'arg2': 5.0})
        item6 = MissionItem(behavior='dummy', args={'arg1': 6.0, 'arg2': 6.0})
        self.stack.add([item5, item6])
        self.assertEqual(len(self.stack.pending), 6)

        first = self.stack.pending[0]
        last = self.stack.pending[-1]
        self.assertEqual(first.behavior, 'dummy')
        self.assertDictEqual(first.args, {'arg1': 1.0, 'arg2': 1.0})
        self.assertEqual(last.behavior, 'dummy')
        self.assertDictEqual(last.args, {'arg1': 6.0, 'arg2': 6.0})

    def test_mission_stack_insert(self):
        """Test mission stack insert."""
        self.stack.insert(MissionItem(behavior='dummy', args={'arg1': 4.0, 'arg2': 4.0}))
        self.assertEqual(len(self.stack.pending), 4)

        item5 = MissionItem(behavior='dummy', args={'arg1': 5.0, 'arg2': 5.0})
        item6 = MissionItem(behavior='dummy', args={'arg1': 6.0, 'arg2': 6.0})
        self.stack.insert([item6, item5])
        self.assertEqual(len(self.stack.pending), 6)

        first = self.stack.pending[0]
        last = self.stack.pending[-1]
        self.assertEqual(first.behavior, 'dummy')
        self.assertDictEqual(first.args, {'arg1': 6.0, 'arg2': 6.0})
        self.assertEqual(last.behavior, 'dummy')
        self.assertDictEqual(last.args, {'arg1': 3.0, 'arg2': 3.0})


if __name__ == '__main__':
    unittest.main()
