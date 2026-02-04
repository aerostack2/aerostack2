"""Mission Stack test."""

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

__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import unittest
from unittest.mock import MagicMock

from as2_python_api.mission_interpreter.mission_stack import MissionStack


class TestMissionStack(unittest.TestCase):

    def setUp(self):
        self.mission_item_1 = MagicMock()
        self.mission_item_1.json.return_value = '{"id": 1}'
        self.mission_item_2 = MagicMock()
        self.mission_item_2.json.return_value = '{"id": 2}'
        self.mission_item_3 = MagicMock()
        self.mission_item_3.json.return_value = '{"id": 3}'

        self.stack = MissionStack([self.mission_item_1, self.mission_item_2])

    def test_initial_state(self):
        self.assertEqual(self.stack.current, None)
        self.assertEqual(self.stack.pending, [self.mission_item_1, self.mission_item_2])
        self.assertEqual(self.stack.done, [])

    def test_next_item(self):
        current = self.stack.next_item()
        self.assertEqual(current, self.mission_item_1)
        self.assertEqual(self.stack.current, self.mission_item_1)
        self.assertEqual(self.stack.pending, [self.mission_item_2])
        self.assertEqual(self.stack.done, [])

        current = self.stack.next_item()
        self.assertEqual(current, self.mission_item_2)
        self.assertEqual(self.stack.current, self.mission_item_2)
        self.assertEqual(self.stack.pending, [])
        self.assertEqual(self.stack.done, [self.mission_item_1])

    def test_previous_item(self):
        self.stack.next_item()
        self.stack.next_item()
        current = self.stack.previous_item()
        self.assertEqual(current, self.mission_item_1)
        self.assertEqual(self.stack.current, self.mission_item_1)
        self.assertEqual(self.stack.pending, [self.mission_item_2])
        self.assertEqual(self.stack.done, [])

        current = self.stack.previous_item()
        self.assertEqual(current, None)
        self.assertEqual(self.stack.current, None)
        self.assertEqual(self.stack.pending, [self.mission_item_1, self.mission_item_2])
        self.assertEqual(self.stack.done, [])

    def test_point_to_item(self):
        self.stack.next_item()
        self.stack.next_item()
        self.stack.point_to_item(0)
        self.assertEqual(self.stack.current, self.mission_item_1)
        self.assertEqual(self.stack.pending, [self.mission_item_2])
        self.assertEqual(self.stack.done, [])

        self.stack.point_to_item(1)
        self.assertEqual(self.stack.current, self.mission_item_2)
        self.assertEqual(self.stack.pending, [])
        self.assertEqual(self.stack.done, [self.mission_item_1])

    def test_add(self):
        self.stack.add(self.mission_item_3)
        self.assertEqual(self.stack.pending, [self.mission_item_1,
                         self.mission_item_2, self.mission_item_3])

    def test_insert(self):
        self.stack.insert(self.mission_item_3)
        self.assertEqual(self.stack.pending, [self.mission_item_3,
                         self.mission_item_1, self.mission_item_2])

    def test_modify(self):
        self.stack.next_item()
        self.mission_item_2.modify.return_value = True
        result = self.stack.modify(1, self.mission_item_3)
        self.assertTrue(result)
        self.mission_item_2.modify.assert_called_once_with(self.mission_item_3)

    def test_modify_out_of_range(self):
        result = self.stack.modify(10, self.mission_item_3)
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()
