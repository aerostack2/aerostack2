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
