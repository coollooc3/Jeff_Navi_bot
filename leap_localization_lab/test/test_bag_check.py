"""Offline patrol policy and CLI checks; no ROS installation or robot is needed."""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


source = Path(__file__).resolve().parents[1] / 'exercises' / 'bag_check.py'
spec = importlib.util.spec_from_file_location('bag_check', source)
bag_check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bag_check)


def recorded_status(state='COMPLETED', **changes):
    status = {'state': state, 'current_waypoint': 'C', 'completed': 3, 'total': 3,
              'reason': '', 'goal_unresolved': False,
              'is_terminal': state in bag_check.PATROL_TERMINAL_STATES}
    status.update(changes)
    return status


class ExpectationTests(unittest.TestCase):
    def test_matching_terminal_states_preserve_input(self):
        for state, completed in [('COMPLETED', 3), ('CANCELED', 0), ('FAILED', 1)]:
            with self.subTest(state=state):
                status = recorded_status(state, completed=completed)
                before = deepcopy(status)
                self.assertEqual(bag_check.check_patrol_expectation(status, state),
                                 {'expected_state': state, 'actual_state': state,
                                  'passed': True, 'reason': ''})
                self.assertEqual(status, before)

    def test_missing_unfinished_unresolved_and_inconsistent_results_fail(self):
        cases = [
            (None, 'patrol_status_missing'),
            (recorded_status('IDLE'), 'patrol_not_terminal'),
            (recorded_status('RUNNING', is_terminal=True), 'patrol_not_terminal'),
            (recorded_status('CANCELING'), 'patrol_not_terminal'),
            (recorded_status(goal_unresolved=True), 'goal_unresolved'),
            (recorded_status(goal_unresolved=None), 'goal_resolution_unknown'),
            (recorded_status(goal_unresolved='false'), 'goal_resolution_unknown'),
            (recorded_status('FAILED'), 'state_mismatch'),
            (recorded_status(completed=2), 'completed_count_mismatch'),
            (recorded_status(completed=4), 'completed_count_mismatch'),
            (recorded_status(completed='3'), 'invalid_completion_counts'),
            (recorded_status(completed=True), 'invalid_completion_counts'),
            (recorded_status(total=-1), 'invalid_completion_counts'),
        ]
        for status, reason in cases:
            with self.subTest(reason=reason, status=status):
                before = deepcopy(status)
                result = bag_check.check_patrol_expectation(status, 'COMPLETED')
                self.assertFalse(result['passed'])
                self.assertEqual(result['reason'], reason)
                self.assertEqual(status, before)

    def test_canceled_with_unresolved_goal_still_fails(self):
        result = bag_check.check_patrol_expectation(
            recorded_status('CANCELED', completed=0, goal_unresolved=True), 'CANCELED')
        self.assertFalse(result['passed'])
        self.assertEqual(result['reason'], 'goal_unresolved')

    def test_invalid_expected_state_is_rejected(self):
        with self.assertRaises(ValueError):
            bag_check.check_patrol_expectation(recorded_status(), 'RUNNING')

    def test_invalid_status_shape_is_rejected(self):
        with self.assertRaises(ValueError):
            bag_check.check_patrol_expectation([], 'COMPLETED')


class CommandTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.bag = self.root / 'input_bag'
        self.bag.mkdir()

    def run_command(self, status, *options):
        report = {'topics': {}, 'total_messages': 0, 'time_basis': 'test recording times',
                  'patrol': {'last_status': status, 'completion_count_check': None}}
        if status is not None:
            report['patrol']['completion_count_check'] = {
                'passed': not bag_check.patrol_completion_count_mismatch(status), 'reason': ''}
        output = io.StringIO()
        with patch('sys.argv', ['bag_check.py', str(self.bag), *options]), \
                patch.object(bag_check, 'summarize_bag', return_value=report) as reader, \
                redirect_stdout(output):
            code = bag_check.main()
        return code, report, reader, output.getvalue()

    def test_expected_state_implies_patrol_decoding(self):
        code, report, reader, output = self.run_command(
            recorded_status(), '--expect-state', 'COMPLETED')
        self.assertEqual(code, 0)
        reader.assert_called_once_with(self.bag.resolve(), 'mcap', include_patrol=True)
        self.assertTrue(report['patrol']['expectation_check']['passed'])
        self.assertIn('PATROL EXPECTATION:', output)

    def test_mismatch_returns_one_and_saves_original_status(self):
        output_path = self.root / 'report.json'
        status = recorded_status('CANCELED', completed=0, reason='operator_cancel')
        code, _, _, _ = self.run_command(
            status, '--expect-state', 'COMPLETED', '--output', str(output_path))
        self.assertEqual(code, 1)
        report = json.loads(output_path.read_text(encoding='utf-8'))
        self.assertEqual(report['patrol']['last_status'], status)
        self.assertEqual(report['patrol']['expectation_check']['reason'], 'state_mismatch')

    def test_missing_status_returns_one(self):
        code, report, _, _ = self.run_command(None, '--expect-state', 'COMPLETED')
        self.assertEqual(code, 1)
        self.assertEqual(report['patrol']['expectation_check']['reason'], 'patrol_status_missing')

    def test_expected_cancellation_returns_zero_with_partial_count(self):
        code, _, _, _ = self.run_command(
            recorded_status('CANCELED', completed=0), '--expect-state', 'CANCELED')
        self.assertEqual(code, 0)

    def test_without_expectation_preserves_data_check_semantics(self):
        code, report, _, _ = self.run_command(recorded_status('FAILED'), '--patrol')
        self.assertEqual(code, 0)
        self.assertNotIn('expectation_check', report['patrol'])

    def test_stats_only_does_not_decode_patrol(self):
        code, _, reader, _ = self.run_command(None)
        self.assertEqual(code, 0)
        reader.assert_called_once_with(self.bag.resolve(), 'mcap', include_patrol=False)

    def test_invalid_cli_state_exits_two_before_reading(self):
        with patch('sys.argv', ['bag_check.py', str(self.bag), '--expect-state', 'RUNNING']), \
                patch.object(bag_check, 'summarize_bag') as reader, \
                redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as raised:
                bag_check.main()
        self.assertEqual(raised.exception.code, 2)
        reader.assert_not_called()


if __name__ == '__main__':
    unittest.main()
