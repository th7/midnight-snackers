import base64
import os
import subprocess
import sys
import unittest

CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'develop_run_cli.py')
JUDGED = 'a' * 40
MAIN = 'b' * 40
STRAY = 'c' * 40


def b64(text):
    return base64.b64encode(text.encode('utf-8')).decode('ascii')


def gathered(**changed):
    handed = {'JUDGING_ENDED': 'success', 'JUDGED': JUDGED, 'SUBJECT': b64('A wheel throws a ball\n'),
              'FORMATTING': 'success', 'TOOL_TESTS': 'success', 'UNIT_TESTS': 'success', 'LACKING': ''}
    handed.update(changed)
    return handed


def called(command, environ, *arguments):
    kept = {'PATH': os.environ.get('PATH', ''), 'PYTHONDONTWRITEBYTECODE': '1'}
    return subprocess.run([sys.executable, CLI, command] + list(arguments), env=dict(kept, **environ),
                          capture_output=True, text=True, timeout=60)


class DecidingWhetherMainMoves(unittest.TestCase):
    def test_a_green_judgement_with_nothing_lacking_hands_the_workflow_a_move_to_the_judged_commit(self):
        done = called('promotion', gathered())

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=move\ncommit=%s\n' % JUDGED, done.stdout)
        self.assertIn('main moves to `%s`' % JUDGED, done.stderr)

    def test_a_held_main_hands_the_workflow_a_hold_and_says_why_in_the_log(self):
        done = called('promotion', gathered(LACKING=b64('%s Fix the README on main\n' % STRAY)))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=hold\n', done.stdout)
        self.assertIn('main holds where it was', done.stderr)
        self.assertIn('%s Fix the README on main' % STRAY, done.stderr)

    def test_an_input_the_workflow_did_not_hand_fails_naming_it_and_hands_nothing(self):
        environ = gathered()
        del environ['LACKING']
        done = called('promotion', environ)

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('LACKING', done.stderr)

    def test_an_input_that_breaks_its_rule_fails_with_the_rule_and_hands_nothing(self):
        done = called('promotion', gathered(UNIT_TESTS='passed'))

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('an outcome is success, failure, cancelled, skipped or nothing', done.stderr)

    def test_a_command_it_does_not_know_fails_naming_those_it_does(self):
        done = called('promote', gathered())

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('promotion', done.stderr)


if __name__ == '__main__':
    unittest.main()
