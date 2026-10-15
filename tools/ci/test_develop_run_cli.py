import base64
import os
import subprocess
import sys
import tempfile
import unittest

import develop_run as run
import develop_run_cli as cli

CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'develop_run_cli.py')
JUDGED = 'a' * 40
MAIN = 'b' * 40
STRAY = 'c' * 40


def b64(text):
    return base64.b64encode(text.encode('utf-8')).decode('ascii')


def gathered(**changed):
    handed = {'JUDGING_ENDED': 'success', 'JUDGED': JUDGED, 'SUBJECT': b64('A wheel throws a ball\n'), 'LACKING': '',
              'MAIN_AHEAD': '', 'WORKFLOWS_CHANGED': '', 'WORKFLOWS_DIFFERING': ''}
    handed.update({cli.outcome_of(step): 'success' for step in run.STEPS})
    handed.update(changed)
    return handed


def reported(**changed):
    handed = gathered(LOOKING_ENDED='success', PROMOTING_ENDED='success', MAIN_FOUND=MAIN, MAIN_TIP=JUDGED,
                      DEVELOP_TIP=JUDGED,
                      PUSHED='success', GIT_SAID='',
                      GITHUB_SERVER_URL='https://github.com', GITHUB_REPOSITORY='th7/midnight-snackers',
                      GITHUB_RUN_ID='123456')
    handed.update(changed)
    return handed


def called(command, environ, *arguments):
    kept = {'PATH': os.environ.get('PATH', ''), 'PYTHONDONTWRITEBYTECODE': '1'}
    return subprocess.run([sys.executable, CLI, command] + list(arguments), env=dict(kept, **environ),
                          capture_output=True, text=True, timeout=60)


class DecidingWhetherMainMoves(unittest.TestCase):
    def test_a_green_judgement_with_nothing_lacking_hands_the_workflow_a_move_naming_the_judged_commit_in_the_log(self):
        done = called('promotion', gathered())

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=move\n', done.stdout)
        self.assertIn('main moves to `%s`' % JUDGED, done.stderr)

    def test_a_held_main_hands_the_workflow_a_hold_and_says_why_in_the_log(self):
        done = called('promotion', gathered(LACKING=b64('%s Fix the README on main\n' % STRAY)))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=hold\n', done.stdout)
        self.assertIn('main holds where it was', done.stderr)
        self.assertIn('%s Fix the README on main' % STRAY, done.stderr)

    def test_a_main_that_already_has_later_commits_of_develop_hands_the_workflow_a_stay(self):
        done = called('promotion', gathered(MAIN_AHEAD=b64('%s A later commit of develop\n' % STRAY)))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=stay\n', done.stdout)
        self.assertIn('main stays where it is', done.stderr)

    def test_commits_since_main_that_change_a_workflow_hand_the_workflow_a_hold_naming_the_files_in_the_log(self):
        done = called('promotion', gathered(WORKFLOWS_CHANGED=b64('.github/workflows/tests.yml\n')))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=hold\n', done.stdout)
        self.assertIn('The commits from main to `%s` change 1 workflow file:\n\n```\n.github/workflows/tests.yml' % JUDGED,
                      done.stderr)
        self.assertIn('git push origin %s:refs/heads/main' % JUDGED, done.stderr)

    def test_a_workflow_main_s_copy_of_which_alone_differs_hands_the_workflow_a_hold_naming_it_apart_in_the_log(self):
        done = called('promotion', gathered(WORKFLOWS_DIFFERING=b64('.github/workflows/tests.yml\n')))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('decision=hold\n', done.stdout)
        self.assertIn("The judged commit's copy of 1 workflow file differs from main's", done.stderr)
        self.assertNotIn('The commits from main to', done.stderr)

    def test_an_input_the_workflow_did_not_hand_fails_naming_it_and_hands_nothing(self):
        for name in ('LACKING', 'WORKFLOWS_CHANGED', 'WORKFLOWS_DIFFERING'):
            environ = gathered()
            del environ[name]
            done = called('promotion', environ)

            self.assertEqual(2, done.returncode)
            self.assertEqual('', done.stdout)
            self.assertIn(name, done.stderr)

    def test_the_outcome_of_each_step_of_the_judging_job_is_handed_under_a_name_of_its_own(self):
        self.assertEqual(['STEP_CHECKOUT', 'STEP_JUDGED', 'STEP_JAVA', 'STEP_GRADLE', 'STEP_NODE', 'STEP_BROWSER',
                          'STEP_FORMATTING', 'STEP_TOOL_TESTS', 'STEP_UNIT_TESTS', 'STEP_RESULTS'],
                         [cli.outcome_of(step) for step in run.STEPS])

    def test_a_step_outcome_the_workflow_did_not_hand_fails_naming_it_and_hands_nothing(self):
        environ = gathered()
        del environ['STEP_BROWSER']
        done = called('promotion', environ)

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('STEP_BROWSER', done.stderr)

    def test_an_input_that_breaks_its_rule_fails_with_the_rule_and_hands_nothing(self):
        done = called('promotion', gathered(STEP_UNIT_TESTS='passed'))

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('an outcome is success, failure, cancelled, skipped or nothing', done.stderr)

    def test_an_input_holding_a_character_beyond_ascii_fails_with_the_rule_and_hands_nothing(self):
        for name in ('SUBJECT', 'LACKING', 'WORKFLOWS_CHANGED', 'WORKFLOWS_DIFFERING'):
            done = called('promotion', gathered(**{name: 'é'}))

            self.assertEqual(2, done.returncode, done.stderr)
            self.assertEqual('', done.stdout)
            self.assertIn('what the workflow gathered is base64', done.stderr)

    def test_a_command_it_does_not_know_fails_naming_those_it_does(self):
        done = called('promote', gathered())

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('promotion', done.stderr)


class SayingWhatTheRunFound(unittest.TestCase):
    def setUp(self):
        self.where = tempfile.TemporaryDirectory()
        self.open_issues = os.path.join(self.where.name, 'open-issues.json')
        self.body = os.path.join(self.where.name, 'issue-body.md')

    def tearDown(self):
        self.where.cleanup()

    def report(self, open_issues, **changed):
        with open(self.open_issues, 'w') as written:
            written.write(open_issues)
        self.done = called('report', reported(**changed), self.open_issues)
        return self.done

    def body_written(self):
        handed = dict(line.split('=', 1) for line in self.done.stdout.split('\n') if line)
        return base64.b64decode(handed['body'], validate=True).decode('utf-8')

    def test_a_red_run_with_no_issue_open_hands_the_workflow_an_issue_to_open_and_writes_its_body(self):
        done = self.report('[{"number": 7, "title": "Launch Sim"}]', JUDGING_ENDED='failure',
                           STEP_UNIT_TESTS='failure', PUSHED='skipped', PROMOTING_ENDED='failure')

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertTrue(done.stdout.startswith('action=open\ntitle=IMPORTANT: develop CI Failures\nbody='), done.stdout)
        self.assertIn('develop was judged red at `%s` `A wheel throws a ball`' % JUDGED, self.body_written())
        self.assertIn('https://github.com/th7/midnight-snackers/actions/runs/123456', self.body_written())
        self.assertIn(self.body_written(), done.stderr)

    def test_a_green_run_with_the_issue_open_hands_the_workflow_the_issue_to_close(self):
        done = self.report('[{"number": 12, "title": "IMPORTANT: develop CI Failures"}]')

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertTrue(done.stdout.startswith('action=close\nissues=12\nbody='), done.stdout)
        self.assertIn('main now stands there', self.body_written())

    def test_a_green_run_after_develop_moved_on_hands_the_workflow_the_issue_to_say_develop_moved_on_on_naming_develop_s_tip(self):
        done = self.report('[{"number": 12, "title": "IMPORTANT: develop CI Failures"}]', DEVELOP_TIP=STRAY)

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertTrue(done.stdout.startswith('action=moved-on\nissues=12\nbody='), done.stdout)
        self.assertIn("develop's tip is now `%s`" % STRAY, self.body_written())

    def test_a_run_whose_main_moved_off_the_judged_commit_after_its_push_hands_the_workflow_nothing_to_do_and_says_why_in_the_log(self):
        done = self.report('[{"number": 12, "title": "IMPORTANT: develop CI Failures"}]', MAIN_TIP=STRAY)

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('action=none\n', done.stdout)
        self.assertIn("but main moved since this run's push left it there, and now stands at `%s`, so this job says "
                      'nothing for the issue' % STRAY, done.stderr)

    def test_a_run_whose_mover_found_main_moved_since_the_look_hands_the_workflow_nothing_to_do_and_says_why_in_the_log(self):
        done = self.report('[]', PUSHED='skipped', MAIN_TIP=STRAY)

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('action=none\n', done.stdout)
        self.assertIn('but main moved since the job that looks found it at `%s`, and now stands at `%s`, so this job '
                      'says nothing for the issue' % (MAIN, STRAY), done.stderr)

    def test_a_held_run_says_main_stays_where_the_job_that_says_read_its_tip_not_where_the_look_found_it(self):
        done = self.report('[]', LOOKING_ENDED='failure', PUSHED='skipped', PROMOTING_ENDED='failure', MAIN_TIP=STRAY,
                           WORKFLOWS_CHANGED=b64('.github/workflows/tests.yml\n'))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertIn('main stays where it was, at `%s`.' % STRAY, self.body_written())

    def test_a_run_that_left_main_where_it_was_ahead_hands_the_workflow_nothing_to_do_and_no_body(self):
        done = self.report('[{"number": 12, "title": "IMPORTANT: develop CI Failures"}]', PUSHED='skipped',
                           MAIN_TIP=MAIN, MAIN_AHEAD=b64('%s A later commit of develop\n' % STRAY))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertEqual('action=none\n', done.stdout)

    def test_a_refused_push_reaches_the_body_with_what_git_said(self):
        done = self.report('[]', PUSHED='failure', PROMOTING_ENDED='failure', MAIN_TIP=MAIN,
                           GIT_SAID=b64('remote: error: GH006: Protected branch update failed\n'))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertTrue(done.stdout.startswith('action=open\ntitle=IMPORTANT: develop CI Failures\nbody='), done.stdout)
        self.assertIn('remote: error: GH006: Protected branch update failed\n', self.body_written())

    def test_a_run_held_over_workflows_hands_the_workflow_the_issue_to_comment_on_with_the_maintainer_s_push(self):
        done = self.report('[{"number": 12, "title": "IMPORTANT: develop CI Failures"}]', PUSHED='skipped',
                           PROMOTING_ENDED='failure', WORKFLOWS_CHANGED=b64('.github/workflows/tests.yml\n'))

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertTrue(done.stdout.startswith('action=comment\nissues=12\nbody='), done.stdout)
        self.assertIn('git push origin %s:refs/heads/main' % JUDGED, self.body_written())

    def test_a_report_not_handed_what_the_look_at_main_found_fails_naming_it_and_hands_nothing(self):
        for name in ('WORKFLOWS_CHANGED', 'WORKFLOWS_DIFFERING', 'LOOKING_ENDED', 'MAIN_FOUND', 'MAIN_TIP', 'DEVELOP_TIP'):
            environ = reported()
            del environ[name]
            with open(self.open_issues, 'w') as written:
                written.write('[]')
            done = called('report', environ, self.open_issues)

            self.assertEqual(2, done.returncode)
            self.assertEqual('', done.stdout)
            self.assertIn(name, done.stderr)

    def test_a_run_whose_look_at_main_ended_before_it_found_main_says_so(self):
        done = self.report('[]', LOOKING_ENDED='failure', MAIN_FOUND='', PROMOTING_ENDED='failure', PUSHED='skipped')

        self.assertEqual(0, done.returncode, done.stderr)
        self.assertTrue(done.stdout.startswith('action=open\ntitle=IMPORTANT: develop CI Failures\nbody='), done.stdout)
        self.assertIn('The job that looks at main ended `failure` before it found where main stands.',
                      self.body_written())

    def test_open_issues_that_are_not_the_json_gh_prints_fail_with_the_rule_and_hand_nothing(self):
        done = self.report('not json')

        self.assertEqual(2, done.returncode)
        self.assertEqual('', done.stdout)
        self.assertIn('JSON', done.stderr)

    def test_a_report_not_told_where_the_open_issues_are_and_nothing_more_fails_and_hands_nothing(self):
        for arguments in ((), (self.open_issues, self.body)):
            done = called('report', reported(), *arguments)

            self.assertEqual(2, done.returncode)
            self.assertEqual('', done.stdout)
            self.assertIn('report takes where the open issues are, and nothing more', done.stderr)


if __name__ == '__main__':
    unittest.main()
