import os
import re
import unittest

import develop_issue as issues
import develop_run as run
import develop_run_cli as cli
from yaml_subset import Rejected, parse

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
WORKFLOW = os.path.join(REPO, '.github', 'workflows', 'tests.yml')
FOR_DEVELOP_ONLY = "${{ !cancelled() && github.event_name == 'push' && github.ref == 'refs/heads/develop' }}"
JUDGED_OR_DEVELOP = "${{ needs.unit-tests.outputs.commit || 'refs/heads/develop' }}"


def read_workflow():
    with open(WORKFLOW, encoding='utf-8') as text:
        return parse(text.read())


def steps_of(job):
    return job.get('steps', [])


def runs_of(job):
    return [step['run'] for step in steps_of(job) if 'run' in step]


def step_named(job, step_id):
    return next(step for step in steps_of(job) if step.get('id') == step_id)


def every_text_in(value):
    if isinstance(value, dict):
        return [text for key, inner in value.items() for text in [key] + every_text_in(inner)]
    if isinstance(value, list):
        return [text for inner in value for text in every_text_in(inner)]
    return [value] if isinstance(value, str) else []


class WorkflowTest(unittest.TestCase):
    def setUp(self):
        self.workflow = read_workflow()
        self.assertNotIsInstance(self.workflow, Rejected, 'tests.yml is outside what the tests read: %r'
                                 % (self.workflow,))
        self.jobs = self.workflow['jobs']


class WhatStartsARun(WorkflowTest):
    def test_a_push_to_develop_and_a_pull_request_start_a_run_and_a_push_to_main_does_not(self):
        self.assertEqual({'push': {'branches': ['develop']}, 'pull_request': None}, self.workflow['on'])

    def test_runs_for_develop_go_one_at_a_time_never_cancelling_one_in_progress_and_pull_requests_queue_apart(self):
        self.assertEqual(
            {'group': "${{ github.event_name == 'push' && 'tests-develop' || "
                      "format('tests-pull-request-{0}', github.event.pull_request.number) }}",
             'cancel-in-progress': "${{ github.event_name == 'pull_request' }}"},
            self.workflow['concurrency'])

    def test_only_a_push_to_develop_reaches_the_jobs_that_move_main_and_write_the_issue_judged_red_or_not(self):
        self.assertEqual({'promote': FOR_DEVELOP_ONLY, 'report': FOR_DEVELOP_ONLY},
                         {name: job['if'] for name, job in self.jobs.items() if 'if' in job})
        self.assertEqual('unit-tests', self.jobs['promote']['needs'])
        self.assertEqual(['unit-tests', 'promote'], self.jobs['report']['needs'])

    def test_no_job_or_step_is_let_fail_without_failing_the_run(self):
        def keys_in(value):
            if isinstance(value, dict):
                return [key for inner_key, inner in value.items() for key in [inner_key] + keys_in(inner)]
            if isinstance(value, list):
                return [key for inner in value for key in keys_in(inner)]
            return []

        self.assertNotIn('continue-on-error', keys_in(self.workflow))

    def test_every_job_ends_long_before_github_s_own_limit_would_read_as_a_stuck_queue(self):
        for name, job in self.jobs.items():
            self.assertLessEqual(int(job['timeout-minutes']), 30, name)


class WhatEachJobMayDo(WorkflowTest):
    def test_nothing_is_granted_to_the_workflow_as_a_whole(self):
        self.assertEqual({}, self.workflow['permissions'])

    def test_each_job_is_granted_only_what_it_needs(self):
        self.assertEqual({'unit-tests': {'contents': 'read'},
                          'promote': {'contents': 'write'},
                          'report': {'contents': 'read', 'issues': 'write'}},
                         {name: job['permissions'] for name, job in self.jobs.items()})

    def test_the_only_job_that_pushes_is_the_only_one_granted_contents_write(self):
        pushing = {name for name, job in self.jobs.items() if any('git push' in each for each in runs_of(job))}
        writing = {name for name, job in self.jobs.items() if job['permissions'].get('contents') == 'write'}

        self.assertEqual({'promote'}, pushing)
        self.assertEqual(pushing, writing)

    def test_the_only_job_that_writes_an_issue_is_the_only_one_granted_issues_write(self):
        writing_issues = {name for name, job in self.jobs.items()
                          if any(re.search(r'gh issue (create|comment|close)', each) for each in runs_of(job))}
        granted = {name for name, job in self.jobs.items() if job['permissions'].get('issues') == 'write'}

        self.assertEqual({'report'}, writing_issues)
        self.assertEqual(writing_issues, granted)

    def test_only_the_job_that_pushes_keeps_its_credentials_in_the_checkout(self):
        kept = {name for name, job in self.jobs.items() for step in steps_of(job)
                if step.get('uses', '').startswith('actions/checkout@')
                and step.get('with', {}).get('persist-credentials') != 'false'}

        self.assertEqual({'promote'}, kept)


class WhatARunJudges(WorkflowTest):
    def test_a_run_for_develop_checks_out_develop_s_tip_as_it_is_when_the_job_starts(self):
        checkout = steps_of(self.jobs['unit-tests'])[0]

        self.assertTrue(checkout['uses'].startswith('actions/checkout@'))
        self.assertEqual("${{ github.event_name == 'push' && 'refs/heads/develop' || github.ref }}",
                         checkout['with']['ref'])

    def test_nothing_judges_or_moves_the_commit_the_event_names(self):
        with open(WORKFLOW, encoding='utf-8') as text:
            self.assertNotIn('github.sha', text.read())

    def test_the_judged_commit_is_named_from_what_was_checked_out(self):
        self.assertEqual(
            'commit=$(git rev-parse HEAD)\n'
            'subject=$(git log -1 --format=%s HEAD | base64 -w0)\n'
            'echo "commit=$commit" >> "$GITHUB_OUTPUT"\n'
            'echo "subject=$subject" >> "$GITHUB_OUTPUT"\n',
            step_named(self.jobs['unit-tests'], 'judged')['run'])

    def test_each_check_is_the_same_command_as_local_and_hands_on_its_outcome(self):
        judging = self.jobs['unit-tests']
        for check in run.CHECKS:
            step = step_named(judging, check.step)
            self.assertEqual((check.name, check.command), (step['name'], step['run']))
            self.assertEqual('${{ steps.%s.outcome }}' % check.step, judging['outputs'][check.step])

    def test_the_job_that_moves_main_checks_out_the_commit_judged_with_all_of_history_so_it_can_count_what_main_has(self):
        self.assertEqual({'name': 'Check out the judged commit', 'uses': 'actions/checkout@v4',
                          'with': {'ref': JUDGED_OR_DEVELOP, 'fetch-depth': '0'}},
                         steps_of(self.jobs['promote'])[0])

    def test_the_job_that_writes_the_issue_checks_out_the_commit_judged_without_keeping_a_credential(self):
        self.assertEqual({'name': 'Check out the judged commit', 'uses': 'actions/checkout@v4',
                          'with': {'ref': JUDGED_OR_DEVELOP, 'persist-credentials': 'false'}},
                         steps_of(self.jobs['report'])[0])


class HowMainMoves(WorkflowTest):
    def test_main_is_moved_only_by_a_plain_push_of_the_commit_the_core_said_moves(self):
        promoting = self.jobs['promote']

        self.assertEqual(
            {'name': 'Move main to the judged commit, never by force',
             'id': 'push',
             'if': "steps.decide.outputs.decision == 'move'",
             'env': {'JUDGED': '${{ steps.decide.outputs.commit }}'},
             'run': 'pushed=0\n'
                    'git push origin "$JUDGED:refs/heads/main" > "$RUNNER_TEMP/git-said" 2>&1 || pushed=$?\n'
                    'cat "$RUNNER_TEMP/git-said"\n'
                    'said=$(base64 -w0 < "$RUNNER_TEMP/git-said")\n'
                    'echo "said=$said" >> "$GITHUB_OUTPUT"\n'
                    'exit "$pushed"\n'},
            step_named(promoting, 'push'))
        self.assertEqual(1, sum(each.count('git push') for job in self.jobs.values() for each in runs_of(job)))

    def test_the_core_decides_from_what_the_judging_found_and_what_main_has_that_develop_lacks(self):
        deciding = step_named(self.jobs['promote'], 'decide')

        self.assertEqual('python3 tools/ci/develop_run_cli.py promotion >> "$GITHUB_OUTPUT"', deciding['run'])
        self.assertEqual(set(cli.JUDGEMENT) | {'LACKING'}, set(deciding['env']))
        self.assertEqual('${{ steps.main.outputs.lacking }}', deciding['env']['LACKING'])

    def test_what_main_lacks_is_found_against_main_as_it_is_just_before_the_decision(self):
        self.assertEqual(
            'git fetch --no-tags origin +refs/heads/main:refs/remotes/origin/main\n'
            'tip=$(git rev-parse refs/remotes/origin/main)\n'
            "lacking=$(git log --format='%H %s' HEAD..refs/remotes/origin/main | base64 -w0)\n"
            'echo "tip=$tip" >> "$GITHUB_OUTPUT"\n'
            'echo "lacking=$lacking" >> "$GITHUB_OUTPUT"\n',
            step_named(self.jobs['promote'], 'main')['run'])

    def test_a_main_the_core_holds_makes_the_run_red(self):
        holding = [step for step in steps_of(self.jobs['promote']) if step.get('name') == 'Hold main where it was']

        self.assertEqual([{'name': 'Hold main where it was', 'if': "steps.decide.outputs.decision != 'move'",
                           'run': 'exit 1'}], holding)

    def test_no_run_merges_commits_or_rewrites_and_no_push_forces(self):
        for job in self.jobs.values():
            for each in runs_of(job):
                for never in ('git merge', 'git commit', 'git reset', 'git rebase', 'gh api', 'gh pr merge'):
                    self.assertNotIn(never, each)
                for line in each.split('\n'):
                    if 'git push' in line:
                        for never in ('--force', ' -f ', '--mirror', '--delete', '--all', ' +', '"+', ':+'):
                            self.assertNotIn(never, line)


class HowTheIssueIsWritten(WorkflowTest):
    def test_the_open_issues_are_listed_as_the_core_reads_them(self):
        self.assertIn('gh issue list --repo "$GITHUB_REPOSITORY" --state open --limit 1000 --json number,title '
                      '> "$RUNNER_TEMP/open-issues.json"', runs_of(self.jobs['report']))

    def test_the_core_says_what_to_write_from_everything_the_run_found(self):
        saying = step_named(self.jobs['report'], 'say')

        self.assertEqual('python3 tools/ci/develop_run_cli.py report "$RUNNER_TEMP/open-issues.json" '
                         '"$RUNNER_TEMP/issue-body.md" >> "$GITHUB_OUTPUT"', saying['run'])
        self.assertEqual(set(cli.JUDGEMENT) | set(cli.PROMOTING), set(saying['env']))

    def test_each_issue_action_the_core_hands_on_is_carried_out_by_one_step_that_does_only_that(self):
        report = self.jobs['report']
        carried = [step for step in steps_of(report) if 'steps.say.outputs.action' in step.get('if', '')]
        handed = [issues.action_outputs(action).split('\n')[0].split('=')[1] for action in
                  (issues.Open(issues.TITLE, ''), issues.CommentOn(1, ''), issues.Close((1,), ''))]

        self.assertEqual(
            [{'name': 'Open the issue',
              'if': "steps.say.outputs.action == '%s'" % handed[0],
              'env': {'GH_TOKEN': '${{ github.token }}', 'TITLE': '${{ steps.say.outputs.title }}'},
              'run': 'gh issue create --repo "$GITHUB_REPOSITORY" --title "$TITLE" '
                     '--body-file "$RUNNER_TEMP/issue-body.md"'},
             {'name': 'Comment on the open issue',
              'if': "steps.say.outputs.action == '%s'" % handed[1],
              'env': {'GH_TOKEN': '${{ github.token }}', 'ISSUE': '${{ steps.say.outputs.issues }}'},
              'run': 'gh issue comment "$ISSUE" --repo "$GITHUB_REPOSITORY" --body-file "$RUNNER_TEMP/issue-body.md"'},
             {'name': 'Close the issue, saying main now stands at the judged commit',
              'if': "steps.say.outputs.action == '%s'" % handed[2],
              'env': {'GH_TOKEN': '${{ github.token }}', 'ISSUES': '${{ steps.say.outputs.issues }}'},
              'run': 'for issue in $ISSUES; do\n'
                     '  gh issue comment "$issue" --repo "$GITHUB_REPOSITORY" --body-file "$RUNNER_TEMP/issue-body.md"\n'
                     '  gh issue close "$issue" --repo "$GITHUB_REPOSITORY"\n'
                     'done\n'}],
            carried)


class WhatAShellIsHanded(WorkflowTest):
    def test_no_run_block_interpolates_an_expression_so_untrusted_text_reaches_a_shell_only_through_env(self):
        for job in self.jobs.values():
            for each in runs_of(job):
                self.assertNotIn('${{', each)

    def test_no_run_block_carries_a_shell_comment(self):
        for job in self.jobs.values():
            for each in runs_of(job):
                for line in each.split('\n'):
                    self.assertFalse(line.lstrip().startswith('#'), line)

    def test_every_run_is_bash_so_a_failing_command_in_a_pipe_fails_its_step(self):
        self.assertEqual({'run': {'shell': 'bash'}}, self.workflow['defaults'])
        for job in self.jobs.values():
            self.assertNotIn('defaults', job)
            for step in steps_of(job):
                self.assertNotIn('shell', step)

    def test_every_step_and_job_output_an_expression_names_exists(self):
        for name, job in self.jobs.items():
            step_ids = {step['id'] for step in steps_of(job) if 'id' in step}
            needs = job.get('needs', [])
            needed = [needs] if isinstance(needs, str) else needs
            for text in every_text_in(job):
                for step_id in re.findall(r'steps\.([A-Za-z0-9_-]+)\.', text):
                    self.assertIn(step_id, step_ids, '%s names step %s' % (name, step_id))
                for needed_job, output in re.findall(r'needs\.([A-Za-z0-9_-]+)\.outputs\.([A-Za-z0-9_-]+)', text):
                    self.assertIn(needed_job, needed, '%s names job %s' % (name, needed_job))
                    self.assertIn(output, self.jobs[needed_job]['outputs'], '%s names %s.%s'
                                  % (name, needed_job, output))
                for needed_job in re.findall(r'needs\.([A-Za-z0-9_-]+)\.result', text):
                    self.assertIn(needed_job, needed, '%s names job %s' % (name, needed_job))


if __name__ == '__main__':
    unittest.main()
