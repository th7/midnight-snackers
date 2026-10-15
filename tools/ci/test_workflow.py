import base64
import http.server
import os
import re
import subprocess
import tempfile
import threading
import unittest

import develop_issue as issues
import develop_run as run
import develop_run_cli as cli
from yaml_subset import Rejected, parse

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
WORKFLOW = os.path.join(REPO, '.github', 'workflows', 'tests.yml')
FOR_DEVELOP_ONLY = "${{ !cancelled() && github.event_name == 'push' && github.ref == 'refs/heads/develop' }}"
JUDGED_OR_DEVELOP = "${{ needs.unit-tests.outputs.commit || 'refs/heads/develop' }}"
CHECKOUT_KEEPS_ITS_TOKEN = 'https://github.com/actions/checkout/blob/v4/src/git-auth-helper.ts'
JUDGEMENT_HANDED = dict({'JUDGING_ENDED': '${{ needs.unit-tests.result }}',
                         'JUDGED': '${{ needs.unit-tests.outputs.commit }}',
                         'SUBJECT': '${{ needs.unit-tests.outputs.subject }}'},
                        **{cli.outcome_of(step): '${{ needs.unit-tests.outputs.%s }}' % step.step for step in run.STEPS})


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

    def test_the_judging_job_s_steps_are_the_ones_the_core_names_in_the_same_order(self):
        self.assertEqual([(step.step, step.name) for step in run.STEPS],
                         [(each.get('id'), each.get('name')) for each in steps_of(self.jobs['unit-tests'])])

    def test_a_run_for_develop_writes_the_gradle_cache_and_a_pull_request_only_reads_it(self):
        self.assertEqual({'name': run.GRADLE.name, 'id': run.GRADLE.step, 'uses': 'gradle/actions/setup-gradle@v4',
                          'with': {'cache-read-only': "${{ github.event_name != 'push' }}"}},
                         step_named(self.jobs['unit-tests'], run.GRADLE.step))
        self.assertEqual('Gradle, its cache written by runs for develop and only read by pull requests', run.GRADLE.name)

    def test_each_check_is_the_same_command_as_local(self):
        for check in run.CHECKS:
            self.assertEqual(check.command, step_named(self.jobs['unit-tests'], check.step)['run'])

    def test_the_judging_job_hands_on_the_commit_it_judged_and_how_each_of_its_steps_ended(self):
        self.assertEqual(dict({'commit': '${{ steps.judged.outputs.commit }}',
                               'subject': '${{ steps.judged.outputs.subject }}'},
                              **{step.step: '${{ steps.%s.outcome }}' % step.step for step in run.STEPS}),
                         self.jobs['unit-tests']['outputs'])

    def test_the_job_that_moves_main_checks_out_the_commit_judged_with_all_of_history_and_the_built_in_token(self):
        self.assertEqual({'name': 'Check out the judged commit', 'uses': 'actions/checkout@v4',
                          'with': {'ref': JUDGED_OR_DEVELOP, 'fetch-depth': '0'}},
                         steps_of(self.jobs['promote'])[0])

    def test_nothing_in_the_workflow_reads_a_secret(self):
        with open(WORKFLOW, encoding='utf-8') as text:
            self.assertNotIn('secrets.', text.read())

    def test_no_job_names_an_environment(self):
        self.assertEqual({}, {name: job['environment'] for name, job in self.jobs.items() if 'environment' in job})

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

    def test_the_core_decides_from_the_judging_what_main_has_and_the_workflow_files_the_commits_since_main_change(self):
        deciding = step_named(self.jobs['promote'], 'decide')

        self.assertEqual('python3 tools/ci/develop_run_cli.py promotion >> "$GITHUB_OUTPUT"', deciding['run'])
        self.assertEqual(dict(JUDGEMENT_HANDED, LACKING='${{ steps.main.outputs.lacking }}',
                              MAIN_AHEAD='${{ steps.main.outputs.ahead }}',
                              WORKFLOWS='${{ steps.main.outputs.workflows }}'), deciding['env'])

    def test_what_main_has_and_lacks_is_found_against_develop_and_the_judged_commit_just_before_the_decision(self):
        self.assertEqual(
            'git fetch --no-tags origin +refs/heads/main:refs/remotes/origin/main '
            '+refs/heads/develop:refs/remotes/origin/develop\n'
            'tip=$(git rev-parse refs/remotes/origin/main)\n'
            "lacking=$(git log --format='%H %s' refs/remotes/origin/develop..refs/remotes/origin/main | base64 -w0)\n"
            "ahead=$(git log --format='%H %s' HEAD..refs/remotes/origin/main | base64 -w0)\n"
            'workflows=$(git log --format= --name-only refs/remotes/origin/main..HEAD -- .github/workflows | base64 -w0)\n'
            'echo "tip=$tip" >> "$GITHUB_OUTPUT"\n'
            'echo "lacking=$lacking" >> "$GITHUB_OUTPUT"\n'
            'echo "ahead=$ahead" >> "$GITHUB_OUTPUT"\n'
            'echo "workflows=$workflows" >> "$GITHUB_OUTPUT"\n',
            step_named(self.jobs['promote'], 'main')['run'])
        self.assertEqual('Find what main has that develop or the judged commit lacks, and the workflow files the '
                         'commits since main change', step_named(self.jobs['promote'], 'main')['name'])

    def test_the_mover_hands_on_what_it_found_of_main_and_how_its_push_went(self):
        self.assertEqual({'main-tip': '${{ steps.main.outputs.tip }}',
                          'lacking': '${{ steps.main.outputs.lacking }}',
                          'main-ahead': '${{ steps.main.outputs.ahead }}',
                          'workflows': '${{ steps.main.outputs.workflows }}',
                          'pushed': '${{ steps.push.outcome }}',
                          'git-said': '${{ steps.push.outputs.said }}'},
                         self.jobs['promote']['outputs'])

    def test_a_main_the_core_neither_moves_nor_leaves_where_it_already_is_makes_the_run_red(self):
        holding = [step for step in steps_of(self.jobs['promote']) if step.get('name') == 'Hold main where it was']
        some_commit = run.Commit(run.CommitHash('a' * 40), 'A commit of develop')
        handed = [run.promotion_outputs(decided).split('\n')[0].split('=')[1] for decided in
                  (run.Move(some_commit), run.Stay(some_commit, (some_commit,)))]

        self.assertEqual([{'name': 'Hold main where it was',
                           'if': "steps.decide.outputs.decision != '%s' && steps.decide.outputs.decision != '%s'"
                                 % tuple(handed),
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


class WhatTheMoverFindsOfMainInARealRepository(WorkflowTest):
    def setUp(self):
        super().setUp()
        self.where = tempfile.TemporaryDirectory()
        self.environ = {'PATH': os.environ.get('PATH', ''), 'HOME': self.where.name, 'GIT_CONFIG_NOSYSTEM': '1',
                        'GIT_CONFIG_GLOBAL': os.devnull, 'GIT_AUTHOR_NAME': 'Coach', 'GIT_AUTHOR_EMAIL': 'c@example.org',
                        'GIT_COMMITTER_NAME': 'Coach', 'GIT_COMMITTER_EMAIL': 'c@example.org', 'LC_ALL': 'C'}
        self.origin = os.path.join(self.where.name, 'origin.git')
        self.work = os.path.join(self.where.name, 'work')
        self.git(self.where.name, 'init', '--bare', '-b', 'main', self.origin)
        self.git(self.where.name, 'init', '-b', 'main', self.work)
        self.git(self.work, 'remote', 'add', 'origin', self.origin)
        self.commit('Where both lines start')
        self.git(self.work, 'push', 'origin', 'main', 'main:develop')

    def tearDown(self):
        self.where.cleanup()

    def git(self, where, *arguments):
        return subprocess.run(['git'] + list(arguments), cwd=where, env=self.environ, check=True, capture_output=True,
                              text=True, timeout=60).stdout.strip()

    def commit(self, subject, written=None, removed=()):
        for path, content in (written or {}).items():
            os.makedirs(os.path.dirname(os.path.join(self.work, path)), exist_ok=True)
            with open(os.path.join(self.work, path), 'w', encoding='utf-8') as file:
                file.write(content)
            self.git(self.work, 'add', path)
        for path in removed:
            self.git(self.work, 'rm', '-q', path)
        self.git(self.work, 'commit', '--allow-empty', '-q', '-m', subject)
        return self.git(self.work, 'rev-parse', 'HEAD')

    def on(self, branch):
        self.git(self.work, 'checkout', '-q', '-B', branch, 'origin/' + branch if branch != 'main' else 'main')

    def push(self, *refspecs):
        self.git(self.work, 'push', '-q', 'origin', *refspecs)

    def ran(self, step_id, runner, **environ):
        outputs = os.path.join(runner, 'outputs-' + step_id)
        done = subprocess.run(['bash', '--noprofile', '--norc', '-eo', 'pipefail', '-c',
                               step_named(self.jobs['promote'], step_id)['run']], cwd=runner,
                              env=dict(self.environ, GITHUB_OUTPUT=outputs, RUNNER_TEMP=runner, **environ),
                              capture_output=True, text=True, timeout=60)
        self.assertEqual(0, done.returncode, done.stdout + done.stderr)
        with open(outputs, encoding='utf-8') as written:
            return dict(line.split('=', 1) for line in written.read().split('\n') if line)

    def mover_found(self, judged_commit):
        runner = tempfile.mkdtemp(prefix='runner-', dir=self.where.name)
        self.git(self.where.name, 'clone', '-q', self.origin, runner)
        self.git(runner, 'checkout', '-q', '--detach', judged_commit)
        return runner, self.ran('main', runner)

    def judgement_of(self, judged_commit, subject):
        steps = tuple('success' for _ in run.STEPS)
        return run.judgement('success', judged_commit, base64.b64encode((subject + '\n').encode()).decode(), steps)

    def decided_for(self, judged_commit, subject):
        _, found = self.mover_found(judged_commit)
        return run.promotion(self.judgement_of(judged_commit, subject), run.listed_commits(found['lacking']),
                             run.listed_commits(found['ahead']), run.workflow_files(found['workflows']))

    def listed(self, *hashes_and_subjects):
        return tuple(run.Commit(run.CommitHash(each), subject) for each, subject in hashes_and_subjects)

    def test_a_main_behind_the_judged_commit_along_develop_moves_to_it(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        judged_commit = self.commit('A commit of develop')
        self.push('develop')

        decided = self.decided_for(judged_commit, 'A commit of develop')

        self.assertEqual(run.Move(run.Commit(run.CommitHash(judged_commit), 'A commit of develop')), decided)

    def test_a_main_moved_past_the_judged_commit_along_develop_stays_and_names_nothing_develop_lacks(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        judged_commit = self.commit('The commit judged')
        later = self.commit('A later commit of develop')
        self.push('develop', 'develop:main')

        decided = self.decided_for(judged_commit, 'The commit judged')

        self.assertEqual(run.Stay(run.Commit(run.CommitHash(judged_commit), 'The commit judged'),
                                  self.listed((later, 'A later commit of develop'))), decided)

    def test_a_commit_that_reached_main_by_itself_holds_main_naming_it(self):
        stray = self.commit('Fix the README on main')
        self.push('main')
        self.git(self.work, 'checkout', '-q', '-b', 'develop', 'HEAD~1')
        judged_commit = self.commit('A commit of develop')
        self.push('develop')

        decided = self.decided_for(judged_commit, 'A commit of develop')

        self.assertEqual(run.Hold((run.MainHasWhatDevelopLacks(self.listed((stray, 'Fix the README on main'))),)),
                         decided)

    def test_commits_since_main_that_change_workflows_hold_main_for_a_maintainer_naming_each_file_once(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Run the tool tests too', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        judged_commit = self.commit('Release from main', {'.github/workflows/tests.yml': 'name: all tests\n',
                                                          '.github/workflows/release.yml': 'name: release\n'})
        self.push('develop')

        decided = self.decided_for(judged_commit, 'Release from main')

        self.assertEqual(run.Hold((run.MainNeedsAMaintainerForWorkflows(
            run.Commit(run.CommitHash(judged_commit), 'Release from main'),
            run.WorkflowFiles(('.github/workflows/release.yml', '.github/workflows/tests.yml'))),)), decided)

    def test_a_workflow_change_a_later_commit_undoes_still_holds_main_naming_it(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Try a nightly run', {'.github/workflows/nightly.yml': 'name: nightly\n'})
        judged_commit = self.commit('No nightly run after all', removed=('.github/workflows/nightly.yml',))
        self.push('develop')

        decided = self.decided_for(judged_commit, 'No nightly run after all')

        self.assertEqual(run.WorkflowFiles(('.github/workflows/nightly.yml',)), decided.findings[0].files)

    def test_a_workflow_change_main_already_has_does_not_hold_main(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Run the tool tests too', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('develop', 'develop:main')
        judged_commit = self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        self.push('develop')

        decided = self.decided_for(judged_commit, 'Say how to build')

        self.assertEqual(run.Move(run.Commit(run.CommitHash(judged_commit), 'Say how to build')), decided)

    def test_a_rerun_after_a_maintainer_pushed_main_to_the_judged_commit_moves_it_nowhere_and_closes_the_issue(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        judged_commit = self.commit('Run the tool tests too', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('develop')
        judgement = self.judgement_of(judged_commit, 'Run the tool tests too')
        self.assertIsInstance(self.decided_for(judged_commit, 'Run the tool tests too'), run.Hold)
        self.push('%s:refs/heads/main' % judged_commit)

        runner, found = self.mover_found(judged_commit)
        decided = run.promotion(judgement, run.listed_commits(found['lacking']), run.listed_commits(found['ahead']),
                                run.workflow_files(found['workflows']))
        self.assertEqual(run.Move(run.Commit(run.CommitHash(judged_commit), 'Run the tool tests too')), decided)
        pushed = self.ran('push', runner, JUDGED=judged_commit)
        promoting = run.promoting('success', found['tip'], found['lacking'], found['ahead'], found['workflows'],
                                  'success', pushed['said'])
        action = issues.issue_action(judgement, promoting, (issues.OpenIssue(12, issues.TITLE),),
                                     issues.run_link('https://github.com', 'th7/midnight-snackers', '123456'))

        self.assertEqual(judged_commit, self.git(self.origin, 'rev-parse', 'refs/heads/main'))
        self.assertIsInstance(action, issues.Close)
        self.assertEqual((12,), action.issues)
        self.assertIn('main now stands there', action.body)

    def test_a_commit_that_reached_main_by_itself_and_was_merged_into_develop_after_the_judging_stays(self):
        stray = self.commit('Fix the README on main')
        self.push('main')
        self.git(self.work, 'checkout', '-q', '-b', 'develop', 'HEAD~1')
        judged_commit = self.commit('A commit of develop')
        self.git(self.work, 'merge', '-q', '--no-edit', 'main')
        self.push('develop')

        decided = self.decided_for(judged_commit, 'A commit of develop')

        self.assertEqual(run.Stay(run.Commit(run.CommitHash(judged_commit), 'A commit of develop'),
                                  self.listed((stray, 'Fix the README on main'))), decided)


def basic(token):
    return 'basic ' + base64.b64encode(('x-access-token:' + token).encode('ascii')).decode('ascii')


class ARefusingServer(http.server.ThreadingHTTPServer):
    def __init__(self, status):
        super().__init__(('127.0.0.1', 0), Refusing)
        self.status = status
        self.heard = []


class Refusing(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.heard.append(self.headers.get_all('Authorization') or [])
        said = b'Permission to th7/midnight-snackers.git denied to a-user.\n'
        self.send_response(self.server.status)
        if self.server.status == 401:
            self.send_header('WWW-Authenticate', 'Basic realm="GitHub"')
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(said)))
        self.end_headers()
        self.wfile.write(said)

    def log_message(self, *arguments):
        pass


class WhatThePushToMainSendsOverHttp(WorkflowTest):
    BUILT_IN = 'the-built-in-token'

    def setUp(self):
        super().setUp()
        self.where = tempfile.TemporaryDirectory()
        self.environ = {'PATH': os.environ.get('PATH', ''), 'HOME': self.where.name, 'GIT_CONFIG_NOSYSTEM': '1',
                        'GIT_CONFIG_GLOBAL': os.devnull, 'GIT_AUTHOR_NAME': 'Coach', 'GIT_AUTHOR_EMAIL': 'c@example.org',
                        'GIT_COMMITTER_NAME': 'Coach', 'GIT_COMMITTER_EMAIL': 'c@example.org', 'LC_ALL': 'C'}
        self.runner = os.path.join(self.where.name, 'runner')
        self.git(self.where.name, 'init', '-q', '-b', 'develop', self.runner)
        self.git(self.runner, 'commit', '--allow-empty', '-q', '-m', 'The commit judged')
        self.judged = self.git(self.runner, 'rev-parse', 'HEAD')

    def tearDown(self):
        self.where.cleanup()

    def git(self, where, *arguments):
        return subprocess.run(['git'] + list(arguments), cwd=where, env=self.environ, check=True, capture_output=True,
                              text=True, timeout=60).stdout.strip()

    def pushed(self, status):
        server = ARefusingServer(status)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        runner = os.path.join(self.where.name, 'runner-%d' % server.server_address[1])
        try:
            url = 'http://127.0.0.1:%d' % server.server_address[1]
            self.git(self.where.name, 'clone', '-q', self.runner, runner)
            self.git(runner, 'remote', 'set-url', 'origin', url + '/th7/midnight-snackers')
            self.git(runner, 'config', 'http.%s/.extraheader' % url, 'AUTHORIZATION: ' + basic(self.BUILT_IN))
            outputs = os.path.join(runner, 'outputs')
            done = subprocess.run(
                ['bash', '--noprofile', '--norc', '-eo', 'pipefail', '-c',
                 step_named(self.jobs['promote'], 'push')['run']],
                cwd=runner, capture_output=True, text=True, timeout=60, start_new_session=True,
                env=dict(self.environ, GITHUB_SERVER_URL=url, GITHUB_OUTPUT=outputs, RUNNER_TEMP=runner,
                         JUDGED=self.judged))
            return done, server.heard
        finally:
            server.shutdown()
            server.server_close()

    def test_the_push_sends_the_built_in_token_the_checkout_kept_and_no_other(self):
        done, heard = self.pushed(403)

        self.assertNotEqual(0, done.returncode, done.stdout)
        self.assertTrue(heard, done.stdout)
        self.assertEqual([[basic(self.BUILT_IN)]] * len(heard), heard,
                         'the checkout keeps the built-in token as the http.extraheader of the server it fetched from '
                         '(%s), which a plain push sends' % CHECKOUT_KEEPS_ITS_TOKEN)


class HowTheIssueIsWritten(WorkflowTest):
    def test_the_open_issues_are_listed_asking_gh_for_as_many_as_the_core_takes_to_be_a_list_cut_short(self):
        self.assertIn('gh issue list --repo "$GITHUB_REPOSITORY" --state open --limit %d --json number,title '
                      '> "$RUNNER_TEMP/open-issues.json"' % issues.LISTED_AT_MOST, runs_of(self.jobs['report']))

    def test_the_core_says_what_to_write_from_everything_the_run_found(self):
        saying = step_named(self.jobs['report'], 'say')

        self.assertEqual('python3 tools/ci/develop_run_cli.py report "$RUNNER_TEMP/open-issues.json" '
                         '"$RUNNER_TEMP/issue-body.md" >> "$GITHUB_OUTPUT"', saying['run'])
        self.assertEqual(dict(JUDGEMENT_HANDED,
                              PROMOTING_ENDED='${{ needs.promote.result }}',
                              MAIN_TIP='${{ needs.promote.outputs.main-tip }}',
                              LACKING='${{ needs.promote.outputs.lacking }}',
                              MAIN_AHEAD='${{ needs.promote.outputs.main-ahead }}',
                              WORKFLOWS='${{ needs.promote.outputs.workflows }}',
                              PUSHED='${{ needs.promote.outputs.pushed }}',
                              GIT_SAID='${{ needs.promote.outputs.git-said }}'),
                         saying['env'])
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
