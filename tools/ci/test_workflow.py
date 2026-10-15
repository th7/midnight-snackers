import base64
import http.server
import itertools
import json
import os
import re
import subprocess
import sys
import tempfile
import threading
import unittest

import develop_issue as issues
import develop_run as run
import develop_run_cli as cli
import workflow_conditions as conditions
from yaml_subset import Rejected, parse

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
WORKFLOW = os.path.join(REPO, '.github', 'workflows', 'tests.yml')
FOR_DEVELOP_ONLY = "${{ !cancelled() && github.event_name == 'push' && github.ref == 'refs/heads/develop' }}"
JUDGED_OR_DEVELOP = "${{ needs.unit-tests.outputs.commit || 'refs/heads/develop' }}"
CHECKOUT_KEEPS_ITS_TOKEN = 'https://github.com/actions/checkout/blob/v4/src/git-auth-helper.ts'
SECRETS_CONTEXT = 'https://docs.github.com/en/actions/reference/workflows-and-actions/contexts#secrets-context'
CHECKOUT_FETCHES_EVERY_BRANCH = 'https://github.com/actions/checkout/blob/v4/src/ref-helper.ts'
REPOSITORY_CODE = re.compile(r'python3? |\./|gradlew|npm |npx |tools/|bash |source |make ')
CONTEXT_NAMES_IGNORE_CASE = ('https://github.com/actions/runner/blob/796cc18ffaaafd5cea525d6231f5a8c93c11c603/'
                             'src/Sdk/DTExpressions2/Expressions2/ExpressionParser.cs#L427')
TESTS_YML = run.WorkflowFiles(('.github/workflows/tests.yml',))
JUDGEMENT_HANDED = dict({'JUDGING_ENDED': '${{ needs.unit-tests.result }}',
                         'JUDGED': '${{ needs.unit-tests.outputs.commit }}',
                         'SUBJECT': '${{ needs.unit-tests.outputs.subject }}'},
                        **{cli.outcome_of(step): '${{ needs.unit-tests.outputs.%s }}' % step.step for step in run.STEPS})
DEVELOP_READ = 'gh api --method GET "repos/$GITHUB_REPOSITORY/git/ref/heads/develop" --jq .object.sha'
MAIN_READ = 'gh api --method GET "repos/$GITHUB_REPOSITORY/git/ref/heads/main" --jq .object.sha'
JUDGED_HASH = 'a' * 40
LATER_HASH = 'b' * 40
STRAY_HASH = 'c' * 40
MOVED_ALONG_DEVELOP = 'Push nothing once main has moved along develop since the job that looks found it, saying so'
HOLDS = ("Fail unless develop holds main's tip as both are now, as GitHub compares them, so that no run is green while "
         'main has a commit develop lacks, whatever the core said')
MOVED_OFF_DEVELOP = ('Fail once main has moved since the job that looks found it to a tip this job could not find in '
                     'develop, pushing nothing and saying so')
COMPARE = 'gh api --method GET "repos/$GITHUB_REPOSITORY/compare/$MAIN...$DEVELOP" --jq .status'
MOVED_ON = ('Comment on the open issue that develop moved on past the commit judged green, only when no step before '
            'failed')
FALLING_BACK = 'Open or comment on the issue saying only that the run is red, when nothing else said so'
COMPARE_STATUS = 'https://docs.github.com/en/rest/commits/commits#compare-two-commits'
FELL_BACK = ('A run for develop is red, and the jobs that say why could not, so only the run itself says what failed: '
             'https://github.com/th7/midnight-snackers/actions/runs/123456')
FAKE_GH = '\n'.join((
    'import json, os, re, subprocess, sys',
    'sent = sys.argv[1:]',
    "read = [open(sent[sent.index('--body-file') + 1], encoding='utf-8').read()] if '--body-file' in sent else []",
    "with open(os.environ['GH_CALLS'], 'a') as calls:",
    "    calls.write(json.dumps(sent + read) + '\\n')",
    "if sent[:2] == ['issue', 'list']:",
    "    sys.stdout.write(open(os.environ['GH_LISTED']).read())",
    "if sent[:2] == ['issue', 'close'] and os.path.exists(os.environ.get('GH_REFUSES_CLOSE', '')):",
    "    sys.exit('HTTP 502: Bad Gateway')",
    "asked = sent[3] if len(sent) == 6 and sent[:3] == ['api', '--method', 'GET'] else ''",
    "repository = 'repos/%s/' % os.environ.get('GITHUB_REPOSITORY', '')",
    "path = asked[len(repository):] if asked.startswith(repository) else ''",
    "refs = {'git/ref/heads/%s' % name: name for name in ('develop', 'main')}",
    "branch = refs.get(path) if sent[4:] == ['--jq', '.object.sha'] else None",
    "hashes = '([0-9a-f]{40}|[0-9a-f]{64})'",
    "compared = re.fullmatch('compare/%s[.]{3}%s' % (hashes, hashes), path) if sent[4:] == ['--jq', '.status'] \\",
    '    else None',
    "if sent[:1] == ['api'] and branch is None and compared is None:",
    "    sys.exit('this gh answers only a read of the ref of develop or main, or a compare of two commits')",
    "if sent[:1] == ['api'] and 'GH_API_FAILS' in os.environ:",
    "    sys.exit('HTTP 502: Bad Gateway')",
    "origin = ['git', '--git-dir', os.environ.get('GH_ORIGIN', '')]",
    "if branch is not None and 'GH_ORIGIN' in os.environ:",
    "    sys.stdout.write(subprocess.run(origin + ['rev-parse', 'refs/heads/' + branch], check=True, capture_output=True,",
    '                                    text=True).stdout)',
    "if branch is not None and 'GH_' + branch.upper() in os.environ:",
    "    sys.stdout.write(os.environ['GH_' + branch.upper()])",
    "if compared is not None and 'GH_ORIGIN' in os.environ:",
    '    base, head = compared.groups()',
    "    if subprocess.run(origin + ['merge-base', base, head], capture_output=True).returncode != 0:",
    "        sys.exit('HTTP 404: Not Found')",
    "    counted = [int(subprocess.run(origin + ['rev-list', '--count', since + '..' + until], check=True,",
    "                                  capture_output=True, text=True).stdout)",
    '               for since, until in ((base, head), (head, base))]',
    "    ahead, behind = (each > 0 for each in counted)",
    "    print({(False, False): 'identical', (True, False): 'ahead', (False, True): 'behind',",
    "           (True, True): 'diverged'}[(ahead, behind)])",
    "if compared is not None and 'GH_COMPARE' in os.environ:",
    "    sys.stdout.write(os.environ['GH_COMPARE'])",
    ''))


def core_word_for_develop_moved_on():
    judgement = run.judgement('success', JUDGED_HASH, base64.b64encode(b'Tune the arm\n').decode('ascii'),
                              tuple('success' for _ in run.STEPS))
    promoting = run.promoting('success', STRAY_HASH, JUDGED_HASH, LATER_HASH, '', '', '', '', 'success', 'success', '')
    action = issues.issue_action(judgement, promoting, (issues.OpenIssue(12, issues.TITLE),),
                                 issues.run_link('https://github.com', 'th7/midnight-snackers', '123456'))
    return issues.action_outputs(action).split('\n')[0].split('=')[1]


def read_workflow():
    with open(WORKFLOW, encoding='utf-8') as text:
        return parse(text.read())


def steps_of(job):
    return job.get('steps', [])


def runs_of(job):
    return [step['run'] for step in steps_of(job) if 'run' in step]


def step_named(job, step_id):
    return next(step for step in steps_of(job) if step.get('id') == step_id)


def key_of(step):
    return step.get('id') or step['name']


def steps_played(job, contexts, act):
    status = conditions.Status(failed=False, cancelled=False)
    steps = {}
    for step in steps_of(job):
        here = dict(contexts, steps=steps)
        runs = conditions.holds(step.get('if', 'success()'), here, status)
        if isinstance(runs, conditions.Rejected):
            return runs
        outcome, outputs = act(step, here, status) if runs else ('skipped', {})
        if isinstance(outcome, conditions.Rejected):
            return outcome
        steps[key_of(step)] = {'outcome': outcome, 'outputs': outputs}
        status = conditions.Status(failed=status.failed or outcome == 'failure', cancelled=False)
    return steps, 'failure' if status.failed else 'success'


def ends_red(step):
    return 'run' in step and step['run'].rstrip('\n').split('\n')[-1] == 'exit 1'


def steps_ran(job, contexts, outputs=None, failing=()):
    def pretended(step, here, status):
        fails = ends_red(step) or key_of(step) in failing
        return 'failure' if fails else 'success', (outputs or {}).get(key_of(step), {})

    done = steps_played(job, contexts, pretended)
    if isinstance(done, conditions.Rejected):
        return done
    steps, ended = done
    return {key: step['outcome'] for key, step in steps.items()}, ended


def as_text(value):
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return value if isinstance(value, str) else str(value)


def evaluated(text, contexts, status):
    if '${{' not in text:
        return text
    value = conditions.value(text, contexts, status)
    return value if isinstance(value, conditions.Rejected) else as_text(value)


def written_outputs(path):
    with open(path, encoding='utf-8') as written:
        return dict(line.split('=', 1) for line in written.read().split('\n') if '=' in line)


def played(job, contexts, cwd, environ, temp):
    def shell_ran(step, here, status):
        if 'run' not in step:
            return 'success', {}
        handed = {name: evaluated(text, here, status) for name, text in step.get('env', {}).items()}
        rejected = [each for each in handed.values() if isinstance(each, conditions.Rejected)]
        if rejected:
            return rejected[0], {}
        outputs = os.path.join(temp, 'outputs-%s' % re.sub(r'[^A-Za-z0-9]+', '-', key_of(step)))
        open(outputs, 'w', encoding='utf-8').close()
        done = subprocess.run(['bash', '--noprofile', '--norc', '-eo', 'pipefail', '-c', step['run']], cwd=cwd,
                              env=dict(environ, GITHUB_OUTPUT=outputs, RUNNER_TEMP=temp, **handed),
                              capture_output=True, text=True, timeout=60)
        return 'success' if done.returncode == 0 else 'failure', written_outputs(outputs)

    done = steps_played(job, contexts, shell_ran)
    if isinstance(done, conditions.Rejected):
        return done
    steps, ended = done
    status = conditions.Status(failed=ended == 'failure', cancelled=False)
    outputs = {name: evaluated(text, {'steps': steps}, status) for name, text in job.get('outputs', {}).items()}
    return {'result': ended, 'outputs': outputs}, steps


def fake_gh(where):
    bin_directory = os.path.join(where, 'bin')
    os.makedirs(bin_directory)
    gh = os.path.join(bin_directory, 'gh')
    with open(gh, 'w', encoding='utf-8') as fake:
        fake.write('#!%s\n%s' % (sys.executable, FAKE_GH))
    os.chmod(gh, 0o755)
    return bin_directory + os.pathsep + os.environ.get('PATH', '')


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
        self.assertEqual({'look': FOR_DEVELOP_ONLY, 'promote': FOR_DEVELOP_ONLY, 'say': FOR_DEVELOP_ONLY,
                          'report': FOR_DEVELOP_ONLY},
                         {name: job['if'] for name, job in self.jobs.items() if 'if' in job})
        self.assertEqual('unit-tests', self.jobs['look']['needs'])
        self.assertEqual(['unit-tests', 'look'], self.jobs['promote']['needs'])
        self.assertEqual(['unit-tests', 'look', 'promote'], self.jobs['say']['needs'])
        self.assertEqual(['unit-tests', 'look', 'promote', 'say'], self.jobs['report']['needs'])

    def test_every_condition_in_the_workflow_is_one_the_tests_evaluate(self):
        for name, job in self.jobs.items():
            for text in [job.get('if', 'success()')] + [step.get('if', 'success()') for step in steps_of(job)]:
                self.assertNotIsInstance(conditions.parsed(text), conditions.Rejected, (name, text))

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
                          'look': {'contents': 'read'},
                          'promote': {'contents': 'write'},
                          'say': {'contents': 'read', 'issues': 'read'},
                          'report': {'contents': 'read', 'issues': 'write'}},
                         {name: job['permissions'] for name, job in self.jobs.items()},
                         'the job that writes the issue compares two commits, which contents: read allows (%s)'
                         % COMPARE_STATUS)

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

    def test_no_job_that_runs_code_it_checked_out_holds_a_write_permission_or_keeps_a_credential(self):
        running = {name for name, job in self.jobs.items() if any(REPOSITORY_CODE.search(each) for each in runs_of(job))}

        self.assertEqual({'unit-tests', 'look', 'say'}, running)
        for name in running:
            self.assertEqual({'read'}, set(self.jobs[name]['permissions'].values()), name)
            for step in steps_of(self.jobs[name]):
                if step.get('uses', '').startswith('actions/checkout@'):
                    self.assertEqual('false', step['with'].get('persist-credentials'), name)

    def test_the_job_that_writes_the_issue_checks_out_nothing(self):
        self.assertEqual([], [step for step in steps_of(self.jobs['report']) if 'uses' in step])


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
        self.assertEqual({'name': 'Check out the judged commit with all of history and the built-in token, running '
                                  'none of its code',
                          'uses': 'actions/checkout@v4',
                          'with': {'ref': JUDGED_OR_DEVELOP, 'fetch-depth': '0'}},
                         steps_of(self.jobs['promote'])[0])

    def test_the_job_that_decides_checks_out_the_commit_judged_with_every_branch_s_history_keeping_no_credential(self):
        self.assertEqual({'name': "Check out the judged commit with every branch's history, keeping no credential",
                          'uses': 'actions/checkout@v4',
                          'with': {'ref': JUDGED_OR_DEVELOP, 'fetch-depth': '0', 'persist-credentials': 'false'}},
                         steps_of(self.jobs['look'])[0],
                         'a checkout of all of history fetches every branch into refs/remotes/origin (%s), so the job '
                         'that looks at main needs no credential of its own to fetch main and develop'
                         % CHECKOUT_FETCHES_EVERY_BRANCH)

    def test_nothing_in_the_workflow_reads_a_secret_in_any_form(self):
        with open(WORKFLOW, encoding='utf-8') as text:
            self.assertEqual([], re.findall(r'.*\bsecrets\b.*', text.read(), re.IGNORECASE),
                             'no step reads a secret, and an expression reaches the secrets context as secrets.NAME, '
                             "as secrets['NAME'] or whole (%s), its name spelled in any case (%s)"
                             % (SECRETS_CONTEXT, CONTEXT_NAMES_IGNORE_CASE))

    def test_the_jobs_that_move_main_and_write_the_issue_hold_exactly_these_steps_so_none_is_added_unseen(self):
        self.assertEqual(
            {'look': [(None, "Check out the judged commit with every branch's history, keeping no credential"),
                      ('main', 'Find where main stands, what main has that develop or the judged commit lacks, and the '
                               'workflow files the commits since main change or main and the judged commit differ in'),
                      ('decide', 'Decide whether main moves'),
                      (None, 'Fail unless main moves or stays, so that re-running the failed jobs looks at main again')],
             'promote': [(None, 'Check out the judged commit with all of history and the built-in token, running none '
                                'of its code'),
                         ('main', "Find where main stands now and whether develop holds it, as this job's checkout "
                                  'fetched them'),
                         ('push', 'Move main to the judged commit, never by force, only from where the job that looks '
                                  'found it'),
                         (None, MOVED_ALONG_DEVELOP),
                         (None, MOVED_OFF_DEVELOP),
                         (None, 'Hold main where it was')],
             'say': [(None, 'List the open issues, before any of the judged commit is on disk'),
                     ('develop', "Read develop's tip as it is now, before any of the judged commit is on disk"),
                     ('main', "Read main's tip as it is now, before any of the judged commit is on disk"),
                     (None, 'Check out the judged commit, keeping no credential'),
                     ('say', 'Say what the run found')],
             'report': [('develop', "Read develop's tip as it is now, before writing anything to the issue"),
                        ('main', "Read main's tip as it is now, before writing anything to the issue"),
                        ('holds', HOLDS),
                        ('open', 'Open the issue'),
                        ('comment', 'Comment on the open issue'),
                        ('moved-on', MOVED_ON),
                        ('close', "Close the issue, saying main now stands at the judged commit, only when every job "
                                  "before passed and that commit is still both develop's tip and main's"),
                        ('behind', "Say on the issue where develop's tip and main's now stand, when every job before "
                                   "passed, the core said close and only main's is still the judged commit"),
                        ('moved', 'Write nothing to the issue once main has moved since this run looked at it, saying '
                                  "so, when every job before passed, the core said close and main's tip is not the "
                                  'judged commit'),
                        (None, FALLING_BACK)]},
            {name: [(step.get('id'), step.get('name')) for step in steps_of(self.jobs[name])]
             for name in ('look', 'promote', 'say', 'report')})

    def test_the_workflow_and_each_job_hold_exactly_their_keys_so_no_env_or_environment_is_added_unseen(self):
        self.assertEqual(['name', 'on', 'concurrency', 'permissions', 'defaults', 'jobs'], list(self.workflow))
        self.assertEqual({'unit-tests': ['runs-on', 'timeout-minutes', 'permissions', 'outputs', 'steps'],
                          'look': ['needs', 'if', 'runs-on', 'timeout-minutes', 'permissions', 'outputs', 'steps'],
                          'promote': ['needs', 'if', 'runs-on', 'timeout-minutes', 'permissions', 'outputs', 'steps'],
                          'say': ['needs', 'if', 'runs-on', 'timeout-minutes', 'permissions', 'outputs', 'steps'],
                          'report': ['needs', 'if', 'runs-on', 'timeout-minutes', 'permissions', 'steps']},
                         {name: list(job) for name, job in self.jobs.items()})

    def test_no_job_names_an_environment(self):
        self.assertEqual({}, {name: job['environment'] for name, job in self.jobs.items() if 'environment' in job})

    def test_the_job_that_says_what_to_write_checks_out_the_commit_judged_without_keeping_a_credential(self):
        self.assertEqual([{'name': 'Check out the judged commit, keeping no credential', 'uses': 'actions/checkout@v4',
                           'with': {'ref': JUDGED_OR_DEVELOP, 'persist-credentials': 'false'}}],
                         [step for step in steps_of(self.jobs['say']) if 'uses' in step])


class HowMainMoves(WorkflowTest):
    def test_main_is_moved_only_by_a_plain_push_of_the_judged_commit_when_the_judging_passed_and_the_core_said_move(self):
        promoting = self.jobs['promote']

        self.assertEqual(
            {'name': 'Move main to the judged commit, never by force, only from where the job that looks found it',
             'id': 'push',
             'if': "needs.unit-tests.result == 'success' && needs.look.result == 'success' && "
                   "needs.look.outputs.decision == 'move' && steps.main.outputs.tip == needs.look.outputs.main-tip",
             'env': {'JUDGED': '${{ needs.unit-tests.outputs.commit }}'},
             'run': 'pushed=0\n'
                    'git push origin "$JUDGED:refs/heads/main" > "$RUNNER_TEMP/git-said" 2>&1 || pushed=$?\n'
                    'cat "$RUNNER_TEMP/git-said"\n'
                    'said=$(base64 -w0 < "$RUNNER_TEMP/git-said")\n'
                    'echo "said=$said" >> "$GITHUB_OUTPUT"\n'
                    'exit "$pushed"\n'},
            step_named(promoting, 'push'))
        self.assertEqual(1, sum(each.count('git push') for job in self.jobs.values() for each in runs_of(job)))

    def test_the_mover_finds_where_main_stands_now_and_whether_develop_holds_it_as_its_checkout_fetched_every_branch_running_none_of_the_judged_code(self):
        self.assertEqual({'name': "Find where main stands now and whether develop holds it, as this job's checkout "
                                  'fetched them',
                          'id': 'main',
                          'run': 'tip=$(git rev-parse refs/remotes/origin/main)\n'
                                 'along=false\n'
                                 'if git merge-base --is-ancestor "$tip" refs/remotes/origin/develop; then\n'
                                 '  along=true\n'
                                 'fi\n'
                                 'echo "tip=$tip" >> "$GITHUB_OUTPUT"\n'
                                 'echo "along=$along" >> "$GITHUB_OUTPUT"\n'},
                         step_named(self.jobs['promote'], 'main'),
                         'a checkout of all of history fetches every branch into refs/remotes/origin (%s)'
                         % CHECKOUT_FETCHES_EVERY_BRANCH)

    def test_the_mover_pushes_nothing_once_main_has_moved_since_the_look_saying_so_by_steps_that_write_nothing_and_ends_red_unless_develop_holds_main_s_tip(self):
        self.assertEqual(
            [{'name': MOVED_ALONG_DEVELOP,
              'if': "needs.unit-tests.result == 'success' && needs.look.result == 'success' && "
                    "steps.main.outputs.tip != needs.look.outputs.main-tip && steps.main.outputs.along == 'true'",
              'env': {'FOUND': '${{ needs.look.outputs.main-tip }}', 'NOW': '${{ steps.main.outputs.tip }}'},
              'run': 'echo "main moved along develop since the job that looks found it at $FOUND: its tip is now $NOW, '
                     'which develop holds, so this run pushes nothing, and a run that looks at main again, for a new '
                     "push to develop or re-running all of this run's jobs, speaks for develop.\"\n"},
             {'name': MOVED_OFF_DEVELOP,
              'if': "needs.unit-tests.result == 'success' && needs.look.result == 'success' && "
                    "steps.main.outputs.tip != needs.look.outputs.main-tip && steps.main.outputs.along != 'true'",
              'env': {'FOUND': '${{ needs.look.outputs.main-tip }}', 'NOW': '${{ steps.main.outputs.tip }}'},
              'run': 'echo "main moved since the job that looks found it at $FOUND: its tip is now $NOW, which this job '
                     "could not find in develop as its checkout fetched it, so this run pushes nothing and is red.\" "
                     '>&2\n'
                     'exit 1\n'}],
            [step for step in steps_of(self.jobs['promote']) if 'NOW' in step.get('env', {})])

    def test_the_core_decides_from_the_judging_what_main_has_and_the_workflow_files_the_commits_since_main_change(self):
        deciding = step_named(self.jobs['look'], 'decide')

        self.assertEqual('python3 tools/ci/develop_run_cli.py promotion >> "$GITHUB_OUTPUT"', deciding['run'])
        self.assertEqual(dict(JUDGEMENT_HANDED, LACKING='${{ steps.main.outputs.lacking }}',
                              MAIN_AHEAD='${{ steps.main.outputs.ahead }}',
                              WORKFLOWS_CHANGED='${{ steps.main.outputs.changed }}',
                              WORKFLOWS_DIFFERING='${{ steps.main.outputs.differing }}'), deciding['env'])

    def test_what_main_has_and_lacks_is_found_against_develop_and_the_judged_commit_as_the_checkout_fetched_them(self):
        self.assertEqual(
            'tip=$(git rev-parse refs/remotes/origin/main)\n'
            "lacking=$(git log --format='%H %s' refs/remotes/origin/develop..refs/remotes/origin/main | base64 -w0)\n"
            "ahead=$(git log --format='%H %s' HEAD..refs/remotes/origin/main | base64 -w0)\n"
            'changed=$(git log --full-history --no-renames --cc --format= --name-only refs/remotes/origin/main..HEAD '
            '-- .github/workflows | base64 -w0)\n'
            'differing=$(git diff --no-renames --name-only refs/remotes/origin/main HEAD -- .github/workflows '
            '| base64 -w0)\n'
            'echo "tip=$tip" >> "$GITHUB_OUTPUT"\n'
            'echo "lacking=$lacking" >> "$GITHUB_OUTPUT"\n'
            'echo "ahead=$ahead" >> "$GITHUB_OUTPUT"\n'
            'echo "changed=$changed" >> "$GITHUB_OUTPUT"\n'
            'echo "differing=$differing" >> "$GITHUB_OUTPUT"\n',
            step_named(self.jobs['look'], 'main')['run'])
        self.assertEqual('Find where main stands, what main has that develop or the judged commit lacks, and the '
                         'workflow files the commits since main change or main and the judged commit differ in',
                         step_named(self.jobs['look'], 'main')['name'])

    def test_the_job_that_looks_hands_on_what_it_found_of_main_and_what_the_core_decided(self):
        self.assertEqual({'main-tip': '${{ steps.main.outputs.tip }}',
                          'lacking': '${{ steps.main.outputs.lacking }}',
                          'main-ahead': '${{ steps.main.outputs.ahead }}',
                          'workflows-changed': '${{ steps.main.outputs.changed }}',
                          'workflows-differing': '${{ steps.main.outputs.differing }}',
                          'decision': '${{ steps.decide.outputs.decision }}'},
                         self.jobs['look']['outputs'])

    def test_the_mover_hands_on_how_its_push_went(self):
        self.assertEqual({'pushed': '${{ steps.push.outcome }}', 'git-said': '${{ steps.push.outputs.said }}'},
                         self.jobs['promote']['outputs'])

    def test_a_main_the_core_neither_moves_nor_leaves_where_it_already_is_fails_the_job_that_looks(self):
        failing = [step for step in steps_of(self.jobs['look']) if step.get('run') == 'exit 1']
        some_commit = run.Commit(run.CommitHash('a' * 40), 'A commit of develop')
        handed = [run.promotion_outputs(decided).split('\n')[0].split('=')[1] for decided in
                  (run.Move(some_commit), run.Stay(some_commit, (some_commit,)))]

        self.assertEqual([{'name': 'Fail unless main moves or stays, so that re-running the failed jobs looks at main '
                                   'again',
                           'if': "steps.decide.outputs.decision != '%s' && steps.decide.outputs.decision != '%s'"
                                 % tuple(handed),
                           'run': 'exit 1'}], failing)

    def test_the_mover_holds_main_red_whenever_the_judging_or_the_look_did_not_pass(self):
        self.assertEqual([{'name': 'Hold main where it was',
                           'if': "needs.unit-tests.result != 'success' || needs.look.result != 'success'",
                           'run': 'exit 1'}],
                         [step for step in steps_of(self.jobs['promote']) if step.get('run') == 'exit 1'])

    def test_no_run_merges_commits_or_rewrites_and_no_push_forces(self):
        for job in self.jobs.values():
            for each in runs_of(job):
                for never in ('git merge', 'git commit', 'git reset', 'git rebase', 'gh pr merge'):
                    self.assertIsNone(re.search(re.escape(never) + r'(?![-\w])', each), (never, each))
                for line in each.split('\n'):
                    if 'git push' in line:
                        for never in ('--force', ' -f ', '--mirror', '--delete', '--all', ' +', '"+', ':+'):
                            self.assertNotIn(never, line)


    def test_the_only_calls_any_run_makes_to_github_s_api_are_reads_of_develop_s_ref_and_main_s_and_one_compare_of_their_tips(self):
        self.assertEqual(['tip=$(%s)' % DEVELOP_READ, 'tip=$(%s)' % MAIN_READ] * 2 + ['status=$(%s)' % COMPARE],
                         [line for job in self.jobs.values() for each in runs_of(job) for line in each.split('\n')
                          if 'gh api' in line])


class WhatReachesThePushToMain(WorkflowTest):
    RESULTS = ('success', 'failure', 'cancelled', 'skipped', '')
    DECISIONS = ('move', 'stay', 'hold', '', 'anything else')
    MAIN_NOW = tuple(itertools.product((LATER_HASH, STRAY_HASH, JUDGED_HASH), ('true', 'false', '')))

    def ran(self, job, contexts, outputs=None, failing=()):
        done = steps_ran(job, dict(contexts, github={'event_name': 'push', 'ref': 'refs/heads/develop'}), outputs, failing)
        self.assertNotIsInstance(done, conditions.Rejected)
        return done

    def looked(self, judging, decision):
        return self.ran(self.jobs['look'], {'needs': {'unit-tests': {'result': judging}}},
                        {'decide': {'decision': decision}})

    def moved(self, judging, looking, decision, failing=(), now=(LATER_HASH, 'true')):
        tip, along = now
        return self.ran(self.jobs['promote'],
                        {'needs': {'unit-tests': {'result': judging, 'outputs': {'commit': JUDGED_HASH}},
                                   'look': {'result': looking, 'outputs': {'decision': decision,
                                                                           'main-tip': LATER_HASH}}}},
                        {'main': {'tip': tip, 'along': along}}, failing)

    def test_a_judging_that_did_not_pass_never_reaches_the_push_whatever_the_core_decided_or_wherever_main_is(self):
        for judging in self.RESULTS[1:]:
            for decision in self.DECISIONS:
                for now in self.MAIN_NOW:
                    _, looking = self.looked(judging, decision)
                    steps, ended = self.moved(judging, looking, decision, now=now)

                    self.assertEqual(('skipped', 'skipped', 'skipped', 'failure'),
                                     (steps['push'], steps[MOVED_ALONG_DEVELOP], steps[MOVED_OFF_DEVELOP], ended),
                                     (judging, decision, now))

    def test_a_look_at_main_that_did_not_pass_never_reaches_the_push_whatever_it_handed_on_or_wherever_main_is(self):
        for looking in self.RESULTS[1:]:
            for decision in self.DECISIONS:
                for now in self.MAIN_NOW:
                    steps, ended = self.moved('success', looking, decision, now=now)

                    self.assertEqual(('skipped', 'skipped', 'skipped', 'failure'),
                                     (steps['push'], steps[MOVED_ALONG_DEVELOP], steps[MOVED_OFF_DEVELOP], ended),
                                     (looking, decision, now))

    def test_the_job_that_looks_fails_unless_the_core_said_main_moves_or_stays_so_a_rerun_of_failed_jobs_looks_again(self):
        for decision in self.DECISIONS:
            _, looking = self.looked('success', decision)

            self.assertEqual('success' if decision in ('move', 'stay') else 'failure', looking, decision)

    def test_main_is_pushed_exactly_when_the_judging_passed_the_core_said_move_and_main_is_where_the_look_found_it(self):
        for judging in self.RESULTS:
            for decision in self.DECISIONS:
                for now in self.MAIN_NOW:
                    _, looking = self.looked(judging, decision)
                    steps, _ = self.moved(judging, looking, decision, now=now)

                    self.assertEqual('success' if (judging, decision, now[0]) == ('success', 'move', LATER_HASH)
                                     else 'skipped', steps['push'], (judging, decision, now))

    def test_the_mover_says_main_moved_along_develop_since_the_look_exactly_when_the_judging_and_the_look_passed_main_is_not_where_the_look_found_it_and_develop_holds_its_tip(self):
        for judging in self.RESULTS:
            for decision in self.DECISIONS:
                for now in self.MAIN_NOW:
                    _, looking = self.looked(judging, decision)
                    steps, _ = self.moved(judging, looking, decision, now=now)
                    said = (judging, looking) == ('success', 'success') and now[0] != LATER_HASH and now[1] == 'true'

                    self.assertEqual('success' if said else 'skipped', steps[MOVED_ALONG_DEVELOP],
                                     (judging, decision, now))

    def test_the_mover_fails_saying_main_moved_since_the_look_exactly_when_the_judging_and_the_look_passed_main_is_not_where_the_look_found_it_and_develop_is_not_found_to_hold_its_tip(self):
        for judging in self.RESULTS:
            for decision in self.DECISIONS:
                for now in self.MAIN_NOW:
                    _, looking = self.looked(judging, decision)
                    steps, _ = self.moved(judging, looking, decision, now=now)
                    said = (judging, looking) == ('success', 'success') and now[0] != LATER_HASH and now[1] != 'true'

                    self.assertEqual('failure' if said else 'skipped', steps[MOVED_OFF_DEVELOP],
                                     (judging, decision, now))

    def test_the_mover_ends_in_success_only_having_pushed_with_main_staying_where_the_core_said_it_stays_or_having_found_main_moved_along_develop_since_the_look(self):
        for judging in self.RESULTS:
            for decision in self.DECISIONS:
                for failing in ((), ('push',)):
                    for now in self.MAIN_NOW:
                        _, looking = self.looked(judging, decision)
                        steps, ended = self.moved(judging, looking, decision, failing, now)
                        stayed = (judging, decision, now[0]) == ('success', 'stay', LATER_HASH)
                        found_moved_along = ((judging, looking) == ('success', 'success') and now[0] != LATER_HASH
                                             and now[1] == 'true')

                        self.assertEqual('success' if steps['push'] == 'success' or stayed or found_moved_along
                                         else 'failure', ended, (judging, decision, failing, now))

    def test_a_mover_whose_read_of_main_failed_neither_pushes_nor_says_main_moved_and_ends_red(self):
        for decision in ('move', 'stay'):
            steps, ended = self.moved('success', 'success', decision, failing=('main',), now=('', ''))

            self.assertEqual(('failure', 'skipped', 'skipped', 'skipped', 'failure'),
                             (steps['main'], steps['push'], steps[MOVED_ALONG_DEVELOP], steps[MOVED_OFF_DEVELOP], ended),
                             decision)


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
        self.calls = os.path.join(self.where.name, 'calls')
        self.refuses_close = os.path.join(self.where.name, 'refuses-close')
        listed = os.path.join(self.where.name, 'listed.json')
        with open(listed, 'w', encoding='utf-8') as written:
            written.write('[{"number": 12, "title": "%s"}]' % issues.TITLE)
        self.on_runner = dict(self.environ, PATH=fake_gh(self.where.name), GH_CALLS=self.calls, GH_LISTED=listed,
                              GH_ORIGIN=self.origin, GH_REFUSES_CLOSE=self.refuses_close,
                              GITHUB_SERVER_URL='https://github.com', GITHUB_REPOSITORY='th7/midnight-snackers',
                              GITHUB_RUN_ID='123456')

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

    def ran(self, job, step_id, runner, **environ):
        outputs = os.path.join(runner, 'outputs-' + step_id)
        done = subprocess.run(['bash', '--noprofile', '--norc', '-eo', 'pipefail', '-c',
                               step_named(self.jobs[job], step_id)['run']], cwd=runner,
                              env=dict(self.on_runner, GITHUB_OUTPUT=outputs, RUNNER_TEMP=runner, **environ),
                              capture_output=True, text=True, timeout=60)
        self.assertEqual(0, done.returncode, done.stdout + done.stderr)
        return written_outputs(outputs)

    def tip_read(self, branch):
        return self.ran('say', branch, tempfile.mkdtemp(prefix='temp-', dir=self.where.name))['tip']

    def judging(self, judged_commit, subject, failing=()):
        outcomes = {step.step: 'failure' if step.step in failing else 'success' for step in run.STEPS}
        return {'result': 'failure' if failing else 'success',
                'outputs': dict(outcomes, commit=judged_commit,
                                subject=base64.b64encode((subject + '\n').encode()).decode())}

    def checked_out(self, judged_commit):
        runner = tempfile.mkdtemp(prefix='runner-', dir=self.where.name)
        self.git(self.where.name, 'clone', '-q', self.origin, runner)
        self.git(runner, 'checkout', '-q', '--detach', judged_commit)
        os.symlink(os.path.join(REPO, 'tools'), os.path.join(runner, 'tools'))
        return runner

    def job(self, name, cwd, ended):
        job = self.jobs[name]
        needs = {needed: ended[needed] for needed in ([job['needs']] if isinstance(job['needs'], str) else job['needs'])}
        contexts = {'needs': needs, 'github': {'event_name': 'push', 'ref': 'refs/heads/develop', 'token': 'a-token'}}
        done = played(job, contexts, cwd, self.on_runner, tempfile.mkdtemp(prefix='temp-', dir=self.where.name))
        self.assertNotIsInstance(done, conditions.Rejected)
        return dict(done[0], steps={key: step['outcome'] for key, step in done[1].items()})

    def jobs_ran(self, names, runner, ended):
        for name in names:
            if name != 'report':
                self.git(runner, 'fetch', '-q', '--prune', 'origin')
            ended[name] = self.job(name, tempfile.mkdtemp(prefix='empty-', dir=self.where.name) if name == 'report'
                                   else runner, ended)
        return ended

    def no_issue_open(self):
        with open(self.on_runner['GH_LISTED'], 'w', encoding='utf-8') as written:
            written.write('[]')

    def issue_writes(self, calls):
        return [[os.path.basename(each) if call[index - 1:index] == ['--body-file'] else each
                 for index, each in enumerate(call)]
                for call in calls if call[:2] in (['issue', 'create'], ['issue', 'comment'], ['issue', 'close'])]

    def fallback_of(self, reported):
        return reported['steps'][key_of(steps_of(self.jobs['report'])[-1])]

    def a_run(self, judged_commit, subject, failing=()):
        runner = self.checked_out(judged_commit)
        return self.jobs_ran(('look', 'promote', 'say', 'report'), runner,
                             {'unit-tests': self.judging(judged_commit, subject, failing)}), runner

    def gh_calls(self):
        if not os.path.exists(self.calls):
            return []
        with open(self.calls, encoding='utf-8') as written:
            calls = [json.loads(line) for line in written.read().split('\n') if line]
        os.remove(self.calls)
        return calls

    def where_main_and_develop_stand(self):
        return tuple(self.git(self.origin, 'rev-parse', 'refs/heads/' + branch) for branch in ('main', 'develop'))

    def mover_found(self, judged_commit):
        runner = tempfile.mkdtemp(prefix='runner-', dir=self.where.name)
        self.git(self.where.name, 'clone', '-q', self.origin, runner)
        self.git(runner, 'checkout', '-q', '--detach', judged_commit)
        return runner, self.ran('look', 'main', runner)

    def judgement_of(self, judged_commit, subject):
        steps = tuple('success' for _ in run.STEPS)
        return run.judgement('success', judged_commit, base64.b64encode((subject + '\n').encode()).decode(), steps)

    def decided_for(self, judged_commit, subject):
        _, found = self.mover_found(judged_commit)
        return run.promotion(self.judgement_of(judged_commit, subject), run.listed_commits(found['lacking']),
                             run.listed_commits(found['ahead']), run.workflows(found['changed'], found['differing']))

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

        both = run.WorkflowFiles(('.github/workflows/release.yml', '.github/workflows/tests.yml'))
        self.assertEqual(run.Hold((run.MainNeedsAMaintainerForWorkflows(
            run.Commit(run.CommitHash(judged_commit), 'Release from main'), run.Workflows(both, both)),)), decided)

    def test_a_workflow_change_a_later_commit_undoes_still_holds_main_naming_it(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Try a nightly run', {'.github/workflows/nightly.yml': 'name: nightly\n'})
        judged_commit = self.commit('No nightly run after all', removed=('.github/workflows/nightly.yml',))
        self.push('develop')

        decided = self.decided_for(judged_commit, 'No nightly run after all')

        self.assertEqual(run.Workflows(run.WorkflowFiles(('.github/workflows/nightly.yml',)), run.WorkflowFiles(())),
                         decided.findings[0].workflows)

    def test_a_workflow_change_main_already_has_does_not_hold_main(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Run the tool tests too', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('develop', 'develop:main')
        judged_commit = self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        self.push('develop')

        decided = self.decided_for(judged_commit, 'Say how to build')

        self.assertEqual(run.Move(run.Commit(run.CommitHash(judged_commit), 'Say how to build')), decided)

    def a_workflow_on_main_and_develop(self):
        self.commit('Run the tests', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('main', 'main:develop')

    def test_a_merge_whose_own_resolution_changes_a_workflow_holds_main_naming_it(self):
        self.a_workflow_on_main_and_develop()
        self.git(self.work, 'checkout', '-q', '-b', 'side')
        self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        self.on('develop')
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.git(self.work, 'merge', '-q', '--no-commit', 'side')
        judged_commit = self.commit('Merge side', {'.github/workflows/tests.yml': 'name: every test\n'})
        self.push('develop')

        decided = self.decided_for(judged_commit, 'Merge side')

        self.assertEqual(run.Hold((run.MainNeedsAMaintainerForWorkflows(
            run.Commit(run.CommitHash(judged_commit), 'Merge side'), run.Workflows(TESTS_YML, TESTS_YML)),)), decided)

    def held_naming_tests_yml(self, judged_commit, subject, changed=None, differing=None):
        self.assertEqual(run.Hold((run.MainNeedsAMaintainerForWorkflows(
            run.Commit(run.CommitHash(judged_commit), subject),
            run.Workflows(changed or TESTS_YML, differing or run.WorkflowFiles(()))),)),
            self.decided_for(judged_commit, subject))

    def test_a_workflow_change_a_merged_branch_undoes_on_itself_still_holds_main_naming_it(self):
        self.a_workflow_on_main_and_develop()
        self.git(self.work, 'checkout', '-q', '-b', 'side')
        self.commit('Try a schedule', {'.github/workflows/tests.yml': 'name: tests\non: schedule\n'})
        self.commit('No schedule after all', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.on('develop')
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.git(self.work, 'merge', '-q', '--no-ff', '-m', 'Merge side', 'side')
        judged_commit = self.git(self.work, 'rev-parse', 'HEAD')
        self.push('develop')

        self.held_naming_tests_yml(judged_commit, 'Merge side')

    def test_a_workflow_change_a_merge_leaves_out_still_holds_main_naming_it(self):
        self.a_workflow_on_main_and_develop()
        self.git(self.work, 'checkout', '-q', '-b', 'side')
        self.commit('Run every test', {'.github/workflows/tests.yml': 'name: every test\n'})
        self.on('develop')
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.git(self.work, 'merge', '-q', '--no-commit', 'side')
        judged_commit = self.commit('Merge side', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('develop')

        self.held_naming_tests_yml(judged_commit, 'Merge side')

    def test_a_workflow_change_one_merge_makes_in_resolving_it_and_a_later_merge_undoes_still_holds_main(self):
        self.a_workflow_on_main_and_develop()
        self.git(self.work, 'checkout', '-q', '-b', 'side1')
        self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        self.on('develop')
        self.git(self.work, 'checkout', '-q', '-b', 'side2')
        self.commit('Draw the field', {'field.txt': 'Twelve feet square.\n'})
        self.on('develop')
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.git(self.work, 'merge', '-q', '--no-commit', 'side1')
        self.commit('Merge side1', {'.github/workflows/tests.yml': 'name: every test\n'})
        self.git(self.work, 'merge', '-q', '--no-commit', 'side2')
        judged_commit = self.commit('Merge side2', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('develop')

        self.held_naming_tests_yml(judged_commit, 'Merge side2')

    def test_a_branch_that_pulled_a_workflow_change_main_already_has_does_not_hold_main_when_merged(self):
        self.a_workflow_on_main_and_develop()
        self.git(self.work, 'checkout', '-q', '-b', 'mine')
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.on('develop')
        self.commit('Run every test', {'.github/workflows/tests.yml': 'name: every test\n'})
        self.push('develop', 'develop:main')
        self.git(self.work, 'checkout', '-q', 'mine')
        self.git(self.work, 'merge', '-q', '--no-edit', 'develop')
        self.on('develop')
        self.git(self.work, 'merge', '-q', '--no-ff', '-m', 'Merge mine', 'mine')
        judged_commit = self.git(self.work, 'rev-parse', 'HEAD')
        self.push('develop')

        self.assertEqual(run.Move(run.Commit(run.CommitHash(judged_commit), 'Merge mine')),
                         self.decided_for(judged_commit, 'Merge mine'))

    def test_a_renamed_workflow_holds_main_naming_it_by_its_old_name_and_its_new(self):
        self.a_workflow_on_main_and_develop()
        self.on('develop')
        self.git(self.work, 'mv', '.github/workflows/tests.yml', '.github/workflows/checks.yml')
        judged_commit = self.commit('Call the workflow checks')
        self.push('develop')

        decided = self.decided_for(judged_commit, 'Call the workflow checks')

        both = run.WorkflowFiles(('.github/workflows/checks.yml', '.github/workflows/tests.yml'))
        self.assertEqual(run.Workflows(both, both), decided.findings[0].workflows)

    def merged_main_keeping_an_older_workflow(self):
        self.a_workflow_on_main_and_develop()
        self.commit('Fix the workflow on main', {'.github/workflows/tests.yml': 'name: fixed tests\n'})
        self.push('main')
        self.on('develop')
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.git(self.work, 'merge', '-q', '--no-commit', 'origin/main')
        judged_commit = self.commit('Merge main', {'.github/workflows/tests.yml': 'name: tests\n'})
        self.push('develop')
        return judged_commit

    def test_a_merge_that_keeps_an_older_workflow_than_main_s_holds_main_naming_it_as_differing_from_main_s_alone(self):
        judged_commit = self.merged_main_keeping_an_older_workflow()

        decided = self.decided_for(judged_commit, 'Merge main')

        self.assertEqual(run.Hold((run.MainNeedsAMaintainerForWorkflows(
            run.Commit(run.CommitHash(judged_commit), 'Merge main'), run.Workflows(run.WorkflowFiles(()), TESTS_YML)),)),
            decided)
        said = run.reasons(decided.findings)
        self.assertIn("The judged commit's copy of 1 workflow file differs from main's though no commit from main to `%s` "
                      "changes it" % judged_commit, said)
        self.assertIn("so a push of `%s` replaces main's copy with the judged commit's" % judged_commit, said)
        self.assertNotIn('The commits from main to', said)

    def test_a_later_commit_that_leaves_a_workflow_kept_older_than_main_s_alone_still_holds_main(self):
        self.merged_main_keeping_an_older_workflow()
        judged_commit = self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        self.push('develop')

        self.held_naming_tests_yml(judged_commit, 'Say how to build', changed=run.WorkflowFiles(()), differing=TESTS_YML)

    def test_a_later_commit_that_gives_develop_main_s_copy_of_a_workflow_back_still_holds_main(self):
        self.merged_main_keeping_an_older_workflow()
        judged_commit = self.commit('Keep the fixed workflow', {'.github/workflows/tests.yml': 'name: fixed tests\n'})
        self.push('develop')

        self.held_naming_tests_yml(judged_commit, 'Keep the fixed workflow')

    def test_a_later_commit_that_leaves_workflows_the_commits_since_main_change_alone_still_holds_main(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Run every test', {'.github/workflows/tests.yml': 'name: every test\n'})
        judged_commit = self.commit('Say how to build', {'README.md': 'Build with Gradle.\n'})
        self.push('develop')

        self.held_naming_tests_yml(judged_commit, 'Say how to build', differing=TESTS_YML)

    def held_for_a_maintainer(self, subject):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        judged_commit = self.commit(subject, {'.github/workflows/tests.yml': 'name: every test\n'})
        self.push('develop')
        first, runner = self.a_run(judged_commit, subject)
        self.assertEqual(('hold', 'failure', 'failure', 'comment'),
                         (first['look']['outputs']['decision'], first['look']['result'], first['promote']['result'],
                          first['say']['outputs']['action']))
        return judged_commit, first, runner

    def test_a_rerun_of_the_failed_jobs_after_a_maintainer_pushed_main_to_the_held_commit_moves_it_nowhere_and_closes_the_issue(self):
        judged_commit, first, runner = self.held_for_a_maintainer('Run every test')
        self.push('%s:refs/heads/main' % judged_commit)
        self.gh_calls()

        rerun = self.jobs_ran(('look', 'promote', 'say', 'report'), runner, {'unit-tests': first['unit-tests']})
        calls = self.gh_calls()

        self.assertEqual((judged_commit, judged_commit), self.where_main_and_develop_stand())
        self.assertEqual(('move', 'success', 'close'), (rerun['look']['outputs']['decision'],
                                                        rerun['promote']['outputs']['pushed'],
                                                        rerun['say']['outputs']['action']))
        self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file', 'issue-body.md', '--', '12',
                           'develop was judged green at `%s` `Run every test`, and main now stands there.\n\nThe run: '
                           'https://github.com/th7/midnight-snackers/actions/runs/123456\n' % judged_commit],
                          ['issue', 'close', '--repo', 'th7/midnight-snackers', '--', '12']],
                         self.issue_writes(calls))
        self.assertEqual('success', rerun['report']['result'])

    def test_a_rerun_of_the_failed_jobs_after_a_maintainer_pushed_main_to_the_held_commit_says_main_already_stood_there_once_develop_moved_on(self):
        judged_commit, first, runner = self.held_for_a_maintainer('Run every test')
        broken = self.commit('Break the drive code', {'drive.txt': 'broken\n'})
        self.push('develop')
        self.push('%s:refs/heads/main' % judged_commit)
        self.gh_calls()

        rerun = self.jobs_ran(('look', 'promote', 'say', 'report'), runner, {'unit-tests': first['unit-tests']})
        calls = self.gh_calls()

        self.assertEqual((judged_commit, broken), self.where_main_and_develop_stand())
        self.assertEqual(('move', 'success', 'moved-on'), (rerun['look']['outputs']['decision'],
                                                          rerun['promote']['outputs']['pushed'],
                                                          rerun['say']['outputs']['action']))
        self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file', 'issue-body.md', '--', '12',
                           "develop was judged green at `%s` `Run every test`, and main already stood there when this run "
                           "looked at it, but develop's tip is now `%s`, which this run did not judge, and a run closes "
                           "this issue only when it finds develop's tip and main's both at the commit it judged, so this "
                           'run does not close it.\n\nThe run: https://github.com/th7/midnight-snackers/actions/runs/'
                           '123456\n' % (judged_commit, broken)]],
                         self.issue_writes(calls))
        self.assertEqual('success', rerun['report']['result'])

    def test_a_rerun_of_the_failed_jobs_of_a_run_whose_push_was_refused_once_pushes_and_writes_nothing_after_a_later_green_run_moved_main_and_closed_the_issue(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        hook = os.path.join(self.origin, 'hooks', 'pre-receive')
        with open(hook, 'w', encoding='utf-8') as refusing:
            refusing.write('#!/bin/sh\necho "502 try again later" >&2\nexit 1\n')
        os.chmod(hook, 0o755)
        first, runner = self.a_run(tuned, 'Tune the arm')
        os.remove(hook)
        self.assertEqual(('move', 'failure', 'comment'),
                         (first['look']['outputs']['decision'], first['promote']['result'], first['say']['outputs']['action']))
        named = self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.push('develop')
        later, _ = self.a_run(named, 'Name the robot')
        self.assertEqual(('close', 'success'), (later['say']['outputs']['action'], later['report']['result']))
        self.assertEqual((named, named), self.where_main_and_develop_stand())
        self.no_issue_open()
        self.gh_calls()

        rerun = self.jobs_ran(('promote', 'say', 'report'), runner,
                              {'unit-tests': first['unit-tests'], 'look': first['look']})
        calls = self.gh_calls()

        self.assertEqual((named, named), self.where_main_and_develop_stand())
        self.assertEqual(('skipped', 'none'), (rerun['promote']['outputs']['pushed'], rerun['say']['outputs']['action']))
        self.assertEqual(('success', 'skipped'), (rerun['promote']['steps'][MOVED_ALONG_DEVELOP],
                                                  rerun['promote']['steps'][MOVED_OFF_DEVELOP]))
        self.assertEqual([], self.issue_writes(calls))
        self.assertEqual({'success'}, {rerun[name]['result'] for name in ('unit-tests', 'look', 'promote', 'say', 'report')})
        self.assertNotIn('failure', rerun['report']['steps'].values())
        self.assertEqual('skipped', self.fallback_of(rerun['report']))

    def test_a_rerun_of_the_failed_jobs_after_main_refused_a_push_once_never_closes_an_issue_a_later_red_tip_holds_open(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        hook = os.path.join(self.origin, 'hooks', 'pre-receive')
        with open(hook, 'w', encoding='utf-8') as refusing:
            refusing.write('#!/bin/sh\necho "502 try again later" >&2\nexit 1\n')
        os.chmod(hook, 0o755)
        first, runner = self.a_run(tuned, 'Tune the arm')
        os.remove(hook)
        self.assertEqual(('move', 'failure', 'comment'),
                         (first['look']['outputs']['decision'], first['promote']['result'], first['say']['outputs']['action']))
        broken = self.commit('Break the drive code', {'drive.txt': 'broken\n'})
        self.push('develop')
        later, _ = self.a_run(broken, 'Break the drive code', failing=(run.UNIT_TESTS.step,))
        self.assertEqual('comment', later['say']['outputs']['action'])
        self.gh_calls()

        rerun = self.jobs_ran(('promote', 'say', 'report'), runner,
                              {'unit-tests': first['unit-tests'], 'look': first['look']})
        calls = self.gh_calls()

        self.assertEqual((tuned, broken), self.where_main_and_develop_stand())
        self.assertEqual(('success', 'moved-on'), (rerun['promote']['result'], rerun['say']['outputs']['action']))
        self.assertEqual([], [call for call in calls if call[:2] == ['issue', 'close']])
        commented = [call for call in calls if call[:2] == ['issue', 'comment']]
        self.assertEqual(1, len(commented), calls)
        self.assertIn("develop's tip is now `%s`" % broken, commented[0][-1])

    def test_a_rerun_of_the_job_that_writes_the_issue_alone_never_closes_an_issue_a_later_red_tip_holds_open(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        open(self.refuses_close, 'w', encoding='utf-8').close()
        first, _ = self.a_run(tuned, 'Tune the arm')
        os.remove(self.refuses_close)
        self.assertEqual(('success', 'close', 'failure'), (first['promote']['result'], first['say']['outputs']['action'],
                                                           first['report']['result']))
        broken = self.commit('Break the drive code', {'drive.txt': 'broken\n'})
        self.push('develop')
        later, _ = self.a_run(broken, 'Break the drive code', failing=(run.UNIT_TESTS.step,))
        self.assertEqual('comment', later['say']['outputs']['action'])
        self.gh_calls()

        reported = self.job('report', tempfile.mkdtemp(prefix='empty-', dir=self.where.name), first)
        calls = self.gh_calls()

        self.assertEqual((tuned, broken), self.where_main_and_develop_stand())
        self.assertEqual([], [call for call in calls if call[:2] == ['issue', 'close']])
        commented = [call for call in calls if call[:2] == ['issue', 'comment']]
        self.assertEqual(1, len(commented), calls)
        self.assertEqual(['--', '12'], commented[0][-2:])
        self.assertIn("develop's tip is now %s" % broken, commented[0][commented[0].index('--body') + 1])
        self.assertEqual('success', reported['result'])

    def stray_on_main(self):
        self.git(self.work, 'fetch', '-q', 'origin')
        self.git(self.work, 'checkout', '-q', '-B', 'main', 'origin/main')
        stray = self.commit('Fix the README on main', {'README.md': 'fixed\n'})
        self.push('main')
        self.git(self.work, 'checkout', '-q', 'develop')
        return stray

    def test_a_rerun_of_the_job_that_writes_the_issue_alone_after_a_commit_reached_main_by_itself_never_closes_the_issue_and_says_on_it_that_the_run_is_red(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        open(self.refuses_close, 'w', encoding='utf-8').close()
        first, _ = self.a_run(tuned, 'Tune the arm')
        os.remove(self.refuses_close)
        self.assertEqual(('success', 'close', 'failure'), (first['promote']['result'], first['say']['outputs']['action'],
                                                           first['report']['result']))
        stray = self.stray_on_main()
        later, _ = self.a_run(tuned, 'Tune the arm')
        self.assertEqual(('hold', 'comment'), (later['look']['outputs']['decision'], later['say']['outputs']['action']))
        self.gh_calls()

        reported = self.job('report', tempfile.mkdtemp(prefix='empty-', dir=self.where.name), first)
        calls = self.gh_calls()

        self.assertEqual((stray, tuned), self.where_main_and_develop_stand())
        self.assertEqual(([self.red_comment_on_12()], 'failure', 'failure', 'skipped', 'skipped', 'success'),
                         (self.issue_writes(calls), reported['result'], reported['steps'].get('holds', 'absent'),
                          reported['steps']['close'], reported['steps']['moved'], self.fallback_of(reported)))

    def test_a_rerun_of_the_jobs_that_say_and_write_after_a_commit_reached_main_by_itself_turns_the_run_red_saying_so_on_the_issue(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        self.on_runner['GH_API_FAILS'] = '1'
        first, runner = self.a_run(tuned, 'Tune the arm')
        del self.on_runner['GH_API_FAILS']
        self.assertEqual(('success', 'success', 'failure'), (first['look']['result'], first['promote']['result'],
                                                             first['say']['result']))
        stray = self.stray_on_main()
        self.gh_calls()

        rerun = self.jobs_ran(('say', 'report'), runner,
                              {'unit-tests': first['unit-tests'], 'look': first['look'], 'promote': first['promote']})
        calls = self.gh_calls()

        self.assertEqual((stray, tuned), self.where_main_and_develop_stand())
        self.assertEqual(('success', 'none'), (rerun['say']['result'], rerun['say']['outputs']['action']))
        self.assertEqual(([self.red_comment_on_12()], 'failure', 'failure', 'success'),
                         (self.issue_writes(calls), rerun['report']['result'],
                          rerun['report']['steps'].get('holds', 'absent'), self.fallback_of(rerun['report'])))

    def test_a_commit_that_reaches_main_by_itself_after_the_look_and_before_the_push_turns_the_run_red_saying_so_on_the_issue(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        runner = self.checked_out(tuned)
        looked = self.jobs_ran(('look',), runner, {'unit-tests': self.judging(tuned, 'Tune the arm')})
        stray = self.stray_on_main()
        self.gh_calls()

        ended = self.jobs_ran(('promote', 'say', 'report'), runner, looked)
        calls = self.gh_calls()

        self.assertEqual((stray, tuned), self.where_main_and_develop_stand())
        self.assertEqual(('move', 'skipped', 'failure'), (ended['look']['outputs']['decision'],
                                                          ended['promote']['outputs']['pushed'],
                                                          ended['promote']['result']))
        self.assertEqual(('skipped', 'failure'), (ended['promote']['steps'][MOVED_ALONG_DEVELOP],
                                                  ended['promote']['steps'][MOVED_OFF_DEVELOP]))
        self.assertEqual(([self.red_comment_on_12()], 'failure', 'failure', 'success'),
                         (self.issue_writes(calls), ended['report']['result'],
                          ended['report']['steps'].get('holds', 'absent'), self.fallback_of(ended['report'])))

    def develop_moved_on_before_the_job_that_writes_the_issue(self, strayed):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        runner = self.checked_out(tuned)
        ended = self.jobs_ran(('look', 'promote'), runner, {'unit-tests': self.judging(tuned, 'Tune the arm')})
        named = self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.push('develop')
        ended = self.jobs_ran(('say',), runner, ended)
        main = self.stray_on_main() if strayed else tuned
        self.assertEqual((main, named), self.where_main_and_develop_stand())
        self.assertEqual(('move', 'success', 'success', 'success'),
                         (ended['look']['outputs']['decision'], ended['promote']['outputs']['pushed'],
                          ended['promote']['result'], ended['say']['result']))
        self.gh_calls()
        return ended, runner

    def reported_after_develop_moved_on(self, ended, runner):
        reported = self.jobs_ran(('report',), runner, ended)['report']
        return (self.issue_writes(self.gh_calls()), ended['say']['outputs']['action'], reported['result'],
                tuple(reported['steps'].get(key, 'absent') for key in ('develop', 'holds', 'comment', 'moved-on'))
                + (self.fallback_of(reported),))

    def test_a_green_run_whose_develop_moved_on_before_the_job_that_says_read_it_says_so_on_the_issue_and_ends_green(self):
        ended, runner = self.develop_moved_on_before_the_job_that_writes_the_issue(strayed=False)
        tuned, named = self.where_main_and_develop_stand()

        self.assertEqual(([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file', 'issue-body.md', '--',
                            '12', "develop was judged green at `%s` `Tune the arm`, and this run moved main there, but "
                                  "develop's tip is now `%s`, which this run did not judge, and a run closes this issue "
                                  "only when it finds develop's tip and main's both at the commit it judged, so this run "
                                  'does not close it.\n\nThe run: https://github.com/th7/midnight-snackers/actions/runs/'
                                  '123456\n' % (tuned, named)]],
                          'moved-on', 'success', ('success', 'success', 'skipped', 'success', 'skipped')),
                         self.reported_after_develop_moved_on(ended, runner))

    def test_a_commit_that_reaches_main_by_itself_after_the_job_that_says_found_develop_moved_on_turns_the_run_red_saying_only_that_on_the_issue(self):
        ended, runner = self.develop_moved_on_before_the_job_that_writes_the_issue(strayed=True)

        self.assertEqual(([self.red_comment_on_12()], 'moved-on', 'failure',
                          ('success', 'failure', 'skipped', 'skipped', 'success')),
                         self.reported_after_develop_moved_on(ended, runner))

    def red_when_the_job_that_writes_the_issue_cannot_read_a_tip_after_develop_moved_on(self, strayed):
        ended, runner = self.develop_moved_on_before_the_job_that_writes_the_issue(strayed)
        self.on_runner['GH_API_FAILS'] = '1'
        reported = self.reported_after_develop_moved_on(ended, runner)
        del self.on_runner['GH_API_FAILS']

        self.assertEqual(([self.red_comment_on_12()], 'moved-on', 'failure',
                          ('failure', 'skipped', 'skipped', 'skipped', 'success')), reported)

    def test_a_job_that_writes_the_issue_and_cannot_read_develop_s_tip_after_the_job_that_says_found_develop_moved_on_turns_the_run_red_saying_only_that_on_the_issue(self):
        self.red_when_the_job_that_writes_the_issue_cannot_read_a_tip_after_develop_moved_on(strayed=False)

    def test_a_job_that_writes_the_issue_and_cannot_read_develop_s_tip_after_develop_moved_on_and_a_commit_reached_main_by_itself_turns_the_run_red_saying_only_that_on_the_issue(self):
        self.red_when_the_job_that_writes_the_issue_cannot_read_a_tip_after_develop_moved_on(strayed=True)

    def red_comment_on_12(self):
        return ['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body', FELL_BACK, '--', '12']

    def red_issue_opened(self):
        return ['issue', 'create', '--repo', 'th7/midnight-snackers', '--title', issues.TITLE, '--body', FELL_BACK]

    def strayed_during_a_run(self, after_the_stray):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        runner = self.checked_out(tuned)
        self.gh_calls()
        ended = self.jobs_ran(tuple(name for name in ('look', 'promote', 'say', 'report') if name not in after_the_stray),
                              runner, {'unit-tests': self.judging(tuned, 'Tune the arm')})
        stray = self.stray_on_main()
        ended = self.jobs_ran(after_the_stray, runner, ended)
        calls = self.gh_calls()
        self.assertEqual((stray, tuned), self.where_main_and_develop_stand())
        self.assertEqual(('move', 'success', 'success'),
                         (ended['look']['outputs']['decision'], ended['promote']['outputs']['pushed'],
                          ended['promote']['result']))
        self.assertEqual({'success'}, {ended[name]['result'] for name in ('unit-tests', 'look', 'promote', 'say')})
        return (ended['say']['outputs']['action'], self.issue_writes(calls), ended['report']['result'],
                tuple(ended['report']['steps'].get(key, 'absent') for key in ('holds', 'close', 'behind', 'moved'))
                + (self.fallback_of(ended['report']),))

    def red_by_the_check(self, said, written):
        return said, [written], 'failure', ('failure', 'skipped', 'skipped', 'skipped', 'success')

    def test_a_commit_that_reaches_main_by_itself_after_the_push_and_before_the_job_that_says_reads_main_turns_the_run_red_opening_the_issue_when_none_is_open(self):
        self.no_issue_open()

        ended = self.strayed_during_a_run(('say', 'report'))

        self.assertEqual(self.red_by_the_check('none', self.red_issue_opened()), ended)

    def test_a_commit_that_reaches_main_by_itself_after_the_push_and_before_the_job_that_says_reads_main_turns_the_run_red_commenting_on_the_open_issue(self):
        ended = self.strayed_during_a_run(('say', 'report'))

        self.assertEqual(self.red_by_the_check('none', self.red_comment_on_12()), ended)

    def test_a_commit_that_reaches_main_by_itself_after_the_job_that_says_read_main_and_before_the_job_that_writes_the_issue_turns_the_run_red_opening_the_issue_when_none_is_open(self):
        self.no_issue_open()

        ended = self.strayed_during_a_run(('report',))

        self.assertEqual(self.red_by_the_check('none', self.red_issue_opened()), ended)

    def test_a_commit_that_reaches_main_by_itself_after_the_job_that_says_read_main_and_before_the_job_that_writes_the_issue_turns_the_run_red_commenting_on_the_issue_it_would_have_closed(self):
        ended = self.strayed_during_a_run(('report',))

        self.assertEqual(self.red_by_the_check('close', self.red_comment_on_12()), ended)

    def test_a_commit_that_reaches_main_by_itself_after_a_run_read_main_for_the_last_time_turns_the_next_run_red_naming_it_on_the_issue(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        first, _ = self.a_run(tuned, 'Tune the arm')
        self.assertEqual(('close', 'success'), (first['say']['outputs']['action'], first['report']['result']))
        stray = self.stray_on_main()
        named = self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.push('develop')
        self.gh_calls()

        later, _ = self.a_run(named, 'Name the robot')
        calls = self.gh_calls()

        self.assertEqual((stray, named), self.where_main_and_develop_stand())
        self.assertEqual(('hold', 'failure', 'failure', 'comment'),
                         (later['look']['outputs']['decision'], later['look']['result'], later['promote']['result'],
                          later['say']['outputs']['action']))
        written = self.issue_writes(calls)
        self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file', 'issue-body.md', '--',
                           '12']], [call[:-1] for call in written])
        self.assertIn('main has 1 commit develop lacks, and every run is red until develop holds it:\n\n```\n%s Fix the '
                      'README on main\n```' % stray, written[0][-1])

    def test_a_green_run_moves_main_to_the_judged_commit_and_closes_the_issue(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        self.gh_calls()

        ended, _ = self.a_run(tuned, 'Tune the arm')
        calls = self.gh_calls()

        self.assertEqual((tuned, tuned), self.where_main_and_develop_stand())
        self.assertEqual(('move', 'success', 'close'), (ended['look']['outputs']['decision'],
                                                        ended['promote']['outputs']['pushed'],
                                                        ended['say']['outputs']['action']))
        self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file', 'issue-body.md', '--', '12',
                           'develop was judged green at `%s` `Tune the arm`, and main now stands there.\n\nThe run: '
                           'https://github.com/th7/midnight-snackers/actions/runs/123456\n' % tuned],
                          ['issue', 'close', '--repo', 'th7/midnight-snackers', '--', '12']],
                         self.issue_writes(calls))
        self.assertEqual({'success'}, {ended[name]['result'] for name in ('unit-tests', 'look', 'promote', 'say', 'report')})

    def compared_now(self):
        done = subprocess.run(['gh', 'api', '--method', 'GET', 'repos/th7/midnight-snackers/compare/%s...%s'
                               % self.where_main_and_develop_stand(), '--jq', '.status'],
                              env=self.on_runner, capture_output=True, text=True, timeout=60)
        return done.stdout if done.returncode == 0 else done.stderr

    def checked_now(self):
        main, _ = self.where_main_and_develop_stand()
        green = {'result': 'success', 'outputs': {}}
        reported = self.job('report', tempfile.mkdtemp(prefix='empty-', dir=self.where.name),
                            {'unit-tests': self.judging(main, 'Where main stands'), 'look': green, 'promote': green,
                             'say': {'result': 'success', 'outputs': {'action': 'none'}}})
        return reported['steps'].get('holds', 'absent'), reported['result']

    def test_the_check_that_develop_holds_main_s_tip_passes_exactly_when_github_compares_main_with_develop_as_ahead_or_identical(self):
        found = [(self.compared_now(), self.checked_now())]
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        found.append((self.compared_now(), self.checked_now()))
        self.push('develop:main')
        self.stray_on_main()
        found.append((self.compared_now(), self.checked_now()))
        self.commit('Name the robot', {'robot.txt': 'Reginald\n'})
        self.push('develop')
        found.append((self.compared_now(), self.checked_now()))
        self.git(self.work, 'checkout', '-q', '--orphan', 'elsewhere')
        self.commit('Start again')
        self.push('+elsewhere:develop')
        found.append((self.compared_now(), self.checked_now()))

        passed, red = ('success', 'success'), ('failure', 'failure')
        self.assertEqual([('identical\n', passed), ('ahead\n', passed), ('behind\n', red), ('diverged\n', red),
                          ('HTTP 404: Not Found\n', red)], found,
                         'GitHub compares base...head as identical, ahead when head has commits base lacks and base '
                         'none head lacks, behind the other way round, diverged when each has commits the other lacks, '
                         'and answers 404 when they share no history (%s)' % COMPARE_STATUS)

    def mover_read(self, judged_commit):
        return self.ran('promote', 'main', self.checked_out(judged_commit))

    def test_the_mover_finds_main_along_develop_exactly_when_develop_holds_main_s_tip_as_its_checkout_fetched_them(self):
        start = self.git(self.work, 'rev-parse', 'HEAD')
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop')
        behind = self.mover_read(tuned)
        self.push('develop:main')
        level = self.mover_read(tuned)
        stray = self.stray_on_main()
        off = self.mover_read(tuned)

        self.assertEqual([{'tip': start, 'along': 'true'}, {'tip': tuned, 'along': 'true'},
                          {'tip': stray, 'along': 'false'}], [behind, level, off])

    def test_a_mover_that_cannot_tell_whether_develop_holds_main_s_tip_finds_main_off_develop(self):
        self.git(self.work, 'checkout', '-q', '-b', 'develop')
        tuned = self.commit('Tune the arm', {'arm.txt': 'tuned\n'})
        self.push('develop', 'develop:main')
        runner = self.checked_out(tuned)
        self.git(runner, 'update-ref', '-d', 'refs/remotes/origin/develop')

        self.assertEqual({'tip': tuned, 'along': 'false'}, self.ran('promote', 'main', runner))

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
    def __init__(self):
        super().__init__(('127.0.0.1', 0), Refusing)
        self.heard = []


class Refusing(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.server.heard.append(self.headers.get_all('Authorization') or [])
        said = b'Permission to th7/midnight-snackers.git denied to a-user.\n'
        self.send_response(403)
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

    def pushed(self):
        server = ARefusingServer()
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
                env=dict(self.environ, GITHUB_OUTPUT=outputs, RUNNER_TEMP=runner, JUDGED=self.judged))
            return done, server.heard
        finally:
            server.shutdown()
            server.server_close()

    def test_the_push_sends_the_built_in_token_the_checkout_kept_and_no_other(self):
        done, heard = self.pushed()

        self.assertNotEqual(0, done.returncode, done.stdout)
        self.assertTrue(heard, done.stdout)
        self.assertEqual([[basic(self.BUILT_IN)]] * len(heard), heard,
                         'the checkout keeps the built-in token as the http.extraheader of the server it fetched from '
                         '(%s), which a plain push sends' % CHECKOUT_KEEPS_ITS_TOKEN)


class HowTheIssueIsWritten(WorkflowTest):
    def test_the_open_issues_are_listed_asking_gh_for_as_many_as_the_core_takes_to_be_a_list_cut_short(self):
        self.assertEqual({'name': 'List the open issues, before any of the judged commit is on disk',
                          'env': {'GH_TOKEN': '${{ github.token }}'},
                          'run': 'gh issue list --repo "$GITHUB_REPOSITORY" --state open --limit %d --json number,title '
                                 '> "$RUNNER_TEMP/open-issues.json"' % issues.LISTED_AT_MOST},
                         steps_of(self.jobs['say'])[0])

    def test_the_core_says_what_to_write_from_everything_the_run_found(self):
        saying = step_named(self.jobs['say'], 'say')

        self.assertEqual('python3 tools/ci/develop_run_cli.py report "$RUNNER_TEMP/open-issues.json" >> "$GITHUB_OUTPUT"',
                         saying['run'])
        self.assertEqual(dict(JUDGEMENT_HANDED,
                              LOOKING_ENDED='${{ needs.look.result }}',
                              MAIN_FOUND='${{ needs.look.outputs.main-tip }}',
                              MAIN_TIP='${{ steps.main.outputs.tip }}',
                              DEVELOP_TIP='${{ steps.develop.outputs.tip }}',
                              LACKING='${{ needs.look.outputs.lacking }}',
                              MAIN_AHEAD='${{ needs.look.outputs.main-ahead }}',
                              WORKFLOWS_CHANGED='${{ needs.look.outputs.workflows-changed }}',
                              WORKFLOWS_DIFFERING='${{ needs.look.outputs.workflows-differing }}',
                              PROMOTING_ENDED='${{ needs.promote.result }}',
                              PUSHED='${{ needs.promote.outputs.pushed }}',
                              GIT_SAID='${{ needs.promote.outputs.git-said }}'),
                         saying['env'])
        self.assertEqual(set(cli.JUDGEMENT) | set(cli.LOOKING) | set(cli.PROMOTING), set(saying['env']))

    def test_the_job_that_says_hands_on_what_the_core_said(self):
        self.assertEqual({'action': '${{ steps.say.outputs.action }}', 'title': '${{ steps.say.outputs.title }}',
                          'issues': '${{ steps.say.outputs.issues }}', 'body': '${{ steps.say.outputs.body }}'},
                         self.jobs['say']['outputs'])

    def test_develop_s_tip_and_main_s_are_each_read_with_the_token_alone_by_the_same_step_in_the_job_that_says_and_the_one_that_writes(self):
        for branch, read in (('develop', DEVELOP_READ), ('main', MAIN_READ)):
            reading = {'env': {'GH_TOKEN': '${{ github.token }}'},
                       'run': 'tip=$(%s)\n'
                              'if [[ ! $tip =~ ^([0-9a-f]{40}|[0-9a-f]{64})$ ]]; then\n'
                              '  echo "gh answered no commit hash for the tip of %s: $tip" >&2\n'
                              '  exit 1\n'
                              'fi\n'
                              'echo "tip=$tip" >> "$GITHUB_OUTPUT"\n' % (read, branch)}

            self.assertEqual(dict(reading, name="Read %s's tip as it is now, before any of the judged commit is on disk"
                                                % branch, id=branch),
                             step_named(self.jobs['say'], branch))
            self.assertEqual(dict(reading, name="Read %s's tip as it is now, before writing anything to the issue" % branch,
                                  id=branch),
                             step_named(self.jobs['report'], branch))

    def test_the_job_that_writes_the_issue_first_reads_develop_s_tip_and_main_s_and_fails_unless_github_finds_develop_ahead_of_main_or_identical(self):
        self.assertEqual(['develop', 'main', 'holds'], [key_of(step) for step in steps_of(self.jobs['report'])[:3]])
        self.assertEqual(
            {'name': HOLDS,
             'id': 'holds',
             'env': {'GH_TOKEN': '${{ github.token }}', 'MAIN': '${{ steps.main.outputs.tip }}',
                     'DEVELOP': '${{ steps.develop.outputs.tip }}'},
             'run': 'status=$(%s)\n'
                    'if [[ $status != ahead && $status != identical ]]; then\n'
                    '  echo "develop does not hold main\'s tip as GitHub compares them now: main\'s tip is $MAIN, develop\'s '
                    'is $DEVELOP, and GitHub answered $status where only ahead or identical passes, so this run is red '
                    'whatever the core said." >&2\n'
                    '  exit 1\n'
                    'fi\n' % COMPARE},
            step_named(self.jobs['report'], 'holds'),
            'GitHub answers how base...head compare as identical, ahead, behind or diverged, ahead meaning head has '
            'commits base lacks and base none head lacks (%s)' % COMPARE_STATUS)

    def test_the_core_is_handed_develop_s_tip_and_main_s_as_the_job_that_says_read_them_not_as_the_job_that_looks_found_them(self):
        looked = {'result': 'success', 'outputs': {'develop-tip': JUDGED_HASH, 'main-tip': JUDGED_HASH}}
        contexts = {'needs': {'look': looked},
                    'steps': {'develop': {'outcome': 'success', 'outputs': {'tip': LATER_HASH}},
                              'main': {'outcome': 'success', 'outputs': {'tip': STRAY_HASH}}}}
        handed = step_named(self.jobs['say'], 'say')['env']
        status = conditions.Status(failed=False, cancelled=False)

        self.assertEqual((LATER_HASH, STRAY_HASH, JUDGED_HASH),
                         tuple(conditions.value(handed[name], contexts, status)
                               for name in ('DEVELOP_TIP', 'MAIN_TIP', 'MAIN_FOUND')))

    def test_each_issue_action_the_core_hands_on_is_carried_out_by_one_step_that_does_only_that(self):
        report = self.jobs['report']
        carried = [step for step in steps_of(report) if step.get('id') in ('open', 'comment', 'moved-on', 'close')]
        handed = [issues.action_outputs(action).split('\n')[0].split('=')[1] for action in
                  (issues.Open(issues.TITLE, ''), issues.CommentOn(1, ''), issues.Close((1,), ''))]
        body = 'base64 -d <<< "$BODY" > "$RUNNER_TEMP/issue-body.md"\n'

        self.assertEqual(
            [{'name': 'Open the issue',
              'id': 'open',
              'if': "${{ !cancelled() && needs.say.outputs.action == '%s' }}" % handed[0],
              'env': {'GH_TOKEN': '${{ github.token }}', 'TITLE': '${{ needs.say.outputs.title }}',
                      'BODY': '${{ needs.say.outputs.body }}'},
              'run': body + 'gh issue create --repo "$GITHUB_REPOSITORY" --title "$TITLE" '
                            '--body-file "$RUNNER_TEMP/issue-body.md"\n'},
             {'name': 'Comment on the open issue',
              'id': 'comment',
              'if': "${{ !cancelled() && needs.say.outputs.action == '%s' }}" % handed[1],
              'env': {'GH_TOKEN': '${{ github.token }}', 'ISSUE': '${{ needs.say.outputs.issues }}',
                      'BODY': '${{ needs.say.outputs.body }}'},
              'run': body + 'gh issue comment --repo "$GITHUB_REPOSITORY" --body-file "$RUNNER_TEMP/issue-body.md" '
                            '-- "$ISSUE"\n'},
             {'name': MOVED_ON,
              'id': 'moved-on',
              'if': "needs.say.outputs.action == '%s'" % core_word_for_develop_moved_on(),
              'env': {'GH_TOKEN': '${{ github.token }}', 'ISSUE': '${{ needs.say.outputs.issues }}',
                      'BODY': '${{ needs.say.outputs.body }}'},
              'run': body + 'gh issue comment --repo "$GITHUB_REPOSITORY" --body-file "$RUNNER_TEMP/issue-body.md" '
                            '-- "$ISSUE"\n'},
             {'name': "Close the issue, saying main now stands at the judged commit, only when every job before passed "
                      "and that commit is still both develop's tip and main's",
              'id': 'close',
              'if': "needs.say.outputs.action == '%s' && needs.unit-tests.result == 'success' && "
                    "needs.look.result == 'success' && needs.promote.result == 'success' && "
                    "needs.say.result == 'success' && steps.develop.outputs.tip == needs.unit-tests.outputs.commit && "
                    "steps.main.outputs.tip == needs.unit-tests.outputs.commit" % handed[2],
              'env': {'GH_TOKEN': '${{ github.token }}', 'ISSUES': '${{ needs.say.outputs.issues }}',
                      'BODY': '${{ needs.say.outputs.body }}'},
              'run': body + 'for issue in $ISSUES; do\n'
                            '  gh issue comment --repo "$GITHUB_REPOSITORY" --body-file "$RUNNER_TEMP/issue-body.md" '
                            '-- "$issue"\n'
                            '  gh issue close --repo "$GITHUB_REPOSITORY" -- "$issue"\n'
                            'done\n'}],
            carried)

    def test_a_green_run_whose_judged_commit_develop_has_moved_off_says_where_they_stand_by_a_step_that_reads_nothing_the_core_wrote(self):
        self.assertEqual(
            {'name': "Say on the issue where develop's tip and main's now stand, when every job before passed, the core "
                     "said close and only main's is still the judged commit",
             'id': 'behind',
             'if': "needs.say.outputs.action == 'close' && needs.unit-tests.result == 'success' && "
                   "needs.look.result == 'success' && needs.promote.result == 'success' && "
                   "needs.say.result == 'success' && steps.main.outputs.tip == needs.unit-tests.outputs.commit && "
                   "steps.develop.outputs.tip != needs.unit-tests.outputs.commit",
             'env': {'GH_TOKEN': '${{ github.token }}', 'ISSUES': '${{ needs.say.outputs.issues }}',
                     'JUDGED': '${{ needs.unit-tests.outputs.commit }}',
                     'DEVELOP_TIP': '${{ steps.develop.outputs.tip }}', 'MAIN_TIP': '${{ steps.main.outputs.tip }}'},
             'run': 'link="$GITHUB_SERVER_URL/$GITHUB_REPOSITORY/actions/runs/$GITHUB_RUN_ID"\n'
                    "body=$(printf '%s\\n\\nThe run: %s' \"develop was judged green at $JUDGED, but develop's tip is now "
                    "$DEVELOP_TIP and main's is $MAIN_TIP, and a run closes this issue only when it finds both at the "
                    'commit it judged, so this run does not close it." "$link")\n'
                    'for issue in $ISSUES; do\n'
                    '  gh issue comment --repo "$GITHUB_REPOSITORY" --body "$body" -- "$issue"\n'
                    'done\n'},
            step_named(self.jobs['report'], 'behind'))

    def test_a_green_run_whose_main_moved_off_the_judged_commit_writes_nothing_to_the_issue_saying_so_by_a_step_that_reads_nothing_the_core_wrote(self):
        self.assertEqual(
            {'name': 'Write nothing to the issue once main has moved since this run looked at it, saying so, when every '
                     "job before passed, the core said close and main's tip is not the judged commit",
             'id': 'moved',
             'if': "needs.say.outputs.action == 'close' && needs.unit-tests.result == 'success' && "
                   "needs.look.result == 'success' && needs.promote.result == 'success' && "
                   "needs.say.result == 'success' && steps.main.outputs.tip != needs.unit-tests.outputs.commit",
             'env': {'JUDGED': '${{ needs.unit-tests.outputs.commit }}', 'MAIN_TIP': '${{ steps.main.outputs.tip }}'},
             'run': 'echo "main moved since this run looked at it: its tip is now $MAIN_TIP, not the judged commit '
                    "$JUDGED, so this run writes nothing to the issue, and a run that looks at main again, for a new "
                    "push to develop or re-running all of this run's jobs, speaks for develop.\"\n"},
            step_named(self.jobs['report'], 'moved'))

    def test_a_red_run_the_core_wrote_nothing_for_opens_or_comments_on_the_issue_by_a_step_that_reads_nothing_it_said(self):
        falling_back = steps_of(self.jobs['report'])[-1]

        self.assertEqual(
            {'name': FALLING_BACK,
             'if': "${{ !cancelled() && steps.open.outcome != 'success' && steps.comment.outcome != 'success' && "
                   "(failure() || needs.unit-tests.result != 'success' || needs.look.result != 'success' || "
                   "needs.promote.result != 'success' || needs.say.result != 'success') }}",
             'env': {'GH_TOKEN': '${{ github.token }}', 'TITLE': issues.TITLE},
             'run': 'link="$GITHUB_SERVER_URL/$GITHUB_REPOSITORY/actions/runs/$GITHUB_RUN_ID"\n'
                    'body="A run for develop is red, and the jobs that say why could not, so only the run itself says '
                    'what failed: $link"\n'
                    'gh issue list --repo "$GITHUB_REPOSITORY" --state open --limit %d --json number,title '
                    '> "$RUNNER_TEMP/open-issues.json"\n'
                    "issue=$(jq -r --arg title \"$TITLE\" '[.[] | select(.title == $title) | .number] | min // empty' "
                    '"$RUNNER_TEMP/open-issues.json")\n'
                    'if [ -n "$issue" ]; then\n'
                    '  gh issue comment --repo "$GITHUB_REPOSITORY" --body "$body" -- "$issue"\n'
                    'else\n'
                    '  gh issue create --repo "$GITHUB_REPOSITORY" --title "$TITLE" --body "$body"\n'
                    'fi\n' % issues.LISTED_AT_MOST},
            falling_back)


class WhatReachesTheIssue(WorkflowTest):
    RESULTS = ('success', 'failure', 'cancelled', 'skipped', '')
    ACTIONS = ('open', 'comment', 'moved-on', 'close', 'none', '', 'anything else')
    TIPS = (JUDGED_HASH, LATER_HASH)
    TIPS_READ = tuple(itertools.product(TIPS, repeat=2))
    GREEN = ('success',) * 4
    GUARD = (('develop',), ('main',), ('holds',))

    def report_ran(self, results, action, tip=JUDGED_HASH, failing=(), main=JUDGED_HASH):
        needs = {name: {'result': result} for name, result in zip(('unit-tests', 'look', 'promote', 'say'), results)}
        needs['unit-tests']['outputs'] = {'commit': JUDGED_HASH}
        needs['say']['outputs'] = {'action': action}
        report = self.jobs['report']
        done = steps_ran(report, {'needs': needs, 'github': {'event_name': 'push', 'ref': 'refs/heads/develop'}},
                         outputs={'develop': {'tip': tip}, 'main': {'tip': main}},
                         failing=tuple(key_of(self.closing()) if each == 'close' else each for each in failing))
        self.assertNotIsInstance(done, conditions.Rejected)
        return done

    def closing(self):
        return next(step for step in steps_of(self.jobs['report']) if 'gh issue close' in step.get('run', ''))

    def reported(self, results, action, tip=JUDGED_HASH, failing=(), main=JUDGED_HASH):
        steps, _ = self.report_ran(results, action, tip, failing, main)
        return {'open': steps['open'], 'comment': steps['comment'], 'moved-on': steps.get('moved-on', 'absent'),
                'read': steps.get('develop', 'absent'),
                'read main': steps.get('main', 'absent'), 'holds': steps.get('holds', 'absent'),
                'close': steps[key_of(self.closing())],
                'behind': steps.get('behind', 'absent'), 'moved': steps.get('moved', 'absent'),
                'fallback': steps[key_of(steps_of(self.jobs['report'])[-1])]}

    def every_red(self):
        return [results for results in itertools.product(self.RESULTS, repeat=4) if results != self.GREEN]

    def green_and_each_job_off_alone(self):
        return [self.GREEN] + [self.GREEN[:at] + (result,) + self.GREEN[at + 1:]
                               for at in range(len(self.GREEN)) for result in self.RESULTS[1:]]

    def test_a_red_run_never_closes_the_issue_whatever_the_core_said(self):
        for results in self.every_red():
            for tip, main in self.TIPS_READ:
                self.assertEqual('skipped', self.reported(results, 'close', tip, main=main)['close'],
                                 (results, tip, main))

    def test_a_green_run_closes_the_issue_when_the_core_says_close_and_develop_s_tip_and_main_s_are_still_the_judged_commit(self):
        self.assertEqual({'open': 'skipped', 'comment': 'skipped', 'read': 'success', 'read main': 'success',
                          'holds': 'success', 'close': 'success', 'behind': 'skipped', 'moved': 'skipped',
                          'moved-on': 'skipped', 'fallback': 'skipped'},
                         self.reported(self.GREEN, 'close'))

    def test_no_run_closes_the_issue_once_develop_s_tip_as_the_job_that_writes_it_reads_it_has_moved_on_whatever_say_said(self):
        for results in self.every_red() + [self.GREEN]:
            for action in self.ACTIONS:
                for main in self.TIPS:
                    self.assertEqual('skipped', self.reported(results, action, LATER_HASH, main=main)['close'],
                                     (results, action, main))

    def test_no_run_closes_the_issue_once_main_s_tip_as_the_job_that_writes_it_reads_it_is_not_the_judged_commit_whatever_say_said(self):
        for results in self.every_red() + [self.GREEN]:
            for action in self.ACTIONS:
                for tip in self.TIPS:
                    self.assertEqual('skipped', self.reported(results, action, tip, main=LATER_HASH)['close'],
                                     (results, action, tip))

    def test_a_green_run_the_core_said_to_close_for_says_on_the_issue_where_develop_and_main_stand_once_develop_alone_moved_off_the_judged_commit(self):
        self.assertEqual({'open': 'skipped', 'comment': 'skipped', 'read': 'success', 'read main': 'success',
                          'holds': 'success', 'close': 'skipped', 'behind': 'success', 'moved': 'skipped',
                          'moved-on': 'skipped', 'fallback': 'skipped'},
                         self.reported(self.GREEN, 'close', LATER_HASH, main=JUDGED_HASH))

    def test_a_green_run_the_core_said_to_close_for_writes_nothing_to_the_issue_and_says_so_in_its_log_once_main_moved_off_the_judged_commit(self):
        for tip in self.TIPS:
            self.assertEqual({'open': 'skipped', 'comment': 'skipped', 'read': 'success', 'read main': 'success',
                              'holds': 'success', 'close': 'skipped', 'behind': 'skipped', 'moved': 'success',
                              'moved-on': 'skipped', 'fallback': 'skipped'},
                             self.reported(self.GREEN, 'close', tip, main=LATER_HASH), tip)

    def test_a_run_says_where_develop_and_main_stand_only_when_every_job_passed_the_core_said_close_and_develop_alone_moved_off_the_judged_commit(self):
        for results in self.every_red() + [self.GREEN]:
            for action in self.ACTIONS:
                for tip, main in self.TIPS_READ:
                    said = (results, action, tip, main) == (self.GREEN, 'close', LATER_HASH, JUDGED_HASH)

                    self.assertEqual('success' if said else 'skipped',
                                     self.reported(results, action, tip, main=main)['behind'],
                                     (results, action, tip, main))

    def test_a_run_says_in_its_log_that_main_moved_only_when_every_job_passed_the_core_said_close_and_main_moved_off_the_judged_commit(self):
        for results in self.every_red() + [self.GREEN]:
            for action in self.ACTIONS:
                for tip, main in self.TIPS_READ:
                    said = (results, action, main) == (self.GREEN, 'close', LATER_HASH)

                    self.assertEqual('success' if said else 'skipped',
                                     self.reported(results, action, tip, main=main)['moved'],
                                     (results, action, tip, main))

    def test_a_read_of_develop_or_main_that_failed_never_closes_the_issue_and_the_run_says_it_is_red(self):
        for failed in ('develop', 'main'):
            for tip, main in self.TIPS_READ:
                reported = self.reported(self.GREEN, 'close', tip, failing=(failed,), main=main)

                self.assertEqual(('failure', 'skipped', 'skipped', 'skipped', 'success'),
                                 (reported['read' if failed == 'develop' else 'read main'], reported['holds'],
                                  reported['close'], reported['behind'], reported['fallback']), (failed, tip, main))

    def test_every_run_reads_develop_s_tip_and_main_s_and_checks_that_develop_holds_main_s_before_writing_anything_whatever_the_core_said(self):
        for results in self.every_red() + [self.GREEN]:
            for action in self.ACTIONS:
                reported = self.reported(results, action, LATER_HASH, main=JUDGED_HASH)

                self.assertEqual(('success', 'success', 'success'),
                                 (reported['read'], reported['read main'], reported['holds']), (results, action))

    def test_a_run_whose_check_that_develop_holds_main_s_tip_failed_or_could_not_read_a_tip_is_red_whatever_the_core_said(self):
        for results in self.green_and_each_job_off_alone():
            for action in self.ACTIONS:
                for failing in self.GUARD:
                    _, ended = self.report_ran(results, action, failing=failing)

                    self.assertEqual('failure', ended, (results, action, failing))

    def test_open_and_the_red_comment_carry_out_what_the_core_said_whatever_became_of_the_check_that_develop_holds_main_s_tip(self):
        for results in self.green_and_each_job_off_alone():
            for action in self.ACTIONS:
                for failing in ((),) + self.GUARD:
                    reported = self.reported(results, action, failing=failing)

                    self.assertEqual(('success' if action == 'open' else 'skipped',
                                      'success' if action == 'comment' else 'skipped'),
                                     (reported['open'], reported['comment']), (results, action, failing))

    def test_no_run_closes_says_where_develop_and_main_stand_or_logs_that_main_moved_once_the_check_that_develop_holds_main_s_tip_failed(self):
        for results in self.green_and_each_job_off_alone():
            for action in self.ACTIONS:
                for tip, main in self.TIPS_READ:
                    for failing in self.GUARD:
                        reported = self.reported(results, action, tip, failing, main)

                        self.assertEqual(('skipped', 'skipped', 'skipped'),
                                         (reported['close'], reported['behind'], reported['moved']),
                                         (results, action, tip, main, failing))

    def test_a_run_whose_check_that_develop_holds_main_s_tip_failed_says_on_the_issue_that_it_is_red_unless_the_core_s_open_or_red_comment_went_through(self):
        for results in self.green_and_each_job_off_alone():
            for action in self.ACTIONS:
                for guard in self.GUARD:
                    for failing in ((), ('open',), ('comment',)):
                        reported = self.reported(results, action, failing=guard + failing)
                        wrote = 'success' in (reported['open'], reported['comment'])

                        self.assertEqual('skipped' if wrote else 'success', reported['fallback'],
                                         (results, action, guard, failing))

    def test_a_green_run_the_core_said_develop_moved_on_for_says_so_on_the_issue_by_a_step_of_its_own_and_ends_green(self):
        said = core_word_for_develop_moved_on()

        self.assertEqual(({'open': 'skipped', 'comment': 'skipped', 'moved-on': 'success', 'read': 'success',
                           'read main': 'success', 'holds': 'success', 'close': 'skipped', 'behind': 'skipped',
                           'moved': 'skipped', 'fallback': 'skipped'}, 'success'),
                         (self.reported(self.GREEN, said, LATER_HASH), self.report_ran(self.GREEN, said, LATER_HASH)[1]))

    def test_no_run_says_on_the_issue_that_develop_moved_on_once_a_read_of_develop_or_main_or_the_check_that_develop_holds_main_s_tip_failed_saying_instead_that_the_run_is_red(self):
        for said in sorted({core_word_for_develop_moved_on(), 'moved-on'}):
            for results in self.green_and_each_job_off_alone():
                for tip, main in self.TIPS_READ:
                    for failing in self.GUARD:
                        reported = self.reported(results, said, tip, failing, main)

                        self.assertEqual(('skipped', 'skipped', 'skipped', 'success'),
                                         (reported['open'], reported['comment'], reported['moved-on'],
                                          reported['fallback']), (said, results, tip, main, failing))

    def test_a_red_run_writes_to_the_issue_whatever_the_core_said_or_failed_to_say(self):
        for results in self.every_red():
            for action in self.ACTIONS:
                for failing in ((), ('open',), ('comment',), ('develop',), ('main',), ('holds',)):
                    written = self.reported(results, action, LATER_HASH, failing)

                    self.assertIn('success', (written['open'], written['comment'], written['fallback']),
                                  (results, action, failing))

    def test_a_red_run_whose_open_or_comment_went_through_writes_nothing_more(self):
        for results in self.every_red():
            self.assertEqual({'open': 'success', 'comment': 'skipped', 'read': 'success', 'read main': 'success',
                              'holds': 'success', 'close': 'skipped', 'behind': 'skipped', 'moved': 'skipped',
                              'moved-on': 'skipped', 'fallback': 'skipped'},
                             self.reported(results, 'open'))
            self.assertEqual({'open': 'skipped', 'comment': 'success', 'read': 'success', 'read main': 'success',
                              'holds': 'success', 'close': 'skipped', 'behind': 'skipped', 'moved': 'skipped',
                              'moved-on': 'skipped', 'fallback': 'skipped'},
                             self.reported(results, 'comment'))

    def test_a_green_run_never_says_it_is_red(self):
        for action in self.ACTIONS:
            for tip, main in self.TIPS_READ:
                self.assertEqual('skipped', self.reported(self.GREEN, action, tip, main=main)['fallback'],
                                 (action, tip, main))

    def test_a_green_run_whose_close_word_that_develop_moved_on_word_of_where_develop_and_main_stand_or_log_that_main_moved_failed_says_it_is_red(self):
        self.assertEqual('success', self.reported(self.GREEN, 'close', failing=('close',))['fallback'])
        self.assertEqual(('failure', 'success'),
                         tuple(self.reported(self.GREEN, core_word_for_develop_moved_on(), LATER_HASH,
                                             failing=('moved-on',))[key] for key in ('moved-on', 'fallback')))
        self.assertEqual('success',
                         self.reported(self.GREEN, 'close', LATER_HASH, failing=('behind',), main=JUDGED_HASH)['fallback'])
        for tip in self.TIPS:
            self.assertEqual('success',
                             self.reported(self.GREEN, 'close', tip, failing=('moved',), main=LATER_HASH)['fallback'], tip)


class WhatTheIssueStepsHandGh(WorkflowTest):
    def setUp(self):
        super().setUp()
        self.where = tempfile.TemporaryDirectory()
        self.calls = os.path.join(self.where.name, 'calls')
        self.listed = os.path.join(self.where.name, 'listed.json')
        self.path = fake_gh(self.where.name)

    def tearDown(self):
        self.where.cleanup()

    def fell_back(self, listed):
        with open(self.listed, 'w', encoding='utf-8') as written:
            written.write(listed)
        return self.ran(steps_of(self.jobs['report'])[-1], {})

    def shell(self, step, environ):
        outputs = os.path.join(self.where.name, 'outputs')
        open(outputs, 'w', encoding='utf-8').close()
        done = subprocess.run(['bash', '--noprofile', '--norc', '-eo', 'pipefail', '-c', step['run']],
                              cwd=self.where.name, capture_output=True, text=True, timeout=60,
                              env=dict({name: text for name, text in step['env'].items() if '${{' not in text},
                                       PATH=self.path, HOME=self.where.name, GH_CALLS=self.calls,
                                       GH_LISTED=self.listed, RUNNER_TEMP=self.where.name, GH_TOKEN='a-token',
                                       GITHUB_OUTPUT=outputs, GITHUB_SERVER_URL='https://github.com',
                                       GITHUB_REPOSITORY='th7/midnight-snackers', GITHUB_RUN_ID='123456', **environ))
        calls = []
        if os.path.exists(self.calls):
            with open(self.calls, encoding='utf-8') as written:
                calls = [json.loads(line) for line in written.read().split('\n') if line]
            os.remove(self.calls)
        return done, calls, written_outputs(outputs)

    def ran(self, step, environ):
        done, calls, _ = self.shell(step, environ)
        self.assertEqual(0, done.returncode, done.stdout + done.stderr)
        return calls

    def test_it_comments_on_the_oldest_issue_so_titled_naming_the_run(self):
        calls = self.fell_back('[{"number": 41, "title": "%s"}, {"number": 7, "title": "Launch Sim"}, '
                               '{"number": 12, "title": "%s"}]' % (issues.TITLE, issues.TITLE))

        self.assertEqual(2, len(calls))
        self.assertEqual(['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body'], calls[1][:5])
        self.assertEqual(['--', '12'], calls[1][-2:])
        self.assertIn('https://github.com/th7/midnight-snackers/actions/runs/123456', calls[1][5])

    def test_each_step_that_carries_out_the_core_s_action_hands_gh_the_body_the_core_wrote_as_it_wrote_it(self):
        body = 'develop was judged red at `x` — é, so\n\n```\nremote: $(rejected)\n```\n'
        handed = issues.action_outputs(issues.Open(issues.TITLE, body)).split('\n')[2][len('body='):]
        report = self.jobs['report']

        self.assertEqual([['issue', 'create', '--repo', 'th7/midnight-snackers', '--title', issues.TITLE, '--body-file',
                           os.path.join(self.where.name, 'issue-body.md'), body]],
                         self.ran(step_named(report, 'open'), {'TITLE': issues.TITLE, 'BODY': handed}))
        for step_id in ('comment', 'moved-on'):
            self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file',
                               os.path.join(self.where.name, 'issue-body.md'), '--', '12', body]],
                             self.ran(step_named(report, step_id), {'ISSUE': '12', 'BODY': handed}), step_id)
        closing = self.ran(step_named(report, 'close'), {'ISSUES': '12 41', 'BODY': handed})
        self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file',
                           os.path.join(self.where.name, 'issue-body.md'), '--', '12', body],
                          ['issue', 'close', '--repo', 'th7/midnight-snackers', '--', '12'],
                          ['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body-file',
                           os.path.join(self.where.name, 'issue-body.md'), '--', '41', body],
                          ['issue', 'close', '--repo', 'th7/midnight-snackers', '--', '41']], closing)

    def test_it_opens_the_issue_when_none_so_titled_is_open(self):
        calls = self.fell_back('[{"number": 5, "title": "IMPORTANT: develop CI Failures (old)"}]')

        self.assertEqual(['issue', 'create', '--repo', 'th7/midnight-snackers', '--title', issues.TITLE, '--body'],
                         calls[1][:7])
        self.assertIn('https://github.com/th7/midnight-snackers/actions/runs/123456', calls[1][7])

    def test_each_read_of_develop_or_main_asks_gh_only_for_that_branch_s_ref_and_hands_on_the_tip_it_answers(self):
        for job in ('say', 'report'):
            for branch in ('develop', 'main'):
                done, calls, outputs = self.shell(step_named(self.jobs[job], branch),
                                                  {'GH_' + branch.upper(): LATER_HASH + '\n'})

                self.assertEqual(0, done.returncode, done.stdout + done.stderr)
                self.assertEqual([['api', '--method', 'GET', 'repos/th7/midnight-snackers/git/ref/heads/' + branch,
                                   '--jq', '.object.sha']], calls, (job, branch))
                self.assertEqual({'tip': LATER_HASH}, outputs, (job, branch))

    def test_a_read_of_develop_or_main_that_gh_failed_or_answered_with_anything_but_one_commit_hash_fails_handing_on_nothing(self):
        for job in ('say', 'report'):
            for branch in ('develop', 'main'):
                answer = 'GH_' + branch.upper()
                for answered in ({'GH_API_FAILS': '1'}, {answer: ''}, {answer: 'null\n'},
                                 {answer: LATER_HASH.upper() + '\n'}, {answer: LATER_HASH[:39] + '\n'},
                                 {answer: '%s\n%s\n' % (LATER_HASH, JUDGED_HASH)},
                                 {answer: '%s\ntip=%s\n' % (LATER_HASH, JUDGED_HASH)}):
                    done, _, outputs = self.shell(step_named(self.jobs[job], branch), answered)

                    self.assertNotEqual(0, done.returncode, (job, branch, answered))
                    self.assertEqual({}, outputs, (job, branch, answered))

    def test_the_check_asks_gh_only_to_compare_main_s_tip_with_develop_s_and_passes_exactly_when_github_answers_ahead_or_identical(self):
        holds = step_named(self.jobs['report'], 'holds')
        for answered, passes in (('ahead\n', True), ('identical\n', True), ('behind\n', False), ('diverged\n', False),
                                 ('', False), ('null\n', False), ('Ahead\n', False), ('ahead \n', False),
                                 (' identical\n', False), ('ahead\nahead\n', False), ('ahead\nidentical\n', False)):
            done, calls, outputs = self.shell(holds, {'MAIN': JUDGED_HASH, 'DEVELOP': LATER_HASH, 'GH_COMPARE': answered})

            self.assertEqual((0 if passes else 1, [['api', '--method', 'GET', 'repos/th7/midnight-snackers/compare/%s...%s'
                                                    % (JUDGED_HASH, LATER_HASH), '--jq', '.status']], {}),
                             (done.returncode, calls, outputs), answered)

    def test_the_check_fails_naming_both_tips_and_github_s_answer_when_develop_does_not_hold_main_s_tip(self):
        done, _, _ = self.shell(step_named(self.jobs['report'], 'holds'),
                                {'MAIN': STRAY_HASH, 'DEVELOP': JUDGED_HASH, 'GH_COMPARE': 'diverged\n'})

        self.assertEqual((1, '', "develop does not hold main's tip as GitHub compares them now: main's tip is %s, develop's "
                                 'is %s, and GitHub answered diverged where only ahead or identical passes, so this run is '
                                 'red whatever the core said.\n' % (STRAY_HASH, JUDGED_HASH)),
                         (done.returncode, done.stdout, done.stderr))

    def test_the_check_fails_when_gh_fails_to_compare_whatever_it_would_have_answered(self):
        for answered in ('ahead\n', 'identical\n'):
            done, calls, outputs = self.shell(step_named(self.jobs['report'], 'holds'),
                                              {'MAIN': JUDGED_HASH, 'DEVELOP': LATER_HASH, 'GH_COMPARE': answered,
                                               'GH_API_FAILS': '1'})

            self.assertNotEqual(0, done.returncode, answered)
            self.assertEqual((1, {}), (len(calls), outputs), answered)

    def test_the_word_of_where_develop_and_main_stand_goes_on_each_issue_the_core_said_to_close_naming_the_commits_and_the_run(self):
        calls = self.ran(step_named(self.jobs['report'], 'behind'),
                         {'ISSUES': '12 41', 'JUDGED': JUDGED_HASH, 'DEVELOP_TIP': JUDGED_HASH, 'MAIN_TIP': STRAY_HASH})
        body = ("develop was judged green at %s, but develop's tip is now %s and main's is %s, and a run closes this "
                'issue only when it finds both at the commit it judged, so this run does not close it.\n\n'
                'The run: https://github.com/th7/midnight-snackers/actions/runs/123456'
                % (JUDGED_HASH, JUDGED_HASH, STRAY_HASH))

        self.assertEqual([['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body', body, '--', '12'],
                          ['issue', 'comment', '--repo', 'th7/midnight-snackers', '--body', body, '--', '41']], calls)


    def test_each_word_that_main_moved_since_the_look_goes_to_the_log_alone_naming_where_main_was_and_is(self):
        promoting = {step['name']: step for step in steps_of(self.jobs['promote'])}
        for step, environ, ended, said, warned in (
                (promoting[MOVED_ALONG_DEVELOP], {'FOUND': JUDGED_HASH, 'NOW': LATER_HASH}, 0,
                 'main moved along develop since the job that looks found it at %s: its tip is now %s, which develop '
                 "holds, so this run pushes nothing, and a run that looks at main again, for a new push to develop or "
                 "re-running all of this run's jobs, speaks for develop.\n" % (JUDGED_HASH, LATER_HASH), ''),
                (promoting[MOVED_OFF_DEVELOP], {'FOUND': JUDGED_HASH, 'NOW': STRAY_HASH}, 1, '',
                 'main moved since the job that looks found it at %s: its tip is now %s, which this job could not find '
                 'in develop as its checkout fetched it, so this run pushes nothing and is red.\n'
                 % (JUDGED_HASH, STRAY_HASH)),
                (step_named(self.jobs['report'], 'moved'), {'JUDGED': JUDGED_HASH, 'MAIN_TIP': STRAY_HASH}, 0,
                 'main moved since this run looked at it: its tip is now %s, not the judged commit %s, so this run '
                 "writes nothing to the issue, and a run that looks at main again, for a new push to develop or "
                 "re-running all of this run's jobs, speaks for develop.\n" % (STRAY_HASH, JUDGED_HASH), '')):
            done, calls, outputs = self.shell(step, environ)

            self.assertEqual((ended, said, warned, [], {}), (done.returncode, done.stdout, done.stderr, calls, outputs),
                             step['name'])


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
