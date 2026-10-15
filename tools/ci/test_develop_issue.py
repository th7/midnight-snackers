import base64
import unittest

import develop_issue as issues
import develop_run as run
from develop_issue import Close, CommentOn, LeaveAlone, LeaveAloneSinceMainMoved, Open, OpenIssue
from develop_run import (Judged, Looked, MainHasWhatDevelopLacks, NeverJudged, StepDidNotPass, StepsEnded,
                         NeverLooked, Outcome, Rejected)

JUDGED = 'a' * 40
MAIN = 'b' * 40
STRAY = 'c' * 40
LATER = 'd' * 40
SUBJECT = 'A ball comes back from a lone still compliant surface at its restitution'
RUN = 'https://github.com/th7/midnight-snackers/actions/runs/123456'
BODY_TOO_LONG = 'https://github.com/golang/go/issues/45998'
GITHUB_LIMITS = 'https://github.com/dead-claudia/github-limits'
GH_ISSUE_LIST = 'https://cli.github.com/manual/gh_issue_list'
EXECVE_LIMITS = 'https://man7.org/linux/man-pages/man2/execve.2.html#NOTES'
MAX_ARG_STRLEN = 32 * 4096


def b64(text):
    return base64.b64encode(text.encode('utf-8')).decode('ascii')


def commit(hash_text=JUDGED, subject=SUBJECT):
    return run.commit_line(hash_text + ' ' + subject)


def steps_ended(**changed):
    outcomes = {step.step: Outcome.SUCCESS for step in run.STEPS}
    outcomes.update({key.replace('_', '-'): value for key, value in changed.items()})
    return StepsEnded(tuple(outcomes[step.step] for step in run.STEPS))


def texts(**changed):
    return tuple(each.value for each in steps_ended(**changed).outcomes)


def judged(ended=Outcome.SUCCESS, the_commit=None, **outcomes):
    return Judged(the_commit or commit(), ended, steps_ended(**outcomes))


def failed():
    return judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE)


def looked(lacking=(), pushed=Outcome.SUCCESS, said='', ahead=(), workflows=(), ended=None, develop=JUDGED, main=MAIN,
           found=MAIN):
    return Looked(run.commit_hash(found), run.commit_hash(main), run.commit_hash(develop), tuple(lacking), tuple(ahead),
                  run.Workflows(run.WorkflowFiles(tuple(workflows)), run.WorkflowFiles(tuple(workflows))), pushed, said,
                  ended if ended is not None else Outcome.SUCCESS if pushed is Outcome.SUCCESS else Outcome.FAILURE)


def held():
    return looked(pushed=Outcome.SKIPPED)


def link():
    return issues.run_link('https://github.com', 'th7/midnight-snackers', '123456')


def issue(number, title=issues.TITLE):
    return OpenIssue(number, title)


def moved_on_found():
    return run.GreenButMovedOn(commit(), run.commit_hash(MAIN), run.commit_hash(LATER))


class WhichIssueIsTouched(unittest.TestCase):
    def test_the_issue_is_titled_exactly_so(self):
        self.assertEqual('IMPORTANT: develop CI Failures', issues.TITLE)

    def test_a_red_run_with_no_issue_open_opens_one(self):
        action = issues.issue_action(failed(), held(), (), link())

        self.assertIsInstance(action, Open)
        self.assertEqual(issues.TITLE, action.title)

    def test_a_red_run_comments_on_the_issue_already_open_the_oldest_when_there_are_several(self):
        action = issues.issue_action(failed(), held(), (issue(41), issue(12)), link())

        self.assertIsInstance(action, CommentOn)
        self.assertEqual(12, action.issue)

    def test_an_issue_titled_otherwise_is_not_the_one(self):
        others = (issue(5, 'IMPORTANT: develop CI Failures (old)'), issue(6, 'important: develop ci failures'))

        self.assertIsInstance(issues.issue_action(failed(), held(), others, link()), Open)

    def test_a_green_run_closes_every_issue_so_titled(self):
        action = issues.issue_action(judged(), looked(main=JUDGED), (issue(41), issue(12), issue(7, 'Launch Sim')),
                                     link())

        self.assertIsInstance(action, Close)
        self.assertEqual((12, 41), action.issues)

    def test_a_green_run_whose_judged_commit_is_no_longer_develop_s_tip_hands_the_workflow_apart_from_a_red_comment_the_word_that_develop_moved_on_for_the_issue_and_leaves_it_open(self):
        action = issues.issue_action(judged(), looked(main=JUDGED, develop=LATER), (issue(41), issue(12)), link())

        self.assertEqual('action=moved-on\nissues=12\nbody=%s\n'
                         % b64("develop was judged green at `%s` `%s`, and this run moved main there, but develop's tip is "
                               "now `%s`, which this run did not judge, and a run closes this issue only when it finds "
                               "develop's tip and main's both at the commit it judged, so this run does not close it."
                               '\n\nThe run: %s\n' % (JUDGED, SUBJECT, LATER, RUN)),
                         issues.action_outputs(action))

    def test_a_green_run_that_found_main_already_at_the_judged_commit_and_develop_moved_on_says_main_already_stood_there(self):
        action = issues.issue_action(judged(), looked(main=JUDGED, develop=LATER, found=JUDGED), (issue(12),), link())

        self.assertEqual('action=moved-on\nissues=12\nbody=%s\n'
                         % b64("develop was judged green at `%s` `%s`, and main already stood there when this run looked "
                               "at it, but develop's tip is now `%s`, which this run did not judge, and a run closes this "
                               "issue only when it finds develop's tip and main's both at the commit it judged, so this "
                               'run does not close it.\n\nThe run: %s\n' % (JUDGED, SUBJECT, LATER, RUN)),
                         issues.action_outputs(action))

    def test_the_only_comment_the_core_hands_the_workflow_as_such_is_a_red_run_s_saying_main_stays_where_it_was(self):
        stray = commit(STRAY, 'Fix the README on main')
        for judgement, promoting in ((failed(), held()), (judged(), looked(lacking=(stray,), pushed=Outcome.SKIPPED)),
                                     (judged(), looked(pushed=Outcome.FAILURE, said='remote: refused\n')),
                                     (judged(), looked(main=JUDGED, develop=LATER)),
                                     (judged(), looked(main=JUDGED, develop=LATER, found=JUDGED)),
                                     (judged(), looked(main=JUDGED)), (judged(), looked(main=STRAY))):
            handed = issues.action_outputs(issues.issue_action(judgement, promoting, (issue(12),), link()))
            said, _, rest = handed.partition('\n')
            body = base64.b64decode(rest.split('\n')[1][len('body='):]).decode('utf-8') if said != 'action=none' else ''

            self.assertEqual('main stays where it was' in body, said == 'action=comment', (promoting, handed))

    def test_a_run_whose_main_moved_off_the_judged_commit_after_its_push_says_nothing_for_the_issue_and_logs_why(self):
        for listed in ((issue(41), issue(12)), ()):
            for develop in (JUDGED, LATER):
                for found in (MAIN, JUDGED):
                    self.assertEqual(
                        LeaveAloneSinceMainMoved(
                            "develop was judged green at `%s` `%s`, but main moved since this run's push left it there, "
                            "and now stands at `%s`, so this job says nothing for the issue: a run that looks at main "
                            "again, for a new push to develop or re-running all of this run's jobs, speaks for develop."
                            % (JUDGED, SUBJECT, STRAY)),
                        issues.issue_action(judged(), looked(main=STRAY, develop=develop, found=found), listed, link()),
                        (listed, develop, found))

    def test_a_run_that_pushed_nothing_to_a_main_moved_since_its_look_says_nothing_for_the_issue_and_logs_why(self):
        later = commit(STRAY, 'A later commit of develop')
        for promoting in (looked(pushed=Outcome.SKIPPED, ended=Outcome.SUCCESS, main=LATER),
                          looked(pushed=Outcome.SKIPPED, ahead=(later,), ended=Outcome.SUCCESS, main=LATER),
                          looked(pushed=Outcome.FAILURE, said=' ! [rejected] main (fetch first)\n', main=LATER)):
            for listed in ((issue(12),), ()):
                self.assertEqual(
                    LeaveAloneSinceMainMoved(
                        'develop was judged green at `%s` `%s`, but main moved since the job that looks found it at `%s`, '
                        'and now stands at `%s`, so this job says nothing for the issue: a run that looks at main again, '
                        "for a new push to develop or re-running all of this run's jobs, speaks for develop."
                        % (JUDGED, SUBJECT, MAIN, LATER)),
                    issues.issue_action(judged(), promoting, listed, link()), (promoting, listed))

    def test_a_green_run_with_develop_moved_on_and_no_issue_open_leaves_issues_alone(self):
        for found in (MAIN, JUDGED):
            self.assertEqual(LeaveAlone(), issues.issue_action(judged(), looked(main=JUDGED, develop=LATER, found=found),
                                                               (issue(7, 'Launch Sim'),), link()), found)

    def test_a_run_that_left_main_where_it_already_had_later_commits_of_develop_leaves_the_issue_alone(self):
        later = commit(STRAY, 'A later commit of develop')

        self.assertEqual(LeaveAlone(), issues.issue_action(judged(), looked(ahead=(later,), pushed=Outcome.SKIPPED,
                                                                            ended=Outcome.SUCCESS),
                                                           (issue(12),), link()))

    def test_a_green_run_with_no_issue_open_leaves_issues_alone(self):
        self.assertEqual(LeaveAlone(), issues.issue_action(judged(), looked(main=JUDGED), (issue(7, 'Launch Sim'),),
                                                           link()))


class WhatTheIssueSays(unittest.TestCase):
    def test_a_red_run_names_the_commit_judged_what_failed_that_main_stays_where_it_was_and_the_run(self):
        body = issues.issue_action(failed(), held(), (), link()).body

        self.assertIn('develop was judged red at `%s` `%s`' % (JUDGED, SUBJECT), body)
        self.assertIn(run.reasons((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),)), body)
        self.assertIn('main stays where it was, at `%s`.' % MAIN, body)
        self.assertIn(RUN, body)

    def test_a_red_run_whose_judging_was_green_says_develop_was_judged_green_and_main_stays_where_it_was(self):
        stray = commit(STRAY, 'Fix the README on main')
        for promoting in (looked(pushed=Outcome.SKIPPED, workflows=('.github/workflows/tests.yml',)),
                          looked(lacking=(stray,), pushed=Outcome.SKIPPED),
                          looked(pushed=Outcome.FAILURE, said='remote: error: GH006: Protected branch update failed\n'),
                          looked(pushed=Outcome.UNREPORTED)):
            body = issues.issue_action(judged(), promoting, (), link()).body

            self.assertTrue(body.startswith('develop was judged green at `%s` `%s`, but main stays where it was, at '
                                            '`%s`.\n\n' % (JUDGED, SUBJECT, MAIN)), body)
            self.assertNotIn('judged red', body)

    def test_a_red_run_whose_judging_failed_before_any_check_names_the_step_that_failed(self):
        body = issues.issue_action(judged(Outcome.FAILURE, browser=Outcome.FAILURE, formatting=Outcome.SKIPPED,
                                          tool_tests=Outcome.SKIPPED, unit_tests=Outcome.SKIPPED), held(), (),
                                   link()).body

        self.assertIn('**%s** failed.' % run.BROWSER.name, body)

    def test_a_red_run_for_what_main_lacks_names_those_commits_and_the_fix(self):
        stray = commit(STRAY, 'Fix the README on main')
        body = issues.issue_action(judged(), looked(lacking=(stray,), pushed=Outcome.SKIPPED), (), link()).body

        self.assertIn(run.reasons((MainHasWhatDevelopLacks((stray,)),)), body)

    def test_a_red_run_whose_push_main_refused_says_what_git_said(self):
        said = 'remote: error: GH006: Protected branch update failed for refs/heads/main.\n'
        body = issues.issue_action(judged(), looked(pushed=Outcome.FAILURE, said=said), (), link()).body

        self.assertIn('main refused the push of `%s`' % JUDGED, body)
        self.assertIn(said, body)
        self.assertIn('main stays where it was, at `%s`.' % MAIN, body)

    def test_a_red_run_whose_push_github_refused_over_a_workflow_quotes_git_as_any_refused_push_does(self):
        said = (' ! [remote rejected] %s -> main (refusing to allow a GitHub App to create or update workflow '
                '`.github/workflows/tests.yml` without `workflows` permission)\n' % JUDGED)
        body = issues.issue_action(judged(), looked(pushed=Outcome.FAILURE, said=said), (), link()).body

        self.assertIn('main refused the push of `%s`. git said:\n\n```\n%s```' % (JUDGED, said), body)
        self.assertIn('main stays where it was, at `%s`.' % MAIN, body)

    def test_a_red_run_held_over_workflows_gives_the_maintainer_the_push_and_says_main_stays_where_it_was(self):
        body = issues.issue_action(judged(), looked(pushed=Outcome.SKIPPED, workflows=('.github/workflows/tests.yml',)),
                                   (issue(12),), link()).body

        self.assertIn('```\n.github/workflows/tests.yml\n```', body)
        self.assertIn('```\ngit push origin %s:refs/heads/main\n```' % JUDGED, body)
        self.assertIn('main stays where it was, at `%s`.' % MAIN, body)
        self.assertIn(RUN, body)

    def test_a_red_run_that_judged_nothing_and_never_looked_at_main_says_main_stays_where_it_was(self):
        never = NeverJudged(Outcome.FAILURE, steps_ended(judged=Outcome.FAILURE, java=Outcome.SKIPPED,
                                                         gradle=Outcome.SKIPPED, node=Outcome.SKIPPED,
                                                         browser=Outcome.SKIPPED, formatting=Outcome.SKIPPED,
                                                         tool_tests=Outcome.SKIPPED, unit_tests=Outcome.SKIPPED))
        body = issues.issue_action(never, NeverLooked(Outcome.FAILURE), (), link()).body

        self.assertIn('judged no commit', body)
        self.assertIn('**%s** failed.' % run.NAMING.name, body)
        self.assertIn('main stays where it was.', body)
        self.assertIn(RUN, body)

    def test_a_green_run_says_which_commit_was_judged_green_and_that_main_now_stands_there(self):
        action = issues.issue_action(judged(), looked(main=JUDGED), (issue(12),), link())
        body = action.body

        self.assertIsInstance(action, Close)

        self.assertIn('develop was judged green at `%s` `%s`' % (JUDGED, SUBJECT), body)
        self.assertIn('main now stands there', body)
        self.assertIn(RUN, body)

    def test_backticks_in_a_subject_never_close_its_quote_early(self):
        tricky = commit(JUDGED, 'Say `x` and ``` then @everyone')
        body = issues.issue_action(judged(Outcome.FAILURE, tricky, unit_tests=Outcome.FAILURE), held(), (),
                                   link()).body

        self.assertIn('```` Say `x` and ``` then @everyone ````', body)


    def test_a_carriage_return_in_a_subject_never_breaks_the_head_line_so_the_reasons_and_the_run_stay_outside_code(self):
        for printed in ('Fix the thing\r```\rrest\n', 'Fix the thing\r===\n'):
            tricky = run.judgement('failure', JUDGED, b64(printed), texts(unit_tests=Outcome.FAILURE))
            body = issues.issue_action(tricky, held(), (), link()).body
            head, _, rest = body.partition('\n')

            self.assertNotIn('\r', body)
            self.assertIn('\\x0d', head)
            self.assertTrue(rest.startswith('\n**'), rest)
            self.assertTrue(body.endswith('\n\nThe run: %s\n' % RUN), body)

    def test_the_body_stays_under_github_s_limit_in_utf_8_bytes_however_long_and_wide_what_it_quotes(self):
        for character in ('x', '\U0001f600'):
            endless = character * 70000
            judged_long = run.judgement('failure', JUDGED, b64(endless), texts(formatting=Outcome.FAILURE,
                                                                               unit_tests=Outcome.FAILURE))
            strays = b64(''.join('%040x %s\n' % (n, endless) for n in range(1, 61)))
            green_long = run.judgement('success', JUDGED, b64(endless), texts())
            bodies = [issues.issue_action(judged_long, promoting, (), link()).body for promoting in (
                run.promoting('failure', MAIN, MAIN, JUDGED, strays, '', '', '', 'failure', 'skipped', ''),
                run.promoting('success', MAIN, MAIN, JUDGED, '', '', '', '', 'failure', 'failure', b64(endless)),
                run.promoting('failure', MAIN, MAIN, JUDGED, '', '', b64(''.join('.github/workflows/%d%s\n' % (n, endless)
                                                                   for n in range(1, 61))), '', 'failure', 'skipped', ''),
                run.promoting('failure', MAIN, MAIN, JUDGED, '', '', '', b64(''.join('.github/workflows/%d%s\n' % (n, endless)
                                                                       for n in range(1, 61))), 'failure', 'skipped', ''))]
            bodies.append(issues.issue_action(green_long, looked(main=JUDGED), (issue(12),), link()).body)

            for body in bodies:
                self.assertLess(len(body.encode('utf-8')), issues.BODY_AT_MOST,
                                'GitHub does not document it, but refuses an issue body over %d characters (%s), counted '
                                'as code points for an issue and bytes for a comment (%s), so a body under that many '
                                'UTF-8 bytes is under either' % (issues.BODY_AT_MOST, BODY_TOO_LONG, GITHUB_LIMITS))


class ReadingWhatTheWorkflowGathered(unittest.TestCase):
    def test_open_issues_are_read_from_the_json_gh_printed(self):
        self.assertEqual((issue(12), issue(7, 'Launch Sim')),
                         issues.open_issues('[{"number": 12, "title": "%s"}, {"number": 7, "title": "Launch Sim"}]'
                                            % issues.TITLE))

    def test_a_list_as_long_as_gh_was_asked_for_is_refused_since_the_issue_may_be_among_those_left_out(self):
        def listing(count):
            return '[%s]' % ', '.join('{"number": %d, "title": "Issue %d"}' % (n, n) for n in range(1, count + 1))

        self.assertEqual(issues.LISTED_AT_MOST - 1, len(issues.open_issues(listing(issues.LISTED_AT_MOST - 1))),
                         'gh lists at most as many issues as --limit asks for: %s' % GH_ISSUE_LIST)
        self.assertEqual(Rejected('gh listed as many open issues as it was asked for, %d, so the list may be cut short '
                                  'and the develop CI issue may be among those left out' % issues.LISTED_AT_MOST),
                         issues.open_issues(listing(issues.LISTED_AT_MOST)))

    def test_open_issues_that_are_not_that_json_are_refused(self):
        for wrong in ('', 'nope', '{}', '[{"number": 0, "title": "x"}]', '[{"number": true, "title": "x"}]',
                      '[{"number": 3}]', '[{"number": "3", "title": "x"}]', '[3]'):
            self.assertIsInstance(issues.open_issues(wrong), Rejected, wrong)

    def test_an_open_issue_made_directly_is_held_to_a_number_from_one_up_and_a_title(self):
        self.assertEqual(12, OpenIssue(12, issues.TITLE).number)
        for number, title in ((0, 'x'), (True, 'x'), ('3', 'x'), (3, None)):
            self.assertIsInstance(OpenIssue(number, title), Rejected, (number, title))

    def test_a_run_link_made_directly_is_held_to_a_run_on_an_https_server(self):
        self.assertEqual(RUN, issues.RunLink(RUN).url)
        for wrong in ('http://github.com/th7/midnight-snackers/actions/runs/1',
                      'https://github.com/th7/midnight-snackers/actions/runs/1a',
                      'https://github.com/th7/midnight-snackers/actions/runs/1 and more', None):
            self.assertIsInstance(issues.RunLink(wrong), Rejected, wrong)

    def test_the_run_link_is_built_from_where_github_said_the_run_is(self):
        self.assertEqual(RUN, link().url)

    def test_a_run_link_from_anywhere_but_an_https_server_is_refused(self):
        self.assertIsInstance(issues.run_link('http://github.com', 'th7/midnight-snackers', '1'), Rejected)
        self.assertIsInstance(issues.run_link('https://github.com', 'th7', '1'), Rejected)
        self.assertIsInstance(issues.run_link('https://github.com', 'th7/midnight-snackers', '1a'), Rejected)


class WhatTheWorkflowIsHanded(unittest.TestCase):
    def test_each_issue_action_hands_the_workflow_what_it_needs_to_carry_it_out_its_body_in_base64(self):
        body = 'develop was judged red at `x` — é\n'

        self.assertEqual('action=open\ntitle=IMPORTANT: develop CI Failures\nbody=%s\n' % b64(body),
                         issues.action_outputs(Open(issues.TITLE, body)))
        self.assertEqual('action=comment\nissues=12\nbody=%s\n' % b64(body), issues.action_outputs(CommentOn(12, body)))
        self.assertEqual('action=close\nissues=12 41\nbody=%s\n' % b64(body),
                         issues.action_outputs(Close((12, 41), body)))
        self.assertEqual('action=none\n', issues.action_outputs(LeaveAlone()))
        self.assertEqual('action=none\n', issues.action_outputs(LeaveAloneSinceMainMoved('main moved')))
        moved_on = issues.CommentDevelopMovedOn(12, moved_on_found(), link())
        self.assertEqual('action=moved-on\nissues=12\nbody=%s\n' % b64(issues.moved_on_body(moved_on_found(), link())),
                         issues.action_outputs(moved_on))

    def test_the_log_of_each_issue_action_is_the_body_it_writes_or_why_it_writes_nothing(self):
        body = 'develop was judged red at `x`\n'

        self.assertEqual([body, body, body, 'main moved', ''],
                         [issues.logged(action) for action in (Open(issues.TITLE, body), CommentOn(12, body),
                                                               Close((12, 41), body), LeaveAloneSinceMainMoved('main moved'),
                                                               LeaveAlone())])
        self.assertEqual(issues.moved_on_body(moved_on_found(), link()),
                         issues.logged(issues.CommentDevelopMovedOn(12, moved_on_found(), link())))

    def test_the_word_that_develop_moved_on_is_held_to_an_open_issue_s_number_a_green_run_whose_develop_moved_on_and_a_run_link(self):
        self.assertEqual(issues.moved_on_body(moved_on_found(), link()),
                         issues.CommentDevelopMovedOn(12, moved_on_found(), link()).body)
        for number, found, linked in ((0, moved_on_found(), link()), (True, moved_on_found(), link()),
                                      ('12', moved_on_found(), link()), (12, run.Green(commit()), link()),
                                      (12, 'moved on', link()), (12, moved_on_found(), RUN)):
            self.assertIsInstance(issues.CommentDevelopMovedOn(number, found, linked), Rejected, (number, found, linked))

    def test_the_longest_body_in_base64_fits_one_environment_string_so_the_job_that_writes_it_is_handed_it_whole(self):
        longest = 'x' * (issues.BODY_AT_MOST - 1)
        handed = issues.action_outputs(Open(issues.TITLE, longest)).split('\n')[2]

        self.assertEqual(longest, base64.b64decode(handed[len('body='):]).decode('ascii'))
        self.assertLess(len(('BODY=' + handed[len('body='):]).encode('ascii')) + 1, MAX_ARG_STRLEN,
                        'Linux refuses an environment string longer than 32 pages, %d bytes with its terminating null '
                        '(%s)' % (MAX_ARG_STRLEN, EXECVE_LIMITS))


if __name__ == '__main__':
    unittest.main()
