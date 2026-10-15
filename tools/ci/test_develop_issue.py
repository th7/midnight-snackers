import unittest

import develop_issue as issues
import develop_run as run
from develop_issue import Close, CommentOn, LeaveAlone, Open, OpenIssue
from develop_run import (CheckDidNotPass, Checked, Judged, Looked, MainHasWhatDevelopLacks, NeverJudged,
                         NeverLooked, Outcome, Rejected)

JUDGED = 'a' * 40
MAIN = 'b' * 40
STRAY = 'c' * 40
SUBJECT = 'A ball comes back from a lone still compliant surface at its restitution'
RUN = 'https://github.com/th7/midnight-snackers/actions/runs/123456'


def commit(hash_text=JUDGED, subject=SUBJECT):
    return run.commit_line(hash_text + ' ' + subject)


def judged(ended=Outcome.SUCCESS, the_commit=None, **outcomes):
    every = dict(formatting=Outcome.SUCCESS, tool_tests=Outcome.SUCCESS, unit_tests=Outcome.SUCCESS)
    every.update(outcomes)
    return Judged(the_commit or commit(), ended, Checked(**every))


def failed():
    return judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE)


def looked(lacking=(), pushed=Outcome.SUCCESS, said=''):
    return Looked(run.commit_hash(MAIN), tuple(lacking), pushed, said,
                  Outcome.SUCCESS if pushed in (Outcome.SUCCESS, Outcome.SKIPPED) else Outcome.FAILURE)


def held():
    return looked(pushed=Outcome.SKIPPED)


def link():
    return issues.run_link('https://github.com', 'th7/midnight-snackers', '123456')


def issue(number, title=issues.TITLE):
    return OpenIssue(number, title)


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
        action = issues.issue_action(judged(), looked(), (issue(41), issue(12), issue(7, 'Launch Sim')), link())

        self.assertIsInstance(action, Close)
        self.assertEqual((12, 41), action.issues)

    def test_a_green_run_with_no_issue_open_leaves_issues_alone(self):
        self.assertEqual(LeaveAlone(), issues.issue_action(judged(), looked(), (issue(7, 'Launch Sim'),), link()))


class WhatTheIssueSays(unittest.TestCase):
    def test_a_red_run_names_the_commit_judged_what_failed_that_main_stays_where_it_was_and_the_run(self):
        body = issues.issue_action(failed(), held(), (), link()).body

        self.assertIn('develop was judged red at `%s` `%s`' % (JUDGED, SUBJECT), body)
        self.assertIn(run.reasons((CheckDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),)), body)
        self.assertIn('main stays where it was, at `%s`.' % MAIN, body)
        self.assertIn(RUN, body)

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

    def test_a_red_run_that_judged_nothing_and_never_looked_at_main_says_main_stays_where_it_was(self):
        body = issues.issue_action(NeverJudged(Outcome.FAILURE), NeverLooked(Outcome.FAILURE), (), link()).body

        self.assertIn('judged no commit', body)
        self.assertIn('main stays where it was.', body)
        self.assertIn(RUN, body)

    def test_a_green_run_says_which_commit_was_judged_green_and_that_main_now_stands_there(self):
        body = issues.issue_action(judged(), looked(), (issue(12),), link()).body

        self.assertIn('develop was judged green at `%s` `%s`' % (JUDGED, SUBJECT), body)
        self.assertIn('main now stands there', body)
        self.assertIn(RUN, body)

    def test_backticks_in_a_subject_never_close_its_quote_early(self):
        tricky = commit(JUDGED, 'Say `x` and ``` then @everyone')
        body = issues.issue_action(judged(Outcome.FAILURE, tricky, unit_tests=Outcome.FAILURE), held(), (),
                                   link()).body

        self.assertIn('```` Say `x` and ``` then @everyone ````', body)


class ReadingWhatTheWorkflowGathered(unittest.TestCase):
    def test_open_issues_are_read_from_the_json_gh_printed(self):
        self.assertEqual((issue(12), issue(7, 'Launch Sim')),
                         issues.open_issues('[{"number": 12, "title": "%s"}, {"number": 7, "title": "Launch Sim"}]'
                                            % issues.TITLE))

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
    def test_each_issue_action_hands_the_workflow_what_it_needs_to_carry_it_out(self):
        self.assertEqual('action=open\ntitle=IMPORTANT: develop CI Failures\n',
                         issues.action_outputs(Open(issues.TITLE, 'body')))
        self.assertEqual('action=comment\nissues=12\n', issues.action_outputs(CommentOn(12, 'body')))
        self.assertEqual('action=close\nissues=12 41\n', issues.action_outputs(Close((12, 41), 'body')))
        self.assertEqual('action=none\n', issues.action_outputs(LeaveAlone()))


if __name__ == '__main__':
    unittest.main()
