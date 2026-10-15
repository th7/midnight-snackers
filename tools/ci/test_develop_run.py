import base64
import unittest

import develop_run as run
from develop_run import (CheckDidNotPass, Checked, Green, Hold, JudgingEnded, Judged, Looked,
                         MainHasWhatDevelopLacks, MainRefusedThePush, Move, NeverJudged, NeverLooked,
                         NothingJudged, Outcome, PromotingEnded, Red, Rejected)

JUDGED = 'a' * 40
MAIN = 'b' * 40
STRAY = 'c' * 40
SUBJECT = 'A ball comes back from a lone still compliant surface at its restitution'


def b64(text):
    return base64.b64encode(text.encode('utf-8')).decode('ascii')


def commit(hash_text=JUDGED, subject=SUBJECT):
    return run.commit_line(hash_text + ' ' + subject)


def checked(formatting=Outcome.SUCCESS, tool_tests=Outcome.SUCCESS, unit_tests=Outcome.SUCCESS):
    return Checked(formatting, tool_tests, unit_tests)


def judged(ended=Outcome.SUCCESS, **outcomes):
    return Judged(commit(), ended, checked(**outcomes))


def looked(lacking=(), pushed=Outcome.SUCCESS, said='', ended=Outcome.SUCCESS):
    return Looked(run.commit_hash(MAIN), tuple(lacking), pushed, said, ended)


class WhetherMainMoves(unittest.TestCase):
    def test_main_moves_to_the_judged_commit_when_every_check_passed_and_main_has_nothing_develop_lacks(self):
        self.assertEqual(Move(commit()), run.promotion(judged(), ()))

    def test_main_holds_naming_the_check_that_failed_and_the_checks_it_kept_from_running(self):
        self.assertEqual(
            Hold((CheckDidNotPass(run.FORMATTING, Outcome.FAILURE),
                  CheckDidNotPass(run.TOOL_TESTS, Outcome.SKIPPED),
                  CheckDidNotPass(run.UNIT_TESTS, Outcome.SKIPPED))),
            run.promotion(judged(Outcome.FAILURE, formatting=Outcome.FAILURE,
                                 tool_tests=Outcome.SKIPPED, unit_tests=Outcome.SKIPPED), ()))

    def test_main_holds_when_main_has_commits_develop_lacks_though_every_check_passed(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual(Hold((MainHasWhatDevelopLacks((stray,)),)), run.promotion(judged(), (stray,)))

    def test_main_holds_for_a_failed_check_and_for_what_main_lacks_together(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual(
            Hold((CheckDidNotPass(run.UNIT_TESTS, Outcome.FAILURE), MainHasWhatDevelopLacks((stray,)))),
            run.promotion(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), (stray,)))

    def test_main_holds_when_the_judging_job_ended_badly_after_every_check_passed(self):
        self.assertEqual(Hold((JudgingEnded(Outcome.FAILURE),)), run.promotion(judged(Outcome.FAILURE), ()))

    def test_main_holds_when_the_judging_job_ended_before_its_checks_reported(self):
        self.assertEqual(
            Hold((CheckDidNotPass(run.FORMATTING, Outcome.UNREPORTED),
                  CheckDidNotPass(run.TOOL_TESTS, Outcome.UNREPORTED),
                  CheckDidNotPass(run.UNIT_TESTS, Outcome.UNREPORTED),
                  JudgingEnded(Outcome.CANCELLED))),
            run.promotion(judged(Outcome.CANCELLED, formatting=Outcome.UNREPORTED,
                                 tool_tests=Outcome.UNREPORTED, unit_tests=Outcome.UNREPORTED), ()))

    def test_main_holds_when_no_commit_was_judged(self):
        self.assertEqual(Hold((NothingJudged(Outcome.FAILURE),)),
                         run.promotion(NeverJudged(Outcome.FAILURE), ()))

    def test_a_check_that_passed_in_a_job_that_passed_is_never_a_reason_to_hold(self):
        self.assertIsInstance(run.promotion(judged(), ()), Move)


class WhatARunForDevelopEndsAs(unittest.TestCase):
    def test_a_run_that_moved_main_to_the_judged_commit_is_green(self):
        self.assertEqual(Green(commit()), run.verdict(judged(), looked()))

    def test_a_run_whose_push_main_refused_is_red_with_what_git_said(self):
        said = ' ! [remote rejected] %s -> main (protected branch hook declined)\n' % JUDGED

        self.assertEqual(Red((MainRefusedThePush(commit(), said),), looked(pushed=Outcome.FAILURE, said=said)),
                         run.verdict(judged(), looked(pushed=Outcome.FAILURE, said=said)))

    def test_a_run_that_held_main_is_red_for_the_reasons_it_held(self):
        promoting = looked(pushed=Outcome.SKIPPED)

        self.assertEqual(Red((CheckDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),), promoting),
                         run.verdict(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), promoting))

    def test_a_run_whose_mover_ended_before_it_looked_at_main_is_red_for_that_and_for_the_judging(self):
        promoting = NeverLooked(Outcome.FAILURE)

        self.assertEqual(Red((CheckDidNotPass(run.UNIT_TESTS, Outcome.FAILURE), PromotingEnded(Outcome.FAILURE)),
                             promoting),
                         run.verdict(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), promoting))

    def test_a_run_that_should_have_moved_main_but_never_pushed_is_red(self):
        promoting = looked(pushed=Outcome.UNREPORTED, ended=Outcome.FAILURE)

        self.assertEqual(Red((PromotingEnded(Outcome.FAILURE),), promoting), run.verdict(judged(), promoting))


class WhatTheReasonsSay(unittest.TestCase):
    def test_a_failed_check_is_named_with_the_command_that_failed(self):
        said = run.reasons((CheckDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),))

        self.assertIn('**%s** failed: `%s`' % (run.UNIT_TESTS.name, run.UNIT_TESTS.command), said)

    def test_a_check_kept_from_running_is_said_not_to_have_run(self):
        said = run.reasons((CheckDidNotPass(run.TOOL_TESTS, Outcome.SKIPPED),))

        self.assertIn('**%s** did not run' % run.TOOL_TESTS.name, said)

    def test_what_main_lacks_is_named_commit_by_commit_with_the_merge_that_fixes_it(self):
        said = run.reasons((MainHasWhatDevelopLacks((commit(STRAY, 'Fix the README on main'),)),))

        self.assertIn('main has 1 commit develop lacks', said)
        self.assertIn('%s Fix the README on main' % STRAY, said)
        self.assertIn('every run is red until develop holds', said)
        self.assertIn('On a branch cut from the latest develop', said)
        self.assertIn('git fetch origin && git merge origin/main', said)
        self.assertIn('push that merge to develop as it is', said)

    def test_a_refused_push_is_said_with_what_git_said(self):
        git_said = 'remote: error: GH006: Protected branch update failed for refs/heads/main.\n'
        said = run.reasons((MainRefusedThePush(commit(), git_said),))

        self.assertIn('main refused the push of `%s`' % JUDGED, said)
        self.assertIn('```\n' + git_said + '```', said)

    def test_a_run_that_judged_nothing_says_so(self):
        self.assertIn('No commit was judged', run.reasons((NothingJudged(Outcome.FAILURE),)))

    def test_a_judging_job_that_ended_badly_says_how_it_ended(self):
        self.assertIn('ended `cancelled`', run.reasons((JudgingEnded(Outcome.CANCELLED),)))

    def test_a_mover_that_ended_before_it_pushed_says_how_it_ended(self):
        self.assertIn('ended `failure` before it pushed', run.reasons((PromotingEnded(Outcome.FAILURE),)))

    def test_backticks_in_what_git_said_never_close_its_quote_early(self):
        git_said = 'remote: ``` and more\n'

        self.assertIn('````\n' + git_said + '````', run.reasons((MainRefusedThePush(commit(), git_said),)))

    def test_a_main_with_more_commits_than_can_be_listed_names_the_first_fifty_and_counts_the_rest(self):
        strays = tuple(commit('%040x' % n, 'stray %d' % n) for n in range(1, 61))
        said = run.reasons((MainHasWhatDevelopLacks(strays),))

        self.assertIn('main has 60 commits develop lacks', said)
        self.assertIn('stray 50\n', said)
        self.assertNotIn('stray 51\n', said)
        self.assertIn('and 10 more', said)

    def test_each_reason_is_a_paragraph_of_its_own(self):
        said = run.reasons((CheckDidNotPass(run.FORMATTING, Outcome.FAILURE), JudgingEnded(Outcome.FAILURE)))

        self.assertEqual(2, len(said.split('\n\n')))


class ReadingWhatTheWorkflowGathered(unittest.TestCase):
    def test_an_outcome_is_one_github_reports_or_nothing(self):
        self.assertEqual([Outcome.SUCCESS, Outcome.FAILURE, Outcome.CANCELLED, Outcome.SKIPPED, Outcome.UNREPORTED],
                         [run.outcome(text) for text in ('success', 'failure', 'cancelled', 'skipped', '')])

    def test_any_other_outcome_is_refused(self):
        self.assertIsInstance(run.outcome('passed'), Rejected)

    def test_a_commit_hash_is_forty_or_sixty_four_lowercase_hex_digits(self):
        self.assertEqual(JUDGED, run.commit_hash(JUDGED).text)
        self.assertEqual('d' * 64, run.commit_hash('d' * 64).text)
        for wrong in ('A' * 40, 'a' * 39, 'g' * 40, ''):
            self.assertIsInstance(run.commit_hash(wrong), Rejected, wrong)

    def test_a_judgement_with_no_commit_named_is_one_that_judged_nothing(self):
        self.assertEqual(NeverJudged(Outcome.FAILURE), run.judgement('failure', '', '', '', '', ''))

    def test_a_judgement_reads_the_subject_git_printed_without_its_newline(self):
        self.assertEqual(Judged(commit(), Outcome.SUCCESS, checked()),
                         run.judgement('success', JUDGED, b64(SUBJECT + '\n'), 'success', 'success', 'success'))

    def test_a_subject_that_is_not_base64_is_refused(self):
        self.assertIsInstance(run.judgement('success', JUDGED, 'not base64!', 'success', 'success', 'success'),
                              Rejected)

    def test_what_main_lacks_is_read_from_the_lines_git_log_printed(self):
        printed = '%s Fix the README on main\n%s Bump a version\n' % (STRAY, MAIN)

        self.assertEqual((commit(STRAY, 'Fix the README on main'), commit(MAIN, 'Bump a version')),
                         run.lacking(b64(printed)))
        self.assertEqual((), run.lacking(''))

    def test_a_line_of_what_main_lacks_that_names_no_commit_is_refused(self):
        self.assertIsInstance(run.lacking(b64('Fix the README on main\n')), Rejected)

    def test_a_mover_that_named_no_main_tip_never_looked_at_main(self):
        self.assertEqual(NeverLooked(Outcome.FAILURE), run.promoting('failure', '', '', '', ''))

    def test_a_mover_that_looked_reads_what_main_lacks_and_what_git_said(self):
        self.assertEqual(looked(pushed=Outcome.FAILURE, said='refused\n', ended=Outcome.FAILURE),
                         run.promoting('failure', MAIN, '', 'failure', b64('refused\n')))


class WhatTheWorkflowIsHanded(unittest.TestCase):
    def test_a_move_hands_the_workflow_the_commit_to_push(self):
        self.assertEqual('decision=move\ncommit=%s\n' % JUDGED, run.promotion_outputs(Move(commit())))

    def test_a_hold_hands_the_workflow_no_commit(self):
        self.assertEqual('decision=hold\n', run.promotion_outputs(Hold((NothingJudged(Outcome.FAILURE),))))

    def test_the_log_of_a_move_names_the_commit_main_moves_to(self):
        self.assertEqual('main moves to `%s` `%s`.' % (JUDGED, SUBJECT), run.explained(Move(commit())))

    def test_the_log_of_a_hold_gives_its_reasons(self):
        hold = Hold((CheckDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),))

        self.assertEqual('main holds where it was:\n\n' + run.reasons(hold.findings), run.explained(hold))


if __name__ == '__main__':
    unittest.main()
