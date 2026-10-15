import base64
import unittest

import develop_run as run
from develop_run import (Green, Hold, JudgingEnded, Judged, Looked, LookingEnded, StepDidNotPass, StepsEnded,
                         MainHasWhatDevelopLacks, MainRefusedThePush, Move, NeverJudged, NeverLooked,
                         NothingJudged, Outcome, PromotingEnded, Red, Rejected, Stay, Stayed,
                         MainNeedsAMaintainerForWorkflows, WorkflowFiles)

JUDGED = 'a' * 40
MAIN = 'b' * 40
STRAY = 'c' * 40
SUBJECT = 'A ball comes back from a lone still compliant surface at its restitution'
NO_WORKFLOWS = WorkflowFiles(())
TESTS_YML = WorkflowFiles(('.github/workflows/tests.yml',))
REFUSALS = (
    ' ! [remote rejected] %s -> main (protected branch hook declined)\n' % JUDGED,
    ' ! [rejected]        %s -> main (fetch first)\n' % JUDGED,
    "fatal: Authentication failed for 'https://github.com/th7/midnight-snackers/'\n",
    ' ! [remote rejected] %s -> main (refusing to allow a GitHub App to create or update workflow '
    '`.github/workflows/tests.yml` without `workflows` permission)\n' % JUDGED)


GITHUB_TOKEN_PERMISSIONS = 'https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax#permissions'
PARTIAL_RERUNS = 'https://github.blog/2022-03-16-save-time-partial-re-runs-github-actions/'


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


def judged(ended=Outcome.SUCCESS, **outcomes):
    return Judged(commit(), ended, steps_ended(**outcomes))


def looked(lacking=(), pushed=Outcome.SUCCESS, said='', ended=Outcome.SUCCESS, ahead=(), workflows=NO_WORKFLOWS):
    return Looked(run.commit_hash(MAIN), tuple(lacking), tuple(ahead), workflows, pushed, said, ended)


class WhetherMainMoves(unittest.TestCase):
    def test_main_moves_to_the_judged_commit_when_every_check_passed_and_main_has_nothing_develop_lacks(self):
        self.assertEqual(Move(commit()), run.promotion(judged(), (), (), NO_WORKFLOWS))

    def test_main_holds_naming_the_check_that_failed_and_the_checks_it_kept_from_running(self):
        self.assertEqual(
            Hold((StepDidNotPass(run.FORMATTING, Outcome.FAILURE),
                  StepDidNotPass(run.TOOL_TESTS, Outcome.SKIPPED),
                  StepDidNotPass(run.UNIT_TESTS, Outcome.SKIPPED))),
            run.promotion(judged(Outcome.FAILURE, formatting=Outcome.FAILURE,
                                 tool_tests=Outcome.SKIPPED, unit_tests=Outcome.SKIPPED), (), (), NO_WORKFLOWS))

    def test_main_holds_when_main_has_commits_develop_lacks_though_every_check_passed(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual(Hold((MainHasWhatDevelopLacks((stray,)),)), run.promotion(judged(), (stray,), (), NO_WORKFLOWS))

    def test_main_holds_for_a_failed_check_and_for_what_main_lacks_together(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual(
            Hold((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE), MainHasWhatDevelopLacks((stray,)))),
            run.promotion(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), (stray,), (), NO_WORKFLOWS))

    def test_main_holds_when_the_judging_job_ended_badly_after_every_check_passed(self):
        self.assertEqual(Hold((JudgingEnded(Outcome.FAILURE),)), run.promotion(judged(Outcome.FAILURE), (), (), NO_WORKFLOWS))

    def test_main_holds_when_the_judging_job_ended_before_its_checks_reported(self):
        self.assertEqual(
            Hold((StepDidNotPass(run.FORMATTING, Outcome.UNREPORTED),
                  StepDidNotPass(run.TOOL_TESTS, Outcome.UNREPORTED),
                  StepDidNotPass(run.UNIT_TESTS, Outcome.UNREPORTED),
                  JudgingEnded(Outcome.CANCELLED))),
            run.promotion(judged(Outcome.CANCELLED, formatting=Outcome.UNREPORTED,
                                 tool_tests=Outcome.UNREPORTED, unit_tests=Outcome.UNREPORTED), (), (), NO_WORKFLOWS))

    def test_main_holds_when_no_commit_was_judged_naming_the_step_that_failed_before_one_was(self):
        never = NeverJudged(Outcome.FAILURE, steps_ended(checkout=Outcome.FAILURE, judged=Outcome.SKIPPED,
                                                         java=Outcome.SKIPPED, gradle=Outcome.SKIPPED,
                                                         node=Outcome.SKIPPED, browser=Outcome.SKIPPED,
                                                         formatting=Outcome.SKIPPED, tool_tests=Outcome.SKIPPED,
                                                         unit_tests=Outcome.SKIPPED))

        self.assertEqual(Hold((NothingJudged(Outcome.FAILURE), StepDidNotPass(run.CHECKOUT, Outcome.FAILURE),
                               StepDidNotPass(run.FORMATTING, Outcome.SKIPPED),
                               StepDidNotPass(run.TOOL_TESTS, Outcome.SKIPPED),
                               StepDidNotPass(run.UNIT_TESTS, Outcome.SKIPPED))),
                         run.promotion(never, (), (), NO_WORKFLOWS))

    def test_main_holds_naming_a_step_before_the_checks_that_failed_and_the_checks_it_kept_from_running(self):
        self.assertEqual(
            Hold((StepDidNotPass(run.BROWSER, Outcome.FAILURE),
                  StepDidNotPass(run.FORMATTING, Outcome.SKIPPED),
                  StepDidNotPass(run.TOOL_TESTS, Outcome.SKIPPED),
                  StepDidNotPass(run.UNIT_TESTS, Outcome.SKIPPED))),
            run.promotion(judged(Outcome.FAILURE, browser=Outcome.FAILURE, formatting=Outcome.SKIPPED,
                                 tool_tests=Outcome.SKIPPED, unit_tests=Outcome.SKIPPED), (), (), NO_WORKFLOWS))

    def test_main_holds_when_the_report_could_not_be_kept_though_every_check_passed(self):
        self.assertEqual(Hold((StepDidNotPass(run.RESULTS, Outcome.FAILURE),)),
                         run.promotion(judged(Outcome.FAILURE, results=Outcome.FAILURE), (), (), NO_WORKFLOWS))

    def test_a_step_cancelled_while_it_ran_is_named_with_the_checks_it_kept_from_their_end(self):
        self.assertEqual(
            Hold((StepDidNotPass(run.GRADLE, Outcome.CANCELLED),
                  StepDidNotPass(run.FORMATTING, Outcome.UNREPORTED),
                  StepDidNotPass(run.TOOL_TESTS, Outcome.UNREPORTED),
                  StepDidNotPass(run.UNIT_TESTS, Outcome.UNREPORTED),
                  JudgingEnded(Outcome.CANCELLED))),
            run.promotion(judged(Outcome.CANCELLED, gradle=Outcome.CANCELLED, node=Outcome.UNREPORTED,
                                 browser=Outcome.UNREPORTED, formatting=Outcome.UNREPORTED,
                                 tool_tests=Outcome.UNREPORTED, unit_tests=Outcome.UNREPORTED,
                                 results=Outcome.UNREPORTED), (), (), NO_WORKFLOWS))

    def test_main_stays_where_it_is_when_it_already_has_commits_of_develop_the_judged_commit_lacks(self):
        later = commit(STRAY, 'A later commit of develop')

        self.assertEqual(Stay(commit(), (later,)), run.promotion(judged(), (), (later,), NO_WORKFLOWS))

    def test_main_holds_for_commits_develop_lacks_though_main_also_has_commits_the_judged_commit_lacks(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual(Hold((MainHasWhatDevelopLacks((stray,)),)), run.promotion(judged(), (stray,), (stray,), NO_WORKFLOWS))

    def test_main_holds_for_a_failed_check_though_main_already_has_commits_the_judged_commit_lacks(self):
        later = commit(STRAY, 'A later commit of develop')

        self.assertEqual(Hold((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),)),
                         run.promotion(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), (), (later,), NO_WORKFLOWS))

    def test_main_holds_for_a_maintainer_when_the_commits_since_main_change_a_workflow_though_every_check_passed(self):
        self.assertEqual(Hold((MainNeedsAMaintainerForWorkflows(commit(), TESTS_YML),)),
                         run.promotion(judged(), (), (), TESTS_YML))

    def test_main_stays_where_it_is_when_it_already_has_later_commits_of_develop_whatever_the_judged_commit_changes(self):
        later = commit(STRAY, 'A later commit of develop')

        self.assertEqual(Stay(commit(), (later,)), run.promotion(judged(), (), (later,), TESTS_YML))

    def test_main_holds_for_a_failed_check_alone_when_the_commits_since_main_also_change_a_workflow(self):
        self.assertEqual(Hold((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),)),
                         run.promotion(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), (), (), TESTS_YML))

    def test_main_holds_for_what_develop_lacks_alone_when_the_commits_since_main_also_change_a_workflow(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual(Hold((MainHasWhatDevelopLacks((stray,)),)), run.promotion(judged(), (stray,), (), TESTS_YML))

    def test_a_check_that_passed_in_a_job_that_passed_is_never_a_reason_to_hold(self):
        self.assertIsInstance(run.promotion(judged(), (), (), NO_WORKFLOWS), Move)


class WhatARunForDevelopEndsAs(unittest.TestCase):
    def test_a_run_that_moved_main_to_the_judged_commit_is_green(self):
        self.assertEqual(Green(commit()), run.verdict(judged(), looked()))

    def test_a_run_that_left_main_where_it_already_had_later_commits_of_develop_stayed(self):
        later = commit(STRAY, 'A later commit of develop')

        self.assertEqual(Stayed(commit()), run.verdict(judged(), looked(ahead=(later,), pushed=Outcome.SKIPPED)))

    def test_a_run_that_held_main_for_a_maintainer_over_workflows_is_red_for_that_having_pushed_nothing(self):
        promoting = looked(pushed=Outcome.SKIPPED, ended=Outcome.FAILURE, workflows=TESTS_YML)

        self.assertEqual(Red((MainNeedsAMaintainerForWorkflows(commit(), TESTS_YML),), promoting),
                         run.verdict(judged(), promoting))

    def test_a_run_whose_push_main_refused_is_red_with_what_git_said(self):
        said = ' ! [remote rejected] %s -> main (protected branch hook declined)\n' % JUDGED

        promoting = looked(pushed=Outcome.FAILURE, said=said, ended=Outcome.FAILURE)

        self.assertEqual(Red((MainRefusedThePush(commit(), said),), promoting), run.verdict(judged(), promoting))

    def test_a_push_main_refused_is_a_refused_push_with_git_s_own_words_whatever_they_are(self):
        for said in REFUSALS:
            promoting = looked(pushed=Outcome.FAILURE, said=said, ended=Outcome.FAILURE)

            self.assertEqual(Red((MainRefusedThePush(commit(), said),), promoting), run.verdict(judged(), promoting),
                             said)

    def test_a_run_that_held_main_is_red_for_the_reasons_it_held(self):
        promoting = looked(pushed=Outcome.SKIPPED, ended=Outcome.FAILURE)

        self.assertEqual(Red((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),), promoting),
                         run.verdict(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), promoting))

    def test_a_run_whose_look_at_main_ended_before_it_found_main_is_red_for_that_and_for_the_judging(self):
        promoting = NeverLooked(Outcome.FAILURE)

        self.assertEqual(Red((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE), LookingEnded(Outcome.FAILURE)),
                             promoting),
                         run.verdict(judged(Outcome.FAILURE, unit_tests=Outcome.FAILURE), promoting))

    def test_a_run_that_should_have_moved_main_but_never_pushed_is_red(self):
        promoting = looked(pushed=Outcome.UNREPORTED, ended=Outcome.FAILURE)

        self.assertEqual(Red((PromotingEnded(Outcome.FAILURE),), promoting), run.verdict(judged(), promoting))

    def test_a_run_that_should_have_moved_main_but_skipped_the_push_is_red_for_how_its_mover_ended(self):
        promoting = looked(pushed=Outcome.SKIPPED, ended=Outcome.FAILURE)

        self.assertEqual(Red((PromotingEnded(Outcome.FAILURE),), promoting), run.verdict(judged(), promoting))


class WhatTheReasonsSay(unittest.TestCase):
    def test_a_failed_check_is_named_with_the_command_that_failed(self):
        said = run.reasons((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),))

        self.assertIn('**%s** failed: `%s`' % (run.UNIT_TESTS.name, run.UNIT_TESTS.command), said)

    def test_a_failed_step_that_is_not_a_check_is_named_by_its_step_name(self):
        self.assertEqual('**%s** failed.' % run.BROWSER.name, run.reasons((StepDidNotPass(run.BROWSER, Outcome.FAILURE),)))

    def test_a_step_cancelled_while_it_ran_is_said_so(self):
        self.assertEqual('**%s** did not run to its end: its outcome is `cancelled`.' % run.GRADLE.name,
                         run.reasons((StepDidNotPass(run.GRADLE, Outcome.CANCELLED),)))

    def test_a_check_kept_from_running_is_said_not_to_have_run(self):
        said = run.reasons((StepDidNotPass(run.TOOL_TESTS, Outcome.SKIPPED),))

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

    def test_a_hold_over_workflows_names_each_file_and_gives_the_maintainer_the_push_and_the_rerun_that_end_it(self):
        files = WorkflowFiles(('.github/workflows/release.yml', '.github/workflows/tests.yml'))
        said = run.reasons((MainNeedsAMaintainerForWorkflows(commit(), files),))

        self.assertIn('the commits from main to `%s` change 2 workflow files' % JUDGED, said)
        self.assertIn('```\n.github/workflows/release.yml\n.github/workflows/tests.yml\n```', said)
        self.assertIn("GitHub's built-in token, the only credential the job that moves main has, may not move a branch "
                      'past a change to a workflow file, so nothing was pushed', said,
                      'no permission a workflow grants its token is one over workflows: %s' % GITHUB_TOKEN_PERMISSIONS)
        self.assertIn('No commit to develop and no session can clear this', said)
        self.assertIn('it waits on a maintainer, who pushes main to `%s` by hand with a credential that may update '
                      'workflows, a plain push, never forced:' % JUDGED, said)
        self.assertIn('```\ngit push origin %s:refs/heads/main\n```' % JUDGED, said)
        self.assertIn('and then re-runs the failed jobs of this run, which re-runs the job that writes this issue too',
                      said, 'a partial re-run runs again every job that depends on one it re-runs: %s' % PARTIAL_RERUNS)
        self.assertIn('finds main at the commit this run judged and closes this issue', said)

    def test_a_hold_over_one_workflow_file_says_exactly_this_and_nothing_more(self):
        self.assertEqual(
            "main needs a maintainer: the commits from main to `%s` change 1 workflow file, and GitHub's built-in token, "
            'the only credential the job that moves main has, may not move a branch past a change to a workflow file, so '
            'nothing was pushed:\n\n```\n.github/workflows/tests.yml\n```\n\nNo commit to develop and no session can '
            'clear this, since every later commit of develop carries these commits too: it waits on a maintainer, who '
            'pushes main to `%s` by hand with a credential that may update workflows, a plain push, never forced:\n\n'
            '```\ngit push origin %s:refs/heads/main\n```\n\nand then re-runs the failed jobs of this run, which re-runs '
            'the job that writes this issue too: the re-run finds main at the commit this run judged and closes this '
            'issue.' % (JUDGED, JUDGED, JUDGED),
            run.reasons((MainNeedsAMaintainerForWorkflows(commit(), TESTS_YML),)))

    def test_a_hold_over_more_workflow_files_than_can_be_listed_names_the_first_fifty_and_counts_the_rest(self):
        files = WorkflowFiles(tuple(sorted('.github/workflows/w%02d.yml' % n for n in range(1, 61))))
        said = run.reasons((MainNeedsAMaintainerForWorkflows(commit(), files),))

        self.assertIn('change 60 workflow files', said)
        self.assertIn('w50.yml\n', said)
        self.assertNotIn('w51.yml', said)
        self.assertIn('and 10 more.', said)

    def test_a_run_that_judged_nothing_says_so(self):
        self.assertIn('No commit was judged', run.reasons((NothingJudged(Outcome.FAILURE),)))

    def test_a_judging_job_that_ended_badly_says_how_it_ended(self):
        self.assertIn('ended `cancelled`', run.reasons((JudgingEnded(Outcome.CANCELLED),)))

    def test_a_mover_that_ended_before_it_pushed_says_how_it_ended(self):
        self.assertEqual('The job that moves main ended `failure` before it pushed.',
                         run.reasons((PromotingEnded(Outcome.FAILURE),)))

    def test_a_look_at_main_that_ended_before_it_found_main_says_how_it_ended(self):
        self.assertEqual('The job that looks at main ended `cancelled` before it found where main stands.',
                         run.reasons((LookingEnded(Outcome.CANCELLED),)))
        self.assertEqual('The job that looks at main ended `unreported` before it found where main stands.',
                         run.reasons((LookingEnded(Outcome.UNREPORTED),)))

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
        said = run.reasons((StepDidNotPass(run.FORMATTING, Outcome.FAILURE), JudgingEnded(Outcome.FAILURE)))

        self.assertEqual(2, len(said.split('\n\n')))


class ReadingWhatTheWorkflowGathered(unittest.TestCase):
    def test_an_outcome_is_one_github_reports_or_nothing(self):
        self.assertEqual([Outcome.SUCCESS, Outcome.FAILURE, Outcome.CANCELLED, Outcome.SKIPPED, Outcome.UNREPORTED],
                         [run.outcome(text) for text in ('success', 'failure', 'cancelled', 'skipped', '')])

    def test_any_other_outcome_is_refused(self):
        self.assertIsInstance(run.outcome('passed'), Rejected)

    def test_an_outcome_that_is_not_text_at_all_is_refused_by_the_same_rule(self):
        self.assertEqual(Rejected("an outcome is success, failure, cancelled, skipped or nothing, never "
                                  "('success', 'failure')"), run.outcome(('success', 'failure')))

    def test_a_commit_hash_is_forty_or_sixty_four_lowercase_hex_digits(self):
        self.assertEqual(JUDGED, run.commit_hash(JUDGED).text)
        self.assertEqual('d' * 64, run.commit_hash('d' * 64).text)
        for wrong in ('A' * 40, 'a' * 39, 'g' * 40, ''):
            self.assertIsInstance(run.commit_hash(wrong), Rejected, wrong)

    def test_a_subject_whose_base64_holds_a_character_beyond_ascii_is_refused_by_the_rule(self):
        self.assertEqual(Rejected("what the workflow gathered is base64, and 'é' is not"),
                         run.judgement('success', JUDGED, 'é', texts()))

    def test_a_control_character_in_a_subject_is_shown_escaped_so_the_subject_stays_one_line(self):
        read = run.judgement('success', JUDGED, b64('Fix the thing\r```\rrest\x1b[31m\u2028end\n'), texts())

        self.assertEqual('Fix the thing\\x0d```\\x0drest\\x1b[31m\\u2028end', read.commit.subject)

    def test_a_subject_longer_than_can_be_shown_is_cut_short_saying_how_much_was_left_out(self):
        read = run.judgement('success', JUDGED, b64('x' * 70000 + '\n'), texts())

        self.assertEqual('x' * run.SUBJECT_SHOWN + '… (%d more characters)' % (70000 - run.SUBJECT_SHOWN),
                         read.commit.subject)

    def test_a_line_of_what_main_lacks_is_shown_escaped_and_cut_short_as_a_subject_is(self):
        printed = '%s Fix\rit%s\n' % (STRAY, 'y' * 1000)

        self.assertEqual('Fix\\x0dit' + 'y' * (run.SUBJECT_SHOWN - 9) + '… (809 more characters)',
                         run.listed_commits(b64(printed))[0].subject)

    def test_a_subject_is_never_cut_inside_the_escape_of_a_character(self):
        read = run.judgement('success', JUDGED, b64('x' * (run.SUBJECT_SHOWN - 2) + '\r\n'), texts())

        self.assertEqual('x' * (run.SUBJECT_SHOWN - 2) + '… (1 more character)', read.commit.subject)

    def test_what_git_said_ends_its_lines_where_a_carriage_return_did_and_shows_other_control_characters_escaped(self):
        read = run.promoting('success', MAIN, '', '', '', 'failure', 'failure',
                             b64('remote: one\r\nremote: two\rthree\x00\x1b[0m\tok\n'))

        self.assertEqual('remote: one\nremote: two\nthree\\x00\\x1b[0m\tok\n', read.said)

    def test_what_git_said_longer_than_can_be_shown_is_cut_short_saying_how_much_was_left_out(self):
        read = run.promoting('success', MAIN, '', '', '', 'failure', 'failure', b64('z' * 70000))

        self.assertEqual('z' * run.SAID_SHOWN + '\n… (%d more characters)\n' % (70000 - run.SAID_SHOWN), read.said)

    def test_a_judgement_with_no_commit_named_is_one_that_judged_nothing(self):
        before_naming = dict(checkout=Outcome.FAILURE, judged=Outcome.SKIPPED, java=Outcome.SKIPPED,
                             gradle=Outcome.SKIPPED, node=Outcome.SKIPPED, browser=Outcome.SKIPPED,
                             formatting=Outcome.SKIPPED, tool_tests=Outcome.SKIPPED, unit_tests=Outcome.SKIPPED)

        self.assertEqual(NeverJudged(Outcome.FAILURE, steps_ended(**before_naming)),
                         run.judgement('failure', '', '', texts(**before_naming)))

    def test_a_judgement_reads_the_subject_git_printed_without_its_newline(self):
        self.assertEqual(Judged(commit(), Outcome.SUCCESS, steps_ended()),
                         run.judgement('success', JUDGED, b64(SUBJECT + '\n'), texts()))

    def test_a_judgement_reads_one_outcome_for_each_step_of_the_judging_job(self):
        for wrong in (texts()[:-1], texts() + ('success',)):
            self.assertEqual(Rejected('a judgement hands one outcome for each of the %d steps of the judging job'
                                      % len(run.STEPS)),
                             run.judgement('success', JUDGED, b64(SUBJECT + '\n'), wrong))
        passed = tuple('passed' if step is run.JAVA else 'success' for step in run.STEPS)

        self.assertIsInstance(run.judgement('success', JUDGED, b64(SUBJECT + '\n'), passed), Rejected)

    def test_a_subject_that_is_not_base64_is_refused(self):
        self.assertIsInstance(run.judgement('success', JUDGED, 'not base64!', texts()),
                              Rejected)

    def test_what_main_lacks_is_read_from_the_lines_git_log_printed(self):
        printed = '%s Fix the README on main\n%s Bump a version\n' % (STRAY, MAIN)

        self.assertEqual((commit(STRAY, 'Fix the README on main'), commit(MAIN, 'Bump a version')),
                         run.listed_commits(b64(printed)))
        self.assertEqual((), run.listed_commits(''))

    def test_a_line_of_what_main_lacks_that_names_no_commit_is_refused(self):
        self.assertIsInstance(run.listed_commits(b64('Fix the README on main\n')), Rejected)

    def test_a_look_that_named_no_main_tip_never_found_main_and_ended_as_the_job_that_looks_did(self):
        self.assertEqual(NeverLooked(Outcome.CANCELLED), run.promoting('cancelled', '', '', '', '', 'failure', 'skipped', ''))

    def test_a_mover_that_looked_reads_what_main_lacks_and_what_git_said(self):
        self.assertEqual(looked(pushed=Outcome.FAILURE, said='refused\n', ended=Outcome.FAILURE),
                         run.promoting('success', MAIN, '', '', '', 'failure', 'failure', b64('refused\n')))

    def test_a_mover_that_looked_reads_what_main_has_beyond_the_judged_commit(self):
        later = commit(STRAY, 'A later commit of develop')

        self.assertEqual(looked(ahead=(later,), pushed=Outcome.SKIPPED),
                         run.promoting('success', MAIN, '', b64('%s A later commit of develop\n' % STRAY), '', 'success',
                                       'skipped', ''))

    def test_a_mover_that_looked_reads_the_workflow_files_the_commits_since_main_change(self):
        self.assertEqual(looked(pushed=Outcome.SKIPPED, ended=Outcome.FAILURE, workflows=TESTS_YML),
                         run.promoting('failure', MAIN, '', '', b64('.github/workflows/tests.yml\n'), 'failure', 'skipped',
                                       ''))

    def test_the_workflow_files_are_read_from_the_lines_git_log_printed_each_once_and_in_order(self):
        printed = ('.github/workflows/tests.yml\n\n.github/workflows/release.yml\n.github/workflows/tests.yml\n\n'
                   '.github/workflows/tests.yml\n')

        self.assertEqual(WorkflowFiles(('.github/workflows/release.yml', '.github/workflows/tests.yml')),
                         run.workflow_files(b64(printed)))
        self.assertEqual(NO_WORKFLOWS, run.workflow_files(''))

    def test_a_workflow_file_git_quoted_is_read_as_git_printed_it(self):
        printed = '".github/workflows/caf\\303\\251.yml"\n'

        self.assertEqual(WorkflowFiles(('".github/workflows/caf\\303\\251.yml"',)), run.workflow_files(b64(printed)))

    def test_a_workflow_file_longer_than_can_be_shown_is_cut_short_saying_how_much_was_left_out(self):
        printed = '.github/workflows/%s.yml\n' % ('x' * 1000)

        self.assertEqual(('.github/workflows/' + 'x' * (run.SUBJECT_SHOWN - 18) + '… (822 more characters)',),
                         run.workflow_files(b64(printed)).paths)

    def test_a_line_that_names_no_file_under_the_workflows_directory_is_refused_by_the_rule(self):
        for printed in ('README.md\n', '%s Change the workflow\n' % STRAY, '.github/workflowsx/tests.yml\n'):
            self.assertEqual(Rejected(run.WORKFLOW_FILES_RULE), run.workflow_files(b64(printed)), printed)
        self.assertIsInstance(run.workflow_files('not base64!'), Rejected)


class MakingAValueDirectly(unittest.TestCase):
    def test_a_commit_hash_made_directly_is_held_to_the_rule_its_reader_checks(self):
        self.assertEqual(JUDGED, run.CommitHash(JUDGED).text)
        self.assertEqual(Rejected("a commit hash is 40 or 64 lowercase hexadecimal digits, never 'not a hash'"),
                         run.CommitHash('not a hash'))
        self.assertIsInstance(run.CommitHash(None), Rejected)

    def test_a_commit_made_directly_is_held_to_a_hash_and_a_one_line_subject(self):
        self.assertEqual(commit(), run.Commit(run.CommitHash(JUDGED), SUBJECT))
        for wrong in ('two\nlines', 'a\rcarriage return', 'a\u2028line separator', 'x' * (run.SUBJECT_SHOWN_AT_MOST + 1)):
            self.assertEqual(Rejected('a commit subject is one line of at most %d characters, none of them a control '
                                      'character or a line separator' % run.SUBJECT_SHOWN_AT_MOST),
                             run.Commit(run.CommitHash(JUDGED), wrong), wrong)
        self.assertIsInstance(run.Commit(JUDGED, SUBJECT), Rejected)
        self.assertIsInstance(run.Commit(run.CommitHash(JUDGED), None), Rejected)

    def test_what_a_mover_saw_made_directly_is_held_to_a_main_tip_commits_workflow_files_outcomes_and_text(self):
        tip = run.CommitHash(MAIN)
        none = NO_WORKFLOWS
        for wrong in ((MAIN, (), (), none, Outcome.SUCCESS, '', Outcome.SUCCESS),
                      (tip, (STRAY,), (), none, Outcome.SUCCESS, '', Outcome.SUCCESS),
                      (tip, [], (), none, Outcome.SUCCESS, '', Outcome.SUCCESS),
                      (tip, (), (STRAY,), none, Outcome.SUCCESS, '', Outcome.SUCCESS),
                      (tip, (), (), ('.github/workflows/tests.yml',), Outcome.SUCCESS, '', Outcome.SUCCESS),
                      (tip, (), (), none, 'success', '', Outcome.SUCCESS),
                      (tip, (), (), none, Outcome.SUCCESS, None, Outcome.SUCCESS),
                      (tip, (), (), none, Outcome.SUCCESS, 'x' * (run.SAID_SHOWN_AT_MOST + 1), Outcome.SUCCESS),
                      (tip, (), (), none, Outcome.SUCCESS, 'a\rb', Outcome.SUCCESS)):
            self.assertIsInstance(Looked(*wrong), Rejected, wrong)
        self.assertIsInstance(Looked(tip, (), (), none, Outcome.SUCCESS, '', Outcome.SUCCESS), Looked)

    def test_what_a_mover_saw_made_directly_holds_what_git_said_only_for_a_push_that_ran(self):
        tip = run.CommitHash(MAIN)
        for never_ran in (Outcome.SKIPPED, Outcome.UNREPORTED):
            self.assertEqual(Rejected(run.SAID_ONLY_OF_A_PUSH_THAT_RAN),
                             Looked(tip, (), (), NO_WORKFLOWS, never_ran, 'refused\n', Outcome.FAILURE), never_ran)
        for ran in (Outcome.SUCCESS, Outcome.FAILURE, Outcome.CANCELLED):
            self.assertIsInstance(Looked(tip, (), (), NO_WORKFLOWS, ran, 'said\n', Outcome.CANCELLED), Looked, ran)

    def test_what_a_mover_saw_made_directly_holds_no_push_to_a_main_the_core_would_have_held_or_left_alone(self):
        tip = run.CommitHash(MAIN)
        held_or_left = ((commit(STRAY, 'Fix the README on main'),), (), NO_WORKFLOWS), \
                       ((), (commit(STRAY, 'A later commit of develop'),), NO_WORKFLOWS), ((), (), TESTS_YML)
        for pushed in (Outcome.SUCCESS, Outcome.FAILURE, Outcome.CANCELLED):
            for lacking, ahead, workflows in held_or_left:
                self.assertEqual(Rejected(run.A_PUSH_RUNS_ONLY_TO_MOVE_MAIN),
                                 Looked(tip, lacking, ahead, workflows, pushed, '', Outcome.FAILURE),
                                 (pushed, lacking, ahead, workflows))
            self.assertIsInstance(Looked(tip, (), (), NO_WORKFLOWS, pushed, '', Outcome.FAILURE), Looked)

    def test_what_a_mover_saw_made_directly_ends_in_success_only_having_moved_main_or_left_it_ahead(self):
        tip = run.CommitHash(MAIN)
        stray = commit(STRAY, 'Fix the README on main')
        later = commit(STRAY, 'A later commit of develop')
        for lacking, ahead, workflows, pushed, said in (((stray,), (), NO_WORKFLOWS, Outcome.SKIPPED, ''),
                                                        ((stray,), (later,), NO_WORKFLOWS, Outcome.SKIPPED, ''),
                                                        ((), (), TESTS_YML, Outcome.SKIPPED, ''),
                                                        ((), (), NO_WORKFLOWS, Outcome.SKIPPED, ''),
                                                        ((), (), NO_WORKFLOWS, Outcome.UNREPORTED, ''),
                                                        ((), (), NO_WORKFLOWS, Outcome.CANCELLED, 'x\n')):
            self.assertEqual(Rejected(run.A_MOVER_SUCCEEDS_ONLY_HAVING_MOVED_MAIN_OR_LEFT_IT_AHEAD),
                             Looked(tip, lacking, ahead, workflows, pushed, said, Outcome.SUCCESS),
                             (lacking, ahead, workflows, pushed))
        for ahead, workflows, pushed in (((later,), NO_WORKFLOWS, Outcome.SKIPPED),
                                         ((later,), TESTS_YML, Outcome.SKIPPED),
                                         ((), NO_WORKFLOWS, Outcome.SUCCESS)):
            self.assertIsInstance(Looked(tip, (), ahead, workflows, pushed, '', Outcome.SUCCESS), Looked,
                                  (ahead, workflows, pushed))

    def test_what_a_mover_saw_made_directly_holds_no_failed_push_in_a_mover_that_succeeded(self):
        tip = run.CommitHash(MAIN)

        self.assertEqual(Rejected(run.A_FAILED_PUSH_FAILS_ITS_JOB),
                         Looked(tip, (), (), NO_WORKFLOWS, Outcome.FAILURE, 'refused\n', Outcome.SUCCESS))

    def test_what_a_mover_hands_on_that_disagrees_with_itself_is_refused_by_the_rule_it_breaks(self):
        self.assertEqual(Rejected(run.SAID_ONLY_OF_A_PUSH_THAT_RAN),
                         run.promoting('success', MAIN, '', '', '', 'failure', 'skipped', b64('refused\n')))
        self.assertEqual(Rejected(run.A_PUSH_RUNS_ONLY_TO_MOVE_MAIN),
                         run.promoting('failure', MAIN, '', '', b64('.github/workflows/tests.yml\n'), 'failure', 'failure',
                                       ''))
        self.assertEqual(Rejected(run.A_FAILED_PUSH_FAILS_ITS_JOB),
                         run.promoting('success', MAIN, '', '', '', 'success', 'failure', b64('refused\n')))
        self.assertEqual(Rejected(run.A_MOVER_SUCCEEDS_ONLY_HAVING_MOVED_MAIN_OR_LEFT_IT_AHEAD),
                         run.promoting('failure', MAIN, '', '', b64('.github/workflows/tests.yml\n'), 'success', 'skipped',
                                       ''))

    def test_workflow_files_made_directly_are_held_to_distinct_paths_under_the_workflows_directory_in_order(self):
        self.assertEqual(('.github/workflows/a.yml', '.github/workflows/b.yml'),
                         WorkflowFiles(('.github/workflows/a.yml', '.github/workflows/b.yml')).paths)
        for wrong in (['.github/workflows/a.yml'], ('.github/workflows/b.yml', '.github/workflows/a.yml'),
                      ('.github/workflows/a.yml', '.github/workflows/a.yml'), ('README.md',), (None,),
                      ('.github/workflows/a\nb.yml',), ('.github/workflows/' + 'x' * run.SUBJECT_SHOWN_AT_MOST,), None):
            self.assertEqual(Rejected(run.WORKFLOW_FILES_RULE), WorkflowFiles(wrong), wrong)

    def test_what_main_lacks_made_directly_is_held_to_at_least_one_commit(self):
        stray = commit(STRAY, 'Fix the README on main')

        self.assertEqual((stray,), MainHasWhatDevelopLacks((stray,)).commits)
        for wrong in ((), [stray], (STRAY,), None):
            self.assertEqual(Rejected(run.WHAT_MAIN_LACKS_NAMES_ITS_COMMITS), MainHasWhatDevelopLacks(wrong), wrong)

    def test_a_refused_push_made_directly_is_held_to_the_commit_it_pushed_and_what_git_said(self):
        self.assertEqual('refused\n', MainRefusedThePush(commit(), 'refused\n').said)
        self.assertEqual(Rejected(run.A_REFUSED_PUSH_NAMES_ITS_COMMIT), MainRefusedThePush(JUDGED, 'refused\n'))
        for wrong in (None, 'a\rb', 'x' * (run.SAID_SHOWN_AT_MOST + 1)):
            self.assertEqual(Rejected(run.WHAT_GIT_SAID_RULE), MainRefusedThePush(commit(), wrong), wrong)

    def test_a_hold_over_workflows_made_directly_is_held_to_the_judged_commit_and_at_least_one_workflow_file(self):
        self.assertEqual(TESTS_YML, MainNeedsAMaintainerForWorkflows(commit(), TESTS_YML).files)
        for wrong in ((commit(), NO_WORKFLOWS), ('x', None), (JUDGED, TESTS_YML),
                      (commit(), ('.github/workflows/tests.yml',))):
            self.assertEqual(Rejected(run.A_HOLD_OVER_WORKFLOWS_NAMES_THEM), MainNeedsAMaintainerForWorkflows(*wrong),
                             wrong)

    def test_a_judgement_made_directly_is_held_to_a_commit_and_outcomes(self):
        self.assertIsInstance(Judged(JUDGED, Outcome.SUCCESS, steps_ended()), Rejected)
        self.assertIsInstance(Judged(commit(), 'success', steps_ended()), Rejected)
        self.assertIsInstance(StepsEnded(('success',) * len(run.STEPS)), Rejected)
        self.assertIsInstance(StepsEnded((Outcome.SUCCESS,) * (len(run.STEPS) - 1)), Rejected)
        self.assertIsInstance(NeverJudged('failure', steps_ended()), Rejected)
        self.assertIsInstance(NeverJudged(Outcome.FAILURE, None), Rejected)


class WhatTheWorkflowIsHanded(unittest.TestCase):
    def test_a_move_hands_the_workflow_only_the_decision_since_the_push_takes_the_commit_from_the_judging(self):
        self.assertEqual('decision=move\n', run.promotion_outputs(Move(commit())))

    def test_a_stay_hands_the_workflow_no_commit(self):
        self.assertEqual('decision=stay\n', run.promotion_outputs(Stay(commit(), (commit(STRAY, 'Later'),))))

    def test_the_log_of_a_stay_says_main_already_has_later_commits_of_develop(self):
        self.assertEqual('main stays where it is: it already has 1 commit of develop that `%s` lacks, which a run for '
                         'a later tip of develop judges.' % JUDGED,
                         run.explained(Stay(commit(), (commit(STRAY, 'Later'),))))

    def test_a_hold_hands_the_workflow_no_commit(self):
        self.assertEqual('decision=hold\n', run.promotion_outputs(Hold((NothingJudged(Outcome.FAILURE),))))

    def test_the_log_of_a_move_names_the_commit_main_moves_to(self):
        self.assertEqual('main moves to `%s` `%s`.' % (JUDGED, SUBJECT), run.explained(Move(commit())))

    def test_the_log_of_a_hold_gives_its_reasons(self):
        hold = Hold((StepDidNotPass(run.UNIT_TESTS, Outcome.FAILURE),))

        self.assertEqual('main holds where it was:\n\n' + run.reasons(hold.findings), run.explained(hold))


if __name__ == '__main__':
    unittest.main()
