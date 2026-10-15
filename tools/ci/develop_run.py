import base64
import re
import unicodedata
from dataclasses import dataclass
from enum import Enum

COMMIT_HASH = re.compile(r'[0-9a-f]{40}|[0-9a-f]{64}')
BASE64 = re.compile(r'(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?')
LISTED_AT_MOST = 50
SUBJECT_SHOWN = 200
SUBJECT_SHOWN_AT_MOST = 256
SAID_SHOWN = 4000
SAID_SHOWN_AT_MOST = 4096
WORKFLOWS_DIRECTORY = '.github/workflows/'
WORKFLOW_FILES_RULE = ('workflow files are distinct paths under .github/workflows, as git prints them, in order, each one '
                       'line of at most %d characters' % SUBJECT_SHOWN_AT_MOST)
WORKFLOWS_RULE = ('what a mover found of the workflows is the workflow files the commits since main change and, kept '
                  'apart, those main and the judged commit differ in')
WHAT_GIT_SAID_RULE = ('what git said is text of at most %d characters, with no control character but a newline or a '
                      'tab' % SAID_SHOWN_AT_MOST)
SAID_ONLY_OF_A_PUSH_THAT_RAN = 'git said something only of a push that ran, never of one skipped or unreported'
A_PUSH_RUNS_ONLY_TO_MOVE_MAIN = ('a push runs only to move main, so never when main has a commit develop or the judged '
                                 'commit lacks, nor when the commits since main change a workflow file or main and the '
                                 'judged commit differ in one')
A_FAILED_PUSH_FAILS_ITS_JOB = 'a push that failed fails the job that moves main, which never ends in success'
A_MOVER_SUCCEEDS_ONLY_HAVING_MOVED_MAIN_LEFT_IT_AHEAD_OR_FOUND_IT_MOVED = (
    'a mover ends in success only when its push succeeded, or when it pushed nothing because main stayed where it was '
    'for commits of develop it already has, having nothing develop lacks, or because main had moved since the job that '
    'looks found it, that job having found that main moves or stays')
WHAT_MAIN_LACKS_NAMES_ITS_COMMITS = 'what main has that develop lacks is named as a tuple of at least one commit'
A_REFUSED_PUSH_NAMES_ITS_COMMIT = 'a refused push names the commit it pushed'
A_HOLD_OVER_WORKFLOWS_NAMES_THEM = ('a hold for a maintainer over workflows names the judged commit and at least one '
                                    'workflow file the commits since main change or main and the judged commit differ '
                                    'in')
A_GREEN_RUN_MOVED_ON_NAMES_DEVELOP_S_TIP_OFF_THE_JUDGED_COMMIT = ("a green run that moved on names the judged commit, "
                                                                 "where the look found main and develop's tip, which is "
                                                                 "not the judged commit")
MAIN_MOVED_SINCE_THE_LOOK_ONLY_OFF_WHERE_THIS_RUN_LEFT_IT = (
    'main moved since the look only when it stands somewhere other than where this run left it, the judged commit '
    'after a push that succeeded and otherwise where the look found it')
THE_MERGE_THAT_FIXES_IT = ('On a branch cut from the latest develop, run `git fetch origin && git merge origin/main`, '
                           'and push that merge to develop as it is, without a rebase, which would drop it.')


@dataclass(frozen=True)
class Rejected:
    rule: str


class Validated:
    def __new__(cls, *fields, **named):
        broken = cls.broken(*fields, **named)
        if broken is not None:
            return Rejected(broken)
        return super().__new__(cls)


def all_of(kind, values):
    return isinstance(values, tuple) and all(isinstance(each, kind) for each in values)


class Outcome(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    CANCELLED = 'cancelled'
    SKIPPED = 'skipped'
    UNREPORTED = ''


@dataclass(frozen=True)
class CommitHash(Validated):
    text: str

    @staticmethod
    def broken(text):
        if not isinstance(text, str) or COMMIT_HASH.fullmatch(text) is None:
            return 'a commit hash is 40 or 64 lowercase hexadecimal digits, never %r' % (text,)
        return None


@dataclass(frozen=True)
class Commit(Validated):
    hash: CommitHash
    subject: str

    @staticmethod
    def broken(hash, subject):
        if not isinstance(hash, CommitHash):
            return 'a commit is named by its hash'
        if (not isinstance(subject, str) or len(subject) > SUBJECT_SHOWN_AT_MOST
                or any(breaks_a_line(character) for character in subject)):
            return ('a commit subject is one line of at most %d characters, none of them a control character or a '
                    'line separator' % SUBJECT_SHOWN_AT_MOST)
        return None


@dataclass(frozen=True)
class Step:
    step: str
    name: str


@dataclass(frozen=True)
class Check:
    step: str
    name: str
    command: str


CHECKOUT = Step('checkout', "Check out develop's tip as it is now, or the pull request")
NAMING = Step('judged', 'Name the judged commit')
JAVA = Step('java', 'Java 21')
GRADLE = Step('gradle', 'Gradle, its cache written by runs for develop and only read by pull requests')
NODE = Step('node', 'Node 22')
BROWSER = Step('browser', 'Browser for the page tests, with the shared libraries Chromium needs')
FORMATTING = Check('formatting', 'Java formatting (same command as local; fix with ./gradlew spotlessApply)',
                   './gradlew spotlessCheck')
TOOL_TESTS = Check('tool-tests', 'Tool tests (same command as local)', 'python3 tools/run_tests.py')
UNIT_TESTS = Check('unit-tests', 'Unit tests (same command as local)', './gradlew :TeamCode:testDebugUnitTest')
RESULTS = Step('results', 'Test report and simulation replays')
CHECKS = (FORMATTING, TOOL_TESTS, UNIT_TESTS)
STEPS = (CHECKOUT, NAMING, JAVA, GRADLE, NODE, BROWSER, FORMATTING, TOOL_TESTS, UNIT_TESTS, RESULTS)


@dataclass(frozen=True)
class StepsEnded(Validated):
    outcomes: tuple

    @staticmethod
    def broken(outcomes):
        if not all_of(Outcome, outcomes) or len(outcomes) != len(STEPS):
            return 'a judgement hands one outcome for each of the %d steps of the judging job' % len(STEPS)
        return None

    def each(self):
        return tuple(zip(STEPS, self.outcomes))


@dataclass(frozen=True)
class Judged(Validated):
    commit: Commit
    ended: Outcome
    steps: StepsEnded

    @staticmethod
    def broken(commit, ended, steps):
        if not isinstance(commit, Commit):
            return 'a judgement names the commit it judged'
        if not isinstance(ended, Outcome) or not isinstance(steps, StepsEnded):
            return 'a judgement says how its job ended and how each of its steps ended'
        return None


@dataclass(frozen=True)
class NeverJudged(Validated):
    ended: Outcome
    steps: StepsEnded

    @staticmethod
    def broken(ended, steps):
        if not isinstance(ended, Outcome) or not isinstance(steps, StepsEnded):
            return 'a judgement says how its job ended and how each of its steps ended'
        return None


def names_a_workflow_file(path):
    return (isinstance(path, str) and path.startswith((WORKFLOWS_DIRECTORY, '"' + WORKFLOWS_DIRECTORY))
            and len(path) <= SUBJECT_SHOWN_AT_MOST and not any(breaks_a_line(character) for character in path))


@dataclass(frozen=True)
class WorkflowFiles(Validated):
    paths: tuple

    @staticmethod
    def broken(paths):
        if (not isinstance(paths, tuple) or not all(names_a_workflow_file(path) for path in paths)
                or list(paths) != sorted(set(paths))):
            return WORKFLOW_FILES_RULE
        return None


@dataclass(frozen=True)
class Workflows(Validated):
    changed: WorkflowFiles
    differing: WorkflowFiles

    @staticmethod
    def broken(changed, differing):
        if not isinstance(changed, WorkflowFiles) or not isinstance(differing, WorkflowFiles):
            return WORKFLOWS_RULE
        return None

    def hold_main(self):
        return bool(self.changed.paths or self.differing.paths)

    def differing_alone(self):
        return tuple(path for path in self.differing.paths if path not in self.changed.paths)


def quotes_what_git_said(said):
    return (isinstance(said, str) and len(said) <= SAID_SHOWN_AT_MOST
            and not any(unsafe_in_a_quote(character) for character in said))


@dataclass(frozen=True)
class Looked(Validated):
    main_found: CommitHash
    main_tip: CommitHash
    develop_tip: CommitHash
    lacking: tuple
    ahead: tuple
    workflows: Workflows
    pushed: Outcome
    said: str
    ended: Outcome

    @staticmethod
    def broken(main_found, main_tip, develop_tip, lacking, ahead, workflows, pushed, said, ended):
        if not all_of(CommitHash, (main_found, main_tip, develop_tip)):
            return ('a mover that looked names where the look found main, main\'s tip and develop\'s tip by their '
                    'hashes')
        if not all_of(Commit, lacking) or not all_of(Commit, ahead):
            return 'what main has that develop or the judged commit lacks is a tuple of commits'
        if not isinstance(workflows, Workflows):
            return ('a mover that looked names the workflow files the commits since main change or main and the judged '
                    'commit differ in')
        if not all_of(Outcome, (pushed, ended)):
            return 'a mover says how its push and its job ended'
        if not quotes_what_git_said(said):
            return WHAT_GIT_SAID_RULE
        if said and pushed in (Outcome.SKIPPED, Outcome.UNREPORTED):
            return SAID_ONLY_OF_A_PUSH_THAT_RAN
        if pushed in (Outcome.SUCCESS, Outcome.FAILURE, Outcome.CANCELLED) and (lacking or ahead or workflows.hold_main()):
            return A_PUSH_RUNS_ONLY_TO_MOVE_MAIN
        if pushed is Outcome.FAILURE and ended is Outcome.SUCCESS:
            return A_FAILED_PUSH_FAILS_ITS_JOB
        looked_well = not lacking and (ahead or not workflows.hold_main())
        stayed = pushed is Outcome.SKIPPED and ahead and looked_well
        found_moved = pushed is Outcome.SKIPPED and looked_well and main_tip != main_found
        if ended is Outcome.SUCCESS and pushed is not Outcome.SUCCESS and not stayed and not found_moved:
            return A_MOVER_SUCCEEDS_ONLY_HAVING_MOVED_MAIN_LEFT_IT_AHEAD_OR_FOUND_IT_MOVED
        return None


@dataclass(frozen=True)
class NeverLooked:
    ended: Outcome


@dataclass(frozen=True)
class StepDidNotPass:
    step: object
    outcome: Outcome


@dataclass(frozen=True)
class JudgingEnded:
    ended: Outcome


@dataclass(frozen=True)
class NothingJudged:
    ended: Outcome


@dataclass(frozen=True)
class MainHasWhatDevelopLacks(Validated):
    commits: tuple

    @staticmethod
    def broken(commits):
        if not all_of(Commit, commits) or not commits:
            return WHAT_MAIN_LACKS_NAMES_ITS_COMMITS
        return None


@dataclass(frozen=True)
class MainRefusedThePush(Validated):
    commit: Commit
    said: str

    @staticmethod
    def broken(commit, said):
        if not isinstance(commit, Commit):
            return A_REFUSED_PUSH_NAMES_ITS_COMMIT
        if not quotes_what_git_said(said):
            return WHAT_GIT_SAID_RULE
        return None


@dataclass(frozen=True)
class MainNeedsAMaintainerForWorkflows(Validated):
    commit: Commit
    workflows: Workflows

    @staticmethod
    def broken(commit, workflows):
        if not isinstance(commit, Commit) or not isinstance(workflows, Workflows) or not workflows.hold_main():
            return A_HOLD_OVER_WORKFLOWS_NAMES_THEM
        return None


@dataclass(frozen=True)
class PromotingEnded:
    ended: Outcome


@dataclass(frozen=True)
class LookingEnded:
    ended: Outcome


@dataclass(frozen=True)
class Move:
    commit: Commit


@dataclass(frozen=True)
class Stay:
    commit: Commit
    ahead: tuple


@dataclass(frozen=True)
class Hold:
    findings: tuple


@dataclass(frozen=True)
class Green:
    commit: Commit


@dataclass(frozen=True)
class GreenButMovedOn(Validated):
    commit: Commit
    main_found: CommitHash
    develop_tip: CommitHash

    @staticmethod
    def broken(commit, main_found, develop_tip):
        if (not isinstance(commit, Commit) or not all_of(CommitHash, (main_found, develop_tip))
                or develop_tip == commit.hash):
            return A_GREEN_RUN_MOVED_ON_NAMES_DEVELOP_S_TIP_OFF_THE_JUDGED_COMMIT
        return None


@dataclass(frozen=True)
class MainMovedSinceTheLook(Validated):
    commit: Commit
    main_found: CommitHash
    main_tip: CommitHash
    pushed: Outcome

    @staticmethod
    def broken(commit, main_found, main_tip, pushed):
        if (not isinstance(commit, Commit) or not all_of(CommitHash, (main_found, main_tip))
                or not isinstance(pushed, Outcome)
                or main_tip == (commit.hash if pushed is Outcome.SUCCESS else main_found)):
            return MAIN_MOVED_SINCE_THE_LOOK_ONLY_OFF_WHERE_THIS_RUN_LEFT_IT
        return None


@dataclass(frozen=True)
class Stayed:
    commit: Commit


@dataclass(frozen=True)
class Red:
    findings: tuple
    promoting: object


def breaks_a_line(character):
    return unicodedata.category(character) in ('Cc', 'Zl', 'Zp')


def unsafe_in_a_quote(character):
    return unicodedata.category(character) == 'Cc' and character not in '\n\t'


def escaped(character):
    code = ord(character)
    return '\\x%02x' % code if code < 0x100 else '\\u%04x' % code


def more(left_out):
    return '… (%d more character%s)' % (left_out, '' if left_out == 1 else 's')


def shown_within(pieces, shown):
    kept = []
    length = 0
    for piece in pieces:
        if length + len(piece) > shown:
            break
        kept.append(piece)
        length += len(piece)
    return kept


def shown_subject(text):
    pieces = [escaped(character) if breaks_a_line(character) else character for character in text]
    kept = shown_within(pieces, SUBJECT_SHOWN)
    left_out = len(pieces) - len(kept)
    return ''.join(kept) + (more(left_out) if left_out else '')


def shown_said(text):
    lines_ended = text.replace('\r\n', '\n').replace('\r', '\n')
    pieces = [escaped(character) if unsafe_in_a_quote(character) else character for character in lines_ended]
    kept = shown_within(pieces, SAID_SHOWN)
    left_out = len(pieces) - len(kept)
    return ''.join(kept) + ('\n' + more(left_out) + '\n' if left_out else '')


def outcome(text):
    for known in Outcome:
        if known.value == text:
            return known
    return Rejected('an outcome is success, failure, cancelled, skipped or nothing, never %r' % (text,))


def commit_hash(text):
    return CommitHash(text)


def commit_line(line):
    hash_text, _, subject = line.partition(' ')
    hashed = commit_hash(hash_text)
    if isinstance(hashed, Rejected):
        return hashed
    return Commit(hashed, shown_subject(subject))


def decoded(text):
    if not isinstance(text, str) or not text.isascii() or BASE64.fullmatch(text) is None:
        return Rejected('what the workflow gathered is base64, and %r is not' % (text,))
    return base64.b64decode(text, validate=True).decode('utf-8', errors='replace')


def first_rejected(*read):
    return next((each for each in read if isinstance(each, Rejected)), None)


def steps_ended(step_texts):
    if not isinstance(step_texts, tuple) or len(step_texts) != len(STEPS):
        return Rejected('a judgement hands one outcome for each of the %d steps of the judging job' % len(STEPS))
    read = tuple(outcome(text) for text in step_texts)
    rejected = first_rejected(*read)
    if rejected is not None:
        return rejected
    return StepsEnded(read)


def judgement(ended_text, judged_text, subject_base64, step_texts):
    ended = outcome(ended_text)
    steps = steps_ended(step_texts)
    rejected = first_rejected(ended, steps)
    if rejected is not None:
        return rejected
    if judged_text == '':
        return NeverJudged(ended, steps)
    hashed = commit_hash(judged_text)
    subject = decoded(subject_base64)
    rejected = first_rejected(hashed, subject)
    if rejected is not None:
        return rejected
    line = subject[:-1] if subject.endswith('\n') else subject
    named = Commit(hashed, shown_subject(line))
    if isinstance(named, Rejected):
        return named
    return Judged(named, ended, steps)


def listed_commits(lines_base64):
    text = decoded(lines_base64)
    if isinstance(text, Rejected):
        return text
    commits = tuple(commit_line(line) for line in text.split('\n') if line != '')
    rejected = first_rejected(*commits)
    if rejected is not None:
        return rejected
    return commits


def workflow_files(lines_base64):
    text = decoded(lines_base64)
    if isinstance(text, Rejected):
        return text
    return WorkflowFiles(tuple(sorted(set(shown_subject(path) for path in text.split('\n') if path != ''))))


def workflows(changed_base64, differing_base64):
    changed = workflow_files(changed_base64)
    differing = workflow_files(differing_base64)
    rejected = first_rejected(changed, differing)
    if rejected is not None:
        return rejected
    return Workflows(changed, differing)


def promoting(looking_text, found_text, main_tip_text, develop_tip_text, lacking_base64, ahead_base64, changed_base64,
              differing_base64, ended_text, pushed_text, said_base64):
    looking = outcome(looking_text)
    ended = outcome(ended_text)
    rejected = first_rejected(looking, ended)
    if rejected is not None:
        return rejected
    if found_text == '':
        return NeverLooked(looking)
    found = commit_hash(found_text)
    main_tip = commit_hash(main_tip_text)
    develop_tip = commit_hash(develop_tip_text)
    lacking = listed_commits(lacking_base64)
    ahead = listed_commits(ahead_base64)
    found_workflows = workflows(changed_base64, differing_base64)
    pushed = outcome(pushed_text)
    said = decoded(said_base64)
    rejected = first_rejected(found, main_tip, develop_tip, lacking, ahead, found_workflows, pushed, said)
    if rejected is not None:
        return rejected
    return Looked(found, main_tip, develop_tip, lacking, ahead, found_workflows, pushed, shown_said(said), ended)


def step_findings(steps):
    return tuple(StepDidNotPass(step, did) for step, did in steps.each()
                 if (isinstance(step, Check) and did is not Outcome.SUCCESS)
                 or did in (Outcome.FAILURE, Outcome.CANCELLED))


def judging_findings(judgement_read):
    unpassed = step_findings(judgement_read.steps)
    if isinstance(judgement_read, NeverJudged):
        return (NothingJudged(judgement_read.ended),) + unpassed
    any_failed = any(finding.outcome is Outcome.FAILURE for finding in unpassed)
    ended_badly = judgement_read.ended is not Outcome.SUCCESS and not any_failed
    return unpassed + ((JudgingEnded(judgement_read.ended),) if ended_badly else ())


def promotion(judgement_read, lacking_commits, ahead_commits, found_workflows):
    findings = judging_findings(judgement_read) + (
        (MainHasWhatDevelopLacks(tuple(lacking_commits)),) if lacking_commits else ())
    if findings or isinstance(judgement_read, NeverJudged):
        return Hold(findings)
    if ahead_commits:
        return Stay(judgement_read.commit, tuple(ahead_commits))
    if found_workflows.hold_main():
        return Hold((MainNeedsAMaintainerForWorkflows(judgement_read.commit, found_workflows),))
    return Move(judgement_read.commit)


def verdict(judgement_read, promoting_read):
    if isinstance(promoting_read, NeverLooked):
        return Red(judging_findings(judgement_read) + (LookingEnded(promoting_read.ended),), promoting_read)
    decided = promotion(judgement_read, promoting_read.lacking, promoting_read.ahead, promoting_read.workflows)
    if isinstance(decided, Hold):
        return Red(decided.findings, promoting_read)
    moved = MainMovedSinceTheLook(decided.commit, promoting_read.main_found, promoting_read.main_tip,
                                  promoting_read.pushed)
    if isinstance(moved, MainMovedSinceTheLook):
        return moved
    if isinstance(decided, Stay):
        return Stayed(decided.commit)
    if promoting_read.pushed is Outcome.SUCCESS and promoting_read.develop_tip == decided.commit.hash:
        return Green(decided.commit)
    if promoting_read.pushed is Outcome.SUCCESS:
        return GreenButMovedOn(decided.commit, promoting_read.main_found, promoting_read.develop_tip)
    if promoting_read.pushed is Outcome.FAILURE:
        return Red((MainRefusedThePush(decided.commit, promoting_read.said),), promoting_read)
    return Red((PromotingEnded(promoting_read.ended),), promoting_read)


def longest_backtick_run(text):
    return max((len(run) for run in re.findall(r'`+', text)), default=0)


def code_span(text):
    ticks = '`' * (longest_backtick_run(text) + 1)
    padded = ' %s ' % text if (text.startswith('`') or text.endswith('`') or ticks != '`') else text
    return ticks + padded + ticks


def fenced(text):
    fence = '`' * max(3, longest_backtick_run(text) + 1)
    return '%s\n%s%s%s' % (fence, text, '' if text.endswith('\n') or text == '' else '\n', fence)


def described(found):
    if isinstance(found, StepDidNotPass) and found.outcome is Outcome.FAILURE and isinstance(found.step, Check):
        return '**%s** failed: %s' % (found.step.name, code_span(found.step.command))
    if isinstance(found, StepDidNotPass) and found.outcome is Outcome.FAILURE:
        return '**%s** failed.' % found.step.name
    if isinstance(found, StepDidNotPass) and found.outcome is Outcome.SKIPPED:
        return '**%s** did not run, since a step before it did not pass.' % found.step.name
    if isinstance(found, StepDidNotPass):
        return '**%s** did not run to its end: its outcome is `%s`.' % (
            found.step.name, found.outcome.value or 'unreported')
    if isinstance(found, JudgingEnded):
        return 'The job that judged it ended `%s` though no check failed.' % found.ended.value
    if isinstance(found, NothingJudged):
        return 'No commit was judged: the job that judges develop ended `%s` before it named one.' % (
            found.ended.value or 'unreported')
    if isinstance(found, MainHasWhatDevelopLacks):
        count = len(found.commits)
        listed = ''.join('%s %s\n' % (each.hash.text, each.subject) for each in found.commits[:LISTED_AT_MOST])
        more = count - LISTED_AT_MOST
        return '%s\n\n%s%s\n\n%s' % (
            'main has %d commit%s develop lacks, and every run is red until develop holds %s:' % (
                count, '' if count == 1 else 's', 'it' if count == 1 else 'them'),
            fenced(listed),
            '\n\nand %d more.' % more if more > 0 else '',
            THE_MERGE_THAT_FIXES_IT)
    if isinstance(found, MainNeedsAMaintainerForWorkflows):
        return held_for_a_maintainer(found)
    if isinstance(found, MainRefusedThePush):
        return 'main refused the push of %s. git said:\n\n%s' % (code_span(found.commit.hash.text),
                                                                  fenced(found.said))
    if isinstance(found, LookingEnded):
        return 'The job that looks at main ended `%s` before it found where main stands.' % (
            found.ended.value or 'unreported')
    return 'The job that moves main ended `%s` before it pushed.' % (found.ended.value or 'unreported')


def workflow_files_counted(paths):
    return '%d workflow file%s' % (len(paths), '' if len(paths) == 1 else 's')


def workflow_files_listed(paths):
    more = len(paths) - LISTED_AT_MOST
    return fenced(''.join(path + '\n' for path in paths[:LISTED_AT_MOST])) + (
        '\n\nand %d more.' % more if more > 0 else '')


def held_for_a_maintainer(found):
    judged = code_span(found.commit.hash.text)
    changed = found.workflows.changed.paths
    differing = found.workflows.differing_alone()
    return '\n\n'.join(
        ("main needs a maintainer: GitHub's built-in token, the only credential the job that moves main has, may not "
         'move a branch past a change to a workflow file, so nothing was pushed.',)
        + (('The commits from main to %s change %s:\n\n%s' % (
            judged, workflow_files_counted(changed), workflow_files_listed(changed)),) if changed else ())
        + (("The judged commit's copy of %s differs from main's though no commit from main to %s changes it, as when "
            "a merge of main into develop keeps develop's copy, so a push of %s replaces main's copy with the judged "
            "commit's:\n\n%s" % (workflow_files_counted(differing), judged, judged,
                                 workflow_files_listed(differing)),) if differing else ())
        + ('No commit to develop and no session can clear this: a later commit of develop either leaves these '
           'workflow files as the judged commit has them or changes them, and main holds for either. It waits on a '
           'maintainer, who pushes main to %s by hand with a credential that may update workflows, a plain push, '
           'never forced, from a clone that has fetched first:\n\n%s\n\nA clone that has main\'s tip but not %s is '
           'told, without the fetch, that the push needs force, which it does not: fetch first, and never force. The '
           'maintainer then re-runs the failed jobs of this run, which re-runs the jobs that look at main, move it and '
           'write this issue too: the re-run finds main at the commit this run judged and closes this issue if that '
           "commit is still both develop's tip and main's, and otherwise leaves it open." % (
               judged, fenced('git fetch origin\ngit push origin %s:refs/heads/main' % found.commit.hash.text),
               judged),))


def reasons(findings):
    return '\n\n'.join(described(found) for found in findings)


def promotion_outputs(decided):
    if isinstance(decided, Move):
        return 'decision=move\n'
    if isinstance(decided, Stay):
        return 'decision=stay\n'
    return 'decision=hold\n'


def named(commit):
    return code_span(commit.hash.text) + (' ' + code_span(commit.subject) if commit.subject else '')


def explained(decided):
    if isinstance(decided, Move):
        return 'main moves to %s.' % named(decided.commit)
    if isinstance(decided, Stay):
        count = len(decided.ahead)
        return ('main stays where it is: it already has %d commit%s of develop that %s lacks, which a run for a later '
                'tip of develop judges.' % (count, '' if count == 1 else 's', code_span(decided.commit.hash.text)))
    return 'main holds where it was:\n\n' + reasons(decided.findings)
