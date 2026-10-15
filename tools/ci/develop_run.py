import base64
import re
from dataclasses import dataclass
from enum import Enum

COMMIT_HASH = re.compile(r'[0-9a-f]{40}|[0-9a-f]{64}')
BASE64 = re.compile(r'(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?')
LISTED_AT_MOST = 50
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
        if not isinstance(subject, str) or '\n' in subject:
            return 'a commit subject is one line'
        return None


@dataclass(frozen=True)
class Check:
    step: str
    name: str
    command: str


FORMATTING = Check('formatting', 'Java formatting (same command as local; fix with ./gradlew spotlessApply)',
                   './gradlew spotlessCheck')
TOOL_TESTS = Check('tool-tests', 'Tool tests (same command as local)', 'python3 tools/run_tests.py')
UNIT_TESTS = Check('unit-tests', 'Unit tests (same command as local)', './gradlew :TeamCode:testDebugUnitTest')
CHECKS = (FORMATTING, TOOL_TESTS, UNIT_TESTS)


@dataclass(frozen=True)
class Checked(Validated):
    formatting: Outcome
    tool_tests: Outcome
    unit_tests: Outcome

    @staticmethod
    def broken(formatting, tool_tests, unit_tests):
        if not all_of(Outcome, (formatting, tool_tests, unit_tests)):
            return 'each check ends in an outcome'
        return None

    def each(self):
        return ((FORMATTING, self.formatting), (TOOL_TESTS, self.tool_tests), (UNIT_TESTS, self.unit_tests))


@dataclass(frozen=True)
class Judged(Validated):
    commit: Commit
    ended: Outcome
    checked: Checked

    @staticmethod
    def broken(commit, ended, checked):
        if not isinstance(commit, Commit):
            return 'a judgement names the commit it judged'
        if not isinstance(ended, Outcome) or not isinstance(checked, Checked):
            return 'a judgement says how its job ended and how each check ended'
        return None


@dataclass(frozen=True)
class NeverJudged:
    ended: Outcome


@dataclass(frozen=True)
class Looked(Validated):
    main_tip: CommitHash
    lacking: tuple
    pushed: Outcome
    said: str
    ended: Outcome

    @staticmethod
    def broken(main_tip, lacking, pushed, said, ended):
        if not isinstance(main_tip, CommitHash):
            return 'a mover that looked names main\'s tip by its hash'
        if not all_of(Commit, lacking):
            return 'what main lacks is a tuple of commits'
        if not all_of(Outcome, (pushed, ended)):
            return 'a mover says how its push and its job ended'
        if not isinstance(said, str):
            return 'what git said is text'
        return None


@dataclass(frozen=True)
class NeverLooked:
    ended: Outcome


@dataclass(frozen=True)
class CheckDidNotPass:
    check: Check
    outcome: Outcome


@dataclass(frozen=True)
class JudgingEnded:
    ended: Outcome


@dataclass(frozen=True)
class NothingJudged:
    ended: Outcome


@dataclass(frozen=True)
class MainHasWhatDevelopLacks:
    commits: tuple


@dataclass(frozen=True)
class MainRefusedThePush:
    commit: Commit
    said: str


@dataclass(frozen=True)
class PromotingEnded:
    ended: Outcome


@dataclass(frozen=True)
class Move:
    commit: Commit


@dataclass(frozen=True)
class Hold:
    findings: tuple


@dataclass(frozen=True)
class Green:
    commit: Commit


@dataclass(frozen=True)
class Red:
    findings: tuple
    promoting: object


def outcome(text):
    for known in Outcome:
        if known.value == text:
            return known
    return Rejected('an outcome is success, failure, cancelled, skipped or nothing, never %r' % text)


def commit_hash(text):
    return CommitHash(text)


def commit_line(line):
    hash_text, _, subject = line.partition(' ')
    hashed = commit_hash(hash_text)
    if isinstance(hashed, Rejected):
        return hashed
    return Commit(hashed, subject)


def decoded(text):
    if not isinstance(text, str) or not text.isascii() or BASE64.fullmatch(text) is None:
        return Rejected('what the workflow gathered is base64, and %r is not' % (text,))
    return base64.b64decode(text, validate=True).decode('utf-8', errors='replace')


def first_rejected(*read):
    return next((each for each in read if isinstance(each, Rejected)), None)


def judgement(ended_text, judged_text, subject_base64, formatting_text, tool_tests_text, unit_tests_text):
    ended = outcome(ended_text)
    if isinstance(ended, Rejected):
        return ended
    if judged_text == '':
        return NeverJudged(ended)
    hashed = commit_hash(judged_text)
    subject = decoded(subject_base64)
    formatting, tool_tests, unit_tests = (outcome(formatting_text), outcome(tool_tests_text),
                                          outcome(unit_tests_text))
    rejected = first_rejected(hashed, subject, formatting, tool_tests, unit_tests)
    if rejected is not None:
        return rejected
    line = subject[:-1] if subject.endswith('\n') else subject
    named = Commit(hashed, line)
    if isinstance(named, Rejected):
        return named
    return Judged(named, ended, Checked(formatting, tool_tests, unit_tests))


def lacking(lines_base64):
    text = decoded(lines_base64)
    if isinstance(text, Rejected):
        return text
    commits = tuple(commit_line(line) for line in text.split('\n') if line != '')
    rejected = first_rejected(*commits)
    if rejected is not None:
        return rejected
    return commits


def promoting(ended_text, main_tip_text, lacking_base64, pushed_text, said_base64):
    ended = outcome(ended_text)
    if isinstance(ended, Rejected):
        return ended
    if main_tip_text == '':
        return NeverLooked(ended)
    main_tip = commit_hash(main_tip_text)
    commits = lacking(lacking_base64)
    pushed = outcome(pushed_text)
    said = decoded(said_base64)
    rejected = first_rejected(main_tip, commits, pushed, said)
    if rejected is not None:
        return rejected
    return Looked(main_tip, commits, pushed, said, ended)


def judging_findings(judgement_read):
    if isinstance(judgement_read, NeverJudged):
        return (NothingJudged(judgement_read.ended),)
    unpassed = tuple(CheckDidNotPass(check, did) for check, did in judgement_read.checked.each()
                     if did is not Outcome.SUCCESS)
    any_failed = any(finding.outcome is Outcome.FAILURE for finding in unpassed)
    ended_badly = judgement_read.ended is not Outcome.SUCCESS and not any_failed
    return unpassed + ((JudgingEnded(judgement_read.ended),) if ended_badly else ())


def promotion(judgement_read, lacking_commits):
    findings = judging_findings(judgement_read) + (
        (MainHasWhatDevelopLacks(tuple(lacking_commits)),) if lacking_commits else ())
    if findings or isinstance(judgement_read, NeverJudged):
        return Hold(findings)
    return Move(judgement_read.commit)


def verdict(judgement_read, promoting_read):
    if isinstance(promoting_read, NeverLooked):
        return Red(judging_findings(judgement_read) + (PromotingEnded(promoting_read.ended),), promoting_read)
    decided = promotion(judgement_read, promoting_read.lacking)
    if isinstance(decided, Hold):
        return Red(decided.findings, promoting_read)
    if promoting_read.pushed is Outcome.SUCCESS:
        return Green(decided.commit)
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
    if isinstance(found, CheckDidNotPass) and found.outcome is Outcome.FAILURE:
        return '**%s** failed: %s' % (found.check.name, code_span(found.check.command))
    if isinstance(found, CheckDidNotPass) and found.outcome is Outcome.SKIPPED:
        return '**%s** did not run, since a step before it did not pass.' % found.check.name
    if isinstance(found, CheckDidNotPass):
        return '**%s** did not run to its end: its outcome is `%s`.' % (
            found.check.name, found.outcome.value or 'unreported')
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
    if isinstance(found, MainRefusedThePush):
        return 'main refused the push of %s. git said:\n\n%s' % (code_span(found.commit.hash.text),
                                                                  fenced(found.said))
    return 'The job that moves main ended `%s` before it pushed.' % (found.ended.value or 'unreported')


def reasons(findings):
    return '\n\n'.join(described(found) for found in findings)


def promotion_outputs(decided):
    if isinstance(decided, Move):
        return 'decision=move\ncommit=%s\n' % decided.commit.hash.text
    return 'decision=hold\n'


def named(commit):
    return code_span(commit.hash.text) + (' ' + code_span(commit.subject) if commit.subject else '')


def explained(decided):
    if isinstance(decided, Move):
        return 'main moves to %s.' % named(decided.commit)
    return 'main holds where it was:\n\n' + reasons(decided.findings)
