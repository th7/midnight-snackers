import base64
import re
import unicodedata
from dataclasses import dataclass
from enum import Enum

COMMIT_HASH = re.compile(r'[0-9a-f]{40}|[0-9a-f]{64}')
BASE64 = re.compile(r'(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?')
LISTED_AT_MOST = 50
MAIN_PUSH_TOKEN = 'MAIN_PUSH_TOKEN'
REFUSED_OVER_WORKFLOWS = re.compile(r'refusing to allow an? [^\n]+? to create or update workflow')
SUBJECT_SHOWN = 200
SUBJECT_SHOWN_AT_MOST = 256
SAID_SHOWN = 4000
SAID_SHOWN_AT_MOST = 4096
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
GRADLE = Step('gradle', 'Gradle')
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


@dataclass(frozen=True)
class Looked(Validated):
    main_tip: CommitHash
    lacking: tuple
    ahead: tuple
    pushed: Outcome
    said: str
    ended: Outcome

    @staticmethod
    def broken(main_tip, lacking, ahead, pushed, said, ended):
        if not isinstance(main_tip, CommitHash):
            return 'a mover that looked names main\'s tip by its hash'
        if not all_of(Commit, lacking) or not all_of(Commit, ahead):
            return 'what main has that develop or the judged commit lacks is a tuple of commits'
        if not all_of(Outcome, (pushed, ended)):
            return 'a mover says how its push and its job ended'
        if (not isinstance(said, str) or len(said) > SAID_SHOWN_AT_MOST
                or any(unsafe_in_a_quote(character) for character in said)):
            return ('what git said is text of at most %d characters, with no control character but a newline or a '
                    'tab' % SAID_SHOWN_AT_MOST)
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
class MainHasWhatDevelopLacks:
    commits: tuple


@dataclass(frozen=True)
class MainRefusedThePush:
    commit: Commit
    said: str


@dataclass(frozen=True)
class MainRefusedOverWorkflows:
    commit: Commit
    said: str


@dataclass(frozen=True)
class PromotingEnded:
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
    return Rejected('an outcome is success, failure, cancelled, skipped or nothing, never %r' % text)


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


def promoting(ended_text, main_tip_text, lacking_base64, ahead_base64, pushed_text, said_base64):
    ended = outcome(ended_text)
    if isinstance(ended, Rejected):
        return ended
    if main_tip_text == '':
        return NeverLooked(ended)
    main_tip = commit_hash(main_tip_text)
    lacking = listed_commits(lacking_base64)
    ahead = listed_commits(ahead_base64)
    pushed = outcome(pushed_text)
    said = decoded(said_base64)
    rejected = first_rejected(main_tip, lacking, ahead, pushed, said)
    if rejected is not None:
        return rejected
    return Looked(main_tip, lacking, ahead, pushed, shown_said(said), ended)


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


def promotion(judgement_read, lacking_commits, ahead_commits):
    findings = judging_findings(judgement_read) + (
        (MainHasWhatDevelopLacks(tuple(lacking_commits)),) if lacking_commits else ())
    if findings or isinstance(judgement_read, NeverJudged):
        return Hold(findings)
    if ahead_commits:
        return Stay(judgement_read.commit, tuple(ahead_commits))
    return Move(judgement_read.commit)


def verdict(judgement_read, promoting_read):
    if isinstance(promoting_read, NeverLooked):
        return Red(judging_findings(judgement_read) + (PromotingEnded(promoting_read.ended),), promoting_read)
    decided = promotion(judgement_read, promoting_read.lacking, promoting_read.ahead)
    if isinstance(decided, Hold):
        return Red(decided.findings, promoting_read)
    if isinstance(decided, Stay):
        return Stayed(decided.commit)
    if promoting_read.pushed is Outcome.SUCCESS:
        return Green(decided.commit)
    if promoting_read.pushed is Outcome.FAILURE and REFUSED_OVER_WORKFLOWS.search(promoting_read.said):
        return Red((MainRefusedOverWorkflows(decided.commit, promoting_read.said),), promoting_read)
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
    if isinstance(found, MainRefusedOverWorkflows):
        hash_text = found.commit.hash.text
        return ('main refused the push of %s: the commits it would bring to main change a file under '
                '`.github/workflows`, and the credential the job that moves main pushes with may not update workflows. '
                'No commit to develop fixes this, so no session can: it waits on a maintainer, who either moves main '
                'to %s by hand with a credential that may update workflows (%s, a plain push, never forced), or gives '
                'the job that moves main such a credential as the repository secret %s, a token that may write '
                'contents and workflows of this repository alone, and then runs the failed jobs of this run again. '
                'git said:\n\n%s' % (code_span(hash_text), code_span(hash_text),
                                     code_span('git push origin %s:refs/heads/main' % hash_text),
                                     code_span(MAIN_PUSH_TOKEN), fenced(found.said)))
    if isinstance(found, MainRefusedThePush):
        return 'main refused the push of %s. git said:\n\n%s' % (code_span(found.commit.hash.text),
                                                                  fenced(found.said))
    return 'The job that moves main ended `%s` before it pushed.' % (found.ended.value or 'unreported')


def reasons(findings):
    return '\n\n'.join(described(found) for found in findings)


def promotion_outputs(decided):
    if isinstance(decided, Move):
        return 'decision=move\ncommit=%s\n' % decided.commit.hash.text
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
