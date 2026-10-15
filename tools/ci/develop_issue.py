import json
import re
from dataclasses import dataclass

import develop_run as run
from develop_run import Green, Judged, Looked, Rejected, Stayed, Validated

TITLE = 'IMPORTANT: develop CI Failures'
BODY_AT_MOST = 65536
LISTED_AT_MOST = 1000
REPOSITORY = re.compile(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+')
RUN_ID = re.compile(r'[0-9]+')
RUN_URL = re.compile(r'https://[^/\s]+/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/actions/runs/[0-9]+')


@dataclass(frozen=True)
class OpenIssue(Validated):
    number: int
    title: str

    @staticmethod
    def broken(number, title):
        if type(number) is not int or number < 1:
            return 'an open issue\'s number is a whole number from 1 up, never %r' % (number,)
        if not isinstance(title, str):
            return 'an open issue\'s title is text, never %r' % (title,)
        return None


@dataclass(frozen=True)
class RunLink(Validated):
    url: str

    @staticmethod
    def broken(url):
        if not isinstance(url, str) or RUN_URL.fullmatch(url) is None:
            return 'a run is linked as https://server/owner/name/actions/runs/digits, never %r' % (url,)
        return None


@dataclass(frozen=True)
class Open:
    title: str
    body: str


@dataclass(frozen=True)
class CommentOn:
    issue: int
    body: str


@dataclass(frozen=True)
class Close:
    issues: tuple
    body: str


@dataclass(frozen=True)
class LeaveAlone:
    pass


def open_issue(entry):
    if not isinstance(entry, dict):
        return Rejected('an open issue is a JSON object with a number and a title')
    return OpenIssue(entry.get('number'), entry.get('title'))


def open_issues(json_text):
    try:
        listed = json.loads(json_text)
    except ValueError:
        return Rejected('the open issues are the JSON list gh prints, and this is not JSON')
    if not isinstance(listed, list):
        return Rejected('the open issues are the JSON list gh prints, and this is not a list')
    if len(listed) >= LISTED_AT_MOST:
        return Rejected('gh listed as many open issues as it was asked for, %d, so the list may be cut short and the '
                        'develop CI issue may be among those left out' % LISTED_AT_MOST)
    read = tuple(open_issue(entry) for entry in listed)
    rejected = run.first_rejected(*read)
    if rejected is not None:
        return rejected
    return read


def run_link(server, repository, run_id):
    if not server.startswith('https://') or '/' in server[len('https://'):]:
        return Rejected('a run is linked from an https server, never %r' % server)
    if REPOSITORY.fullmatch(repository) is None:
        return Rejected('a repository is an owner and a name, never %r' % repository)
    if RUN_ID.fullmatch(run_id) is None:
        return Rejected('a run id is digits, never %r' % run_id)
    return RunLink('%s/%s/actions/runs/%s' % (server, repository, run_id))


def where_main_stays(promoting):
    if isinstance(promoting, Looked):
        return 'main stays where it was, at %s.' % run.code_span(promoting.main_tip.text)
    return 'main stays where it was.'


def red_head(judgement):
    if isinstance(judgement, Judged):
        return 'develop was judged red at %s, so' % run.named(judgement.commit)
    return 'A run for develop judged no commit, so'


def red_body(judgement, found, link):
    return '%s %s\n\n%s\n\nThe run: %s\n' % (red_head(judgement), where_main_stays(found.promoting),
                                             run.reasons(found.findings), link.url)


def green_body(found, link):
    return 'develop was judged green at %s, and main now stands there.\n\nThe run: %s\n' % (
        run.named(found.commit), link.url)


def issue_action(judgement, promoting, listed, link):
    found = run.verdict(judgement, promoting)
    ours = tuple(sorted(each.number for each in listed if each.title == TITLE))
    if isinstance(found, Stayed):
        return LeaveAlone()
    if isinstance(found, Green):
        return Close(ours, green_body(found, link)) if ours else LeaveAlone()
    body = red_body(judgement, found, link)
    return CommentOn(ours[0], body) if ours else Open(TITLE, body)


def action_outputs(action):
    if isinstance(action, Open):
        return 'action=open\ntitle=%s\n' % action.title
    if isinstance(action, CommentOn):
        return 'action=comment\nissues=%d\n' % action.issue
    if isinstance(action, Close):
        return 'action=close\nissues=%s\n' % ' '.join(str(number) for number in action.issues)
    return 'action=none\n'
