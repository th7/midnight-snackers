import os
import sys

import develop_issue as issues
import develop_run as run

JUDGEMENT = ('JUDGING_ENDED', 'JUDGED', 'SUBJECT', 'FORMATTING', 'TOOL_TESTS', 'UNIT_TESTS')
PROMOTING = ('PROMOTING_ENDED', 'MAIN_TIP', 'LACKING', 'PUSHED', 'GIT_SAID')
RUN = ('GITHUB_SERVER_URL', 'GITHUB_REPOSITORY', 'GITHUB_RUN_ID')


def handed(environ, names):
    missing = [name for name in names if name not in environ]
    if missing:
        return run.Rejected('the workflow did not hand %s' % ', '.join(missing))
    return tuple(environ[name] for name in names)


def promotion(environ, arguments, stdout, stderr):
    inputs = handed(environ, JUDGEMENT + ('LACKING',))
    if isinstance(inputs, run.Rejected):
        return inputs
    judgement = run.judgement(*inputs[:6])
    lacking = run.lacking(inputs[6])
    rejected = run.first_rejected(judgement, lacking)
    if rejected is not None:
        return rejected
    decided = run.promotion(judgement, lacking)
    print(run.explained(decided), file=stderr)
    stdout.write(run.promotion_outputs(decided))
    return 0


def report(environ, arguments, stdout, stderr):
    if len(arguments) != 2:
        return run.Rejected('report takes where the open issues are and where to write the body, in that order')
    inputs = handed(environ, JUDGEMENT + PROMOTING + RUN)
    if isinstance(inputs, run.Rejected):
        return inputs
    with open(arguments[0], encoding='utf-8') as listed:
        open_issues = issues.open_issues(listed.read())
    judgement = run.judgement(*inputs[:6])
    promoting = run.promoting(*inputs[6:11])
    link = issues.run_link(*inputs[11:])
    rejected = run.first_rejected(open_issues, judgement, promoting, link)
    if rejected is not None:
        return rejected
    action = issues.issue_action(judgement, promoting, open_issues, link)
    if not isinstance(action, issues.LeaveAlone):
        with open(arguments[1], 'w', encoding='utf-8') as body:
            body.write(action.body)
        print(action.body, file=stderr)
    stdout.write(issues.action_outputs(action))
    return 0


COMMANDS = {'promotion': promotion, 'report': report}


def main(arguments, environ, stdout, stderr):
    command = COMMANDS.get(arguments[0] if arguments else '')
    if command is None:
        print('say which: %s' % ', '.join(sorted(COMMANDS)), file=stderr)
        return 2
    done = command(environ, arguments[1:], stdout, stderr)
    if isinstance(done, run.Rejected):
        print(done.rule, file=stderr)
        return 2
    return done


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:], os.environ, sys.stdout, sys.stderr))
