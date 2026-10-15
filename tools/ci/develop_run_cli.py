import os
import sys

import develop_issue as issues
import develop_run as run

def outcome_of(step):
    return 'STEP_' + step.step.upper().replace('-', '_')


JUDGED = ('JUDGING_ENDED', 'JUDGED', 'SUBJECT')
STEP_OUTCOMES = tuple(outcome_of(step) for step in run.STEPS)
JUDGEMENT = JUDGED + STEP_OUTCOMES
WORKFLOWS = ('WORKFLOWS_CHANGED', 'WORKFLOWS_DIFFERING')
LOOKING = ('LOOKING_ENDED', 'MAIN_TIP', 'DEVELOP_TIP', 'LACKING', 'MAIN_AHEAD') + WORKFLOWS
PROMOTING = ('PROMOTING_ENDED', 'PUSHED', 'GIT_SAID')
RUN = ('GITHUB_SERVER_URL', 'GITHUB_REPOSITORY', 'GITHUB_RUN_ID')


def judgement_from(inputs):
    return run.judgement(*inputs[:len(JUDGED)], inputs[len(JUDGED):len(JUDGEMENT)])


def handed(environ, names):
    missing = [name for name in names if name not in environ]
    if missing:
        return run.Rejected('the workflow did not hand %s' % ', '.join(missing))
    return tuple(environ[name] for name in names)


def promotion(environ, arguments, stdout, stderr):
    inputs = handed(environ, JUDGEMENT + ('LACKING', 'MAIN_AHEAD') + WORKFLOWS)
    if isinstance(inputs, run.Rejected):
        return inputs
    judgement = judgement_from(inputs)
    lacking = run.listed_commits(inputs[len(JUDGEMENT)])
    ahead = run.listed_commits(inputs[len(JUDGEMENT) + 1])
    workflows = run.workflows(*inputs[len(JUDGEMENT) + 2:])
    rejected = run.first_rejected(judgement, lacking, ahead, workflows)
    if rejected is not None:
        return rejected
    decided = run.promotion(judgement, lacking, ahead, workflows)
    print(run.explained(decided), file=stderr)
    stdout.write(run.promotion_outputs(decided))
    return 0


def report(environ, arguments, stdout, stderr):
    if len(arguments) != 1:
        return run.Rejected('report takes where the open issues are, and nothing more')
    inputs = handed(environ, JUDGEMENT + LOOKING + PROMOTING + RUN)
    if isinstance(inputs, run.Rejected):
        return inputs
    with open(arguments[0], encoding='utf-8') as listed:
        open_issues = issues.open_issues(listed.read())
    judgement = judgement_from(inputs)
    found = len(JUDGEMENT) + len(LOOKING) + len(PROMOTING)
    promoting = run.promoting(*inputs[len(JUDGEMENT):found])
    link = issues.run_link(*inputs[found:])
    rejected = run.first_rejected(open_issues, judgement, promoting, link)
    if rejected is not None:
        return rejected
    action = issues.issue_action(judgement, promoting, open_issues, link)
    if not isinstance(action, issues.LeaveAlone):
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
