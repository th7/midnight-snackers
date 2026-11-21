import os
import sys

import develop_issue as issues
import develop_run as run

def named(text):
    return text.upper().replace('-', '_')


def ended_of(job):
    return named(job.job) + '_ENDED'


def outcome_of(job, step):
    return 'STEP_%s_%s' % (named(job.job), named(step.step))


JUDGED = ('JUDGED', 'SUBJECT')
JOB_OUTCOMES = tuple(name for job in run.JOBS for name in (ended_of(job),) + tuple(outcome_of(job, step)
                                                                                 for step in job.steps))
JUDGEMENT = JUDGED + JOB_OUTCOMES
WORKFLOWS = ('WORKFLOWS_CHANGED', 'WORKFLOWS_DIFFERING')
LOOKING = ('LOOKING_ENDED', 'MAIN_FOUND', 'MAIN_TIP', 'DEVELOP_TIP', 'LACKING', 'MAIN_AHEAD') + WORKFLOWS
PROMOTING = ('PROMOTING_ENDED', 'PUSHED', 'GIT_SAID')
RUN = ('GITHUB_SERVER_URL', 'GITHUB_REPOSITORY', 'GITHUB_RUN_ID')


def judgement_from(inputs):
    handed = inputs[len(JUDGED):len(JUDGEMENT)]
    job_texts = []
    for job in run.JOBS:
        job_texts.append((handed[0], tuple(handed[1:1 + len(job.steps)])))
        handed = handed[1 + len(job.steps):]
    return run.judgement(*inputs[:len(JUDGED)], tuple(job_texts))


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
    said = issues.logged(action)
    if said:
        print(said, file=stderr)
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
