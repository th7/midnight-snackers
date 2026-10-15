import os
import sys

import develop_run as run

JUDGEMENT = ('JUDGING_ENDED', 'JUDGED', 'SUBJECT', 'FORMATTING', 'TOOL_TESTS', 'UNIT_TESTS')


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


COMMANDS = {'promotion': promotion}


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
