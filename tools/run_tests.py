#!/usr/bin/env python3
"""Run the tests for the scripts under tools/. The same command locally and in CI:

    python3 tools/run_tests.py

Discovery that finds nothing reports success, which is the failure this guards against: a
renamed directory or a pattern that stops matching would leave a green step that judged nothing.
A run that discovers no tests, or that could not import a test module, fails here instead.

It also runs two checks that need more than Python. The renderer check loads the field's
visual model with the three.js the page uses; the browser check opens the page in Chromium and
sees that it draws. Both need node, the second needs a browser, and a missing one fails the run
rather than skipping the check: a gate that quietly steps aside is worse than no gate, because
the run still comes out green.
"""
import os
import shutil
import subprocess
import sys
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
# Each script directory is its own top level, so a test imports the module it tests by name.
PACKAGES = ['field']


def main():
    suite = unittest.TestSuite()
    loader = unittest.TestLoader()
    for package in PACKAGES:
        directory = os.path.join(TOOLS, package)
        if not os.path.isdir(directory):
            print('tools/' + package + ' is gone; this runner names it in PACKAGES.', file=sys.stderr)
            return 2
        sys.path.insert(0, directory)
        suite.addTests(loader.discover(directory, pattern='test_*.py', top_level_dir=directory))
    if loader.errors:
        for error in loader.errors:
            print(error, file=sys.stderr)
        return 2
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if result.testsRun == 0:
        print('No tests were discovered under tools/. That is a broken runner, not a pass.',
              file=sys.stderr)
        return 2
    if not result.wasSuccessful():
        return 1
    for check in (renderer_check, browser_check):
        wrong = check()
        if wrong:
            return wrong
    return 0


def node_at(*where):
    """The path to a check script, and node to run it with, or a reason it cannot be run."""
    check = os.path.join(TOOLS, *where)
    if not os.path.isfile(check):
        return None, check + ' is gone; this runner expects it.'
    node = shutil.which('node')
    if node is None:
        return None, ('node is not installed, so ' + os.path.basename(check) + ' could not run. '
                      'That is a failure, not a pass: install node, or take the check out '
                      'deliberately.')
    return (node, check), None


def renderer_check():
    """Load the field's visual model with the real three.js. Needs node."""
    run, wrong = node_at('renderer', 'check.mjs')
    if wrong:
        print(wrong, file=sys.stderr)
        return 2
    print()
    return subprocess.call(list(run))


def browser_check():
    """Open the field page in Chromium and see that it draws. Needs node and a browser."""
    run, wrong = node_at('browser', 'check.mjs')
    if wrong:
        print(wrong, file=sys.stderr)
        return 2
    if not os.path.isdir(os.path.join(TOOLS, 'browser', 'node_modules')):
        print('tools/browser has no node_modules, so the page was never opened. Run `npm ci` in '
              'tools/browser. Not skipped: a page nobody opened is a page nobody tested.',
              file=sys.stderr)
        return 2
    print()
    return subprocess.call(list(run))


if __name__ == '__main__':
    sys.exit(main())
