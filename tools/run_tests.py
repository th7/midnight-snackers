#!/usr/bin/env python3
"""Run the tests for the scripts under tools/. The same command locally and in CI:

    python3 tools/run_tests.py

Discovery that finds nothing reports success, which is the failure this guards against: a
renamed directory or a pattern that stops matching would leave a green step that judged nothing.
A run that discovers no tests, or that could not import a test module, fails here instead.

It also runs the renderer check, which loads the field's visual model with the three.js the page
uses. That needs node, and a missing node fails the run rather than skipping the check: a gate
that quietly steps aside is worse than no gate, because the run still comes out green.
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
    return renderer_check()


def renderer_check():
    """Load the field's visual model with the real three.js. Needs node."""
    check = os.path.join(TOOLS, 'renderer', 'check.mjs')
    if not os.path.isfile(check):
        print(check + ' is gone; this runner expects it.', file=sys.stderr)
        return 2
    node = shutil.which('node')
    if node is None:
        print('node is not installed, so the renderer check could not run. That is a failure, '
              'not a pass: install node, or take the check out deliberately.', file=sys.stderr)
        return 2
    print()
    return subprocess.call([node, check])


if __name__ == '__main__':
    sys.exit(main())
