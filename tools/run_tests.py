#!/usr/bin/env python3
"""Run the tests for the scripts under tools/. The same command locally and in CI:

    python3 tools/run_tests.py

Discovery that finds nothing reports success, which is the failure this guards against: a
renamed directory or a pattern that stops matching would leave a green step that judged nothing.
A run that discovers no tests, or that could not import a test module, fails here instead.
"""
import os
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
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
