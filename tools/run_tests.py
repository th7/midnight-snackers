#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
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
    for check in (renderer_check, browser_check, dashboard_check, replay_check, cost_check):
        wrong = check()
        if wrong:
            return wrong
    return 0

def node_at(*where):
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
    run, wrong = node_at('renderer', 'check.mjs')
    if wrong:
        print(wrong, file=sys.stderr)
        return 2
    print()
    return subprocess.call(list(run))

def browser_check():
    return in_a_browser('check.mjs')


def dashboard_check():
    return in_a_browser('dashboard.mjs')


def replay_check():
    return in_a_browser('replay.mjs')


def cost_check():
    return in_a_browser('cost.mjs')


def in_a_browser(script):
    run, wrong = node_at('browser', script)
    if wrong:
        print(wrong, file=sys.stderr)
        return 2
    if not os.path.isdir(os.path.join(TOOLS, 'browser', 'node_modules')):
        print('tools/browser has no node_modules, so ' + script + ' never opened a page. Run '
              '`npm ci` in tools/browser. Not skipped: a page nobody opened is a page nobody tested.',
              file=sys.stderr)
        return 2
    print()
    return subprocess.call(list(run))

if __name__ == '__main__':
    sys.exit(main())
