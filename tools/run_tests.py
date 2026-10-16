#!/usr/bin/env python3
import os
import shutil
import subprocess
import sys
import time
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
PACKAGES = ['field', 'ci', 'suite_cost', 'start']

def python_suite(tools, packages, loader):
    suite = unittest.TestSuite()
    for package in packages:
        directory = os.path.join(tools, package)
        if not os.path.isdir(directory):
            return 'tools/' + package + ' is gone; this runner names it in PACKAGES.'
        sys.path.insert(0, directory)
        found = loader.discover(directory, pattern='test_*.py', top_level_dir=directory)
        if loader.errors:
            return '\n'.join(loader.errors)
        if found.countTestCases() == 0:
            return 'tools/' + package + ' discovered no tests. That is a broken runner, not a pass.'
        suite.addTests(found)
    return suite

def main():
    suite = python_suite(TOOLS, PACKAGES, unittest.TestLoader())
    if isinstance(suite, str):
        print(suite, file=sys.stderr)
        return 2
    began = time.monotonic()
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    took = [('python', time.monotonic() - began)]
    if result.testsRun == 0:
        print('No tests were discovered under tools/. That is a broken runner, not a pass.',
              file=sys.stderr)
        return 2
    if not result.wasSuccessful():
        return 1
    for check in (renderer_check, browser_check, dashboard_check, login_check, admin_check, replay_check, launch_check, cost_check):
        began = time.monotonic()
        wrong = check()
        took.append((check.__name__.replace('_check', ''), time.monotonic() - began))
        if wrong:
            print_times_never_judged(took)
            return wrong
    print_times_never_judged(took)
    return 0

def print_times_never_judged(took):
    print()
    print('what the tool tests cost')
    for what, seconds in took:
        print('  %-12s %6.1fs' % (what, seconds))
    print('  %-12s %6.1fs' % ('all of it', sum(seconds for _, seconds in took)))

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


def login_check():
    return in_a_browser('login.mjs')


def admin_check():
    return in_a_browser('admin.mjs')


def replay_check():
    return in_a_browser('replay.mjs')


def launch_check():
    return in_a_browser('launch.mjs')


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
