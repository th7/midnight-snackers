import os
import sys
import tempfile
import unittest

TOOLS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, TOOLS)

import run_tests

A_TEST = 'import unittest\n\n\nclass Holds(unittest.TestCase):\n    def test_it_holds(self):\n        pass\n'


class WhatThePythonSuiteDiscovers(unittest.TestCase):
    def setUp(self):
        self.where = tempfile.TemporaryDirectory()
        self.tools = self.where.name

    def tearDown(self):
        self.where.cleanup()

    def package(self, name, files):
        directory = os.path.join(self.tools, name)
        os.makedirs(directory)
        for file_name, text in files.items():
            with open(os.path.join(directory, file_name), 'w', encoding='utf-8') as written:
                written.write(text)

    def test_every_package_s_tests_are_gathered_into_one_suite(self):
        self.package('ran_alpha', {'test_ran_alpha_holds.py': A_TEST})
        self.package('ran_beta', {'test_ran_beta_holds.py': A_TEST})

        gathered = run_tests.python_suite(self.tools, ['ran_alpha', 'ran_beta'], unittest.TestLoader())

        self.assertIsInstance(gathered, unittest.TestSuite)
        self.assertEqual(2, gathered.countTestCases())

    def test_a_package_that_discovers_no_tests_is_a_broken_runner_named_by_the_package(self):
        self.package('empty_gamma', {'test_empty_gamma_holds.py': A_TEST})
        self.package('empty_delta', {'renamed_test_empty_delta.py': A_TEST})

        gathered = run_tests.python_suite(self.tools, ['empty_gamma', 'empty_delta'], unittest.TestLoader())

        self.assertEqual('tools/empty_delta discovered no tests. That is a broken runner, not a pass.', gathered)

    def test_a_package_that_is_gone_is_named(self):
        self.package('gone_epsilon', {'test_gone_epsilon_holds.py': A_TEST})

        gathered = run_tests.python_suite(self.tools, ['gone_epsilon', 'gone_zeta'], unittest.TestLoader())

        self.assertEqual('tools/gone_zeta is gone; this runner names it in PACKAGES.', gathered)

    def test_a_test_module_that_does_not_import_is_named(self):
        self.package('broken_eta', {'test_broken_eta_imports.py': 'import a_module_nobody_wrote\n'})

        gathered = run_tests.python_suite(self.tools, ['broken_eta'], unittest.TestLoader())

        self.assertIsInstance(gathered, str)
        self.assertIn('test_broken_eta_imports', gathered)

    def test_the_runner_names_every_package_whose_tests_it_runs(self):
        self.assertEqual(['field', 'ci', 'suite_cost', 'start'], run_tests.PACKAGES)


if __name__ == '__main__':
    unittest.main()
