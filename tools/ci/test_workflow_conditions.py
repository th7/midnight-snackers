import unittest

from workflow_conditions import Rejected, Status, holds, value

EXPRESSIONS = 'https://docs.github.com/en/actions/reference/workflows-and-actions/expressions'
STATUS_CHECKS = EXPRESSIONS + '#status-check-functions'
NOTHING_FAILED = Status(failed=False, cancelled=False)
SOMETHING_FAILED = Status(failed=True, cancelled=False)
CANCELLED = Status(failed=False, cancelled=True)
CONTEXTS = {'needs': {'unit-tests': {'result': 'success', 'outputs': {'commit': 'a' * 40, 'empty': ''}},
                      'look': {'result': 'failure', 'outputs': {'decision': 'move'}}},
            'steps': {'decide': {'outcome': 'success', 'outputs': {'decision': 'stay'}}}}


def evaluated(text, status=NOTHING_FAILED):
    return value(text, CONTEXTS, status)


class ComparingValues(unittest.TestCase):
    def test_a_context_is_read_by_its_dotted_path_hyphens_and_all(self):
        self.assertEqual('success', evaluated('needs.unit-tests.result'))
        self.assertEqual('stay', evaluated('steps.decide.outputs.decision'))

    def test_a_property_no_context_holds_is_null(self):
        self.assertIsNone(evaluated('needs.promote.result'))
        self.assertIsNone(evaluated('needs.look.outputs.decision.more'))

    def test_strings_are_compared_without_regard_to_case(self):
        self.assertTrue(evaluated("needs.look.outputs.decision == 'MOVE'"),
                        'GitHub ignores case when comparing strings: %s' % EXPRESSIONS)
        self.assertFalse(evaluated("needs.look.outputs.decision != 'Move'"))

    def test_values_of_different_types_are_compared_as_numbers_so_null_equals_the_empty_string(self):
        self.assertTrue(evaluated("needs.promote.result == ''"),
                        'GitHub coerces values of different types to numbers, null and the empty string to 0: %s'
                        % EXPRESSIONS)
        self.assertTrue(evaluated("needs.unit-tests.outputs.empty == null"))
        self.assertFalse(evaluated("needs.promote.result == 'success'"))

    def test_a_single_quoted_string_doubles_its_quotes(self):
        self.assertEqual("it's", evaluated("'it''s'"))


class CombiningConditions(unittest.TestCase):
    def test_and_gives_its_first_falsy_operand_or_its_last_and_or_its_first_truthy_or_its_last(self):
        self.assertEqual('', evaluated("needs.unit-tests.outputs.empty && 'never'"))
        self.assertEqual('last', evaluated("needs.unit-tests.result && 'last'"))
        self.assertEqual('success', evaluated("needs.unit-tests.result || 'never'"))
        self.assertEqual('develop', evaluated("needs.unit-tests.outputs.empty || 'develop'"),
                         'a logical operator gives one of its operands, not a boolean: %s' % EXPRESSIONS)

    def test_not_binds_tighter_than_a_comparison_which_binds_tighter_than_and_which_binds_tighter_than_or(self):
        self.assertTrue(evaluated("!needs.promote.result == true"))
        self.assertTrue(evaluated("needs.look.result == 'success' && false || true"))
        self.assertFalse(evaluated("needs.look.result == 'success' && (false || true)"))

    def test_parentheses_group_and_the_dollar_braces_around_an_expression_are_read_through(self):
        self.assertTrue(evaluated("${{ !(needs.look.result == 'success') }}"))


class TheStatusAStepOrJobRunsUnder(unittest.TestCase):
    def test_a_condition_naming_no_status_check_runs_only_when_nothing_before_it_failed_or_was_cancelled(self):
        self.assertTrue(holds("needs.unit-tests.result == 'success'", CONTEXTS, NOTHING_FAILED))
        for status in (SOMETHING_FAILED, CANCELLED):
            self.assertFalse(holds("needs.unit-tests.result == 'success'", CONTEXTS, status),
                             'success() is applied unless a condition names a status check: %s' % STATUS_CHECKS)

    def test_a_condition_naming_a_status_check_decides_alone(self):
        self.assertTrue(holds('failure()', CONTEXTS, SOMETHING_FAILED))
        self.assertFalse(holds('failure()', CONTEXTS, NOTHING_FAILED))
        self.assertTrue(holds("${{ !cancelled() && needs.unit-tests.result == 'success' }}", CONTEXTS,
                              SOMETHING_FAILED))
        self.assertFalse(holds('${{ !cancelled() }}', CONTEXTS, CANCELLED))
        self.assertTrue(holds('always()', CONTEXTS, CANCELLED))

    def test_success_is_false_once_something_failed_or_was_cancelled(self):
        self.assertEqual([True, False, False],
                         [holds('success()', CONTEXTS, status) for status in (NOTHING_FAILED, SOMETHING_FAILED,
                                                                               CANCELLED)])


class WhatItDoesNotRead(unittest.TestCase):
    def test_an_expression_outside_what_it_reads_is_refused_by_the_rule_it_breaks(self):
        for text, rule in (("format('{0}', github.ref)", 'a function this reader does not take'),
                           ('contains(github.ref)', 'a function this reader does not take'),
                           ('github.event.commits[0]', 'an expression this reader does not take'),
                           ("(needs.look.result == 'success'", 'a parenthesis left open'),
                           ("needs.look.result ==", 'an expression that ends where an operand was due'),
                           ("needs.look.result 'x'", 'an expression with more after its end'),
                           ('&& true', 'an operator where an operand was due'),
                           ('1 == 1', 'an expression this reader does not take')):
            read = evaluated(text)

            self.assertIsInstance(read, Rejected, text)
            self.assertTrue(read.rule.startswith(rule), (text, read))
            self.assertEqual(read, holds(text, CONTEXTS, NOTHING_FAILED), text)


if __name__ == '__main__':
    unittest.main()
