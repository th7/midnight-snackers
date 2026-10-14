import unittest

from yaml_subset import Rejected, parse


def lines(*text):
    return '\n'.join(text) + '\n'


class ReadingTheSubset(unittest.TestCase):
    def test_a_mapping_of_plain_scalars_is_a_dict_of_their_text(self):
        self.assertEqual({'name': 'tests', 'timeout-minutes': '30'},
                         parse(lines('name: tests', 'timeout-minutes: 30')))

    def test_a_key_with_nothing_under_it_is_none(self):
        self.assertEqual({'on': {'pull_request': None}}, parse(lines('on:', '  pull_request:')))

    def test_a_mapping_indented_under_its_key_is_that_key_s_value(self):
        self.assertEqual({'jobs': {'judge': {'runs-on': 'ubuntu-latest'}}, 'name': 'tests'},
                         parse(lines('jobs:', '  judge:', '    runs-on: ubuntu-latest', 'name: tests')))

    def test_a_sequence_indented_under_its_key_is_a_list(self):
        self.assertEqual({'needs': ['judge', 'promote']},
                         parse(lines('needs:', '  - judge', '  - promote')))

    def test_a_sequence_item_may_open_a_mapping_on_its_dash_line(self):
        self.assertEqual(
            {'steps': [{'uses': 'actions/checkout@v4', 'with': {'ref': 'develop'}},
                       {'name': 'Formatting', 'run': './gradlew spotlessCheck'}]},
            parse(lines('steps:',
                        '  - uses: actions/checkout@v4',
                        '    with:',
                        '      ref: develop',
                        '  - name: Formatting',
                        '    run: ./gradlew spotlessCheck')))

    def test_a_literal_block_keeps_its_lines_their_inner_indentation_and_one_final_newline(self):
        self.assertEqual({'run': 'if true; then\n  echo yes\nfi\n', 'name': 'after'},
                         parse(lines('run: |', '  if true; then', '    echo yes', '  fi', 'name: after')))

    def test_a_literal_block_keeps_the_blank_lines_inside_it(self):
        self.assertEqual({'run': 'one\n\ntwo\n'}, parse(lines('run: |', '  one', '', '  two')))

    def test_a_stripped_literal_block_drops_its_final_newline(self):
        self.assertEqual({'run': 'one\ntwo'}, parse(lines('run: |-', '  one', '  two')))

    def test_a_flow_sequence_of_scalars_is_a_list(self):
        self.assertEqual({'branches': ['develop', 'nugget-develop']},
                         parse(lines('branches: [develop, nugget-develop]')))

    def test_an_empty_flow_mapping_is_an_empty_dict(self):
        self.assertEqual({'permissions': {}}, parse(lines('permissions: {}')))

    def test_a_single_quoted_scalar_is_its_text_with_doubled_quotes_undone(self):
        self.assertEqual({'java-version': '21', 'say': "it's"},
                         parse(lines("java-version: '21'", "say: 'it''s'")))

    def test_an_expression_is_a_plain_scalar_and_keeps_its_quotes(self):
        self.assertEqual({'if': "${{ github.event_name == 'push' }}"},
                         parse(lines("if: ${{ github.event_name == 'push' }}")))


class RefusingWhatLiesOutsideTheSubset(unittest.TestCase):
    def assertRefused(self, text, naming):
        read = parse(text)
        self.assertIsInstance(read, Rejected, 'read as %r' % (read,))
        self.assertIn(naming, read.rule)

    def test_a_comment_line_is_refused(self):
        self.assertRefused(lines('name: tests', '# what it does'), 'comment')

    def test_a_comment_after_a_value_is_refused(self):
        self.assertRefused(lines('name: tests # what it does'), 'comment')

    def test_a_tab_in_the_indentation_is_refused(self):
        self.assertRefused('jobs:\n\tjudge: x\n', 'tab')

    def test_an_anchor_is_refused(self):
        self.assertRefused(lines('defaults: &shared', '  run: x'), 'anchor')

    def test_an_alias_is_refused(self):
        self.assertRefused(lines('defaults: *shared'), 'anchor')

    def test_a_flow_mapping_with_anything_in_it_is_refused(self):
        self.assertRefused(lines('permissions: {contents: read}'), 'flow mapping')

    def test_a_key_given_twice_is_refused(self):
        self.assertRefused(lines('name: one', 'name: two'), 'twice')

    def test_a_folded_block_is_refused(self):
        self.assertRefused(lines('run: >', '  one'), 'folded')

    def test_a_line_indented_under_a_scalar_is_refused(self):
        self.assertRefused(lines('name: tests', '  more: x'), 'indented')

    def test_a_sequence_at_its_key_s_own_indentation_is_refused(self):
        self.assertRefused(lines('needs:', '- judge'), 'indented under its key')

    def test_a_plain_scalar_holding_a_colon_and_a_space_is_refused(self):
        self.assertRefused(lines('name: one: two'), 'colon')

    def test_a_document_that_is_not_a_mapping_is_refused(self):
        self.assertRefused(lines('- one'), 'mapping')

    def test_a_document_marker_is_refused(self):
        self.assertRefused(lines('---', 'name: tests'), 'document marker')

    def test_a_refusal_names_the_line_it_found(self):
        self.assertRefused(lines('name: tests', 'on:', '  push: # here'), 'line 3')


if __name__ == '__main__':
    unittest.main()
