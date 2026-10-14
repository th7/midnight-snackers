import re
from dataclasses import dataclass

KEY_LINE = re.compile(r'([A-Za-z_][A-Za-z0-9_-]*):(?: +(.*))?')
SINGLE_QUOTED = re.compile(r"'((?:[^']|'')*)'")
FLOW_ITEM = re.compile(r'[A-Za-z0-9_./-]+')
DOCUMENT_MARKERS = ('---', '...')


@dataclass(frozen=True)
class Rejected:
    rule: str


def parse(text):
    rows = tuple(text.split('\n'))
    if rows and rows[-1] == '':
        rows = rows[:-1]
    first = significant(rows, 0)
    if isinstance(first, Rejected):
        return first
    if first == len(rows):
        return {}
    if indent_of(rows[first]) != 0:
        return at(first, 'the document starts indented')
    if is_item(rows[first]):
        return at(first, 'the document is a sequence, and this reader takes only a mapping')
    read = mapping(rows, first, 0, None)
    if isinstance(read, Rejected):
        return read
    value, after = read
    if after != len(rows):
        return at(after, 'a line indented under nothing that holds it')
    return value


def at(index, rule):
    return Rejected('line %d: %s' % (index + 1, rule))


def indent_of(row):
    return len(row) - len(row.lstrip(' '))


def is_item(text):
    return text == '-' or text.startswith('- ')


def refusal_of_row(rows, index):
    row = rows[index]
    stripped = row.lstrip(' ')
    if '\r' in row:
        return at(index, 'a carriage return')
    if stripped.startswith('\t'):
        return at(index, 'a tab in the indentation')
    if stripped.startswith('#'):
        return at(index, 'a comment, which is prose')
    if row in DOCUMENT_MARKERS or any(row.startswith(marker + ' ') for marker in DOCUMENT_MARKERS):
        return at(index, 'a document marker')
    return None


def significant(rows, start):
    for index in range(start, len(rows)):
        refusal = refusal_of_row(rows, index)
        if refusal is not None:
            return refusal
        if rows[index].strip(' ') != '':
            return index
    return len(rows)


def block(rows, index, indent):
    if is_item(rows[index][indent:]):
        return sequence(rows, index, indent)
    return mapping(rows, index, indent, None)


def mapping(rows, index, indent, first):
    entries = {}
    here = index
    while here < len(rows):
        row_indent = indent if (here == index and first is not None) else indent_of(rows[here])
        if row_indent < indent:
            return entries, here
        if row_indent > indent:
            return at(here, 'a line indented under nothing that holds it')
        text = first if (here == index and first is not None) else rows[here][indent:]
        if is_item(text):
            return at(here, 'a sequence item among keys; a sequence sits indented under its key')
        entry = KEY_LINE.fullmatch(text)
        if entry is None:
            return at(here, 'not a key and its value')
        key, rest = entry.group(1), (entry.group(2) or '').rstrip(' ')
        if key in entries:
            return at(here, 'the key %s given twice' % key)
        read = value_after(rows, here, indent, rest)
        if isinstance(read, Rejected):
            return read
        entries[key], after = read
        here = significant(rows, after)
        if isinstance(here, Rejected):
            return here
    return entries, here


def sequence(rows, index, indent):
    items = []
    here = index
    while here < len(rows):
        row_indent = indent_of(rows[here])
        if row_indent < indent:
            return items, here
        if row_indent > indent:
            return at(here, 'a line indented under nothing that holds it')
        text = rows[here][indent:]
        if not is_item(text):
            return at(here, 'a key among sequence items')
        after_dash = text[1:]
        item = after_dash.lstrip(' ')
        item_indent = indent + 1 + len(after_dash) - len(item)
        read = item_value(rows, here, indent, item_indent, item.rstrip(' '))
        if isinstance(read, Rejected):
            return read
        value, after = read
        items.append(value)
        here = significant(rows, after)
        if isinstance(here, Rejected):
            return here
    return items, here


def item_value(rows, index, indent, item_indent, item):
    if item == '':
        return nested_or_none(rows, index, indent)
    if KEY_LINE.fullmatch(item) is not None:
        return mapping(rows, index, item_indent, item)
    value = scalar(item)
    if isinstance(value, Rejected):
        return at(index, value.rule)
    return value, index + 1


def nested_or_none(rows, index, indent):
    following = significant(rows, index + 1)
    if isinstance(following, Rejected):
        return following
    if following < len(rows) and indent_of(rows[following]) > indent:
        return block(rows, following, indent_of(rows[following]))
    if following < len(rows) and indent_of(rows[following]) == indent and is_item(rows[following][indent:]):
        return at(following, 'a sequence at its key\'s own indentation; a sequence sits indented under its key')
    return None, index + 1


def value_after(rows, index, indent, rest):
    if rest == '':
        return nested_or_none(rows, index, indent)
    if rest in ('|', '|-'):
        return literal(rows, index + 1, indent, rest == '|')
    if rest.startswith('>'):
        return at(index, 'a folded block, which this reader does not take')
    if rest.startswith('|'):
        return at(index, 'a literal block with an indicator this reader does not take')
    value = scalar(rest)
    if isinstance(value, Rejected):
        return at(index, value.rule)
    return value, index + 1


def literal(rows, start, parent_indent, keep_final_newline):
    collected = []
    block_indent = None
    here = start
    while here < len(rows):
        row = rows[here]
        if '\r' in row:
            return at(here, 'a carriage return')
        if row.strip(' ') == '':
            collected.append('')
            here += 1
            continue
        row_indent = indent_of(row)
        if row_indent <= parent_indent:
            break
        if block_indent is None:
            block_indent = row_indent
        if row_indent < block_indent:
            return at(here, 'a literal block line indented less than the block\'s first line')
        collected.append(row[block_indent:])
        here += 1
    while collected and collected[-1] == '':
        collected.pop()
    if not collected:
        return '', here
    return '\n'.join(collected) + ('\n' if keep_final_newline else ''), here


def scalar(text):
    if text.startswith('#'):
        return Rejected('a comment, which is prose')
    if text.startswith("'"):
        quoted = SINGLE_QUOTED.fullmatch(text)
        if quoted is None:
            return Rejected('a single-quoted scalar that does not end where its line does')
        return quoted.group(1).replace("''", "'")
    if text.startswith('"'):
        return Rejected('a double-quoted scalar, which this reader does not take')
    if text.startswith('['):
        return flow_sequence(text)
    if text == '{}':
        return {}
    if text.startswith('{'):
        return Rejected('a flow mapping, which this reader does not take')
    if text[0] in '&*':
        return Rejected('an anchor or an alias, which this reader does not take')
    if text[0] in '!|>%@`,?':
        return Rejected('a plain scalar starting with %s' % text[0])
    if ' #' in text:
        return Rejected('a comment after a value, which is prose')
    if ': ' in text or text.endswith(':'):
        return Rejected('a plain scalar holding a colon and a space')
    return text


def flow_sequence(text):
    if not text.endswith(']'):
        return Rejected('a flow sequence that does not end where its line does')
    inner = text[1:-1].strip(' ')
    if inner == '':
        return []
    items = [item.strip(' ') for item in inner.split(',')]
    for item in items:
        if FLOW_ITEM.fullmatch(item) is None:
            return Rejected('a flow sequence item this reader does not take: %s' % item)
    return items
