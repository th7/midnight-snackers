import math
import re
from dataclasses import dataclass

TOKEN = re.compile(r" *(?:(?P<text>'(?:[^']|'')*')|(?P<operator>==|!=|&&|\|\||!|\(|\))"
                   r"|(?P<call>[A-Za-z_][A-Za-z0-9_]*)\(|(?P<path>[A-Za-z_][A-Za-z0-9_-]*(?:\.[A-Za-z_][A-Za-z0-9_-]*)*)"
                   r"(?![A-Za-z0-9_.(\[-]))")
WRAPPED = re.compile(r'\$\{\{(.*)\}\}', re.DOTALL)
STATUS_CHECKS = ('success', 'failure', 'cancelled', 'always')
LITERALS = {'true': True, 'false': False, 'null': None}
COMPARISONS = {'==': 'equal', '!=': 'unequal'}
COMBINING = ('not', 'and', 'or', 'equal', 'unequal')


@dataclass(frozen=True)
class Rejected:
    rule: str


@dataclass(frozen=True)
class Status:
    failed: bool
    cancelled: bool


def tokens(text):
    found = []
    here = 0
    while text[here:].strip(' ') != '':
        match = TOKEN.match(text, here)
        if match is None:
            return Rejected('an expression this reader does not take: %r' % text[here:].strip(' '))
        kind, name = match.lastgroup, match.group(match.lastgroup)
        if kind == 'call' and (name not in STATUS_CHECKS or not text[match.end():].lstrip(' ').startswith(')')):
            return Rejected('a function this reader does not take: %s(...)' % name)
        found.append((kind, name))
        here = match.end()
    return tuple(found)


def binary(read, at, operators, operand):
    first = operand(read, at)
    if isinstance(first, Rejected):
        return first
    tree, here = first
    while here < len(read) and read[here][0] == 'operator' and read[here][1] in operators:
        following = operand(read, here + 1)
        if isinstance(following, Rejected):
            return following
        right, after = following
        tree, here = (operators[read[here][1]], tree, right), after
    return tree, here


def either(read, at):
    return binary(read, at, {'||': 'or'}, both)


def both(read, at):
    return binary(read, at, {'&&': 'and'}, compared)


def compared(read, at):
    return binary(read, at, COMPARISONS, negated)


def negated(read, at):
    if at < len(read) and read[at] == ('operator', '!'):
        inner = negated(read, at + 1)
        if isinstance(inner, Rejected):
            return inner
        tree, here = inner
        return ('not', tree), here
    return primary(read, at)


def primary(read, at):
    if at >= len(read):
        return Rejected('an expression that ends where an operand was due')
    kind, text = read[at]
    if (kind, text) == ('operator', '('):
        inner = either(read, at + 1)
        if isinstance(inner, Rejected):
            return inner
        tree, here = inner
        if here >= len(read) or read[here] != ('operator', ')'):
            return Rejected('a parenthesis left open')
        return tree, here + 1
    if kind == 'text':
        return ('value', text[1:-1].replace("''", "'")), at + 1
    if kind == 'call':
        return ('status', text), at + 2
    if kind == 'path' and text in LITERALS:
        return ('value', LITERALS[text]), at + 1
    if kind == 'path':
        return ('path', tuple(text.split('.'))), at + 1
    return Rejected('an operator where an operand was due: %s' % text)


def parsed(text):
    wrapped = WRAPPED.fullmatch(text.strip(' '))
    read = tokens(wrapped.group(1) if wrapped else text)
    if isinstance(read, Rejected):
        return read
    result = either(read, 0)
    if isinstance(result, Rejected):
        return result
    tree, after = result
    if after != len(read):
        return Rejected('an expression with more after its end: %r' % (read[after:],))
    return tree


def number(each):
    if each is None:
        return 0.0
    if isinstance(each, bool):
        return 1.0 if each else 0.0
    if isinstance(each, str) and each.strip() == '':
        return 0.0
    if isinstance(each, str) and re.fullmatch(r'-?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?', each.strip()):
        return float(each.strip())
    return math.nan


def loosely_equal(left, right):
    if isinstance(left, str) and isinstance(right, str):
        return left.casefold() == right.casefold()
    if isinstance(left, (dict, list)) or isinstance(right, (dict, list)):
        return left is right
    if type(left) is type(right):
        return left == right
    return number(left) == number(right)


def truthy(each):
    if each is None or each is False or (isinstance(each, str) and each == ''):
        return False
    if isinstance(each, float) and (each == 0 or math.isnan(each)):
        return False
    return True


def looked_up(contexts, path):
    here = contexts
    for name in path:
        if not isinstance(here, dict) or name not in here:
            return None
        here = here[name]
    return here


def status_check(name, status):
    if name == 'success':
        return not status.failed and not status.cancelled
    if name == 'failure':
        return status.failed
    if name == 'cancelled':
        return status.cancelled
    return True


def evaluated(tree, contexts, status):
    kind = tree[0]
    if kind == 'value':
        return tree[1]
    if kind == 'path':
        return looked_up(contexts, tree[1])
    if kind == 'status':
        return status_check(tree[1], status)
    if kind == 'not':
        return not truthy(evaluated(tree[1], contexts, status))
    left = evaluated(tree[1], contexts, status)
    if kind == 'and':
        return evaluated(tree[2], contexts, status) if truthy(left) else left
    if kind == 'or':
        return left if truthy(left) else evaluated(tree[2], contexts, status)
    equal = loosely_equal(left, evaluated(tree[2], contexts, status))
    return equal if kind == 'equal' else not equal


def names_a_status_check(tree):
    if tree[0] == 'status':
        return True
    if tree[0] in COMBINING:
        return any(names_a_status_check(each) for each in tree[1:])
    return False


def value(text, contexts, status):
    tree = parsed(text)
    if isinstance(tree, Rejected):
        return tree
    return evaluated(tree, contexts, status)


def holds(text, contexts, status):
    tree = parsed(text)
    if isinstance(tree, Rejected):
        return tree
    checked = tree if names_a_status_check(tree) else ('and', ('status', 'success'), tree)
    return truthy(evaluated(checked, contexts, status))
