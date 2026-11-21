import os
import shutil
import subprocess
import tempfile
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
ATTRIBUTES = os.path.join(REPO, '.gitattributes')
GIT_FOR_WINDOWS_CHECKS_OUT_WITH_AUTOCRLF = 'https://github.com/git-for-windows/build-extra/blob/main/installer/install.iss'
BINARY_EXTENSIONS = ('.png', '.wav', '.glb', '.jar', '.pdf', '.keystore')


def git(*args, cwd=REPO, config=()):
    command = ['git']
    for setting in config:
        command += ['-c', setting]
    return subprocess.run(command + list(args), cwd=cwd, check=True, capture_output=True).stdout


def tracked():
    return [name.decode() for name in git('ls-files', '-z').split(b'\0') if name]


def attribute(name, paths):
    said = git('check-attr', name, '--', *paths).decode().splitlines()
    return {line.split(': ')[0]: line.split(': ')[2] for line in said}


def starts_as_a_script(path):
    with open(os.path.join(REPO, path), 'rb') as f:
        return f.read(2) == b'#!'


class LineEndingsTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(os.path.isfile(ATTRIBUTES), 'the repository has no .gitattributes')
        self.scratch = tempfile.mkdtemp(prefix='line-endings-')
        self.addCleanup(shutil.rmtree, self.scratch, ignore_errors=True)

    def test_every_file_the_index_holds_is_lf_or_binary(self):
        lines = git('ls-files', '--eol').decode().splitlines()
        crlf = [line.split('\t')[-1] for line in lines if line.split()[0] in ('i/crlf', 'i/mixed')]

        self.assertEqual([], crlf)

    def test_every_script_is_checked_out_lf_wherever_it_is_cloned(self):
        scripts = [path for path in tracked() if starts_as_a_script(path)]
        self.assertIn('gradlew', scripts)

        eol = attribute('eol', scripts)

        self.assertEqual({path: 'lf' for path in scripts}, eol)

    def test_every_binary_file_is_never_converted(self):
        binaries = [path for path in tracked() if path.lower().endswith(BINARY_EXTENSIONS)]
        self.assertTrue(binaries)

        text = attribute('text', binaries)

        self.assertEqual({path: 'unset' for path in binaries}, text)

    def test_a_windows_clone_changes_nothing_and_what_its_editor_saves_crlf_is_committed_lf(self):
        origin = os.path.join(self.scratch, 'origin')
        os.makedirs(os.path.join(origin, 'src'))
        shutil.copy(ATTRIBUTES, origin)
        files = {'gradlew': b'#!/bin/sh\necho gradle\n',
                 'gradlew.bat': b'@echo off\r\necho gradle\r\n',
                 'src/Plans.java': b'class Plans {\n}\n',
                 'src/field.glb': b'glTF\r\n\0\x01\r\n'}
        for name, content in files.items():
            with open(os.path.join(origin, name), 'wb') as f:
                f.write(content)
        identity = ('user.name=test', 'user.email=test@example.com', 'core.autocrlf=false', 'init.defaultBranch=main')
        git('init', '-q', cwd=origin, config=identity)
        git('add', '-A', cwd=origin, config=identity)
        git('commit', '-qm', 'as committed', cwd=origin, config=identity)

        clone = os.path.join(self.scratch, 'clone')
        windows = identity[:2] + ('core.autocrlf=true',)
        git('clone', '-q', '-c', 'core.autocrlf=true', origin, clone, cwd=self.scratch, config=windows)
        with open(os.path.join(clone, 'gradlew'), 'rb') as f:
            self.assertEqual(files['gradlew'], f.read(), 'a script was checked out CRLF')
        with open(os.path.join(clone, 'src', 'field.glb'), 'rb') as f:
            self.assertEqual(files['src/field.glb'], f.read(), 'a binary file was converted')
        self.assertEqual(b'', git('status', '--porcelain', cwd=clone, config=windows),
                         'a fresh clone made as Git for Windows makes one by default, '
                         + GIT_FOR_WINDOWS_CHECKS_OUT_WITH_AUTOCRLF + ', shows changes nobody made')

        for loops, asked in ((1, windows), (2, identity[:2] + ('core.autocrlf=false',))):
            with open(os.path.join(clone, 'src', 'Plans.java'), 'wb') as f:
                f.write(b'class Plans {\r\n    int loops = %d;\r\n}\r\n' % loops)
            git('commit', '-qam', 'an edit saved CRLF', cwd=clone, config=asked)

            committed = git('show', 'HEAD:src/Plans.java', cwd=clone)

            self.assertEqual(b'class Plans {\n    int loops = %d;\n}\n' % loops, committed,
                             'committed with ' + asked[-1])

    def test_a_windows_clone_checks_out_text_as_committed_so_a_test_reading_a_file_reads_the_same_bytes_anywhere(self):
        origin = os.path.join(self.scratch, 'origin')
        os.makedirs(os.path.join(origin, 'resources'))
        shutil.copy(ATTRIBUTES, origin)
        committed = b"export const DEFAULT_RESOLUTION = 'high';\nexport const FIELD_SAID = {};\n"
        with open(os.path.join(origin, 'resources', 'field-default.js'), 'wb') as f:
            f.write(committed)
        identity = ('user.name=test', 'user.email=test@example.com', 'core.autocrlf=false', 'init.defaultBranch=main')
        git('init', '-q', cwd=origin, config=identity)
        git('add', '-A', cwd=origin, config=identity)
        git('commit', '-qm', 'as committed', cwd=origin, config=identity)

        clone = os.path.join(self.scratch, 'clone')
        windows = identity[:2] + ('core.autocrlf=true',)
        git('clone', '-q', '-c', 'core.autocrlf=true', origin, clone, cwd=self.scratch, config=windows)

        with open(os.path.join(clone, 'resources', 'field-default.js'), 'rb') as f:
            self.assertEqual(committed, f.read(), 'a clone made as Git for Windows makes one by default, '
                             + GIT_FOR_WINDOWS_CHECKS_OUT_WITH_AUTOCRLF + ', checked a text file out CRLF')


if __name__ == '__main__':
    unittest.main()
