import os
import shutil
import subprocess
import tempfile
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
START = os.path.join(REPO, 'start')
TASK = ':TeamCode:codingServer'
APP_SETTING = 'ANDROID_STUDIO_APP'
DEFAULT_APP = '/Applications/Android Studio.app'
JAVA_HOME_IN_APP = os.path.join('Contents', 'jbr', 'Contents', 'Home')
RUN_SECONDS = 30

FAKE_GRADLEW = '''#!/bin/sh
printf '%s\\n' "$JAVA_HOME" "$@" > "$(pwd)/ran"
echo "gradlew ran with JAVA_HOME=$JAVA_HOME"
'''


def executable(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)
    os.chmod(path, 0o755)


class StartRunsTheCodingServerWithAndroidStudiosJava(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.mkdtemp(prefix='start-')
        self.addCleanup(shutil.rmtree, scratch, True)
        self.checkout = os.path.join(scratch, 'checkout')
        os.makedirs(self.checkout)
        shutil.copy2(START, os.path.join(self.checkout, 'start'))
        executable(os.path.join(self.checkout, 'gradlew'), FAKE_GRADLEW)
        self.elsewhere = os.path.join(scratch, 'elsewhere')
        os.makedirs(self.elsewhere)
        self.app = os.path.join(scratch, 'Applications', 'Android Studio.app')
        self.ran = os.path.join(self.checkout, 'ran')

    def install_android_studio(self):
        executable(os.path.join(self.app, JAVA_HOME_IN_APP, 'bin', 'java'), '#!/bin/sh\n')
        return os.path.join(self.app, JAVA_HOME_IN_APP)

    def start(self, app, *arguments):
        env = dict(os.environ, JAVA_HOME='/not/android/studios/java')
        env.pop(APP_SETTING, None)
        if app is not None:
            env[APP_SETTING] = app
        run = subprocess.run(
            [os.path.join(self.checkout, 'start')] + list(arguments),
            cwd=self.elsewhere, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, timeout=RUN_SECONDS)
        return run.returncode, run.stdout, run.stderr

    def what_gradlew_was_given(self):
        with open(self.ran) as f:
            return f.read().splitlines()

    def test_it_runs_the_coding_server_from_its_own_checkout_with_android_studios_java(self):
        java_home = self.install_android_studio()

        code, out, err = self.start(self.app)

        self.assertEqual(0, code, out + err)
        self.assertEqual([java_home, TASK], self.what_gradlew_was_given())

    def test_it_hands_gradle_what_it_is_given_after_the_task(self):
        java_home = self.install_android_studio()

        code, out, err = self.start(self.app, '--offline', '--info')

        self.assertEqual(0, code, out + err)
        self.assertEqual([java_home, TASK, '--offline', '--info'], self.what_gradlew_was_given())

    def test_without_android_studio_it_fails_naming_where_it_looked_and_runs_nothing(self):
        code, out, err = self.start(self.app)

        self.assertNotEqual(0, code, out + err)
        self.assertIn(os.path.join(self.app, JAVA_HOME_IN_APP), err)
        self.assertIn(APP_SETTING, err)
        self.assertFalse(os.path.exists(self.ran), 'start ran gradlew without a Java to give it')

    def test_it_looks_in_the_applications_folder_unless_told_where_android_studio_is(self):
        code, out, err = self.start(None)

        self.assertIn(os.path.join(DEFAULT_APP, JAVA_HOME_IN_APP), out + err)


class TheTaskStartRunsIsOneTheBuildHas(unittest.TestCase):
    def read(self, *path):
        with open(os.path.join(REPO, *path)) as f:
            return f.read()

    def test_the_project_it_names_is_in_the_build(self):
        project, _ = TASK.rsplit(':', 1)

        self.assertIn("include '%s'" % project, self.read('settings.gradle'))

    def test_that_project_registers_the_task_it_names(self):
        project, name = TASK.rsplit(':', 1)

        self.assertIn("tasks.register('%s'" % name, self.read(project.lstrip(':'), 'build.gradle'))


if __name__ == '__main__':
    unittest.main()
