import json
import os
import platform
import shutil
import subprocess
import tempfile
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
GRADLEW = os.path.join(REPO, 'gradlew')
JUDGE = os.path.join(REPO, 'TeamCode', 'suite-cost.gradle')
RUN_SECONDS = 600

REPORT = 'what the fixture spent: one test, nothing dear'
MET = 'the fixture spent what its budget pins'
MISSED = 'the fixture spent more than its budget pins'
FAILING_TESTS = 'There were failing tests'
NO_VERDICT = 'wrote no cost verdict'


def groovy(text):
    return "'" + text.replace('\\', '\\\\').replace("'", "\\'") + "'"


BUILD = '''plugins {
    id 'java'
}

repositories {
    mavenCentral()
}

dependencies {
    testImplementation 'junit:junit:4.13.2'
}

apply from: %s

def costDir = layout.buildDirectory.dir('cost').get().asFile
tasks.named('test', Test) {
    systemProperty 'suiteCost.ledger', costDir.absolutePath
}
judgeTheCostOf(tasks.named('test', Test), costDir, file('passed-over.json'))
''' % groovy(JUDGE)

TEST = '''import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.Assert;
import org.junit.Assume;
import org.junit.Test;

public class SpendsTest {
    @Test
    public void writesUpWhatItCost() throws Exception {
        Path ledger = Paths.get(System.getProperty("suiteCost.ledger"));
        Files.createDirectories(ledger);
        Files.write(ledger.resolve("report.txt"), %s.getBytes(StandardCharsets.UTF_8));
        %s
        Assert.assertTrue("this fixture's test fails when it is asked to", %s);
    }
%s}
'''

PASSES_ITSELF_OVER = '''
    @Test
    public void meansNothingHere() {
        Assume.assumeTrue("this fixture's second test passes itself over", false);
    }
'''
SKIPPED = 'SpendsTest.meansNothingHere'
WHY = 'there is nothing for it to mean on this platform'
THIS_PLATFORM = {'Linux': 'linux', 'Darwin': 'mac', 'Windows': 'windows'}[platform.system()]


class TheCostOfATestRunIsJudgedWhateverItsTestsDid(unittest.TestCase):
    def run_fixture(self, verdict, passes, skips=False, passed_over=None):
        fixture = tempfile.mkdtemp(prefix='suite-cost-')
        self.addCleanup(shutil.rmtree, fixture, True)
        with open(os.path.join(fixture, 'settings.gradle'), 'w') as f:
            f.write("rootProject.name = 'suite-cost-fixture'\n")
        with open(os.path.join(fixture, 'gradle.properties'), 'w') as f:
            f.write('org.gradle.jvmargs=-Xmx256m\n')
        with open(os.path.join(fixture, 'build.gradle'), 'w') as f:
            f.write(BUILD)
        sources = os.path.join(fixture, 'src', 'test', 'java')
        os.makedirs(sources)
        writes = ('' if verdict is None else
                  'Files.write(ledger.resolve("verdict.txt"), %s.getBytes(StandardCharsets.UTF_8));'
                  % json.dumps(verdict))
        with open(os.path.join(sources, 'SpendsTest.java'), 'w') as f:
            f.write(TEST % (json.dumps(REPORT), writes, 'true' if passes else 'false',
                            PASSES_ITSELF_OVER if skips else ''))
        if passed_over is not None:
            with open(os.path.join(fixture, 'passed-over.json'), 'w') as f:
                json.dump(passed_over, f)
        run = subprocess.run(
            [GRADLEW, '-p', fixture, '--no-daemon', '--console=plain', 'test'],
            cwd=fixture, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=RUN_SECONDS)
        return run.returncode, run.stdout

    def assertSaid(self, said, output):
        self.assertIn(said, output, 'the run never said it. It printed:\n' + output)

    def assertFailedSaying(self, said, output):
        self.assertIn('> ' + said, output.splitlines(),
                      'the run did not fail saying it. It printed:\n' + output)

    def test_a_run_whose_test_fails_still_prints_what_it_cost_and_the_budget_it_met(self):
        code, output = self.run_fixture('MET\n' + MET, passes=False)

        self.assertNotEqual(0, code, output)
        self.assertSaid(FAILING_TESTS, output)
        self.assertSaid(REPORT, output)
        self.assertSaid(MET, output)

    def test_a_run_whose_test_fails_still_fails_on_a_budget_it_missed(self):
        code, output = self.run_fixture('MISSED\n' + MISSED, passes=False)

        self.assertNotEqual(0, code, output)
        self.assertSaid(FAILING_TESTS, output)
        self.assertSaid(REPORT, output)
        self.assertFailedSaying(MISSED, output)

    def test_a_run_whose_test_fails_and_whose_test_jvm_wrote_no_verdict_says_it_has_not_checked_what_it_cost(self):
        code, output = self.run_fixture(None, passes=False)

        self.assertNotEqual(0, code, output)
        self.assertSaid(FAILING_TESTS, output)
        self.assertSaid(NO_VERDICT, output)

    def test_a_run_whose_tests_pass_prints_what_it_cost_and_fails_on_a_budget_it_missed(self):
        code, output = self.run_fixture('MISSED\n' + MISSED, passes=True)

        self.assertNotEqual(0, code, output)
        self.assertNotIn(FAILING_TESTS, output)
        self.assertSaid(REPORT, output)
        self.assertFailedSaying(MISSED, output)

    def test_a_run_whose_tests_pass_prints_what_it_cost_and_the_budget_it_met(self):
        code, output = self.run_fixture('MET\n' + MET, passes=True)

        self.assertEqual(0, code, output)
        self.assertSaid(REPORT, output)
        self.assertSaid(MET, output)

    def test_a_run_says_how_many_tests_it_ran_on_which_platform_and_how_many_it_passed_over(self):
        code, output = self.run_fixture('MET\n' + MET, passes=True, skips=True,
                                        passed_over={THIS_PLATFORM: {SKIPPED: WHY}})

        self.assertEqual(0, code, output)
        self.assertSaid('test ran 2 tests on %s, 1 of them passed over:' % THIS_PLATFORM, output)
        self.assertSaid('  %s -- %s' % (SKIPPED, WHY), output)

    def test_a_test_passed_over_that_the_list_does_not_name_for_this_platform_fails_the_run_naming_it(self):
        code, output = self.run_fixture('MET\n' + MET, passes=True, skips=True,
                                        passed_over={'elsewhere': {SKIPPED: WHY}})

        self.assertNotEqual(0, code, output)
        self.assertSaid(SKIPPED, output)
        self.assertSaid('passed over on %s without being named' % THIS_PLATFORM, output)

    def test_a_test_the_list_names_for_this_platform_that_ran_after_all_fails_the_run_naming_it(self):
        code, output = self.run_fixture('MET\n' + MET, passes=True,
                                        passed_over={THIS_PLATFORM: {'SpendsTest.writesUpWhatItCost': WHY}})

        self.assertNotEqual(0, code, output)
        self.assertSaid('named as passed over on %s, but ran: SpendsTest.writesUpWhatItCost' % THIS_PLATFORM,
                        output)
