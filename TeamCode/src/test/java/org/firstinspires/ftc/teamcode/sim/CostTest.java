package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.Test;

public class CostTest {

    private final Cost.Ledger ledger = new Cost.Ledger();

    private static Map<Cost.Kind, Integer> budgetOf(Object... pairs) {
        Map<Cost.Kind, Integer> budget = new LinkedHashMap<>();
        for (Cost.Kind kind : Cost.Kind.values()) {
            budget.put(kind, 0);
        }
        for (int i = 0; i < pairs.length; i += 2) {
            budget.put((Cost.Kind) pairs[i], (Integer) pairs[i + 1]);
        }
        return budget;
    }

    @Test
    public void aTestsOwnLedgerIsNotTheOneTheSuiteIsKeeping() {
        Map<Cost.Kind, Cost.Tally> before = Cost.taken();

        spend(Cost.Kind.GIT, 3);

        assertEquals(
                "a test that counts its own spending must not land in the suite's ledger",
                before.get(Cost.Kind.GIT).count,
                Cost.taken().get(Cost.Kind.GIT).count);
        assertNotSame(before, Cost.taken());
    }

    @Test
    public void somethingExpensiveIsCountedOnceAndItsTimeAddedUp() {
        try (Cost.Spent ignored = ledger.start(Cost.Kind.GIT)) {
            spendAbout(5);
        }
        try (Cost.Spent ignored = ledger.start(Cost.Kind.GIT)) {
            spendAbout(5);
        }

        Cost.Tally git = ledger.taken().get(Cost.Kind.GIT);
        assertEquals(2, git.count);
        assertTrue("both spells are in the time: " + git.seconds, git.seconds >= 0.008);
    }

    @Test
    public void everyKindIsInTheLedgerEvenTheOnesNothingSpent() {
        spend(Cost.Kind.GIT, 1);

        Map<Cost.Kind, Cost.Tally> taken = ledger.taken();

        assertEquals(
                "an absent kind reads as none spent, never as a kind that is missing",
                Cost.Kind.values().length,
                taken.size());
        assertEquals(0, taken.get(Cost.Kind.COMPILE).count);
        assertEquals(0, taken.get(Cost.Kind.COMPILE).seconds, 0);
    }

    @Test
    public void whatThrewStillCostWhatItSpent() {
        assertThrows(IllegalStateException.class, () -> {
            try (Cost.Spent ignored = ledger.start(Cost.Kind.CHILD_JVM)) {
                spendAbout(5);
                throw new IllegalStateException("the child would not start");
            }
        });

        assertEquals(
                "a child that failed to start still cost the time it took to fail",
                1,
                ledger.taken().get(Cost.Kind.CHILD_JVM).count);
        assertTrue(ledger.taken().get(Cost.Kind.CHILD_JVM).seconds >= 0.004);
    }

    @Test
    public void closingTwiceCountsOnce() {
        Cost.Spent spent = ledger.start(Cost.Kind.COMPILE);
        spent.close();
        spent.close();

        assertEquals(1, ledger.taken().get(Cost.Kind.COMPILE).count);
    }

    @Test
    public void whatSeveralThreadsSpendAddsUp() throws InterruptedException {
        Thread[] threads = new Thread[8];
        for (int i = 0; i < threads.length; i++) {
            threads[i] = new Thread(() -> spend(Cost.Kind.PROJECT_COPY, 25));
            threads[i].start();
        }
        for (Thread thread : threads) {
            thread.join();
        }

        assertEquals(200, ledger.taken().get(Cost.Kind.PROJECT_COPY).count);
    }

    @Test
    public void aSuiteThatSpentWhatItIsBudgetedHasMetIt() {
        spend(Cost.Kind.GIT, 3);

        Cost.Verdict verdict = Cost.verdictOn(ledger.taken(), budgetOf(Cost.Kind.GIT, 3), true);

        assertEquals(Cost.Verdict.Outcome.MET, verdict.outcome);
        assertTrue(verdict.ok());
    }

    @Test
    public void spendingMoreThanTheBudgetNamesTheKindTheCountsAndHowToRepinIt() {
        spend(Cost.Kind.CHILD_JVM, 5);

        Cost.Verdict verdict = Cost.verdictOn(ledger.taken(), budgetOf(Cost.Kind.CHILD_JVM, 2), true);

        assertEquals(Cost.Verdict.Outcome.MISSED, verdict.outcome);
        assertFalse(verdict.ok());
        assertTrue(verdict.said, verdict.said.contains(Cost.Kind.CHILD_JVM.key));
        assertTrue("it says what was pinned", verdict.said.contains("pins 2"));
        assertTrue("and what the suite did", verdict.said.contains("spent 5"));
        assertTrue("and how to pin the new number", verdict.said.contains(Cost.REGENERATE));
    }

    @Test
    public void spendingLessThanTheBudgetMissesItToo() {
        spend(Cost.Kind.CHILD_JVM, 1);

        Cost.Verdict verdict = Cost.verdictOn(ledger.taken(), budgetOf(Cost.Kind.CHILD_JVM, 4), true);

        assertEquals(
                "a budget is a change detector, not a ceiling: cheaper is a change too",
                Cost.Verdict.Outcome.MISSED,
                verdict.outcome);
        assertTrue(verdict.said, verdict.said.contains("fewer"));
    }

    @Test
    public void aKindNoBudgetPinsIsNotJudgedSilently() {
        spend(Cost.Kind.GIT, 1);
        Map<Cost.Kind, Integer> budget = budgetOf();
        budget.remove(Cost.Kind.GIT);

        Cost.Verdict verdict = Cost.verdictOn(ledger.taken(), budget, true);

        assertEquals(Cost.Verdict.Outcome.MISSED, verdict.outcome);
        assertTrue(verdict.said, verdict.said.contains(Cost.Kind.GIT.key));
        assertTrue(verdict.said, verdict.said.contains("pins"));
    }

    @Test
    public void aBudgetThatIsNotThereIsNotAPass() {
        spend(Cost.Kind.GIT, 1);

        Cost.Verdict verdict = Cost.verdictOn(ledger.taken(), null, true);

        assertEquals(Cost.Verdict.Outcome.MISSED, verdict.outcome);
        assertTrue(verdict.said, verdict.said.contains("nothing is being checked"));
    }

    @Test
    public void aRunOfPartOfTheSuiteSaysItCouldNotJudgeRatherThanPassing() {
        spend(Cost.Kind.GIT, 1);

        Cost.Verdict verdict = Cost.verdictOn(ledger.taken(), budgetOf(Cost.Kind.GIT, 99), false);

        assertEquals(Cost.Verdict.Outcome.COULD_NOT_JUDGE, verdict.outcome);
        assertTrue("it is not a failure either", verdict.ok());
        assertTrue(verdict.said, verdict.said.contains("could not judge"));
    }

    @Test
    public void theReportSaysWhatEachKindCostAndWhatItIs() {
        spend(Cost.Kind.CHILD_JVM, 2);

        String report = Cost.report(ledger.taken());

        assertTrue(report, report.contains(Cost.Kind.CHILD_JVM.key));
        assertTrue("the count is pinned, so it is printed", report.contains("2"));
        assertTrue(
                "and what the kind is, so a reader need not know the code", report.contains(Cost.Kind.CHILD_JVM.what));
        for (Cost.Kind kind : Cost.Kind.values()) {
            assertTrue("every kind is in the report: " + kind, report.contains(kind.key));
        }
    }

    @Test
    public void aBudgetSurvivesBeingWrittenAndReadBack() {
        spend(Cost.Kind.GIT, 3);
        spend(Cost.Kind.COMPILE, 7);

        Map<Cost.Kind, Integer> read = Cost.budgetFrom(Cost.budgetJson(ledger.taken()));

        assertEquals(Integer.valueOf(3), read.get(Cost.Kind.GIT));
        assertEquals(Integer.valueOf(7), read.get(Cost.Kind.COMPILE));
        assertEquals(
                "what is written is every kind, so a new kind is never silently unpinned",
                Cost.Kind.values().length,
                read.size());
    }

    @Test
    public void aBudgetNamingSomethingThatIsNoLongerAKindIsNotReadPastInSilence() {
        String json = "{\"child-jvm\": 1, \"steam-engine\": 4}";

        IllegalStateException wrong = assertThrows(IllegalStateException.class, () -> Cost.budgetFrom(json));

        assertTrue(wrong.getMessage(), wrong.getMessage().contains("steam-engine"));
    }

    @Test
    public void theTimesAreNeverJudged() {
        spend(Cost.Kind.GIT, 1);
        Map<Cost.Kind, Cost.Tally> slow = new LinkedHashMap<>(ledger.taken());
        slow.put(Cost.Kind.GIT, new Cost.Tally(1, 600.0));

        assertEquals(
                "a count is the suite's and a time is the machine's, so only the count is a verdict",
                Cost.Verdict.Outcome.MET,
                Cost.verdictOn(slow, budgetOf(Cost.Kind.GIT, 1), true).outcome);
    }

    private void spend(Cost.Kind kind, int times) {
        for (int i = 0; i < times; i++) {
            ledger.start(kind).close();
        }
    }

    private static void spendAbout(long millis) {
        long until = System.nanoTime() + millis * 1_000_000L;
        while (System.nanoTime() < until) {
            Thread.onSpinWait();
        }
    }
}
