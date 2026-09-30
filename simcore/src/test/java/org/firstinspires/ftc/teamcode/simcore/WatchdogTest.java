package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class WatchdogTest {
    private static final Seconds STARTUP = seconds(60);
    private static final Seconds SILENCE = seconds(5);
    private static final Seconds UNWATCHED = seconds(300);
    private static final Seconds KILL_GRACE = seconds(2);
    private static final Seconds VIEW = seconds(30);
    private static final Watchdog.Waits WAITS = new Watchdog.Waits(STARTUP, SILENCE, UNWATCHED, KILL_GRACE, VIEW);

    private static final Moment LAUNCHED = at(1000);

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static Moment at(double seconds) {
        return new Moment((long) (seconds * 1e9));
    }

    private static Watchdog.Verdict at(RunState run, double seconds) {
        return Watchdog.verdict(run, at(seconds), WAITS);
    }

    @Test
    public void aRunIsNotWatchedWhileItsSourcesAreBuilding() {
        assertEquals(Watchdog.Verdict.WATCHING, at(new RunState.Building(), 1e6));
    }

    @Test
    public void aRunThatIsOverHasNothingLeftToWatch() {
        assertEquals(Watchdog.Verdict.OVER, at(new RunState.Over(), 0));
    }

    @Test
    public void aChildHasTheStartupForTheOpModesTimeToBegin() {
        RunState starting = new RunState.Starting(LAUNCHED);

        assertEquals(Watchdog.Verdict.WATCHING, at(starting, 1000));
        assertEquals(Watchdog.Verdict.WATCHING, at(starting, 1060));
        assertEquals(Watchdog.Verdict.NEVER_STARTED, at(starting, 1060.1));
    }

    @Test
    public void whileAChildStartsNeitherItsSilenceNorItsWatchersAreJudged() {
        RunState starting = new RunState.Starting(LAUNCHED);

        assertEquals(
                "a JVM that is loading says nothing, and the startup is how long it may take",
                Watchdog.Verdict.WATCHING,
                at(starting, 1000 + SILENCE.value() + 1));
    }

    @Test
    public void aChildHeldForItsViewIsWaitedOnForTheViewWaitAndThenPlacedWithoutIt() {
        RunState held = new RunState.Held(at(990));

        assertEquals(Watchdog.Verdict.WATCHING, at(held, 1000));
        assertEquals(Watchdog.Verdict.WATCHING, at(held, 1020));
        assertEquals(Watchdog.Verdict.VIEW_LATE, at(held, 1020.1));
    }

    @Test
    public void aChildHeldForItsViewIsNotJudgedForTheStartupItWasNotLetUse() {
        Seconds aSecond = seconds(1);
        Watchdog.Waits viewOutlastsStartup = new Watchdog.Waits(aSecond, aSecond, aSecond, KILL_GRACE, VIEW);

        assertEquals(
                "the bench held it, so the time held is not the child's to have started in",
                Watchdog.Verdict.WATCHING,
                Watchdog.verdict(new RunState.Held(at(1000)), at(1025), viewOutlastsStartup));
    }

    @Test
    public void onlyTheVerdictsThatEndNothingKeepTheWatchdogWatching() {
        assertTrue(Watchdog.Verdict.WATCHING.keepsWatching());
        assertTrue(
                "a child placed without its view still has a run to watch", Watchdog.Verdict.VIEW_LATE.keepsWatching());
        for (Watchdog.Verdict ending : List.of(
                Watchdog.Verdict.OVER,
                Watchdog.Verdict.NEVER_STARTED,
                Watchdog.Verdict.SILENT,
                Watchdog.Verdict.UNWATCHED,
                Watchdog.Verdict.IGNORED_STOP)) {
            assertFalse(ending.name(), ending.keepsWatching());
        }
    }

    @Test
    public void aRunningChildThatSaysNothingForLongerThanTheSilenceHasHung() {
        RunState running = new RunState.Running(at(1010), at(1010));

        assertEquals(Watchdog.Verdict.WATCHING, at(running, 1015));
        assertEquals(Watchdog.Verdict.SILENT, at(running, 1015.1));
    }

    @Test
    public void aRunNobodyHasAskedAboutForLongerThanItMayGoUnwatchedIsStopped() {
        Moment lastLook = at(1010);

        assertEquals(Watchdog.Verdict.WATCHING, at(new RunState.Running(at(1310), lastLook), 1310));
        assertEquals(Watchdog.Verdict.UNWATCHED, at(new RunState.Running(at(1310), lastLook), 1310.1));
    }

    @Test
    public void aChildThatHasHungIsKilledForThatEvenWhenNobodyIsWatchingEither() {
        assertEquals(Watchdog.Verdict.SILENT, at(new RunState.Running(at(1010), at(1010)), 2000));
    }

    @Test
    public void aChildToldToStopHasItsGraceToEndIn() {
        RunState stopping = new RunState.Stopping(at(1100));

        assertEquals(Watchdog.Verdict.WATCHING, at(stopping, 1102));
        assertEquals(Watchdog.Verdict.IGNORED_STOP, at(stopping, 1102.1));
    }

    @Test
    public void onceAChildIsToldToStopItsGraceIsTheOnlyClockOnIt() {
        Seconds aSecond = seconds(1);
        Watchdog.Waits shortOtherwise = new Watchdog.Waits(aSecond, aSecond, aSecond, KILL_GRACE, aSecond);

        assertEquals(
                "Stop was pressed, so what ends it is Stop or its grace, not a startup, silence or look",
                Watchdog.Verdict.WATCHING,
                Watchdog.verdict(new RunState.Stopping(at(1100)), at(1101.5), shortOtherwise));
    }

    @Test
    public void aMomentReadBeforeWhatItIsComparedWithIsNoTimeAtAll() {
        // The watchdog reads the run, then the clock; a line heard in between is heard "after now".
        assertEquals(Watchdog.Verdict.WATCHING, at(new RunState.Running(at(1010), at(1010)), 1009));
        assertEquals(Watchdog.Verdict.WATCHING, at(new RunState.Starting(LAUNCHED), 999));
        assertEquals(Watchdog.Verdict.WATCHING, at(new RunState.Stopping(at(1100)), 1099));
        assertEquals(Watchdog.Verdict.WATCHING, at(new RunState.Held(at(1000)), 999));
    }

    @Test
    public void aMomentIsAReadingOfAClockThatMayWrap() {
        assertEquals(1.5, at(2.5).secondsSince(at(1)), 1e-12);
        assertEquals(-1.5, at(1).secondsSince(at(2.5)), 1e-12);
        assertEquals(
                "System.nanoTime may wrap, and a difference across the wrap is still the time between",
                1e-9,
                new Moment(Long.MIN_VALUE).secondsSince(new Moment(Long.MAX_VALUE)),
                0);
    }
}
