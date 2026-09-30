package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ViewTest {
    private static final Seconds WAIT = Valid.value(Seconds.of(30));

    private static Moment at(double seconds) {
        return new Moment((long) (seconds * 1e9));
    }

    @Test
    public void aRunNobodyAskedToWaitIsPlacedAsSoonAsItsChildCanBe() {
        assertFalse(new View.NotAwaited().holdsTheStart(at(0), WAIT));
    }

    @Test
    public void aRunWaitingForItsViewHoldsItsStartForTheViewWaitAndNoLonger() {
        View awaited = new View.Awaited(at(100));

        assertTrue(awaited.holdsTheStart(at(100), WAIT));
        assertTrue(awaited.holdsTheStart(at(130), WAIT));
        assertFalse(awaited.holdsTheStart(at(130.1), WAIT));
    }

    @Test
    public void aViewThatIsReadyHoldsNothing() {
        assertFalse(new View.Ready().holdsTheStart(at(100), WAIT));
    }

    @Test
    public void onlyAViewStillAwaitedIsWaitedFor() {
        assertTrue(new View.Awaited(at(100)).waitedFor());
        assertFalse(new View.NotAwaited().waitedFor());
        assertFalse(new View.Ready().waitedFor());
    }

    @Test
    public void aMomentReadBeforeTheWaitBeganIsNoTimeWaited() {
        assertTrue(new View.Awaited(at(100)).holdsTheStart(at(99), WAIT));
    }
}
