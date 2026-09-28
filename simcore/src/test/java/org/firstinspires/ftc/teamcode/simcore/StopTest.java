package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class StopTest {
    private static final Moment NOW = new Moment(0);

    @Test
    public void stopEndsARunStillBuildingWhereItIsSinceThereIsNoChildToTell() {
        assertEquals(Stop.END_UNBUILT, new RunState.Building().onStop());
    }

    @Test
    public void stopIsToldToAChildWhetherItsOpModeHasStartedOrNot() {
        assertEquals(Stop.TELL, new RunState.Starting(NOW).onStop());
        assertEquals(Stop.TELL, new RunState.Running(NOW, NOW).onStop());
    }

    @Test
    public void aChildIsToldToStopOnceAndItsGraceRunsFromThen() {
        assertEquals(Stop.NOTHING, new RunState.Stopping(NOW).onStop());
    }

    @Test
    public void stopDoesNothingToARunThatIsOver() {
        assertEquals(Stop.NOTHING, new RunState.Over().onStop());
    }

    @Test
    public void onlyARunStillBuildingTakesTheChildBuiltForIt() {
        assertTrue(new RunState.Building().takesAChild());
        assertFalse("Stop ended it while it built", new RunState.Over().takesAChild());
        assertFalse("it has one", new RunState.Starting(NOW).takesAChild());
        assertFalse("it has one", new RunState.Running(NOW, NOW).takesAChild());
        assertFalse("it has one", new RunState.Stopping(NOW).takesAChild());
    }
}
