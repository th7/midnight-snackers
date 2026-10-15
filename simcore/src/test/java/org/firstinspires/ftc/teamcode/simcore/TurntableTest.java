package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TurntableTest {
    private static final double DELTA = 1e-9;
    private static final int TICKS_PER_REVOLUTION = 1700;
    private static final Turntable TURNTABLE = Valid.value(Turntable.of(TICKS_PER_REVOLUTION, Constants.defaults()));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    @Test
    public void theTurntableTurnsCounterClockwiseWithItsEncoderARevolutionForItsTicks() {
        assertEquals(2 * Math.PI, TURNTABLE.radians(TICKS_PER_REVOLUTION), DELTA);
        assertEquals(Math.PI / 2, TURNTABLE.radians(TICKS_PER_REVOLUTION / 4), DELTA);
        assertEquals(-Math.PI, TURNTABLE.radians(-TICKS_PER_REVOLUTION / 2), DELTA);
    }

    @Test
    public void theTurntableTurnsAtItsFullSpeedTimesItsPowerInWholeTicks() {
        assertEquals(
                100 + (int) Turntable.TICKS_PER_SECOND_AT_FULL_POWER,
                TURNTABLE.turned(100, Power.clamped(1), seconds(1)));
        assertEquals(100 - 850, TURNTABLE.turned(100, Power.clamped(-0.5), seconds(1)));
        assertEquals("4.25 ticks is 4", 104, TURNTABLE.turned(100, Power.clamped(0.5), seconds(0.005)));
        assertEquals(100, TURNTABLE.turned(100, Power.none(), seconds(1)));
    }

    @Test
    public void aTurntableHasTicksInARevolution() {
        assertTrue(Turntable.of(0, Constants.defaults()) instanceof Checked.Rejected);
        assertTrue(Turntable.of(-1700, Constants.defaults()) instanceof Checked.Rejected);
    }

    private static Constants constants(Constant constant, double value) {
        java.util.Map<Constant, Double> set = new java.util.EnumMap<>(Constant.class);
        set.put(constant, value);
        return Valid.value(Constants.of(set));
    }

    @Test
    public void theTurntableTurnsAtTheSpeedItIsGiven() {
        Turntable quicker = Valid.value(Turntable.of(TICKS_PER_REVOLUTION, constants(Constant.TURNTABLE_SPEED, 3400)));

        assertEquals(100 + 3400, quicker.turned(100, Power.clamped(1), seconds(1)));
        assertEquals(100 - 1700, quicker.turned(100, Power.clamped(-0.5), seconds(1)));
    }

    @Test
    public void theLaunchersFrameIsTheRobotsPlaceFacingItsHeadingTurnedByTheTurntable() {
        Launcher.Frame quarter = Valid.value(TURNTABLE.launcherFrame(new Vec2(10, -20), 0.5, TICKS_PER_REVOLUTION / 4));
        Launcher.Frame back = Valid.value(TURNTABLE.launcherFrame(new Vec2(10, -20), 0.5, -TICKS_PER_REVOLUTION / 2));

        assertEquals(Valid.value(Launcher.Frame.of(10, -20, 0.5 + Math.PI / 2)), quarter);
        assertEquals(Valid.value(Launcher.Frame.of(10, -20, 0.5 - Math.PI)), back);
    }

    @Test
    public void aRobotWhosePlaceOrHeadingNoFrameCouldHaveGivesNoLaunchersFrameAndTheRefusalNamesTheFramesRule() {
        String far = String.valueOf(TURNTABLE.launcherFrame(new Vec2(20000, 0), 0, 0));
        String turning = String.valueOf(TURNTABLE.launcherFrame(new Vec2(10, -20), Double.NaN, 0));

        assertTrue(far, far.contains("a launcher's frame is within"));
        assertTrue(turning, turning.contains("a launcher's frame faces a finite yaw"));
    }
}
