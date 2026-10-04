package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class LaunchTest {
    private static final double DELTA = 1e-9;
    private static final Vec2 STILL = new Vec2(0, 0);
    private static final Vec2 AT = new Vec2(10, 20);
    private static final int TICKS_PER_REVOLUTION = 1700;
    private static final Turntable TURNTABLE = Valid.value(Turntable.of(TICKS_PER_REVOLUTION, Constants.defaults()));

    private static Seconds seconds(double value) {
        return Valid.value(Seconds.of(value));
    }

    private static void assertVec(Vec3 expected, Vec3 actual) {
        assertEquals("x of " + actual, expected.x(), actual.x(), DELTA);
        assertEquals("y of " + actual, expected.y(), actual.y(), DELTA);
        assertEquals("z of " + actual, expected.z(), actual.z(), DELTA);
    }

    @Test
    public void aBallLeavesAheadOfTheRobotAtTheLaunchHeight() {
        Launch launch = Launch.from(AT, 0, 0, 1000, STILL, Constants.defaults());

        assertVec(new Vec3(10 + Launch.AHEAD_IN, 20, Launch.HEIGHT_IN), launch.at());
    }

    @Test
    public void itLeavesAtTheLaunchAngleAtASpeedSetByTheFlywheels() {
        Launch launch = Launch.from(AT, 0, 0, 1000, STILL, Constants.defaults());

        double speed = 1000 * Launch.IN_PER_S_PER_TICK_PER_S;
        assertVec(
                new Vec3(speed * Math.cos(Launch.ANGLE_RADIANS), 0, speed * Math.sin(Launch.ANGLE_RADIANS)),
                launch.velocity());
        assertEquals(speed, launch.velocity().length(), DELTA);
        assertEquals(Math.toRadians(70), Launch.ANGLE_RADIANS, 0);
    }

    @Test
    public void aFlywheelTurningBackwardsThrowsAsHard() {
        assertVec(
                Launch.from(AT, 0, 0, 1000, STILL, Constants.defaults()).velocity(),
                Launch.from(AT, 0, 0, -1000, STILL, Constants.defaults()).velocity());
    }

    @Test
    public void theTurntableAimsItFromTheRobotsHeading() {
        Launch launch = Launch.from(AT, Math.PI / 4, Math.PI / 4, 1000, STILL, Constants.defaults());

        assertVec(new Vec3(10, 20 + Launch.AHEAD_IN, Launch.HEIGHT_IN), launch.at());
        assertEquals(0, launch.velocity().x(), DELTA);
        assertTrue(launch.velocity().y() > 0);
    }

    @Test
    public void theRobotsOwnMotionCarriesTheBall() {
        Vec3 still =
                Launch.from(AT, 0.3, 0.2, 1000, STILL, Constants.defaults()).velocity();
        Vec3 moving = Launch.from(AT, 0.3, 0.2, 1000, new Vec2(5, -3), Constants.defaults())
                .velocity();

        assertVec(still.plus(new Vec3(5, -3, 0)), moving);
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
    public void aBallLeavesAtTheSpeedTheLauncherThrowGivesTheFlywheel() {
        Launch harder = Launch.from(AT, 0.3, 0.2, 1000, STILL, constants(Constant.LAUNCH_THROW, 0.3));

        assertEquals(1000 * 0.3, harder.velocity().length(), DELTA);
        assertVec(
                Launch.from(AT, 0.3, 0.2, 1000, STILL, Constants.defaults())
                        .velocity()
                        .times(0.3 / Launch.IN_PER_S_PER_TICK_PER_S),
                harder.velocity());
    }

    @Test
    public void theTurntableTurnsAtTheSpeedItIsGiven() {
        Turntable quicker = Valid.value(Turntable.of(TICKS_PER_REVOLUTION, constants(Constant.TURNTABLE_SPEED, 3400)));

        assertEquals(100 + 3400, quicker.turned(100, Power.clamped(1), seconds(1)));
        assertEquals(100 - 1700, quicker.turned(100, Power.clamped(-0.5), seconds(1)));
    }
}
