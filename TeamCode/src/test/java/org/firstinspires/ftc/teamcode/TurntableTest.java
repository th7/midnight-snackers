package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

/**
 * The turntable is told where to point and decides for itself whether to listen. What it is
 * following -- the goal, straight ahead, or the driver's own hand -- is its own business, so
 * nobody has to change a mode in one module to make a call on another one stick.
 */
public class TurntableTest {
    private static final double DELTA = 0.0001;

    private final SimDevices devices = new SimDevices();
    private final Robot robot = new Robot(devices.hardware(), Alliance.RELATIVE, new FakeTelemetry());
    private final FakeDcMotorEx motor = devices.turnTable;
    private final Turntable turntable = robot.turntable;

    @Test
    public void followsTheGoalItIsAimedAtFromTheStart() {
        turntable.aimAt(Math.PI / 2);

        turntable.loop();

        assertTrue(motor.power > 0);
    }

    @Test
    public void stopsInsideTheDeadband() {
        turntable.aimAt(0);
        motor.currentPosition = 3;

        turntable.loop();

        assertEquals(0, motor.power, DELTA);
    }

    @Test
    public void weakCommandsAreRaisedToTheMinimumPower() {
        turntable.aimAt(0);
        motor.currentPosition = -8;

        turntable.loop();

        assertEquals(0.1, motor.power, DELTA);
    }

    @Test
    public void wrapsTargetsPastHalfARevolutionToTurnTheShortWay() {
        turntable.aimAt(3 * Math.PI / 2);

        turntable.loop();

        assertTrue(motor.power < 0);
    }

    @Test
    public void reportsTheOffsetInRadians() {
        motor.currentPosition = 1700;

        assertEquals(2 * Math.PI, turntable.offsetRadians(), DELTA);
    }

    /**
     * The ordering constraint this replaced: a nudge used to add ten ticks to the same field the
     * brain overwrote on its next tick, so it survived only because the line above it in Driver
     * flipped a boolean inside a third module. Deleting that line still compiled, still ran, and
     * the nudge was gone twenty milliseconds later.
     */
    @Test
    public void aNudgeTakesTheTurntableByHandSoALaterAimDoesNotMoveIt() {
        turntable.nudgeLeft();

        turntable.aimAt(Math.PI / 2);
        motor.currentPosition = 10;
        turntable.loop();

        assertEquals("still on the nudge, inside its deadband", 0, motor.power, DELTA);
    }

    /** And the whole robot ticking, brain included, does not take it back either. */
    @Test
    public void aNudgeSurvivesTheBrainsNextTick() {
        turntable.nudgeRight();

        robot.loop();
        motor.currentPosition = -10;
        turntable.loop();

        assertEquals(0, motor.power, DELTA);
    }

    @Test
    public void nudgesStepTenTicksEachWayAndAccumulate() {
        turntable.nudgeLeft();
        turntable.nudgeLeft();

        motor.currentPosition = 20;
        turntable.loop();

        assertEquals(0, motor.power, DELTA);
    }

    @Test
    public void parkingStraightAheadIgnoresTheAimUntilItIsToldToFollowAgain() {
        turntable.parkStraightAhead();

        turntable.aimAt(Math.PI / 2);
        turntable.loop();

        assertEquals("parked at zero, inside the deadband", 0, motor.power, DELTA);
    }

    @Test
    public void followingTheGoalAgainResumesTheAim() {
        turntable.parkStraightAhead();
        turntable.aimAt(Math.PI / 2);

        turntable.followTheGoal();
        turntable.aimAt(Math.PI / 2);
        turntable.loop();

        assertTrue(motor.power > 0);
    }

    /** A nudge is the driver taking it over, so following the goal again is what gives it back. */
    @Test
    public void followingTheGoalAgainTakesItBackFromTheDriversHand() {
        turntable.nudgeLeft();
        turntable.aimAt(Math.PI / 2);

        turntable.followTheGoal();
        turntable.aimAt(Math.PI / 2);
        turntable.loop();

        assertTrue(motor.power > 0);
    }
}
