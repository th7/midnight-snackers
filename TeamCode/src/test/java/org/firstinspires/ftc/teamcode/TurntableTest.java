package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

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

    @Test
    public void aNudgeTakesTheTurntableByHandSoALaterAimDoesNotMoveIt() {
        turntable.nudgeLeft();

        turntable.aimAt(Math.PI / 2);
        motor.currentPosition = 10;
        turntable.loop();

        assertEquals("still on the nudge, inside its deadband", 0, motor.power, DELTA);
    }

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
