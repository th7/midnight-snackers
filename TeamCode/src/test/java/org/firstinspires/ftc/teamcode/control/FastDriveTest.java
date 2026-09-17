package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.control.FastDrive.Steering;
import org.junit.Test;

/**
 * One call says everything about one loop of steering: the three powers, whether the robot has
 * arrived, and how far off it is.
 */
public class FastDriveTest {
    private static final float DELTA = 0.0001f;

    private final FastDrive fastDrive = new FastDrive();

    private Steering steerFrom(double x, double y, double heading, Pose2d destination) {
        return fastDrive.steer(new Pose2d(x, y, heading), destination);
    }

    @Test
    public void atTheDestinationAllPowersAreZeroAndItHasArrived() {
        Steering steering = steerFrom(0, 0, 0, new Pose2d(0, 0, 0));

        assertEquals(0f, steering.straight, DELTA);
        assertEquals(0f, steering.strafe, DELTA);
        assertEquals(0f, steering.turn, DELTA);
        assertTrue(steering.arrived);
    }

    @Test
    public void farAheadDrivesStraightAtFullPower() {
        Steering steering = steerFrom(0, 0, 0, new Pose2d(48, 0, 0));

        assertEquals(1f, steering.straight, DELTA);
        assertEquals(0f, steering.strafe, DELTA);
        assertEquals(0f, steering.turn, DELTA);
        assertFalse(steering.arrived);
        assertFalse(steering.nearStraight);
    }

    /**
     * Steering hard in reverse is not arriving. The powers are signed, so a test of "moving" that
     * compares them to a floor rather than their size answers that a robot reversing at full power
     * is at rest.
     */
    @Test
    public void farBehindReversesAtFullPowerAndHasNotArrived() {
        Steering steering = steerFrom(0, 0, 0, new Pose2d(-48, 0, 0));

        assertEquals(-1f, steering.straight, DELTA);
        assertFalse(steering.arrived);
        assertFalse(steering.nearStraight);
    }

    @Test
    public void headingErrorTurnsWithoutTranslating() {
        Steering steering = steerFrom(0, 0, 0, new Pose2d(0, 0, Math.PI / 2));

        assertEquals(0f, steering.straight, DELTA);
        assertEquals(0f, steering.strafe, DELTA);
        assertEquals(1f, Math.abs(steering.turn), DELTA);
        assertFalse(steering.nearTurn);
        assertFalse(steering.arrived);
    }

    @Test
    public void withinTolerancesCountsAsArrived() {
        Steering steering = steerFrom(0, 0, 0, new Pose2d(0.5, 0.3, 0.01));

        assertTrue(steering.nearStraight);
        assertTrue(steering.nearStrafe);
        assertTrue(steering.nearTurn);
        assertTrue(steering.arrived);
    }

    /** The error is what the powers were read off, so telemetry and the powers cannot disagree. */
    @Test
    public void theErrorIsTheOneThesePowersWereReadOff() {
        Steering steering = steerFrom(10, 0, 0, new Pose2d(34, 0, 0));

        assertEquals(24, steering.error.position.x, DELTA);
        assertEquals(0, steering.error.position.y, DELTA);
    }
}
