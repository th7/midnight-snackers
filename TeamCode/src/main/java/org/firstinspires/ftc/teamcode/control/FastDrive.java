package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;

/**
 * Steers the robot toward a pose. Given where the robot is and where it is going, one call answers
 * everything about one loop of steering: the three powers to drive at, whether the robot has
 * arrived, and the error they were all read off.
 *
 * <p>One call, because it is one answer. The powers and "arrived" come off the same error, and a
 * caller that could ask for them one at a time had to remember to set the destination, then
 * update, then read -- an order nothing enforced -- and could ask "am I moving?" on its own, which
 * was a question with a wrong answer: the powers are signed, so a robot reversing at full power
 * compared less than the minimum power and read as at rest. Nothing can ask that now.
 *
 * <p>What the controller keeps between calls is what a PID must: its accumulated error and its
 * last measurement.
 */
public class FastDrive {
    /** What one loop of steering decided. Every field is read off the same {@link #error}. */
    public static final class Steering {
        /** Forward power, -1 to 1. */
        public final float straight;
        /** Leftward power, -1 to 1. */
        public final float strafe;
        /** Counterclockwise power, -1 to 1. */
        public final float turn;
        /** Whether the robot is within tolerance on all three axes, and so has arrived. */
        public final boolean arrived;
        /** Whether it is within tolerance along its own forward axis. */
        public final boolean nearStraight;
        /** Whether it is within tolerance sideways. */
        public final boolean nearStrafe;
        /** Whether it is within tolerance in heading. */
        public final boolean nearTurn;
        /** How far the destination is from here, in the destination's frame. */
        public final Pose2d error;

        Steering(
                float straight,
                float strafe,
                float turn,
                boolean nearStraight,
                boolean nearStrafe,
                boolean nearTurn,
                Pose2d error) {
            this.straight = straight;
            this.strafe = strafe;
            this.turn = turn;
            this.nearStraight = nearStraight;
            this.nearStrafe = nearStrafe;
            this.nearTurn = nearTurn;
            this.arrived = nearStraight && nearStrafe && nearTurn;
            this.error = error;
        }
    }

    private final double positionXCloseEnough = 0.7;
    private final double positionYCloseEnough = 0.5;
    private final double headingCloseEnoughRads = 0.017;
    private final float straightMinPower = 0.1f; // 0.05f
    private final float strafeMinPower = 0.1f; // 0.1f
    private final float turnMinPower = 0.1f; // 0.03f
    private final double positionP = 0.01;
    private final double positionI = 0;
    private final double positionD = 0.02;
    private final double headingP = 2;
    private final double headingI = 0;
    private final double headingD = 4;
    private final MiniPID xPID = new MiniPID(positionP, positionI, positionD);
    private final MiniPID yPID = new MiniPID(positionP * 2, positionI, positionD);
    private final MiniPID hPID = new MiniPID(headingP, headingI, headingD);

    /** What to drive at this loop to get from {@code currentPose} to {@code destination}. */
    public Steering steer(Pose2d currentPose, Pose2d destination) {
        Pose2d error = destination.minusExp(currentPose);

        boolean nearStraight = Math.abs(error.position.x) < positionXCloseEnough;
        float straightPower;
        if (nearStraight) {
            straightPower = 0;
        } else {
            double exaggeratedXError = error.position.x * Math.abs(error.position.x);
            double xOutput = xPID.getOutput(-exaggeratedXError, 0);
            straightPower = clamp(xOutput, straightMinPower, 1);
        }

        boolean nearStrafe = Math.abs(error.position.y) < positionYCloseEnough;
        float strafePower;
        if (nearStrafe) {
            strafePower = 0;
        } else {
            double exaggeratedYError = error.position.y * Math.abs(error.position.y);
            double yOutput = yPID.getOutput(-exaggeratedYError, 0);
            strafePower = clamp(yOutput, strafeMinPower, 1);
        }

        double headingErrorRads = error.heading.minus(Rotation2d.exp(0));
        boolean nearTurn = Math.abs(headingErrorRads) < headingCloseEnoughRads;
        float turnPower;
        if (nearTurn) {
            turnPower = 0;
        } else {
            double exaggeratedHError = headingErrorRads * Math.abs(headingErrorRads);
            double hOutput = hPID.getOutput(-exaggeratedHError, 0);
            turnPower = clamp(hOutput, turnMinPower, 1);
        }

        return new Steering(straightPower, strafePower, turnPower, nearStraight, nearStrafe, nearTurn, error);
    }

    private float clamp(double unclamped, float min, float max) {
        if (unclamped < 0) {
            return (float) Math.min(-min, Math.max(unclamped, -max));
        }
        if (unclamped > 0) {
            return (float) Math.max(min, Math.min(unclamped, max));
        }

        return 0f;
    }
}
