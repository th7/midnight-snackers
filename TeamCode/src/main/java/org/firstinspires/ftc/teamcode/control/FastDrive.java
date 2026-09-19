package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;

public class FastDrive {
    public static final class Steering {
        public final float straight;

        public final float strafe;

        public final float turn;

        public final boolean arrived;

        public final boolean nearStraight;

        public final boolean nearStrafe;

        public final boolean nearTurn;

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

    private final double positionXCloseEnoughInches = 0.7;
    private final double positionYCloseEnoughInches = 0.5;
    private final double headingCloseEnoughRads = 0.017;
    private final float straightMinPower = 0.1f;
    private final float strafeMinPower = 0.1f;
    private final float turnMinPower = 0.1f;
    private final double positionP = 0.01;
    private final double positionI = 0;
    private final double positionD = 0.02;
    private final double headingP = 2;
    private final double headingI = 0;
    private final double headingD = 4;
    private final MiniPID xPID = new MiniPID(positionP, positionI, positionD);
    private final MiniPID yPID = new MiniPID(positionP * 2, positionI, positionD);
    private final MiniPID hPID = new MiniPID(headingP, headingI, headingD);

    public Steering steer(Pose2d currentPose, Pose2d destination) {
        Pose2d error = destination.minusExp(currentPose);

        boolean nearStraight = Math.abs(error.position.x) < positionXCloseEnoughInches;
        float straightPower;
        if (nearStraight) {
            straightPower = 0;
        } else {
            double exaggeratedXError = error.position.x * Math.abs(error.position.x);
            double xOutput = xPID.getOutput(-exaggeratedXError, 0);
            straightPower = clamp(xOutput, straightMinPower, 1);
        }

        boolean nearStrafe = Math.abs(error.position.y) < positionYCloseEnoughInches;
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
