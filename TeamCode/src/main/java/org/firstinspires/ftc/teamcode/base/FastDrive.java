package org.firstinspires.ftc.teamcode.base;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;

public class FastDrive {
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
    private Pose2d error;
    private Pose2d destination;
    private float straightPower;
    private float strafePower;
    private float turnPower;

    public void update(Pose2d currentPose) {
        this.error = destination.minusExp(currentPose);

        if (Math.abs(error.position.x) < positionXCloseEnough) {
            straightPower = 0;
        } else {
            double exaggeratedXError = error.position.x * Math.abs(error.position.x);
            double xOutput = xPID.getOutput(-exaggeratedXError, 0);
            straightPower = clamp(xOutput, straightMinPower, 1);
        }

        if (Math.abs(error.position.y) < positionYCloseEnough) {
            strafePower = 0;
        } else {
            double exaggeratedYError = error.position.y * Math.abs(error.position.y);
            double yOutput = yPID.getOutput(-exaggeratedYError, 0);
            strafePower = clamp(yOutput, strafeMinPower, 1);
        }

        double headingErrorRads = error.heading.minus(Rotation2d.exp(0));
        if (Math.abs(headingErrorRads) < headingCloseEnoughRads) {
            turnPower = 0;
        } else {
            double exaggeratedHError = headingErrorRads * Math.abs(headingErrorRads);
            double hOutput = hPID.getOutput(-exaggeratedHError, 0);
            turnPower = clamp(hOutput, turnMinPower, 1);
        }
    }

    public void setDestination(Pose2d newDestination) {
        this.destination = newDestination;
    }

    public Pose2d error() {
        return error;
    }

    public float straightPower() {
        return straightPower;
    }

    public float strafePower() {
        return strafePower;
    }

    public float turnPower() {
        return turnPower;
    }

    public boolean doneMoving() {
        if (error == null) {
            return false;
        }

        return nearDestination() && notMoving();
    }

    public boolean nearDestination() {
        if (error == null) {
            return false;
        }
        return nearXDestination() && nearYDestination() && nearHDestination();
    }

    public boolean nearXDestination() {
        if (error == null) {
            return false;
        }
        return Math.abs(error.position.x) < positionXCloseEnough;
    }

    public boolean nearYDestination() {
        if (error == null) {
            return false;
        }
        return Math.abs(error.position.y) < positionYCloseEnough;
    }

    public boolean nearHDestination() {
        if (error == null) {
            return false;
        }

        double headingError = error.heading.minus(Rotation2d.exp(0));
        return Math.abs(headingError) < headingCloseEnoughRads;
    }

    public boolean notMoving() {
        return straightPower < straightMinPower && strafePower < strafeMinPower && turnPower < turnMinPower;
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
