package org.firstinspires.ftc.teamcode.base;

import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

public class FastDrive {
//    private Pose2d currentPose;
//    private long currentPoseAt;
//    private Pose2d lastPose;
//    private long lastPoseAt;
    private Pose2d destination;
//
//    private Twist2d positionDelta;
//    private Twist2d velocity;
//    private MoveData moveData;
    private float straightPower;
    private float strafePower;
    private float turnPower;
    public Twist2d poseError;
    public Pose2d error;

    private final double positionXCloseEnough = 0.7;
    private final double positionYCloseEnough = 0.5;
    private final double headingCloseEnoughRads = 0.017;

    private final float straightMinPower = 0.1f; // 0.05f
    private final float strafeMinPower = 0.1f; // 0.1f
    private final float turnMinPower = 0.1f; // 0.03f
    public double positionP = 0.01;
    public double positionI = 0;
    public double positionD = 0.02;
    public double headingP = 2;
    public double headingI = 0;
    public double headingD = 4;

    private MiniPID xPID = new MiniPID(positionP, positionI, positionD);
    private MiniPID yPID = new MiniPID(positionP * 2, positionI, positionD);
    private MiniPID hPID = new MiniPID(headingP, headingI, headingD);


    public void update(Pose2d currentPose) {
        this.poseError = destination.minus(currentPose);
        this.error = destination.minusExp(currentPose);





//        double xOutput = xPID.getOutput(-exaggeratedXError, 0);
//        straightPower = clamp(xOutput, straightMinPower, 1);
//        double yOutput = yPID.getOutput(-exaggeratedYError, 0);
//        strafePower = clamp(yOutput, strafeMinPower, 1);
//        double hOutput = hPID.getOutput(-exaggeratedHError, 0);
//        turnPower = clamp(hOutput, turnMinPower, 1);

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

    public void setDestination(Vector2d position, double headingRadians) {
        this.destination = new Pose2d(position, headingRadians);
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
        if (error == null) { return false; }

        return nearDestination() && notMoving();
    }

    public boolean nearDestination() {
        if (error == null) { return false; }
        return nearXDestination() && nearYDestination() && nearHDestination();
    }

    public boolean nearXDestination() {
        if (error == null) { return false; }
        return Math.abs(error.position.x) < positionXCloseEnough;
    }

    public boolean nearYDestination() {
        if (error == null) { return false; }
        return Math.abs(error.position.y) < positionYCloseEnough;
    }

    public boolean nearHDestination() {
        if (error == null) { return false; }

        double headingError = error.heading.minus(Rotation2d.exp(0));
        return Math.abs(headingError) < headingCloseEnoughRads;
    }

    public boolean notMoving() {
        return straightPower < straightMinPower && strafePower < strafeMinPower && turnPower < turnMinPower;
    }

    public void increasePositionP() {
        positionP = positionP + 0.01;
        xPID = new MiniPID(positionP, positionI, positionD);
        yPID = new MiniPID(positionP * 2, positionI, positionD);
    }

    public void decreasePositionP() {
        positionP = positionP - 0.01;
        xPID = new MiniPID(positionP, positionI, positionD);
        yPID = new MiniPID(positionP * 2, positionI, positionD);
    }

    public void increasePositionI() {
        positionI = positionI + 0.001;
        xPID = new MiniPID(positionP, positionI, positionD);
        yPID = new MiniPID(positionP * 2, positionI, positionD);
    }

    public void decreasePositionI() {
        positionI = positionI - 0.001;
        xPID = new MiniPID(positionP, positionI, positionD);
        yPID = new MiniPID(positionP * 2, positionI, positionD);
    }

    public void increasePositionD() {
        positionD = positionD + 0.01;
        xPID = new MiniPID(positionP, positionI, positionD);
        yPID = new MiniPID(positionP * 2, positionI, positionD);
    }

    public void decreasePositionD() {
        positionD = positionD - 0.01;
        xPID = new MiniPID(positionP, positionI, positionD);
        yPID = new MiniPID(positionP * 2, positionI, positionD);
    }

    public void increaseHeadingP() {
        headingP = headingP + 0.1;
        hPID = new MiniPID(headingP, headingI, headingD);
    }

    public void decreaseHeadingP() {
        headingP = headingP - 0.1;
        hPID = new MiniPID(headingP, headingI, headingD);
    }

    public void increaseHeadingI() {
        headingI = headingI + 0.01;
        hPID = new MiniPID(headingP, headingI, headingD);
    }

    public void decreaseHeadingI() {
        headingI = headingI - 0.01;
        hPID = new MiniPID(headingP, headingI, headingD);
    }

    public void increaseHeadingD() {
        headingD = headingD + 0.1;
        hPID = new MiniPID(headingP, headingI, headingD);
    }

    public void decreaseHeadingD() {
        headingD = headingD - 0.1;
        hPID = new MiniPID(headingP, headingI, headingD);
    }


    private float clamp(double unclamped, float min, float max) {
        if (unclamped < 0) {
            return (float) MathUtils.clamp(unclamped, -max, -min);
        }
        if (unclamped > 0) {
            return (float) MathUtils.clamp(unclamped, min, max);
        }

        return 0f;
    }


//    private double speed() {
//        return delta().line.norm();
//    }
}
