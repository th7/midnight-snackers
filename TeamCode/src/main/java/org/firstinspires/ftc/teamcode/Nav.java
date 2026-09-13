package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * 2025-2026 Season
 * Treat all coordinates and heading as if you are playing as blue.
 * If you're red, they will automatically be adjusted.
 * Positive y coordinates will move toward your goal.
 * Positive headings will turn toward your goal.
 */
public class Nav extends SubSystem {
    // COORDINATES!!! ARGH
    // Imagine facing the field from the audience. Blue goal is forward left, red goal is forward right. Any further mention of left/right or forward/backward is relative to this perspective.

    // FTC Coordinates
    // +x backward, +y right, straight forward heading is PI/2

    // Roadrunner Coordinates (this is what we use)
    // +x forward, +y left, straight forward heading is 0
    private final int headingSign;
    private final int ySign;
    private final Vector2d launchTarget;
    private final double targetLaunchDistance = 40;
    private final MecanumDrive mecanumDrive;
    private boolean fieldPositionKnown = false;

    public Nav(MecanumDrive mecanumDrive, Alliance alliance) {
        this.mecanumDrive = mecanumDrive;
        this.headingSign = alliance.headingSign;
        this.ySign = alliance.ySign;
        this.launchTarget = alliance.launchTarget;
    }

    @Override
    protected void onLoop() {
        mecanumDrive.localizer.update();
    }

    public double relativeHeadingToTarget() {
        if (!fieldPositionKnown) {
            return 0;
        }
        Pose2d launchPose = launchPose().pose2d;
        double launchPoseHeadingRads = launchPose.heading.minus(Rotation2d.exp(0));
        double currentPoseHeadingRads = getPose().heading.minus(Rotation2d.exp(0));
        double headingRads = -(launchPoseHeadingRads - currentPoseHeadingRads);

        return headingRads;
    }

    private double angleRadians(Vector2d from, Vector2d to) {
        return Math.atan2(to.y - from.y, to.x - from.x);
    }

    private double distanceInches(Vector2d from, Vector2d to) {
        return Math.sqrt(Math.pow(to.y - from.y, 2) + Math.pow(to.x - from.x, 2));
    }

    private Vector2d pointAtDistanceInDirection(Vector2d from, double distance, double directionRadians) {
        double newX = from.x + Math.cos(directionRadians) * distance;
        double newY = from.y + Math.sin(directionRadians) * distance;
        return new Vector2d(newX, newY);
    }

    public Pose launchPose() {
        Vector2d from = getPose().position;
        double bearingToTarget = angleRadians(from, launchTarget);
        double distanceToTarget = distanceInches(from, launchTarget);
        double distanceError = distanceToTarget - targetLaunchDistance;
        Vector2d position = pointAtDistanceInDirection(from, distanceError, bearingToTarget);
        return new Pose(new Pose2d(position, bearingToTarget));
    }

    public Pose pose(double x, double y, double heading) {
        return new Pose(new Pose2d(x, y * this.ySign, heading * this.headingSign));
    }

    public Pose currentPose() {
        return new Pose(getPose());
    }

    private Pose2d getPose() {
        return mecanumDrive.localizer.getPose();
    }

    public void setPose(Pose2d pose) {
        mecanumDrive.localizer.setPose(pose);
    }

    public void setPose(Pose pose) {
        setPose(pose.pose2d);
    }

    public void setFieldPosition(Pose2d pose) {
        if (!fieldPositionKnown) {
            fieldPositionKnown = true;
            setPose(pose);
            return;
        }

        Pose2d currentPose = getPose();

        double xError = pose.position.x - currentPose.position.x;
        double yError = pose.position.y - currentPose.position.y;

        if (xError > 1) {
            xError = 1;
        } else if (xError < -1) {
            xError = -1;
        }

        if (yError > 1) {
            yError = 1;
        } else if (yError < -1) {
            yError = -1;
        }

        Vector2d adjustedPosition = new Vector2d(currentPose.position.x + xError, currentPose.position.y + yError);
        setPose(new Pose2d(adjustedPosition, currentPose.heading));
    }

    public Action backwardPath(Pose... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose());
        for (Pose pose : poseList) {
            builder = builder.setReversed(true).splineToSplineHeading(pose.pose2d, Math.PI);
        }
        return builder.build();
    }

    public Action backwardTo(double x, double y, double heading) {
        Pose pose = pose(x, y, heading);
        return backwardPath(pose);
    }

    public Action strafePath(Pose... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose());
        for (Nav.Pose pose : poseList) {
            builder = builder.strafeToSplineHeading(pose.pose2d.position, pose.pose2d.heading);
        }
        return builder.build();
    }

    public Action strafeTo(double x, double y, double heading) {
        Pose pose = pose(x, y, heading);
        return strafePath(pose);
    }

    public boolean closeTo(double x, double y, double heading) {
        Pose targetPose = pose(x, y, heading);
        double headinglimit = Math.PI * 2 / 60;
        Pose2d currentPose = mecanumDrive.localizer.getPose();
        double headingerror = currentPose.heading.minus(targetPose.pose2d.heading);
        if (Math.abs(headingerror) > headinglimit) {
            return false;
        }

        double xError = currentPose.position.x - targetPose.x();
        if (Math.abs(xError) > 3) {
            return false;
        }

        double yError = currentPose.position.y - targetPose.pose2d.position.y;
        if (Math.abs(yError) > 3) {
            return false;
        }

        return true;
    }

    public static class Pose {
        public final Pose2d pose2d;

        /**
         * Represents a position and heading. Call .pose2d to use with RoadRunner.
         */
        public Pose(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public double x() { return pose2d.position.x; }
    }
}
