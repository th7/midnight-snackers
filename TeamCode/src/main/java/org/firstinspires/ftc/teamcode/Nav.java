package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Use Nav.blue, Nav.red, or Nav.relative to get an instance of this class.
 * <p>
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
    private static final double blueAprilTagX = 58.3;
    private static final double blueAprilTagY = 55.6;
    private static final double blueLaunchTargetX = blueAprilTagX + 9;
    private static final double blueLaunchTargetY = blueAprilTagY + 9;
    private static final Vector2d blueLaunchTarget = new Vector2d(blueLaunchTargetX, blueLaunchTargetY);
    private static final Vector2d redLaunchTarget = new Vector2d(blueLaunchTargetX, -blueLaunchTargetY);

    private final int headingSign;
    private final int ySign;
    private final Vector2d launchTarget;
    private final double targetLaunchDistance = 40;
    private MecanumDrive mecanumDrive;
    private boolean fieldPositionKnown = false;

    private Nav(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry, int headingSign, int ySign, Vector2d launchTarget) {
        super(hardwareMap, runtime, telemetry);
        this.headingSign = headingSign;
        this.ySign = ySign;
        this.launchTarget = launchTarget;
    }

    public static Nav blue(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        return new Nav(hardwareMap, runtime, telemetry, 1, 1, blueLaunchTarget);
    }

    public static Nav red(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        return new Nav(hardwareMap, runtime, telemetry, -1, -1, redLaunchTarget);
    }

    public static Nav relative(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        return new Nav(hardwareMap, runtime, telemetry, 1, 1, null);
    }

    @Override
    public void init() {
        this.mecanumDrive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
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

        return headingRads;//launchPose.minus().heading.toDouble().minus(currentPose.heading);
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

    public void setZeroPosition() {
        setPose(new Pose2d(0, 0, 0));
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

    public Action forwardPath(Pose... poseList) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose());
        for (Nav.Pose pose : poseList) {
            builder = builder.splineToSplineHeading(pose.pose2d, 0);
        }
        return builder.build();
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

    /**
     * Builds one continuous spline trajectory through every pose. Consecutive spline
     * segments stay velocity-continuous, so the robot rounds each waypoint instead of
     * stopping at it. Heading is held constant (constant-heading splines); each
     * waypoint's path tangent points toward the next waypoint.
     */
    public Action smoothPath(Pose... poseList) {
        Pose2d startPose = mecanumDrive.localizer.getPose();
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(startPose);
        if (poseList.length > 0) {
            // The robot's heading need not point at the first waypoint (e.g. it starts
            // facing the top vertex while the first move is behind-left). Aim the initial
            // path tangent at the first waypoint so the opening spline is a valid forward
            // curve instead of a degenerate one the follower can't leave.
            Vector2d first = poseList[0].pose2d.position;
            double startTangent = Math.atan2(
                    first.y - startPose.position.y, first.x - startPose.position.x);
            builder = builder.setTangent(startTangent);
        }
        for (int i = 0; i < poseList.length; i++) {
            Vector2d position = poseList[i].pose2d.position;
            double tangent;
            if (i + 1 < poseList.length) {
                Vector2d next = poseList[i + 1].pose2d.position;
                tangent = Math.atan2(next.y - position.y, next.x - position.x);
            } else if (i > 0) {
                Vector2d prev = poseList[i - 1].pose2d.position;
                tangent = Math.atan2(position.y - prev.y, position.x - prev.x);
            } else {
                tangent = poseList[i].pose2d.heading.toDouble();
            }
            builder = builder.splineToConstantHeading(position, tangent);
        }
        return builder.build();
    }

    /**
     * Like {@link #smoothPath} but the robot drives nose-first: instead of holding a
     * constant heading, it rotates to face its direction of travel and turns and
     * translates at the same time. Each waypoint's target heading is the direction
     * travelled to reach it, so the robot arrives driving forward into the point
     * (including turning toward the first point on the way there). The path tangent
     * aims at the next waypoint to keep the curve smooth and continuous.
     */
    public Action smoothForwardPath(Pose... poseList) {
        Pose2d startPose = mecanumDrive.localizer.getPose();
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(startPose);
        Vector2d previous = startPose.position;
        for (int i = 0; i < poseList.length; i++) {
            Vector2d position = poseList[i].pose2d.position;
            // Face the direction we travelled to get here (drive forward into the point).
            double heading = Math.atan2(position.y - previous.y, position.x - previous.x);
            // Aim the path tangent at the next point so the corner rounds into the next
            // leg; on the last point just keep the arrival direction.
            double tangent;
            if (i + 1 < poseList.length) {
                Vector2d next = poseList[i + 1].pose2d.position;
                tangent = Math.atan2(next.y - position.y, next.x - position.x);
            } else {
                tangent = heading;
            }
            if (i == 0) {
                // The robot may not already face the first point; leave along the travel
                // direction so the opening spline is a valid forward curve.
                builder = builder.setTangent(heading);
            }
            builder = builder.splineToLinearHeading(new Pose2d(position, heading), tangent);
            previous = position;
        }
        return builder.build();
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
