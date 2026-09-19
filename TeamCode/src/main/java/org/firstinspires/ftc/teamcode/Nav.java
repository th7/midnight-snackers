package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;

public class Nav implements Loopable {
    public static class Pose {
        public final Pose2d pose2d;

        public Pose(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public double xInches() {
            return pose2d.position.x;
        }

        public double yInches() {
            return pose2d.position.y;
        }

        public double headingRadians() {
            return pose2d.heading.toDouble();
        }

        public Pose rotated(double radians) {
            return new Pose(new Pose2d(pose2d.position, pose2d.heading.plus(radians)));
        }

        @Override
        public String toString() {
            return String.format("(%.1f, %.1f, %.2f)", xInches(), yInches(), headingRadians());
        }
    }

    public static final double LAUNCH_DISTANCE_INCHES = 40;

    private static final double NEAR_INCHES = 3;
    private static final double NEAR_RADIANS = Math.PI * 2 / 60;

    private static final double SIGHTING_NUDGE_INCHES = 1;

    private final int headingSign;
    private final int ySign;

    private final Vector2d launchTarget;

    private final Localizer localizer;

    private boolean onTheField = false;

    public Nav(Localizer localizer, Alliance alliance) {
        this.localizer = localizer;
        this.headingSign = alliance.headingSign;
        this.ySign = alliance.ySign;
        this.launchTarget = alliance.launchTarget;
    }

    public Pose pose(double x, double y, double heading) {
        return new Pose(new Pose2d(x, y * this.ySign, heading * this.headingSign));
    }

    public Pose currentPose() {
        return new Pose(getPose());
    }

    public void placeAt(Pose pose) {
        onTheField = true;
        localizer.setPose(pose.pose2d);
    }

    public Optional<Pose> launchPose() {
        if (launchTarget == null) {
            return Optional.empty();
        }
        Vector2d from = getPose().position;
        double bearingToTarget = angleRadians(from, launchTarget);
        double distanceToTarget = distanceInches(from, launchTarget);
        double distanceError = distanceToTarget - LAUNCH_DISTANCE_INCHES;
        Vector2d position = pointAtDistanceInDirection(from, distanceError, bearingToTarget);
        return Optional.of(new Pose(new Pose2d(position, bearingToTarget)));
    }

    public double relativeHeadingToTarget() {
        if (!onTheField) {
            return 0;
        }
        Optional<Pose> launchPose = launchPose();
        if (launchPose.isEmpty()) {
            return 0;
        }
        double launchPoseHeadingRads = launchPose.get().pose2d.heading.minus(Rotation2d.exp(0));
        double currentPoseHeadingRads = getPose().heading.minus(Rotation2d.exp(0));
        return -(launchPoseHeadingRads - currentPoseHeadingRads);
    }

    public void sighted(Pose sighting) {
        if (!onTheField) {
            placeAt(sighting);
            return;
        }

        Pose2d currentPose = getPose();
        double xError = clamp(sighting.xInches() - currentPose.position.x, SIGHTING_NUDGE_INCHES);
        double yError = clamp(sighting.yInches() - currentPose.position.y, SIGHTING_NUDGE_INCHES);
        Vector2d adjustedPosition = new Vector2d(currentPose.position.x + xError, currentPose.position.y + yError);
        localizer.setPose(new Pose2d(adjustedPosition, currentPose.heading));
    }

    public boolean near(Pose target) {
        Pose2d currentPose = getPose();
        double headingError = currentPose.heading.minus(target.pose2d.heading);
        return Math.abs(headingError) <= NEAR_RADIANS
                && Math.abs(currentPose.position.x - target.xInches()) <= NEAR_INCHES
                && Math.abs(currentPose.position.y - target.yInches()) <= NEAR_INCHES;
    }

    private Pose2d getPose() {
        return localizer.pose();
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    private static double angleRadians(Vector2d from, Vector2d to) {
        return Math.atan2(to.y - from.y, to.x - from.x);
    }

    private static double distanceInches(Vector2d from, Vector2d to) {
        return Math.hypot(to.x - from.x, to.y - from.y);
    }

    private static Vector2d pointAtDistanceInDirection(Vector2d from, double distance, double directionRadians) {
        return new Vector2d(
                from.x + Math.cos(directionRadians) * distance, from.y + Math.sin(directionRadians) * distance);
    }

    @Override
    public void loop() {}
}
