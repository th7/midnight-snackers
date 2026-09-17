package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;

/**
 * Where the robot is on the field, and where the places worth going are. Everything in and out is a
 * {@link Pose} on the field: {@link #pose} makes one from coordinates given the blue way, and
 * the alliance's mirroring happens there and nowhere else.
 * <p>
 * 2025-2026 Season. Treat all coordinates and headings as if you are playing as blue; if you're
 * red, they are mirrored for you. Positive y moves toward your goal, positive headings turn
 * toward your goal.
 */
public class Nav implements Loopable {
    // COORDINATES!!! ARGH
    // Imagine facing the field from the audience. Blue goal is forward left, red goal is forward right. Any further
    // mention of left/right or forward/backward is relative to this perspective.

    // FTC Coordinates
    // +x backward, +y right, straight forward heading is PI/2

    // Roadrunner Coordinates (this is what we use)
    // +x forward, +y left, straight forward heading is 0

    /**
     * A position and heading on the field, in Road Runner coordinates, as the alliance plays it.
     * Only Road Runner itself and the drive's controllers read the {@link #pose2d} inside.
     */
    public static class Pose {
        public final Pose2d pose2d;

        public Pose(Pose2d pose2d) {
            this.pose2d = pose2d;
        }

        public double x() {
            return pose2d.position.x;
        }

        public double y() {
            return pose2d.position.y;
        }

        /** Radians, counterclockwise from +x. */
        public double heading() {
            return pose2d.heading.toDouble();
        }

        /** The same place, turned by {@code radians}. */
        public Pose rotated(double radians) {
            return new Pose(new Pose2d(pose2d.position, pose2d.heading.plus(radians)));
        }

        @Override
        public String toString() {
            return String.format("(%.1f, %.1f, %.2f)", x(), y(), heading());
        }
    }

    private static final double LAUNCH_DISTANCE = 40;
    private static final double NEAR_INCHES = 3;
    private static final double NEAR_RADIANS = Math.PI * 2 / 60;
    /** The most a later sighting may move the robot, per axis, so one bad frame cannot teleport it. */
    private static final double SIGHTING_NUDGE_INCHES = 1;

    private final int headingSign;
    private final int ySign;
    /** Where this alliance's goal is; null when playing for no alliance. */
    private final Vector2d launchTarget;

    private final Localizer localizer;

    /**
     * Whether the robot's pose means something on the field: somebody has said where it is, by
     * hand or through the camera. Until then the localizer's pose is only distance travelled from
     * wherever it was switched on, which says nothing about where the goal is.
     */
    private boolean onTheField = false;

    public Nav(Localizer localizer, Alliance alliance) {
        this.localizer = localizer;
        this.headingSign = alliance.headingSign;
        this.ySign = alliance.ySign;
        this.launchTarget = alliance.launchTarget;
    }

    /** A pose from coordinates given the blue way, mirrored for the alliance. */
    public Pose pose(double x, double y, double heading) {
        return new Pose(new Pose2d(x, y * this.ySign, heading * this.headingSign));
    }

    /** Where the localizer believes the robot is. */
    public Pose currentPose() {
        return new Pose(getPose());
    }

    /**
     * The robot is here: where a person set it down, or where a plan's first step says it started.
     * Placing it is what puts it on the field, so everything that needs to know where the goal is
     * works from this moment on.
     */
    public void placeAt(Pose pose) {
        onTheField = true;
        localizer.setPose(pose.pose2d);
    }

    /**
     * Where to launch from: {@value #LAUNCH_DISTANCE} inches short of the goal on the line from
     * the robot to it, facing the goal. Empty when playing for no alliance, which has no goal.
     */
    public Optional<Pose> launchPose() {
        if (launchTarget == null) {
            return Optional.empty();
        }
        Vector2d from = getPose().position;
        double bearingToTarget = angleRadians(from, launchTarget);
        double distanceToTarget = distanceInches(from, launchTarget);
        double distanceError = distanceToTarget - LAUNCH_DISTANCE;
        Vector2d position = pointAtDistanceInDirection(from, distanceError, bearingToTarget);
        return Optional.of(new Pose(new Pose2d(position, bearingToTarget)));
    }

    /**
     * How far the turntable must turn from straight ahead to face the goal: zero until the camera
     * has placed the robot on the field, or when there is no goal.
     */
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

    /**
     * The camera saw where the robot is. A sighting that arrives while the robot is not yet on the
     * field places it there, since there is nothing to nudge from; once it is on the field --
     * however it got there, by hand or by an earlier sighting -- a sighting nudges its position by
     * at most {@value #SIGHTING_NUDGE_INCHES} inch per axis and leaves the heading to the
     * localizer, so one bad frame cannot teleport a robot somebody measured and put down.
     */
    public void sighted(Pose sighting) {
        if (!onTheField) {
            placeAt(sighting);
            return;
        }

        Pose2d currentPose = getPose();
        double xError = clamp(sighting.x() - currentPose.position.x, SIGHTING_NUDGE_INCHES);
        double yError = clamp(sighting.y() - currentPose.position.y, SIGHTING_NUDGE_INCHES);
        Vector2d adjustedPosition = new Vector2d(currentPose.position.x + xError, currentPose.position.y + yError);
        localizer.setPose(new Pose2d(adjustedPosition, currentPose.heading));
    }

    /** Whether the robot is within {@value #NEAR_INCHES} inches and six degrees of {@code target}. */
    public boolean near(Pose target) {
        Pose2d currentPose = getPose();
        double headingError = currentPose.heading.minus(target.pose2d.heading);
        return Math.abs(headingError) <= NEAR_RADIANS
                && Math.abs(currentPose.position.x - target.x()) <= NEAR_INCHES
                && Math.abs(currentPose.position.y - target.y()) <= NEAR_INCHES;
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
