package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.base.SubSystem;
import org.firstinspires.ftc.teamcode.control.DriveRunner;
import org.firstinspires.ftc.teamcode.control.FastDrive;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

/**
 * Moves the robot. Whoever is driving says what they want each loop and the drive writes the
 * motors itself: {@link #manual} powers from the sticks, {@link #toward} a pose the drive steers
 * to on its own, or {@link #follow} a Road Runner action that runs until it is done or
 * {@link #cancel}led, which also stops the robot. An action being followed owns the wheels: manual and toward do nothing
 * until it is done or cancelled.
 */
public class Drive extends SubSystem {
    /**
     * The axes a driver holds while the drive steers {@link #toward} a pose: an axis held is
     * driven at the driver's power, one not held is the drive's to steer.
     */
    public static final class Held {
        /** Nothing held: every axis is the drive's. */
        public static final Held NONE = new Held(null, null, null);

        private final Float straight;
        private final Float strafe;
        private final Float turn;

        private Held(Float straight, Float strafe, Float turn) {
            this.straight = straight;
            this.strafe = strafe;
            this.turn = turn;
        }

        public Held straight(float power) {
            return new Held(power, strafe, turn);
        }

        public Held strafe(float power) {
            return new Held(straight, power, turn);
        }

        public Held turn(float power) {
            return new Held(straight, strafe, power);
        }

        private float straightOr(float drives) {
            return straight != null ? straight : drives;
        }

        private float strafeOr(float drives) {
            return strafe != null ? strafe : drives;
        }

        private float turnOr(float drives) {
            return turn != null ? turn : drives;
        }
    }

    private DriveRunner driveRunner;
    private final FastDrive fastDrive = new FastDrive();
    /** What the last {@link #toward} decided, for telemetry; null until the drive has steered. */
    private FastDrive.Steering steering;

    private final Wheels wheels;
    private final MecanumDrive mecanumDrive;
    private final Localizer localizer;
    private final Dashboard dashboard;

    public Drive(Wheels wheels, MecanumDrive mecanumDrive, Localizer localizer, Dashboard dashboard) {
        this.wheels = wheels;
        this.mecanumDrive = mecanumDrive;
        this.localizer = localizer;
        this.dashboard = dashboard;
    }

    /**
     * The drive runner draws the robot where Nav says it is, so the dashboard field view shows
     * the robot whenever the op mode is running, not only during RoadRunner actions.
     */
    @Override
    protected void onInit() {
        driveRunner = new DriveRunner(dashboard, localizer::pose);
    }

    /** The drive runner is the drive's, so the drive ticks it: this is what Drive does each loop. */
    @Override
    protected void onLoop() {
        driveRunner.loop();
    }

    /**
     * Drives at these powers, -1 to 1 each, this loop. Nothing happens while an action is being
     * followed.
     */
    public void manual(float straight, float strafe, float turn) {
        power(straight, strafe, turn);
    }

    /**
     * Steers toward {@code target} from where Nav says the robot is, this loop.
     *
     * @return whether the robot has arrived and come to rest there
     */
    public boolean toward(Nav.Pose target) {
        return toward(target, Held.NONE);
    }

    /**
     * Steers toward {@code target} on the axes the driver is not holding, this loop. Nothing
     * moves while an action is being followed.
     *
     * @return whether the robot has arrived and come to rest there
     */
    public boolean toward(Nav.Pose target, Held held) {
        steering = fastDrive.steer(localizer.pose(), target.pose2d);
        power(held.straightOr(steering.straight), held.strafeOr(steering.strafe), held.turnOr(steering.turn));
        return steering.arrived;
    }

    /**
     * Follows a Road Runner action, loop by loop, until it is done or cancelled.
     *
     * @throws IllegalStateException while another action is still being followed
     */
    public void follow(Action action) {
        driveRunner.drive(action);
    }

    /**
     * Strafes through the poses, from where the robot is now, until it arrives or is cancelled:
     * the robot faces where each pose says while it goes, rather than turning to face the way it
     * is travelling.
     *
     * @throws IllegalStateException while another action is still being followed
     */
    public void strafeTo(Nav.Pose... path) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(localizer.pose());
        for (Nav.Pose pose : path) {
            builder = builder.strafeToSplineHeading(pose.pose2d.position, pose.pose2d.heading);
        }
        follow(builder.build());
    }

    /**
     * Backs through the poses, from where the robot is now, until it arrives or is cancelled.
     *
     * @throws IllegalStateException while another action is still being followed
     */
    public void backwardTo(Nav.Pose... path) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(localizer.pose());
        for (Nav.Pose pose : path) {
            builder = builder.setReversed(true).splineToSplineHeading(pose.pose2d, Math.PI);
        }
        follow(builder.build());
    }

    /** Whether no action is being followed. */
    public boolean done() {
        return driveRunner.done();
    }

    /**
     * Stops following the action, if any, and stops the robot.
     *
     * <p>Cancelling is whoever was driving saying they are done with it, and a robot nobody is
     * driving should not still be driving. The wheels are asked for nothing, which brakes them
     * rather than letting them coast. Whoever cancels is free to give an intent in the same loop,
     * and that is what the robot will do.
     */
    public void cancel() {
        driveRunner.cancel();
        wheels.stop();
    }

    private void power(float straight, float strafe, float turn) {
        if (!done()) {
            return;
        }
        wheels.drive(new PoseVelocity2d(new Vector2d(straight, strafe), turn));
    }

    /** What the last steer decided: one loop's numbers, all read off the one error. */
    @Override
    protected void onTelemetry() {
        if (steering == null) {
            telemetry.addData("steering", "the drive has not steered toward a pose yet");
            return;
        }

        telemetry.addData("steeringError.x", steering.error.position.x);
        telemetry.addData("steeringError.y", steering.error.position.y);
        telemetry.addData("steeringError.h", Rotation2d.exp(0).minus(steering.error.heading));
        telemetry.addData("steeringStraightPower", steering.straight);
        telemetry.addData("steeringStrafePower", steering.strafe);
        telemetry.addData("steeringTurnPower", steering.turn);
        telemetry.addData("steeringArrived", steering.arrived);
        telemetry.addData("steeringNearStraight", steering.nearStraight);
        telemetry.addData("steeringNearStrafe", steering.nearStrafe);
        telemetry.addData("steeringNearTurn", steering.nearTurn);
    }
}
