package org.firstinspires.ftc.framework.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.framework.hardware.Dashboard;
import org.firstinspires.ftc.framework.hardware.Wheels;
import org.firstinspires.ftc.framework.plans.Step;
import org.firstinspires.ftc.framework.robot.Prints;
import org.firstinspires.ftc.framework.robot.SubSystem;

public class Chassis extends SubSystem {
    public static final class Held {
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

    private static final boolean NOT_ARRIVED = false;

    private final Prints telemetry;
    private final DriveRunner driveRunner;
    private final FastDrive fastDrive = new FastDrive();

    private FastDrive.Steering steering;

    private final Wheels wheels;
    private final MecanumDrive mecanumDrive;
    private final Localizer localizer;
    private final Nav nav;

    public Chassis(
            Wheels wheels,
            MecanumDrive mecanumDrive,
            Localizer localizer,
            Nav nav,
            Dashboard dashboard,
            Prints telemetry) {
        this.wheels = wheels;
        this.mecanumDrive = mecanumDrive;
        this.localizer = localizer;
        this.nav = nav;
        this.telemetry = telemetry;
        this.driveRunner = new DriveRunner(dashboard, localizer::pose);
    }

    public void manual(float straight, float strafe, float turn) {
        power(straight, strafe, turn);
    }

    public boolean toward(Nav.Pose target) {
        return toward(target, Held.NONE);
    }

    public boolean toward(Nav.Pose target, Held held) {
        if (anActionOwnsTheWheels()) {
            return NOT_ARRIVED;
        }
        steering = fastDrive.steer(localizer.pose(), target.pose2d);
        power(held.straightOr(steering.straight), held.strafeOr(steering.strafe), held.turnOr(steering.turn));
        return steering.arrived;
    }

    private boolean anActionOwnsTheWheels() {
        return !done();
    }

    public void follow(Action action) {
        driveRunner.drive(action);
    }

    public void strafeTo(Nav.Pose... path) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(localizer.pose());
        for (Nav.Pose pose : path) {
            builder = builder.strafeToSplineHeading(pose.pose2d.position, pose.pose2d.heading);
        }
        follow(builder.build());
    }

    public void backwardTo(Nav.Pose... path) {
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(localizer.pose());
        for (Nav.Pose pose : path) {
            builder = builder.setReversed(true).splineToSplineHeading(pose.pose2d, Math.PI);
        }
        follow(builder.build());
    }

    public Step to(double x, double y, double heading) {
        Nav.Pose target = nav.pose(x, y, heading);
        return new Step("drive to " + target, () -> strafeTo(target), this::done);
    }

    public Step near(double x, double y, double heading) {
        Nav.Pose target = nav.pose(x, y, heading);
        return new Step("drive near " + target, () -> strafeTo(target), () -> {
            if (nav.near(target)) {
                cancel();
                return true;
            }
            return false;
        });
    }

    public boolean done() {
        return driveRunner.done();
    }

    public void cancel() {
        driveRunner.cancel();
        wheels.stop();
    }

    private void power(float straight, float strafe, float turn) {
        if (anActionOwnsTheWheels()) {
            return;
        }
        wheels.drive(new PoseVelocity2d(new Vector2d(straight, strafe), turn));
    }

    @Override
    public void loop() {
        driveRunner.loop();

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
