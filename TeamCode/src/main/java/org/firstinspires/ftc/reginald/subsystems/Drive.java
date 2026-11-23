package org.firstinspires.ftc.reginald.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Optional;
import org.firstinspires.ftc.reginald.control.FastDrive;
import org.firstinspires.ftc.reginald.framework.SubSystem;
import org.firstinspires.ftc.reginald.hardware.Wheels;
import org.firstinspires.ftc.reginald.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.control.DriveRunner;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;
import org.firstinspires.ftc.teamcode.planrunner.Step;

public class Drive extends SubSystem {
    private final Prints telemetry;

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

    private static final float HELD = 0.05f;

    private static final float TAKEOVER = 0.2f;

    private DriveRunner driveRunner;
    private final FastDrive fastDrive = new FastDrive();

    private FastDrive.Steering steering;

    private final Wheels wheels;
    private final MecanumDrive mecanumDrive;
    private final Localizer localizer;
    private final Nav nav;
    private final Dashboard dashboard;

    public Drive(
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
        this.dashboard = dashboard;
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
    public void teleOp(Gamepads gamepads) {
        Gamepad gamepad1 = gamepads.gamepad1;
        if (Math.abs(gamepad1.left_stick_x) > TAKEOVER
                || Math.abs(gamepad1.left_stick_y) > TAKEOVER
                || Math.abs(gamepad1.right_stick_x) > TAKEOVER
                || Math.abs(gamepad1.right_stick_y) > TAKEOVER) {
            cancel();
        }

        if (gamepad1.left_bumper) {
            Held held = Held.NONE;
            if (Math.abs(gamepad1.left_stick_x) > HELD) {
                held = held.strafe(-gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            aim(gamepad1, held);
        } else if (gamepad1.right_bumper) {
            Held held = Held.NONE.straight(-gamepad1.left_stick_y).strafe(-gamepad1.left_stick_x);
            if (Math.abs(gamepad1.right_stick_x) > HELD) {
                held = held.turn(-gamepad1.right_stick_x);
            }
            aim(gamepad1, held);
        } else {
            manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }
    }

    private void aim(Gamepad gamepad1, Held held) {
        Optional<Nav.Pose> launchPose = nav.launchPose();
        if (launchPose.isPresent()) {
            toward(launchPose.get(), held);
        } else {
            manual(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
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
