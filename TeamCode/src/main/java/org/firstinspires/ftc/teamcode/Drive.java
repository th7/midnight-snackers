package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.base.DriveRunner;
import org.firstinspires.ftc.teamcode.base.FastDrive;
import org.firstinspires.ftc.teamcode.base.MoveData;
import org.firstinspires.ftc.teamcode.base.SubSystem;

/**
 * Moves the robot. Whoever is driving says what they want each loop and the drive writes the
 * motors itself: {@link #manual} powers from the sticks, {@link #toward} a pose the drive steers
 * to on its own, or {@link #follow} a Road Runner action that runs until it is done or
 * {@link #cancel}led. An action being followed owns the wheels: manual and toward do nothing
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
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;
    private boolean telemetryOn = false;

    public Drive(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
    }

    /**
     * The drive runner draws the robot where Nav says it is, so the dashboard field view shows
     * the robot whenever the op mode is running, not only during RoadRunner actions.
     */
    @Override
    public void init() {
        driveRunner = add(new DriveRunner(robot.dashboard, () -> robot.nav.currentPose().pose2d));
    }

    @Override
    protected void onLoop() {
        if (telemetryOn) {
            setTelemetry();
        }
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
        fastDrive.setDestination(target.pose2d);
        fastDrive.update(robot.nav.currentPose().pose2d);
        power(held.straightOr(fastDrive.straightPower()), held.strafeOr(fastDrive.strafePower()),
                held.turnOr(fastDrive.turnPower()));
        return fastDrive.doneMoving();
    }

    /**
     * Follows a Road Runner action, loop by loop, until it is done or cancelled.
     *
     * @throws IllegalStateException while another action is still being followed
     */
    public void follow(Action action) {
        driveRunner.drive(action);
    }

    /** Whether no action is being followed. */
    public boolean done() {
        return driveRunner.done();
    }

    /** Stops following the action, if any; the wheels keep their last power until the next intent. */
    public void cancel() {
        driveRunner.cancel();
    }

    public void toggleTelemetry() {
        telemetryOn = !telemetryOn;
    }

    private void power(float straight, float strafe, float turn) {
        if (!done()) {
            return;
        }
        MoveData moveData = MoveData.straight(straight, 0f, 1f)
                .add(MoveData.strafe(strafe, 0f, 1f), MoveData.turn(turn, 0f, 1f));
        leftFront.setPower(moveData.frontLeftPower());
        rightFront.setPower(moveData.frontRightPower());
        leftBack.setPower(moveData.rearLeftPower());
        rightBack.setPower(moveData.rearRightPower());
    }

    private void setTelemetry() {
        telemetry.addData("Drive", "telemetry on");

        Pose2d error = fastDrive.error();
        if (error != null) {
            telemetry.addData("fastDriveError.x", error.position.x);
            telemetry.addData("fastDriveError.y", error.position.y);
            telemetry.addData("fastDriveError.h", Rotation2d.exp(0).minus(error.heading));
        }

        telemetry.addData("fastDriveStraightPower", fastDrive.straightPower());
        telemetry.addData("fastDriveStrafePower", fastDrive.strafePower());
        telemetry.addData("fastDriveTurnPower", fastDrive.turnPower());
        telemetry.addData("fastDrive.atDestination();", fastDrive.doneMoving());
        telemetry.addData("fastDrive.nearXDestination();", fastDrive.nearXDestination());
        telemetry.addData("fastDrive.nearYDestination();", fastDrive.nearYDestination());
        telemetry.addData("fastDrive.nearHDestination();", fastDrive.nearHDestination());
        telemetry.addData("fastDrive.notMoving();", fastDrive.notMoving());
    }
}
