package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;

/**
 * The four wheels, and the only thing that turns numbers into them turning.
 *
 * <p>Everything that wants the robot to move — a driver's sticks, the drive steering itself toward
 * a pose, a Road Runner trajectory being followed — ends here. That there is one such place is the
 * point: while two of them wrote the same four motors, what the robot did came down to which ran
 * last in the loop, and nothing in the code said which that was. {@code DriveOwnsTheWheelsTest}
 * asks the motors who wrote them and holds the answer to this class.
 *
 * <p>It also does the wiring the motors need once, when the robot is built: they brake when asked
 * for nothing, and the two on the back run the other way round because of how they are mounted.
 */
public final class Wheels {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final DcMotorEx rightFront;

    public Wheels(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightBack, DcMotorEx rightFront) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Drives at these powers in the robot's own frame: forward, left, and counterclockwise, -1 to
     * 1 each. This is the one place three of those become four, so there is one answer to what a
     * mixed command means and one answer to what happens when it asks for more than there is.
     *
     * <p>What happens is that the whole command is scaled to fit. A wheel asked for more than full
     * power cannot give it, and clipping that one wheel would leave the others as they were, which
     * is a different command than the one given: the robot would go somewhere other than where it
     * was pointed. Scaled down whole, it goes where it was pointed, slower.
     */
    public void drive(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheels =
                new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

        double most = 1;
        for (DualNum<Time> power : wheels.all()) {
            most = Math.max(most, Math.abs(power.value()));
        }

        set(
                wheels.leftFront.get(0) / most,
                wheels.leftBack.get(0) / most,
                wheels.rightBack.get(0) / most,
                wheels.rightFront.get(0) / most);
    }

    /**
     * Drives the four wheels, each at its own power, -1 to 1. For a caller that has already worked
     * out what each wheel should do, such as a trajectory being followed.
     *
     * <p>The order is Road Runner's, which is the order its kinematics answer in and the order
     * everything here says out loud, so that a wheel is never quietly swapped for its neighbour.
     */
    public void set(double leftFront, double leftBack, double rightBack, double rightFront) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
        this.rightFront.setPower(rightFront);
    }

    /** Asks all four for nothing, which is a stop and not a coast: they are set to brake. */
    public void stop() {
        set(0, 0, 0, 0);
    }

    /** The left-hand motors, front first: what Road Runner's tuning op modes drive a side by. */
    public List<DcMotorEx> left() {
        return List.of(leftFront, leftBack);
    }

    /** The right-hand motors, front first. */
    public List<DcMotorEx> right() {
        return List.of(rightFront, rightBack);
    }
}
