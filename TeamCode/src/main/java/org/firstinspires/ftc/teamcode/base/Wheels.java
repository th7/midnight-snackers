package org.firstinspires.ftc.teamcode.base;

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
     * Drives the four wheels, each at its own power, -1 to 1.
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
