package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;

public class Turntable implements Loopable {
    public static final String CHANNEL = "Turntable";

    private final Prints telemetry;
    public static final int TICKS_PER_REVOLUTION = 1700;

    private static final int NUDGE_TICKS = 10;

    private static final int DEADBAND_TICKS = 5;
    private static final double MINIMUM_POWER = 0.1;
    private static final double TICKS_PER_FULL_POWER = 100;

    private enum Following {
        THE_GOAL,

        STRAIGHT_AHEAD,

        THE_DRIVERS_HAND
    }

    private final DcMotorEx turnTable;
    private Following following = Following.THE_GOAL;
    private int turnTableTargetPosition;

    public Turntable(DcMotorEx turnTable, Prints telemetry) {
        this.turnTable = turnTable;
        this.telemetry = telemetry;
    }

    private float clampMinPower(double power, double min) {
        if (power > 0 && power < min) {
            return (float) min;
        } else if (power < 0 && power > -min) {
            return (float) -min;
        } else {
            return (float) power;
        }
    }

    public void aimAt(double relativeHeadingRadians) {
        if (following != Following.THE_GOAL) {
            return;
        }
        turnTableTargetPosition = wrapped(ticks(relativeHeadingRadians));
    }

    public void followTheGoal() {
        following = Following.THE_GOAL;
    }

    public void parkStraightAhead() {
        following = Following.STRAIGHT_AHEAD;
        turnTableTargetPosition = 0;
    }

    public void nudgeLeft() {
        nudge(NUDGE_TICKS);
    }

    public void nudgeRight() {
        nudge(-NUDGE_TICKS);
    }

    private void nudge(int ticks) {
        following = Following.THE_DRIVERS_HAND;
        turnTableTargetPosition += ticks;
    }

    public double offsetRadians() {
        double revolutions = (double) turnTable.getCurrentPosition() / TICKS_PER_REVOLUTION;
        return revolutions * (Math.PI * 2);
    }

    private static int ticks(double radians) {
        return (int) (radians * (TICKS_PER_REVOLUTION / (Math.PI * 2)));
    }

    private static int wrapped(int ticks) {
        int middle = ticks % TICKS_PER_REVOLUTION;
        if (middle > TICKS_PER_REVOLUTION / 2) {
            return middle - TICKS_PER_REVOLUTION;
        } else if (middle < -TICKS_PER_REVOLUTION / 2) {
            return middle + TICKS_PER_REVOLUTION;
        }
        return middle;
    }

    @Override
    public void loop() {
        double turnTableError = turnTableTargetPosition - turnTable.getCurrentPosition();
        double turnTablePower = turnTableError / TICKS_PER_FULL_POWER;

        if (turnTableError < DEADBAND_TICKS && turnTableError > -DEADBAND_TICKS) {
            turnTable.setPower(0);
        } else {
            turnTable.setPower(clampMinPower(turnTablePower, MINIMUM_POWER));
        }

        telemetry.addData("turnTableFollowing", following);
        telemetry.addData("turnTableRotationTicks", turnTable.getCurrentPosition());
        telemetry.addData("turnTableOffsetRadians", offsetRadians());
        telemetry.addData("turnTableTargetPosition", turnTableTargetPosition);
        telemetry.addData("turnTablePower", turnTable.getPower());
    }
}
