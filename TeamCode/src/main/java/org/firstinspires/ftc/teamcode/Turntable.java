package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.base.Prints;

/**
 * The turntable the launcher and the camera ride on, and the only thing that decides where it
 * points. It is given an <b>aim</b> each tick -- how far from straight ahead the goal is -- and
 * what it does with one is its own business, because what it is <b>following</b> is its own state:
 * the goal, straight ahead, or the driver's own hand.
 *
 * <p>That is the point of it being here. The aim, the parking and the nudging used to be three
 * modules' worth of booleans over one target field, so a nudge survived only because the line
 * above it in {@code Driver} flipped a flag on {@code Brain}; delete that line and the nudge still
 * compiled, still ran, and was overwritten on the brain's next tick twenty milliseconds later. A
 * nudge now <em>is</em> the driver taking it over, so there is no ordering left to forget.
 *
 * <p>A turntable follows the goal until it is told otherwise, and {@link #followTheGoal()} is what
 * gives it back after parking or a nudge.
 */
public class Turntable implements Loopable {
    /** The name this prints under: said here, where the printing is, and nowhere else. */
    public static final String CHANNEL = "Turntable";

    private final Prints telemetry;
    public static final int TICKS_PER_REVOLUTION = 1700;

    /** How far one nudge steps the target, in ticks. */
    private static final int NUDGE_TICKS = 10;

    private static final int DEADBAND_TICKS = 5;
    private static final double MINIMUM_POWER = 0.1;
    private static final double TICKS_PER_FULL_POWER = 100;

    /** What the turntable is following, and so whose say the target is. */
    private enum Following {
        /** The aim it is given each tick. */
        THE_GOAL,
        /** Straight ahead, whatever the aim says. */
        STRAIGHT_AHEAD,
        /** The driver's nudges, whatever the aim says. */
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

    /**
     * Point this far from straight ahead, turning whichever way is shorter. Ignored while the
     * turntable is parked or in the driver's hand: this is the brain's standing request, given
     * every tick, not an order.
     */
    public void aimAt(double relativeHeadingRadians) {
        if (following != Following.THE_GOAL) {
            return;
        }
        turnTableTargetPosition = wrapped(ticks(relativeHeadingRadians));
    }

    /** Follow the aim again, after parking or a nudge. */
    public void followTheGoal() {
        following = Following.THE_GOAL;
    }

    /** Hold straight ahead, whatever the aim says, until told to follow the goal again. */
    public void parkStraightAhead() {
        following = Following.STRAIGHT_AHEAD;
        turnTableTargetPosition = 0;
    }

    /** The driver takes the turntable over and steps it {@value #NUDGE_TICKS} ticks to the left. */
    public void nudgeLeft() {
        nudge(NUDGE_TICKS);
    }

    /** The driver takes the turntable over and steps it {@value #NUDGE_TICKS} ticks to the right. */
    public void nudgeRight() {
        nudge(-NUDGE_TICKS);
    }

    private void nudge(int ticks) {
        following = Following.THE_DRIVERS_HAND;
        turnTableTargetPosition += ticks;
    }

    /** How far the turntable has turned from straight ahead, which is where the camera faces. */
    public double offsetRadians() {
        double revolutions = (double) turnTable.getCurrentPosition() / TICKS_PER_REVOLUTION;
        return revolutions * (Math.PI * 2);
    }

    private static int ticks(double radians) {
        return (int) (radians * (TICKS_PER_REVOLUTION / (Math.PI * 2)));
    }

    /** The same place, expressed as the shorter way round: never more than half a revolution. */
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
