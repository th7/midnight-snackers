package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.base.Loopable;

/**
 * The intake: what takes pollen off the floor and into the hopper. It is on or off and nothing
 * else — on, its motor runs at {@link #RUNNING_POWER}; off, the motor stops — and it starts
 * {@link #isOn() on}, so a robot that is never told anything is taking pollen in from the moment
 * it is built. Everything the mechanism will need beyond that (a sensor saying the hopper is full,
 * reversing to clear a jam) is still to be built, and the simulator holds the intake to this much:
 * the pollen the front of the robot meets go in while the motor is running.
 */
public class Intake implements Loopable {
    /** What the motor runs at while the intake is on: all of it, since on is all this says. */
    public static final double RUNNING_POWER = 1;

    private final DcMotorEx intake;
    private boolean on = true;

    public Intake(DcMotorEx intake) {
        this.intake = intake;
        // on from the moment the robot is built, not from its first loop
        driveTheMotor();
    }

    /** Take pollen in. */
    public void on() {
        on = true;
    }

    /** Stop taking pollen in. */
    public void off() {
        on = false;
    }

    /** Whether the intake is taking pollen in. */
    public boolean isOn() {
        return on;
    }

    private void driveTheMotor() {
        intake.setPower(on ? RUNNING_POWER : 0);
    }

    @Override
    public void loop() {
        driveTheMotor();
    }
}
