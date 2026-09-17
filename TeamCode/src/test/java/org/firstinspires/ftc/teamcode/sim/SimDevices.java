package org.firstinspires.ftc.teamcode.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * A robot's devices as fakes, and a clock: everything the robot code can reach, and nothing that
 * moves on its own.
 *
 * <p>This is the other adapter at the {@link Hardware} seam. {@link SimRobot} is the first: it
 * holds one of these and writes the sensors from where the physics put the robot. A test of one
 * subsystem wants neither the physics nor the field -- a turntable turns because its own motor
 * says so -- and it wants time it can move by hand, which is what {@link #advance} is. Before
 * this, both came only as a side effect of building a rigid-body world, so a test of the intake
 * loaded the ball model, and a launcher waiting a tenth of a second moved the balls to get there.
 *
 * <p>Two adapters, one seam, and the seam is the one the robot is built on either way.
 */
public final class SimDevices {
    /** The battery a robot with no noise runs on, in volts. */
    public static final double BATTERY_VOLTS = 12.5;

    public final FakeDcMotorEx leftFront = new FakeDcMotorEx();
    public final FakeDcMotorEx rightFront = new FakeDcMotorEx();
    public final FakeDcMotorEx leftBack = new FakeDcMotorEx();
    public final FakeDcMotorEx rightBack = new FakeDcMotorEx();
    public final FakeDcMotorEx launcher = new FakeDcMotorEx();
    public final FakeDcMotorEx turnTable = new FakeDcMotorEx();
    public final FakeDcMotorEx intake = new FakeDcMotorEx();
    public final FakeServo topGate = new FakeServo();
    public final FakeServo bottomGate = new FakeServo();
    public final FakeImu imu = new FakeImu();
    public final FakeVoltageSensor voltageSensor = new FakeVoltageSensor(BATTERY_VOLTS);
    public final FakeDashboard dashboard = new FakeDashboard();

    /** Nanoseconds since these devices were made; only {@link #advance} moves it. */
    private long nanos = 0;

    /** The devices, wired the way {@link Hardware#fromHardwareMap} wires the real ones. */
    public Hardware hardware() {
        return hardware(ArrayList::new);
    }

    /**
     * The same devices, with a camera that sees {@code aprilTags}: nothing here has vision of its
     * own, so whoever wants the robot to see something says what.
     */
    public Hardware hardware(Supplier<List<AprilTagDetection>> aprilTags) {
        return Hardware.builder()
                .launcher(launcher)
                .topGate(topGate)
                .bottomGate(bottomGate)
                .leftFront(leftFront)
                .rightFront(rightFront)
                .leftBack(leftBack)
                .rightBack(rightBack)
                .turnTable(turnTable)
                .intake(intake)
                .imu(() -> imu)
                .voltageSensor(voltageSensor)
                .aprilTags(aprilTags)
                .dashboard(dashboard)
                .clock(this::nanoTime)
                .build();
    }

    /** The clock the robot code reads through its hardware, in nanoseconds. */
    public long nanoTime() {
        return nanos;
    }

    /**
     * Moves the clock on by {@code seconds}. Nothing else moves: these devices read what was last
     * written to them, so time passing is all this says.
     */
    public void advance(double seconds) {
        nanos += Math.round(seconds * 1e9);
    }
}
