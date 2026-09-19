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

public final class SimDevices {
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

    private long nanos = 0;

    public Hardware hardware() {
        return hardware(ArrayList::new);
    }

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
                .nanoClock(this::nanoTime)
                .build();
    }

    public long nanoTime() {
        return nanos;
    }

    public void advance(double seconds) {
        nanos += Math.round(seconds * 1e9);
    }
}
