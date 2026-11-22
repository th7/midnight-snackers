package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.hardware.Dashboard;
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

    private final Map<String, FakeDcMotorEx> motors = Map.of(
            "leftFront", leftFront,
            "rightFront", rightFront,
            "leftBack", leftBack,
            "rightBack", rightBack,
            "launcher", launcher,
            "turnTable", turnTable,
            "intake", intake);
    private final Map<String, FakeServo> servos = Map.of("topGate", topGate, "bottomGate", bottomGate);

    private long nanos = 0;

    public Hardware hardware() {
        return hardware(ArrayList::new);
    }

    public Hardware hardware(Supplier<List<AprilTagDetection>> aprilTags) {
        return new Hardware() {
            @Override
            public DcMotorEx motor(String name) {
                return named(name, "motor", motors, "servo", servos);
            }

            @Override
            public Servo servo(String name) {
                return named(name, "servo", servos, "motor", motors);
            }

            @Override
            public LazyImu imu() {
                return () -> imu;
            }

            @Override
            public VoltageSensor voltageSensor() {
                return voltageSensor;
            }

            @Override
            public Supplier<List<AprilTagDetection>> aprilTags() {
                return aprilTags;
            }

            @Override
            public Dashboard dashboard() {
                return dashboard;
            }

            @Override
            public LongSupplier nanoClock() {
                return SimDevices.this::nanoTime;
            }
        };
    }

    private static <D> D named(
            String name, String kind, Map<String, ? extends D> ofKind, String otherKind, Map<String, ?> ofOtherKind) {
        D device = ofKind.get(name);
        if (device != null) {
            return device;
        }
        if (ofOtherKind.containsKey(name)) {
            throw new IllegalStateException("the robot asks for a " + kind + " named " + name + ", and the simulator's "
                    + name + " is a " + otherKind);
        }
        throw new IllegalStateException(
                "the robot asks for a " + kind + " named " + name + ", and the simulator has no " + name);
    }

    public long nanoTime() {
        return nanos;
    }

    public void advance(double seconds) {
        nanos += Math.round(seconds * 1e9);
    }
}
