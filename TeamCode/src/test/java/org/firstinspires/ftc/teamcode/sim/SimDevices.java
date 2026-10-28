package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
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
        return wired(Hardware.builder(), offered(aprilTags)).build();
    }

    private Map<String, Object> offered(Supplier<List<AprilTagDetection>> aprilTags) {
        Map<String, Object> offered = new LinkedHashMap<>();
        offered.put("launcher", launcher);
        offered.put("topGate", topGate);
        offered.put("bottomGate", bottomGate);
        offered.put("leftFront", leftFront);
        offered.put("rightFront", rightFront);
        offered.put("leftBack", leftBack);
        offered.put("rightBack", rightBack);
        offered.put("turnTable", turnTable);
        offered.put("intake", intake);
        offered.put("imu", (LazyImu) () -> imu);
        offered.put("voltageSensor", voltageSensor);
        offered.put("aprilTags", aprilTags);
        offered.put("dashboard", dashboard);
        offered.put("nanoClock", (LongSupplier) this::nanoTime);
        return offered;
    }

    static <B> B wired(B builder, Map<String, Object> offered) {
        for (Method setter : builder.getClass().getMethods()) {
            if (setter.getDeclaringClass() != builder.getClass() || setter.getParameterCount() != 1) {
                continue;
            }
            String name = setter.getName();
            Class<?> kind = setter.getParameterTypes()[0];
            Object device = offered.get(name);
            if (device == null) {
                throw new IllegalStateException(
                        "the robot's hardware asks for " + name + ", and the simulator has no " + name + " to hand it");
            }
            if (!kind.isInstance(device)) {
                throw new IllegalStateException("the robot's hardware asks for " + name + " as a "
                        + kind.getSimpleName() + ", and the simulator's " + name + " is not one");
            }
            try {
                setter.invoke(builder, device);
            } catch (IllegalAccessException e) {
                throw new IllegalStateException("the simulator could not hand the robot's hardware its " + name, e);
            } catch (InvocationTargetException e) {
                throw new IllegalStateException("the robot's hardware refused the simulator's " + name, e.getCause());
            }
        }
        return builder;
    }

    public long nanoTime() {
        return nanos;
    }

    public void advance(double seconds) {
        nanos += Math.round(seconds * 1e9);
    }
}
