package org.firstinspires.ftc.teamcode.simcore;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public final class Configuration {
    public sealed interface Device permits Motor, Imu, ContinuousServo, Servo, VoltageSensor {
        String name();
    }

    public record Motor(String name, MotorType type, EncoderReads encoder) implements Device {}

    public record Imu(String name) implements Device {}

    public record ContinuousServo(String name) implements Device {}

    public record Servo(String name) implements Device {}

    public record VoltageSensor(String name) implements Device {}

    private final List<Device> devices;

    private Configuration(List<Device> devices) {
        this.devices = List.copyOf(devices);
    }

    public static Configuration reginalds() {
        return new Configuration(List.of(
                new Motor("leftFront", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.PERPENDICULAR_DEAD_WHEEL),
                new Motor("rightFront", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.NOTHING),
                new Motor("leftBack", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.NOTHING),
                new Motor("rightBack", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.PARALLEL_DEAD_WHEEL),
                new Motor("intake", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.OWN_MOTOR),
                new Motor("flywheelFront", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.OWN_MOTOR),
                new Motor("flywheelBack", MotorType.ULTRAPLANETARY_HD_HEX, EncoderReads.OWN_MOTOR),
                new Imu("imu"),
                new ContinuousServo("spinner"),
                new Servo("ramp"),
                new VoltageSensor("Control Hub")));
    }

    public static Checked<Configuration> of(List<Device> devices) {
        Set<String> names = new HashSet<>();
        Map<EncoderReads, String> deadWheels = new EnumMap<>(EncoderReads.class);
        for (Device device : devices) {
            String name = device.name();
            if (name.isBlank()) {
                return Checked.rejected("a device's name is not blank, not '" + name + "'");
            }
            if (!name.strip().equals(name)) {
                return Checked.rejected("a device's name has no space around it, not '" + name + "'");
            }
            if (!names.add(name)) {
                return Checked.rejected("two devices are named '" + name + "'");
            }
            if (device instanceof Motor motor && motor.encoder().deadWheel()) {
                String already = deadWheels.putIfAbsent(motor.encoder(), name);
                if (already != null) {
                    return Checked.rejected(motor.encoder() + " is read on one port at most, not on both '" + already
                            + "' and '" + name + "'");
                }
            }
        }
        return Checked.ok(new Configuration(devices));
    }

    public List<Device> devices() {
        return devices;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Configuration that && devices.equals(that.devices);
    }

    @Override
    public int hashCode() {
        return devices.hashCode();
    }

    @Override
    public String toString() {
        return devices.toString();
    }
}
