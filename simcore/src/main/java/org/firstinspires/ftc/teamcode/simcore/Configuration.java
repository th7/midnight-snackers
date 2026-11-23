package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public final class Configuration {
    public sealed interface Device permits Motor, Imu, ContinuousServo, Servo, VoltageSensor {
        String name();

        String kind();

        Device named(String name);
    }

    public record Motor(String name, MotorType type, EncoderReads encoder) implements Device {
        @Override
        public String kind() {
            return "motor";
        }

        @Override
        public Device named(String name) {
            return new Motor(name, type, encoder);
        }
    }

    public record Imu(String name) implements Device {
        @Override
        public String kind() {
            return "imu";
        }

        @Override
        public Device named(String name) {
            return new Imu(name);
        }
    }

    public record ContinuousServo(String name) implements Device {
        @Override
        public String kind() {
            return "continuous_servo";
        }

        @Override
        public Device named(String name) {
            return new ContinuousServo(name);
        }
    }

    public record Servo(String name) implements Device {
        @Override
        public String kind() {
            return "servo";
        }

        @Override
        public Device named(String name) {
            return new Servo(name);
        }
    }

    public record VoltageSensor(String name) implements Device {
        @Override
        public String kind() {
            return "voltage_sensor";
        }

        @Override
        public Device named(String name) {
            return new VoltageSensor(name);
        }
    }

    public record Committed(String as, Device device) {}

    private final List<Committed> committed;

    private Configuration(List<Committed> committed) {
        this.committed = List.copyOf(committed);
    }

    private static Configuration underTheirOwnNames(List<Device> devices) {
        List<Committed> committed = new ArrayList<>();
        for (Device device : devices) {
            committed.add(new Committed(device.name(), device));
        }
        return new Configuration(committed);
    }

    public static Configuration reginalds() {
        return underTheirOwnNames(List.of(
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
        return checked(devices).map(Configuration::underTheirOwnNames);
    }

    public static Checked<Configuration> committedAs(List<Committed> committed) {
        Set<String> committedNames = new HashSet<>();
        List<Device> devices = new ArrayList<>();
        for (Committed one : committed) {
            if (!committedNames.add(one.as())) {
                return Checked.rejected("two devices are committed as '" + one.as() + "'");
            }
            devices.add(one.device());
        }
        return checked(devices).map(valid -> new Configuration(committed));
    }

    private static Checked<List<Device>> checked(List<Device> devices) {
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
        return Checked.ok(devices);
    }

    public List<Device> devices() {
        List<Device> devices = new ArrayList<>();
        for (Committed one : committed) {
            devices.add(one.device());
        }
        return List.copyOf(devices);
    }

    public List<Committed> committed() {
        return committed;
    }

    public boolean committedUnderTheirOwnNames() {
        for (Committed one : committed) {
            if (!one.as().equals(one.device().name())) {
                return false;
            }
        }
        return true;
    }

    public Optional<Device> committed(String as) {
        for (Committed one : committed) {
            if (one.as().equals(as)) {
                return Optional.of(one.device());
            }
        }
        return Optional.empty();
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Configuration that && committed.equals(that.committed);
    }

    @Override
    public int hashCode() {
        return committed.hashCode();
    }

    @Override
    public String toString() {
        return devices().toString();
    }
}
