package org.firstinspires.ftc.teamcode.simcore;

public final class Drivetrain {
    private final PerWheel<Sense> mounted;
    private final PerWheel<Feedforward> motors;
    private final double inPerTick;

    private Drivetrain(PerWheel<Sense> mounted, PerWheel<Feedforward> motors, double inPerTick) {
        this.mounted = mounted;
        this.motors = motors;
        this.inPerTick = inPerTick;
    }

    public record Setting(Sense direction, Power power, ZeroPower atZero) {}

    public static Checked<Drivetrain> of(
            Drivebase drivebase, Feedforward tuned, double inPerTick, PerWheel<Noise.Motor> noise) {
        if (!(inPerTick > 0 && Double.isFinite(inPerTick))) {
            return Checked.rejected("a wheel turns a positive, finite distance a tick, not " + inPerTick + " in");
        }
        return PerWheel.allOf(drivebase.turning(noise).map(motor -> tuned.scaledBy(motor)))
                .map(motors -> new Drivetrain(drivebase.mounted(), motors, inPerTick));
    }

    public Checked<PerWheel<Double>> accelerations(
            PerWheel<Setting> settings, double volts, PerWheel<Double> velocities, Seconds dt, Traction traction) {
        return PerWheel.allOf(PerWheel.each(
                wheel -> acceleration(wheel, settings.of(wheel), volts, velocities.of(wheel), dt, traction)));
    }

    private Checked<Double> acceleration(
            Wheel wheel, Setting setting, double batteryVolts, double velocity, Seconds dt, Traction traction) {
        Feedforward motor = motors.of(wheel);
        double volts =
                mounted.of(wheel).of(setting.direction().of(setting.power().value())) * batteryVolts;
        double kSVolts = motor.kSVolts();
        double kAVoltSecondsSquaredPerTick = motor.kAVoltSecondsSquaredPerTick();
        double ticksPerSecond = velocity / inPerTick;
        boolean creeping = Math.abs(ticksPerSecond) <= kSVolts / kAVoltSecondsSquaredPerTick * dt.value();
        if (creeping && Math.abs(volts) <= kSVolts) {
            return Checked.ok(traction.gives(dt.value() > 0 ? -velocity / dt.value() : 0));
        }
        double sign = ticksPerSecond != 0 ? Math.signum(ticksPerSecond) : Math.signum(volts);
        return backEmf(wheel, setting, motor.kVVoltSecondsPerTick() * ticksPerSecond)
                .map(backEmf ->
                        traction.gives((volts - kSVolts * sign - backEmf) / kAVoltSecondsSquaredPerTick * inPerTick));
    }

    private static Checked<Double> backEmf(Wheel wheel, Setting setting, double turning) {
        if (setting.power().value() != 0) {
            return Checked.ok(turning);
        }
        return switch (setting.atZero()) {
            case BRAKE -> Checked.ok(turning);
            case FLOAT -> Checked.ok(0.0);
            case UNKNOWN ->
                Checked.rejected("the " + wheel + " wheel is rolling at zero power with its zero power"
                        + " behavior UNKNOWN: set BRAKE or FLOAT on it, as MecanumDrive does, so this simulation knows"
                        + " whether its motor holds it back or lets it roll");
        };
    }
}
