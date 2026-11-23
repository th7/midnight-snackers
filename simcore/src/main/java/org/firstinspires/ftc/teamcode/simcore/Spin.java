package org.firstinspires.ftc.teamcode.simcore;

public record Spin(double ticks, double ticksPerSecond) {
    public static final double NOMINAL_VOLTS = 12;
    public static final double SPIN_UP_SECONDS = 0.1;

    public static Spin still() {
        return new Spin(0, 0);
    }

    public Spin next(MotorType type, Power power, double volts, Seconds dt) {
        double free = type.freeTicksPerSecond() * power.value() * volts / NOMINAL_VOLTS;
        double velocity = free + (ticksPerSecond - free) * StrictMath.exp(-dt.value() / SPIN_UP_SECONDS);
        double turned = free * dt.value()
                + (ticksPerSecond - free) * SPIN_UP_SECONDS * (1 - StrictMath.exp(-dt.value() / SPIN_UP_SECONDS));
        return new Spin(ticks + turned, velocity);
    }

    public Spin next(Feedforward motor, Power power, double volts, ZeroPower atZero, Seconds dt) {
        double driving = volts * power.value();
        double backEmf = power.value() == 0 && atZero == ZeroPower.FLOAT ? 0 : motor.kVVoltSecondsPerTick();
        double sense = ticksPerSecond != 0 ? StrictMath.signum(ticksPerSecond) : starting(motor, driving);
        if (sense == 0) {
            return new Spin(ticks, 0);
        }
        Course course = Course.of(motor, driving - motor.kSVolts() * sense, backEmf, ticksPerSecond);
        double stopsAfter = course.stopsAfter();
        if (!(stopsAfter < dt.value())) {
            return new Spin(ticks + course.turnedBy(dt.value()), course.speedAt(dt.value()));
        }
        double after = dt.value() - stopsAfter;
        double turned = ticks + course.turnedBy(stopsAfter);
        double restarting = starting(motor, driving);
        if (restarting == 0) {
            return new Spin(turned, 0);
        }
        Course again = Course.of(motor, driving - motor.kSVolts() * restarting, backEmf, 0);
        return new Spin(turned + again.turnedBy(after), again.speedAt(after));
    }

    private static double starting(Feedforward motor, double driving) {
        return StrictMath.abs(driving) > motor.kSVolts() ? StrictMath.signum(driving) : 0;
    }

    private record Course(double from, double toward, double seconds, double accelerating) {
        static Course of(Feedforward motor, double pushing, double backEmf, double from) {
            double kA = motor.kAVoltSecondsSquaredPerTick();
            if (backEmf == 0) {
                return new Course(from, 0, 0, pushing / kA);
            }
            return new Course(from, pushing / backEmf, kA / backEmf, 0);
        }

        double speedAt(double t) {
            if (seconds == 0) {
                return from + accelerating * t;
            }
            return toward + (from - toward) * StrictMath.exp(-t / seconds);
        }

        double turnedBy(double t) {
            if (seconds == 0) {
                return from * t + accelerating * t * t / 2;
            }
            return toward * t + (from - toward) * seconds * (1 - StrictMath.exp(-t / seconds));
        }

        double stopsAfter() {
            if (seconds == 0) {
                return from * accelerating < 0 ? -from / accelerating : Double.POSITIVE_INFINITY;
            }
            return from * toward < 0 ? seconds * StrictMath.log((from - toward) / -toward) : Double.POSITIVE_INFINITY;
        }
    }

    public Count count() {
        return new Count(ticks, ticksPerSecond);
    }
}
