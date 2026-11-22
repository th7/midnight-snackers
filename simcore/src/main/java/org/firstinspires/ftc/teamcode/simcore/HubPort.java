package org.firstinspires.ftc.teamcode.simcore;

public final class HubPort {
    public static final double MOST_TICKS_PER_SECOND = 32767;

    private HubPort() {}

    public record Wrote(MotorType type, Sense direction, RunMode mode, Written written) {}

    public record State(VelocityLoop loop, double zeroTicks) {
        public static State fresh() {
            return new State(VelocityLoop.idle(), 0);
        }
    }

    public record Driven(Power power, State next) {}

    public static Sense operational(Turning declared, Sense direction) {
        if (declared == Turning.CLOCKWISE) {
            return direction;
        }
        return direction == Sense.FORWARD ? Sense.REVERSE : Sense.FORWARD;
    }

    public static Checked<Driven> drive(Wrote wrote, State state, Count measured, Seconds dt) {
        Sense operational = operational(wrote.type().declared(), wrote.direction());
        double achievable = wrote.type().achievableTicksPerSecond();
        return switch (wrote.mode()) {
            case RUN_WITHOUT_ENCODER ->
                Checked.ok(new Driven(
                        Power.clamped(operational.of(openLoopPower(wrote.written(), achievable))),
                        new State(VelocityLoop.idle(), state.zeroTicks())));
            case RUN_USING_ENCODER -> {
                VelocityLoop.Stepped stepped = state.loop()
                        .toward(operational.of(targetTicksPerSecond(wrote.written(), achievable)), measured, dt);
                yield Checked.ok(new Driven(stepped.power(), new State(stepped.loop(), state.zeroTicks())));
            }
            case STOP_AND_RESET_ENCODER ->
                Checked.ok(new Driven(Power.none(), new State(VelocityLoop.idle(), measured.ticks())));
            case RUN_TO_POSITION ->
                Checked.rejected("the simulator does not run a motor in RUN_TO_POSITION yet: drive it with"
                        + " RUN_WITHOUT_ENCODER or RUN_USING_ENCODER, or ask for RUN_TO_POSITION in an issue");
        };
    }

    public static Encoder reports(Wrote wrote, State state, Count raw) {
        Sense operational = operational(wrote.type().declared(), wrote.direction());
        return Encoder.asTheHubReports(
                operational.of(raw.ticks() - state.zeroTicks()), operational.of(raw.ticksPerSecond()));
    }

    private static double openLoopPower(Written written, double achievable) {
        if (written instanceof Written.AtVelocity velocity) {
            return velocity.ticksPerSecond() / achievable;
        }
        return written instanceof Written.Powered powered ? powered.power() : 0;
    }

    private static double targetTicksPerSecond(Written written, double achievable) {
        if (written instanceof Written.AtVelocity velocity) {
            return StrictMath.max(
                    -MOST_TICKS_PER_SECOND,
                    StrictMath.min(MOST_TICKS_PER_SECOND, StrictMath.floor(velocity.ticksPerSecond() + 0.5)));
        }
        return written instanceof Written.Powered powered
                ? Power.clamped(powered.power()).value() * achievable
                : 0;
    }
}
