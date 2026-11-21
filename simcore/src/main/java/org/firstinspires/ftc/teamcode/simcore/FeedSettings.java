package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public final class FeedSettings {
    private final EnumMap<FeedSetting, Double> values;

    private FeedSettings(EnumMap<FeedSetting, Double> values) {
        this.values = values;
    }

    public static FeedSettings defaults() {
        EnumMap<FeedSetting, Double> values = new EnumMap<>(FeedSetting.class);
        for (FeedSetting setting : FeedSetting.values()) {
            values.put(setting, setting.byDefault());
        }
        return new FeedSettings(values);
    }

    public static Checked<FeedSettings> of(Map<FeedSetting, Double> given) {
        EnumMap<FeedSetting, Double> values = new EnumMap<>(FeedSetting.class);
        for (FeedSetting setting : FeedSetting.values()) {
            Double asked = given.get(setting);
            Checked<Double> admitted = setting.admits(asked == null ? setting.byDefault() : asked);
            if (admitted instanceof Checked.Rejected<Double> rejected) {
                return Checked.rejected(rejected.rule());
            }
            values.put(setting, admitted.orElse(setting.byDefault()));
        }
        FeedSettings settings = new FeedSettings(values);
        return settings.ramp().then(ramp -> settings.withinTheRun());
    }

    private Checked<FeedSettings> withinTheRun() {
        double run = value(FeedSetting.RUN);
        double lowered = value(FeedSetting.RAMP_LOWERED_AT);
        if (!(lowered <= run)) {
            return Checked.rejected("a ramp is lowered within the run of " + run + " s, not at " + lowered + " s");
        }
        return Checked.ok(this);
    }

    private Checked<Feed.RampSchedule> ramp() {
        return Feeder.RampAngle.ofDegrees(value(FeedSetting.RAMP_DOWN))
                .then(down -> Feeder.RampAngle.ofDegrees(value(FeedSetting.RAMP_UP))
                        .then(up -> Seconds.of(value(FeedSetting.RAMP_RAISED_AT))
                                .then(raised -> Seconds.of(value(FeedSetting.RAMP_LOWERED_AT))
                                        .then(lowered -> Feed.RampSchedule.of(down, up, raised, lowered)))));
    }

    public Checked<Feed.Plan> plan() {
        return Feed.Plan.defaults()
                .then(plan -> Feeder.Shape.cad()
                        .then(cad -> Feeder.Window.of(Feeder.Window.BACK_HALF_WIDTH_MM, value(FeedSetting.BACK_SILL))
                                .then(back -> cad.withBackWindow(back)))
                        .map(shape -> plan.withShape(shape)))
                .then(plan -> Feeder.Spinner.of(value(FeedSetting.SPINNER_SPEED), value(FeedSetting.SPINNER_STALL))
                        .map(spinner -> plan.withSpinner(spinner)))
                .then(plan -> Power.of(value(FeedSetting.SPINNER_POWER)).map(power -> plan.withSpinnerPower(power)))
                .then(plan ->
                        Feeder.Ramp.of(value(FeedSetting.RAMP_SERVO_SPEED)).map(servo -> plan.withRampServo(servo)))
                .then(plan -> ramp().map(ramp -> plan.withRamp(ramp)))
                .then(plan -> Flywheel.TicksPerTurn.of(Feed.FLYWHEEL_TICKS_PER_TURN)
                        .map(ticks ->
                                plan.withFlywheel(Flywheel.AngularSpeed.from(value(FeedSetting.FLYWHEEL), ticks))))
                .then(plan -> pollen().map(pollen -> plan.withPollen(pollen)))
                .then(plan -> Seconds.of(value(FeedSetting.RUN)).then(run -> plan.lasting(run)));
    }

    private Checked<List<Feed.Pollen>> pollen() {
        long count = StrictMath.round(value(FeedSetting.POLLEN));
        Checked<List<Feed.Pollen>> placed = Checked.ok(List.of());
        long placing = 0;
        for (FeedSetting where : List.of(FeedSetting.FIRST_POLLEN, FeedSetting.SECOND_POLLEN)) {
            placing++;
            if (placing <= count) {
                placed = placed.then(
                        before -> Feed.Pollen.againstTheWall(value(where)).map(one -> appended(before, one)));
            }
        }
        return placed;
    }

    private static List<Feed.Pollen> appended(List<Feed.Pollen> before, Feed.Pollen one) {
        List<Feed.Pollen> after = new ArrayList<>(before);
        after.add(one);
        return after;
    }

    public double value(FeedSetting setting) {
        return values.getOrDefault(setting, setting.byDefault());
    }

    public Map<FeedSetting, Double> changed() {
        EnumMap<FeedSetting, Double> changed = new EnumMap<>(FeedSetting.class);
        for (FeedSetting setting : FeedSetting.values()) {
            double value = value(setting);
            if (Double.compare(value, setting.byDefault()) != 0) {
                changed.put(setting, value);
            }
        }
        return changed;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof FeedSettings that && values.equals(that.values);
    }

    @Override
    public int hashCode() {
        return values.hashCode();
    }

    @Override
    public String toString() {
        Map<FeedSetting, Double> changed = changed();
        return changed.isEmpty() ? "the defaults" : changed.toString();
    }
}
