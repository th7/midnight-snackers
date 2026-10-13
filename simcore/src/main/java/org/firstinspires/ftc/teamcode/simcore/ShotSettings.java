package org.firstinspires.ftc.teamcode.simcore;

import java.util.EnumMap;
import java.util.Map;

public final class ShotSettings {
    private final EnumMap<ShotSetting, Double> values;

    private ShotSettings(EnumMap<ShotSetting, Double> values) {
        this.values = values;
    }

    public static ShotSettings defaults() {
        EnumMap<ShotSetting, Double> values = new EnumMap<>(ShotSetting.class);
        for (ShotSetting setting : ShotSetting.values()) {
            values.put(setting, setting.byDefault());
        }
        return new ShotSettings(values);
    }

    public static Checked<ShotSettings> of(Map<ShotSetting, Double> given) {
        EnumMap<ShotSetting, Double> values = new EnumMap<>(ShotSetting.class);
        for (ShotSetting setting : ShotSetting.values()) {
            Checked<Double> admitted = setting.admits(givenOrDefault(given, setting));
            if (admitted instanceof Checked.Rejected<Double> rejected) {
                return Checked.rejected(rejected.rule());
            }
            values.put(setting, admitted.orElse(setting.byDefault()));
        }
        return Checked.ok(new ShotSettings(values));
    }

    private static double givenOrDefault(Map<ShotSetting, Double> given, ShotSetting setting) {
        Double value = given.get(setting);
        return value == null ? setting.byDefault() : value;
    }

    public double value(ShotSetting setting) {
        return values.getOrDefault(setting, setting.byDefault());
    }

    public boolean on(ShotSetting setting) {
        return value(setting) != 0;
    }

    public Map<ShotSetting, Double> changed() {
        EnumMap<ShotSetting, Double> changed = new EnumMap<>(ShotSetting.class);
        for (ShotSetting setting : ShotSetting.values()) {
            double value = value(setting);
            if (Double.compare(value, setting.byDefault()) != 0) {
                changed.put(setting, value);
            }
        }
        return changed;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ShotSettings that && values.equals(that.values);
    }

    @Override
    public int hashCode() {
        return values.hashCode();
    }

    @Override
    public String toString() {
        Map<ShotSetting, Double> changed = changed();
        return changed.isEmpty() ? "the defaults" : changed.toString();
    }
}
