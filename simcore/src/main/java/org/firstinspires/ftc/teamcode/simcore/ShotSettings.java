package org.firstinspires.ftc.teamcode.simcore;

import java.util.EnumMap;
import java.util.Map;

public final class ShotSettings {
    private final Constants described;
    private final EnumMap<ShotSetting, Double> values;

    private ShotSettings(Constants described, EnumMap<ShotSetting, Double> values) {
        this.described = described;
        this.values = values;
    }

    public static ShotSettings defaults(Constants described) {
        EnumMap<ShotSetting, Double> values = new EnumMap<>(ShotSetting.class);
        for (ShotSetting setting : ShotSetting.values()) {
            values.put(setting, setting.byDefault(described));
        }
        return new ShotSettings(described, values);
    }

    public static Checked<ShotSettings> of(Map<ShotSetting, Double> given, Constants described) {
        EnumMap<ShotSetting, Double> values = new EnumMap<>(ShotSetting.class);
        for (ShotSetting setting : ShotSetting.values()) {
            double byDefault = setting.byDefault(described);
            Double asked = given.get(setting);
            Checked<Double> admitted = setting.admits(asked == null ? byDefault : asked);
            if (admitted instanceof Checked.Rejected<Double> rejected) {
                return Checked.rejected(rejected.rule());
            }
            values.put(setting, admitted.orElse(byDefault));
        }
        return Checked.ok(new ShotSettings(described, values));
    }

    public double value(ShotSetting setting) {
        return values.getOrDefault(setting, byDefault(setting));
    }

    public double byDefault(ShotSetting setting) {
        return setting.byDefault(described);
    }

    public boolean on(ShotSetting setting) {
        return value(setting) != 0;
    }

    public Map<ShotSetting, Double> changed() {
        EnumMap<ShotSetting, Double> changed = new EnumMap<>(ShotSetting.class);
        for (ShotSetting setting : ShotSetting.values()) {
            double value = value(setting);
            if (Double.compare(value, byDefault(setting)) != 0) {
                changed.put(setting, value);
            }
        }
        return changed;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ShotSettings that && described.equals(that.described) && values.equals(that.values);
    }

    @Override
    public int hashCode() {
        return 31 * described.hashCode() + values.hashCode();
    }

    @Override
    public String toString() {
        Map<ShotSetting, Double> changed = changed();
        return changed.isEmpty() ? "the defaults" : changed.toString();
    }
}
