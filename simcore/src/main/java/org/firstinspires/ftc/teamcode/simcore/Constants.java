package org.firstinspires.ftc.teamcode.simcore;

import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class Constants {
    private final EnumMap<Constant, Double> values;

    private Constants(EnumMap<Constant, Double> values) {
        this.values = values;
    }

    public static Constants defaults() {
        EnumMap<Constant, Double> values = new EnumMap<>(Constant.class);
        for (Constant constant : Constant.values()) {
            values.put(constant, constant.byDefault());
        }
        return new Constants(values);
    }

    public static Checked<Constants> of(Map<Constant, Double> set) {
        EnumMap<Constant, Double> values = new EnumMap<>(Constant.class);
        for (Constant constant : Constant.values()) {
            double value = set.getOrDefault(constant, constant.byDefault());
            Checked<Double> admitted = constant.admits(value);
            if (admitted instanceof Checked.Rejected<Double> rejected) {
                return Checked.rejected(rejected.rule());
            }
            values.put(constant, value);
        }
        Constants constants = new Constants(values);
        for (Ordered pair : ordered()) {
            double lower = constants.value(pair.lower());
            double upper = constants.value(pair.upper());
            if (lower > upper) {
                return Checked.rejected(pair.lower().label()
                        + " may not be more than " + pair.upper().label().toLowerCase(Locale.ROOT)
                        + ", not " + lower + " " + pair.lower().unit()
                        + " against " + upper + " " + pair.upper().unit());
            }
        }
        return Checked.ok(constants);
    }

    private record Ordered(Constant lower, Constant upper) {}

    private static List<Ordered> ordered() {
        return List.of(
                new Ordered(Constant.FLATTEST_VOLTS, Constant.FRESHEST_VOLTS),
                new Ordered(Constant.LEAST_TRACTION_G, Constant.MOST_TRACTION_G),
                new Ordered(Constant.LEAST_LOOP_SECONDS, Constant.LOOP_SECONDS),
                new Ordered(Constant.LOOP_SECONDS, Constant.SHORTEST_HICCUP_SECONDS),
                new Ordered(Constant.SHORTEST_HICCUP_SECONDS, Constant.LONGEST_HICCUP_SECONDS));
    }

    public double value(Constant constant) {
        return values.getOrDefault(constant, constant.byDefault());
    }

    public Map<Constant, Double> changed() {
        EnumMap<Constant, Double> changed = new EnumMap<>(Constant.class);
        for (Constant constant : Constant.values()) {
            double value = value(constant);
            if (Double.compare(value, constant.byDefault()) != 0) {
                changed.put(constant, value);
            }
        }
        return changed;
    }

    public boolean asBuilt() {
        return changed().isEmpty();
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Constants that && values.equals(that.values);
    }

    @Override
    public int hashCode() {
        return values.hashCode();
    }

    @Override
    public String toString() {
        return asBuilt() ? "as built" : changed().toString();
    }
}
