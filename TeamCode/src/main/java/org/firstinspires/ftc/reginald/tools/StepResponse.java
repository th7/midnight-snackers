package org.firstinspires.ftc.reginald.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

public final class StepResponse {
    public record Sample(double seconds, double value) {}

    private final double target;
    private final List<Sample> samples = new ArrayList<>();

    public StepResponse(double target) {
        this.target = target;
    }

    public void add(double seconds, double value) {
        samples.add(new Sample(seconds, value));
    }

    public Optional<Sample> peak() {
        Sample peak = null;
        for (Sample sample : samples) {
            if (peak == null || Math.abs(sample.value) > Math.abs(peak.value)) {
                peak = sample;
            }
        }
        return Optional.ofNullable(peak);
    }

    public OptionalDouble settledWithin(double fraction) {
        double band = Math.abs(target) * fraction;
        OptionalDouble settled = OptionalDouble.empty();
        for (Sample sample : samples) {
            boolean inside = Math.abs(sample.value - target) <= band;
            if (!inside) {
                settled = OptionalDouble.empty();
            } else if (settled.isEmpty()) {
                settled = OptionalDouble.of(sample.seconds);
            }
        }
        return settled;
    }
}
