package org.firstinspires.ftc.teamcode.simcore;

final class Compliance {
    static final String STIFFNESS_SOURCE = "doc/launch-sim.md takes the pollen as an indoor 26-hole pickleball, and"
            + " Pickleball Science's force-gauge tests put one between two plates at 16.4-22.8 N/mm: two contacts in"
            + " series, so 33-46 N/mm a contact";
    static final double STIFFNESS_N_PER_MM = 40;
    static final String RESTITUTION_SOURCE = "a pickleball on stone comes back at about 0.65 (Pickleball Science)";
    static final double RESTITUTION = 0.6;
    static final String FRICTION_SOURCE = "a rubber-like tread on the wheel and printed plastic on the hood, assumed:"
            + " in a model of this launcher the throw moved 0.2 % over 0.3-1.0 on the wheel and 0.4 % over 0.15-0.5"
            + " on the hood";
    static final double WHEEL_FRICTION = 0.6;
    static final double HOOD_FRICTION = 0.3;
    static final String REGULARISATION_SOURCE =
            "in a model of this launcher the launch is the same from 0.001 to 0.3" + " m/s";
    static final double REGULARISATION_M_PER_S = 0.02;
    static final double MILLIMETRES_PER_METRE = 1000;
    static final String DAMPING_SOURCE = "a linear spring and dashpot that never pulls lets go once its force reaches"
            + " nothing, before its depth does, and so returns a ball at exp(-2 z acos(z) / sqrt(1 - z^2)) for a"
            + " damping ratio z under 1, exp(-2) at 1 and exp(-2 z acosh(z) / sqrt(z^2 - 1)) above it (Schwager and"
            + " Poeschel, Granular Matter 9, 465, 2007), which falls as z grows";
    static final int MOST_HALVINGS = 1100;

    private final double stiffness;
    private final double restitution;
    private final double friction;
    private final double regularisation;
    private final int rings;
    private final double dampingRatio;

    private Compliance(
            double stiffness,
            double restitution,
            double friction,
            double regularisation,
            int rings,
            double dampingRatio) {
        this.stiffness = stiffness;
        this.restitution = restitution;
        this.friction = friction;
        this.regularisation = regularisation;
        this.rings = rings;
        this.dampingRatio = dampingRatio;
    }

    private static double dampingRatioFor(double restitution) {
        double lost = -StrictMath.log(restitution) / 2;
        double low = 0;
        double high = 1 + 1 / StrictMath.sqrt(restitution);
        double middle = low + (high - low) / 2;
        for (int halvings = 0; halvings < MOST_HALVINGS && low < middle && middle < high; halvings++) {
            if (halfTheLogLostAt(middle) < lost) {
                low = middle;
            } else {
                high = middle;
            }
            middle = low + (high - low) / 2;
        }
        return low;
    }

    private static double halfTheLogLostAt(double ratio) {
        if (ratio < 1) {
            return ratio * StrictMath.acos(ratio) / StrictMath.sqrt((1 - ratio) * (1 + ratio));
        }
        if (ratio > 1) {
            double over = StrictMath.sqrt(1 - 1 / (ratio * ratio));
            return (StrictMath.log(ratio) + StrictMath.log1p(over)) / over;
        }
        return 1;
    }

    static Checked<Compliance> of(
            double newtonsPerMillimetre, double restitution, double friction, double regularisationMetresPerSecond) {
        if (!(newtonsPerMillimetre > 0 && Double.isFinite(newtonsPerMillimetre))) {
            return Checked.rejected("a compliant surface's stiffness is a positive, finite number of newtons a"
                    + " millimetre, not " + newtonsPerMillimetre);
        }
        if (!(restitution > 0 && restitution <= 1)) {
            return Checked.rejected(
                    "a compliant surface's restitution is more than 0 and at most 1, not " + restitution);
        }
        if (!(friction >= 0 && Double.isFinite(friction))) {
            return Checked.rejected(
                    "a compliant surface's friction coefficient is a finite number not below 0, not " + friction);
        }
        if (!(regularisationMetresPerSecond > 0 && Double.isFinite(regularisationMetresPerSecond))) {
            return Checked.rejected("a compliant surface's regularisation speed is a positive, finite number of"
                    + " metres a second, not " + regularisationMetresPerSecond);
        }
        return Checked.ok(new Compliance(
                newtonsPerMillimetre,
                restitution,
                friction,
                regularisationMetresPerSecond,
                1,
                dampingRatioFor(restitution)));
    }

    Checked<Compliance> sharedBy(int rings) {
        if (rings < 1) {
            return Checked.rejected("a compliant surface is shared by at least one ring, not " + rings);
        }
        return Checked.ok(new Compliance(stiffness, restitution, friction, regularisation, rings, dampingRatio));
    }

    double newtonsPerMetre() {
        return stiffness * MILLIMETRES_PER_METRE / rings;
    }

    double dampingFor(Ball ball) {
        return 2 * dampingRatio * StrictMath.sqrt(stiffness * MILLIMETRES_PER_METRE * ball.kilograms()) / rings;
    }

    double friction() {
        return friction;
    }

    double regularisation() {
        return regularisation;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof Compliance that
                && Double.compare(stiffness, that.stiffness) == 0
                && Double.compare(restitution, that.restitution) == 0
                && Double.compare(friction, that.friction) == 0
                && Double.compare(regularisation, that.regularisation) == 0
                && rings == that.rings;
    }

    @Override
    public int hashCode() {
        int hash = Double.hashCode(stiffness);
        hash = hash * 31 + Double.hashCode(restitution);
        hash = hash * 31 + Double.hashCode(friction);
        hash = hash * 31 + Double.hashCode(regularisation);
        return hash * 31 + rings;
    }

    @Override
    public String toString() {
        return stiffness + " N/mm shared by " + rings + ", restitution " + restitution + ", friction " + friction
                + ", regularised over " + regularisation + " m/s";
    }
}
