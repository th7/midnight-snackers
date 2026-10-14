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

    private final double stiffness;
    private final double restitution;
    private final double friction;
    private final double regularisation;
    private final int rings;
    private final double dampingRatio;

    private Compliance(double stiffness, double restitution, double friction, double regularisation, int rings) {
        this.stiffness = stiffness;
        this.restitution = restitution;
        this.friction = friction;
        this.regularisation = regularisation;
        this.rings = rings;
        double lost = StrictMath.abs(StrictMath.log(restitution));
        this.dampingRatio = lost / StrictMath.sqrt(StrictMath.PI * StrictMath.PI + lost * lost);
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
        return Checked.ok(
                new Compliance(newtonsPerMillimetre, restitution, friction, regularisationMetresPerSecond, 1));
    }

    Checked<Compliance> sharedBy(int rings) {
        if (rings < 1) {
            return Checked.rejected("a compliant surface is shared by at least one ring, not " + rings);
        }
        return Checked.ok(new Compliance(stiffness, restitution, friction, regularisation, rings));
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
