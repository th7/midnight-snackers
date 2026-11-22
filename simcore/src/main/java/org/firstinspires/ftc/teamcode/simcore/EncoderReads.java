package org.firstinspires.ftc.teamcode.simcore;

public enum EncoderReads {
    OWN_MOTOR("its own motor"),
    PARALLEL_DEAD_WHEEL("the parallel dead wheel"),
    PERPENDICULAR_DEAD_WHEEL("the perpendicular dead wheel"),
    NOTHING("nothing");

    private final String label;

    EncoderReads(String label) {
        this.label = label;
    }

    public boolean deadWheel() {
        return this == PARALLEL_DEAD_WHEEL || this == PERPENDICULAR_DEAD_WHEEL;
    }

    public Checked<Count> count(Checked<Count> ownMotor, DeadWheels.Counts deadWheels) {
        return switch (this) {
            case OWN_MOTOR -> ownMotor;
            case PARALLEL_DEAD_WHEEL -> Checked.ok(deadWheels.par());
            case PERPENDICULAR_DEAD_WHEEL -> Checked.ok(deadWheels.perp());
            case NOTHING -> Checked.ok(Count.none());
        };
    }

    @Override
    public String toString() {
        return label;
    }
}
