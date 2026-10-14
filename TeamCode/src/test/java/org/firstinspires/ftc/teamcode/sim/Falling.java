package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.simcore.Flight;

final class Falling {
    static final double BOUNCES_FOR_EACH_IN_PER_S_OF_ITS_FIRST_IMPACT = 1 / 129.0;
    static final double ROLLS_OUT_AND_LANDS_SECONDS = 0.5;
    static final double ROLLS_THE_LENGTH_OF_A_CELL_SECONDS = 0.5;

    private Falling() {}

    static double settledAfter(double inches) {
        double g = Flight.GRAVITY_IN_PER_S2;
        return Math.sqrt(2 * inches / g)
                + Math.sqrt(2 * g * inches) * BOUNCES_FOR_EACH_IN_PER_S_OF_ITS_FIRST_IMPACT
                + ROLLS_OUT_AND_LANDS_SECONDS;
    }
}
