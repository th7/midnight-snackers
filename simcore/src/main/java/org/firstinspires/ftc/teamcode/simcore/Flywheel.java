package org.firstinspires.ftc.teamcode.simcore;

public final class Flywheel {
    public static final String MOTOR = "goBILDA 5203-2402-0001, 1:1: 6,000 rpm with no load at 12 V, an encoder of 28"
            + " counts a turn of its output shaft,"
            + " https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/";
    public static final double MOTOR_RPM = 6000;
    public static final double MOTOR_TICKS_PER_TURN = 28;
    public static final double TOP_TICKS_PER_SECOND = MOTOR_RPM / 60 * MOTOR_TICKS_PER_TURN;

    private Flywheel() {}

    public static double turnsAt(double commandedTicksPerSecond) {
        if (Double.isNaN(commandedTicksPerSecond)) {
            return 0;
        }
        return StrictMath.max(-TOP_TICKS_PER_SECOND, StrictMath.min(TOP_TICKS_PER_SECOND, commandedTicksPerSecond));
    }
}
