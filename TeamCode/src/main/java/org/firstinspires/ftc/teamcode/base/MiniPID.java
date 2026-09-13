package org.firstinspires.ftc.teamcode.base;

/**
 * A plain PID controller: proportional on the error, integral on the accumulated error, derivative
 * on the change in the measurement. Trimmed from Tekdemo's MiniPID to the parts {@link FastDrive}
 * uses: there are no output limits, ramp rate, filter, or feed-forward.
 * <p>
 * Usage: {@code output = pid.getOutput(actual, setpoint)} once per loop.
 */
public class MiniPID {
    private final double p;
    private final double i;
    private final double d;
    private double errorSum = 0;
    private double lastActual = 0;
    private boolean firstRun = true;

    public MiniPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getOutput(double actual, double setpoint) {
        double error = setpoint - actual;

        // On the first run there is no previous measurement; assume it was where it is now.
        if (firstRun) {
            lastActual = actual;
            firstRun = false;
        }

        // The derivative is on the measurement, not the error, so a setpoint change does not spike it.
        double derivative = -d * (actual - lastActual);
        lastActual = actual;

        double output = p * error + i * errorSum + derivative;
        errorSum += error;
        return output;
    }
}
