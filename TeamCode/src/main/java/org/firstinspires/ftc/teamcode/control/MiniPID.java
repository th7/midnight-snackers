package org.firstinspires.ftc.teamcode.control;

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

        if (firstRun) {
            lastActual = actual;
            firstRun = false;
        }

        double derivative = -d * (actual - lastActual);
        lastActual = actual;

        double output = p * error + i * errorSum + derivative;
        errorSum += error;
        return output;
    }
}
