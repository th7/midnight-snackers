package org.firstinspires.ftc.teamcode.planrunner;

public class StepData {
    public final long nanoStartTime;

    public StepData(long nanoStartTime) {
        this.nanoStartTime = nanoStartTime;
    }

    public boolean secondsElapsed(double seconds) {
        return System.nanoTime() - nanoStartTime > seconds * 1_000_000_000;
    }
}
