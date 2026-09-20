package org.firstinspires.ftc.teamcode.sim;

public final class SystemClock implements Clock {
    @Override
    public long nanos() {
        return System.nanoTime();
    }

    @Override
    public long millisSinceEpoch() {
        return System.currentTimeMillis();
    }

    @Override
    public void sleep(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
