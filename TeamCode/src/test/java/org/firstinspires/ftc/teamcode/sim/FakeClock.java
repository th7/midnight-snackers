package org.firstinspires.ftc.teamcode.sim;

import java.util.concurrent.atomic.AtomicLong;

/**
 * A clock whose time moves only when a test moves it. A sleep passes none of it: it gives the sleeper
 * a moment of real time, so a thread polling this clock waits for the test rather than spinning
 * through minutes of it before the thread it races has said a word.
 */
public final class FakeClock implements Clock {
    private final AtomicLong nanos = new AtomicLong();
    private final AtomicLong epochMillis = new AtomicLong(1_700_000_000_000L);

    @Override
    public long nanos() {
        return nanos.get();
    }

    @Override
    public long millisSinceEpoch() {
        return epochMillis.get();
    }

    @Override
    public void sleep(double seconds) {
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public FakeClock advance(double seconds) {
        long by = (long) (seconds * 1e9);
        nanos.addAndGet(by);
        epochMillis.addAndGet(by / 1_000_000L);
        return this;
    }
}
