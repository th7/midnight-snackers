package org.firstinspires.ftc.teamcode.sim;

import java.util.concurrent.atomic.AtomicLong;

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
        advance(seconds);
    }

    public FakeClock advance(double seconds) {
        long by = (long) (seconds * 1e9);
        nanos.addAndGet(by);
        epochMillis.addAndGet(by / 1_000_000L);
        return this;
    }
}
