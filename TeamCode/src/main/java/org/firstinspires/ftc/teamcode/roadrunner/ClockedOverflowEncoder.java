package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.function.LongSupplier;

public final class ClockedOverflowEncoder implements Encoder {
    private static final int WRAP = 65536;

    private static final int WRAP_STEP = 5 * WRAP;

    public final RawEncoder encoder;

    private final LongSupplier clock;

    private final double[] recent = new double[3];

    private int next = 0;
    private int lastPosition;
    private long lastReadNanos;
    private double estimate = 0;

    public ClockedOverflowEncoder(RawEncoder encoder, LongSupplier clock) {
        this.encoder = encoder;
        this.clock = clock;
        this.lastPosition = encoder.getPositionAndVelocity().position;
        this.lastReadNanos = clock.getAsLong();
    }

    @Override
    public PositionVelocityPair getPositionAndVelocity() {
        PositionVelocityPair reading = encoder.getPositionAndVelocity();
        long now = clock.getAsLong();
        long elapsedNanos = now - lastReadNanos;

        if (elapsedNanos > 0) {
            estimate = median((reading.position - lastPosition) / (elapsedNanos * 1e-9));
            lastPosition = reading.position;
            lastReadNanos = now;
        }
        return new PositionVelocityPair(
                reading.position,
                reading.velocity == null ? null : unwrapped(reading.velocity, estimate),
                reading.rawPosition,
                reading.rawVelocity);
    }

    @Override
    public DcMotorSimple.Direction getDirection() {
        return encoder.getDirection();
    }

    @Override
    public void setDirection(DcMotorSimple.Direction direction) {
        encoder.setDirection(direction);
    }

    private static int unwrapped(int velocity, double estimate) {
        int real = velocity & 0xFFFF;
        real += ((real % 20) / 4) * WRAP;
        real += (int) Math.rint((estimate - real) / WRAP_STEP) * WRAP_STEP;
        return real;
    }

    private double median(double speed) {
        recent[next] = speed;
        next = (next + 1) % recent.length;
        return Math.max(Math.min(recent[0], recent[1]), Math.min(Math.max(recent[0], recent[1]), recent[2]));
    }
}
