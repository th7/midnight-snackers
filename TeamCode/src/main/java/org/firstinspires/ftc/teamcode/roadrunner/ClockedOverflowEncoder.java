package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.function.LongSupplier;

/**
 * An encoder that puts the hub's wrapped velocity count back together, on the robot's clock.
 *
 * <p>The hub reports a motor's velocity in sixteen bits, so a wheel turning quickly wraps it and
 * the true count has to be recovered. The only way to know which wrap it is on is to measure the
 * wheel a second way — how far it turned, over how long — and pick the wrap nearest that. The
 * <em>how long</em> is a clock.
 *
 * <p>Road Runner's own {@code OverflowEncoder} does exactly this and reads the wall clock for it,
 * which is right on the robot and wrong anywhere the robot's time is not the machine's. In the
 * simulator a loop is twenty milliseconds of the robot's time and a fraction of a millisecond of
 * the machine's, so the wall clock says the wheel turned that far in almost no time, the estimate
 * comes out tens of times too large, and the wrong wrap is chosen: a robot rolling to a stop
 * measures itself doing a hundred and seventy inches a second. The pose survives that, because the
 * pose is built from position and not from velocity; what does not survive is everything that asks
 * how fast the robot is going, such as whether a trajectory has finished.
 *
 * <p>So this reads the clock it is given, which on the robot is {@code System::nanoTime} and in the
 * simulator is the simulation's — the same clock every other timer in the robot code reads. The
 * arithmetic is Road Runner's, unchanged, so on the robot this measures what its encoder measures.
 */
public final class ClockedOverflowEncoder implements Encoder {
    /** What the hub's velocity count wraps at. */
    private static final int WRAP = 65536;
    /**
     * The hub reports velocity in steps of 20 ticks per second and wraps every five of its counts,
     * so the recoverable values are this far apart.
     */
    private static final int WRAP_STEP = 5 * WRAP;

    /** The encoder underneath, reading the port raw; Road Runner's tuning op modes want this one. */
    public final RawEncoder encoder;

    private final LongSupplier clock;
    /** The last three of our own speed estimates, for the median that resists a single bad read. */
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
        // No time has passed, so there is nothing new to measure and the last estimate still
        // stands. Dividing here would be a distance over no time, which is not a speed.
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

    /**
     * The hub's wrapped count as the whole number it stands for: the wrap nearest {@code estimate}.
     * Road Runner's arithmetic, kept as it is so that this encoder and its own agree on a robot.
     */
    private static int unwrapped(int velocity, double estimate) {
        int real = velocity & 0xFFFF;
        real += ((real % 20) / 4) * WRAP;
        real += (int) Math.rint((estimate - real) / WRAP_STEP) * WRAP_STEP;
        return real;
    }

    /** The middle of the last three estimates, this one included. */
    private double median(double speed) {
        recent[next] = speed;
        next = (next + 1) % recent.length;
        return Math.max(Math.min(recent[0], recent[1]), Math.min(Math.max(recent[0], recent[1]), recent[2]));
    }
}
