package org.firstinspires.ftc.teamcode.simcore;

public final class ServoPort {
    private static final int FIRST_PULSE_US = 1;
    private static final int LAST_PULSE_US = 65535;

    private final Written written;
    private final boolean enabled;

    private ServoPort(Written written, boolean enabled) {
        this.written = written;
        this.enabled = enabled;
    }

    public static ServoPort unwritten() {
        return new ServoPort(new Unwritten(), true);
    }

    public Checked<ServoPort> pulsed(double microseconds) {
        int sent = (int) microseconds;
        if (sent < FIRST_PULSE_US || sent > LAST_PULSE_US) {
            return Checked.rejected("illegal pulse width: " + sent);
        }
        return Checked.ok(new ServoPort(new Pulsed(sent), true));
    }

    public Checked<ServoPort> positioned(double position, PulseRange range) {
        if (!(position >= 0 && position <= 1)) {
            return Checked.rejected("a servo position is from 0 to 1, not " + position);
        }
        if (written instanceof Positioned already && Double.compare(already.position(), position) == 0) {
            return Checked.ok(this);
        }
        double scaled = scale(position, 0, 1, range.lowerUs(), range.upperUs());
        int sent = (int) clip(scaled, FIRST_PULSE_US, LAST_PULSE_US);
        return Checked.ok(new ServoPort(new Positioned(position, sent), true));
    }

    public ServoPort forgotten() {
        if (written instanceof Positioned positioned) {
            return new ServoPort(new Pulsed(positioned.pulseWidth()), enabled);
        }
        return this;
    }

    public ServoPort enabled(boolean enabled) {
        return new ServoPort(written, enabled);
    }

    public boolean enabled() {
        return enabled;
    }

    public PositionRead read(PulseRange range) {
        if (written instanceof Positioned positioned) {
            return new PositionRead(positioned.position(), this);
        }
        if (written instanceof Pulsed pulsed) {
            double position = clip(scale(pulsed.pulseWidth(), range.lowerUs(), range.upperUs(), 0, 1), 0, 1);
            return new PositionRead(position, new ServoPort(new Positioned(position, pulsed.pulseWidth()), enabled));
        }
        return new PositionRead(0, this);
    }

    public double pulseWidth() {
        return written.pulseWidth();
    }

    public ServoSignal signal() {
        return enabled && written.pulseWidth() != 0 ? new ServoSignal.Pulse(written.pulseWidth()) : ServoSignal.none();
    }

    private static double scale(double n, double x1, double x2, double y1, double y2) {
        double a = (y1 - y2) / (x1 - x2);
        double b = y1 - x1 * (y1 - y2) / (x1 - x2);
        return a * n + b;
    }

    private static double clip(double n, double min, double max) {
        if (n < min) {
            return min;
        }
        if (n > max) {
            return max;
        }
        return n;
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ServoPort that && written.equals(that.written) && enabled == that.enabled;
    }

    @Override
    public int hashCode() {
        return 31 * written.hashCode() + Boolean.hashCode(enabled);
    }

    @Override
    public String toString() {
        return written + (enabled ? "" : ", disabled");
    }

    public record PositionRead(double position, ServoPort port) {}

    private sealed interface Written permits Unwritten, Positioned, Pulsed {
        int pulseWidth();
    }

    private record Unwritten() implements Written {
        @Override
        public int pulseWidth() {
            return 0;
        }
    }

    private record Positioned(double position, int pulseWidth) implements Written {}

    private record Pulsed(int pulseWidth) implements Written {}
}
