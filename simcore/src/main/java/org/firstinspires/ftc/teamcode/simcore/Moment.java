package org.firstinspires.ftc.teamcode.simcore;

/**
 * A reading of a clock that only goes forward, in nanoseconds from an origin of the clock's own. Every
 * long is one: readings are compared by their difference, which stays right across a wrap.
 */
public record Moment(long nanos) {
    /** How long after earlier this is, in seconds: below zero when it came first. */
    public double secondsSince(Moment earlier) {
        return (nanos - earlier.nanos) / 1e9;
    }
}
