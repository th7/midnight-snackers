package org.firstinspires.ftc.teamcode.sim;

public interface Clock {

    long nanos();

    long millisSinceEpoch();

    void sleep(double seconds);
}
