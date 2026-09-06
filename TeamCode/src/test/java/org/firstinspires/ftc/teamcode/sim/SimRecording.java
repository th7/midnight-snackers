package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Everything observed during one simulated run, one entry per op mode loop: the true pose, the
 * plan's current step, the drive powers, and the dashboard packets the robot code drew that loop.
 * Safe to read from another thread (the live view) while the run is still adding to it.
 */
public final class SimRecording {
    public static final class Tick {
        public final double seconds;
        public final Pose2d truePose;
        public final String step;
        /** leftFront, rightFront, leftBack, rightBack */
        public final double[] wheelPowers;
        public final List<TelemetryPacket> packets;

        public Tick(double seconds, Pose2d truePose, String step, double[] wheelPowers, List<TelemetryPacket> packets) {
            this.seconds = seconds;
            this.truePose = truePose;
            this.step = step;
            this.wheelPowers = wheelPowers;
            this.packets = packets;
        }
    }

    private final String name;
    private final List<Tick> ticks = new ArrayList<>();
    private String outcome = null;

    public SimRecording(String name) {
        this.name = name;
    }

    public String name() {
        return name;
    }

    public synchronized void add(Tick tick) {
        ticks.add(tick);
    }

    /**
     * A snapshot of every tick so far.
     */
    public synchronized List<Tick> ticks() {
        return Collections.unmodifiableList(new ArrayList<>(ticks));
    }

    /**
     * A snapshot of the ticks from index {@code from} onward.
     */
    public synchronized List<Tick> ticksFrom(int from) {
        return Collections.unmodifiableList(new ArrayList<>(ticks.subList(Math.min(from, ticks.size()), ticks.size())));
    }

    public List<Pose2d> poses() {
        List<Tick> snapshot = ticks();
        List<Pose2d> poses = new ArrayList<>(snapshot.size());
        for (Tick tick : snapshot) {
            poses.add(tick.truePose);
        }
        return poses;
    }

    /**
     * How the run ended, e.g. "done" or "timed out after 40.0s".
     */
    public synchronized void finish(String outcome) {
        this.outcome = outcome;
    }

    public synchronized boolean finished() {
        return outcome != null;
    }

    public synchronized String outcome() {
        return outcome;
    }
}
