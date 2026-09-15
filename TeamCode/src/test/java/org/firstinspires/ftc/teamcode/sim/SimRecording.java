package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.JsonArray;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;

/**
 * Everything observed during one simulated run, one entry per op mode loop: the true pose, the
 * plan's current step, the drive powers, the dashboard packets the robot code drew that loop, where
 * the loose game pieces are, and for a TeleOp what the driver's gamepads read. Safe to read from another thread (the live view)
 * while the run is still adding to it. A source for the replay page, in this JVM.
 */
public final class SimRecording implements SimReplayPage.Source {
    public static final class Tick {
        public final double seconds;
        public final Pose2d truePose;
        /** The plan's current step; empty for a TeleOp, which has no plan. */
        public final String step;
        /** leftFront, rightFront, leftBack, rightBack */
        public final double[] wheelPowers;

        public final List<TelemetryPacket> packets;
        /** What gamepad 1 read this loop; null for an auto, which is not driven. */
        public final State gamepad1;
        /** What gamepad 2 read this loop; null for an auto. */
        public final State gamepad2;
        /**
         * Where the balls are, {x, y, z} each: the field's loose game pieces in
         * {@link SimField#loosePieces}' order, then the nectar the hives are set up with in
         * {@link SimField#cellPieces}' order, then the pollen the flowers are set up with in
         * {@link SimField#flowerPieces}' order, then the robot's preload; a ball held in the robot
         * is null. Null as a whole when the tick does not say, and they are where the field was
         * set up.
         */
        public final double[][] pieces;
        /** How many balls the robot holds. */
        public final int held;
        /** How many balls each alliance has in its hive, by "Blue" and "Red"; absent means none. */
        public final Map<String, Integer> scored;
        /**
         * How far each alliance's hive leans now, in degrees, by "Blue" and "Red"; empty means
         * every hive leans the way the field was set up.
         */
        public final Map<String, Double> tilt;

        public Tick(double seconds, Pose2d truePose, String step, double[] wheelPowers, List<TelemetryPacket> packets) {
            this(seconds, truePose, step, wheelPowers, packets, null, null);
        }

        public Tick(
                double seconds,
                Pose2d truePose,
                String step,
                double[] wheelPowers,
                List<TelemetryPacket> packets,
                State gamepad1,
                State gamepad2) {
            this(seconds, truePose, step, wheelPowers, packets, gamepad1, gamepad2, null);
        }

        public Tick(
                double seconds,
                Pose2d truePose,
                String step,
                double[] wheelPowers,
                List<TelemetryPacket> packets,
                State gamepad1,
                State gamepad2,
                double[][] pieces) {
            this(seconds, truePose, step, wheelPowers, packets, gamepad1, gamepad2, pieces, 0, Map.of());
        }

        public Tick(
                double seconds,
                Pose2d truePose,
                String step,
                double[] wheelPowers,
                List<TelemetryPacket> packets,
                State gamepad1,
                State gamepad2,
                double[][] pieces,
                int held,
                Map<String, Integer> scored) {
            this(seconds, truePose, step, wheelPowers, packets, gamepad1, gamepad2, pieces, held, scored, Map.of());
        }

        public Tick(
                double seconds,
                Pose2d truePose,
                String step,
                double[] wheelPowers,
                List<TelemetryPacket> packets,
                State gamepad1,
                State gamepad2,
                double[][] pieces,
                int held,
                Map<String, Integer> scored,
                Map<String, Double> tilt) {
            this.seconds = seconds;
            this.truePose = truePose;
            this.step = step;
            this.wheelPowers = wheelPowers;
            this.packets = packets;
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
            this.pieces = pieces;
            this.held = held;
            this.scored = scored;
            this.tilt = tilt;
        }
    }

    private final String name;
    private final String kind;
    private final List<Tick> ticks = new ArrayList<>();
    private String outcome = null;

    /** A recording of an auto. */
    public SimRecording(String name) {
        this(name, SimCatalog.AUTO);
    }

    /**
     * @param kind {@link SimCatalog#AUTO} or {@link SimCatalog#TELEOP}
     */
    public SimRecording(String name, String kind) {
        this.name = name;
        this.kind = kind;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public String kind() {
        return kind;
    }

    @Override
    public JsonArray ticksJson(int from) {
        return SimRunStream.ticksJson(ticksFrom(from));
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
     * How the run ended, one of {@link SimRunStream.Outcome}'s. The first outcome stands: a
     * later call changes nothing.
     */
    public synchronized void finish(String outcome) {
        if (this.outcome == null) {
            this.outcome = outcome;
        }
    }

    public synchronized boolean finished() {
        return outcome != null;
    }

    @Override
    public synchronized String outcome() {
        return outcome;
    }
}
