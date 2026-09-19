package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.JsonArray;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;

public final class SimRecording implements SimReplayPage.Source {
    public static final class Tick {
        public final double seconds;
        public final Pose2d truePose;

        public final String step;

        public final double[] wheelPowers;

        public final List<TelemetryPacket> packets;

        public final State gamepad1;

        public final State gamepad2;

        public final double[][] pieces;

        public final int held;

        public final Map<String, Integer> scored;

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

    public SimRecording(String name) {
        this(name, SimCatalog.AUTO);
    }

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

    public synchronized List<Tick> ticks() {
        return Collections.unmodifiableList(new ArrayList<>(ticks));
    }

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
