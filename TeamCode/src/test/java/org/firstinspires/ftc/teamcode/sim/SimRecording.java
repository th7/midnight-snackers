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

        private Tick(Builder building) {
            this.seconds = building.seconds;
            this.truePose = building.truePose;
            this.step = building.step;
            this.wheelPowers = building.wheelPowers;
            this.packets = building.packets;
            this.gamepad1 = building.gamepad1;
            this.gamepad2 = building.gamepad2;
            this.pieces = building.pieces;
            this.held = building.held;
            this.scored = building.scored;
            this.tilt = building.tilt;
        }

        public static Builder at(
                double seconds, Pose2d truePose, String step, double[] wheelPowers, List<TelemetryPacket> packets) {
            return new Builder(seconds, truePose, step, wheelPowers, packets);
        }

        public static final class Builder {
            private final double seconds;
            private final Pose2d truePose;
            private final String step;
            private final double[] wheelPowers;
            private final List<TelemetryPacket> packets;
            private State gamepad1;
            private State gamepad2;
            private double[][] pieces;
            private int held;
            private Map<String, Integer> scored = Map.of();
            private Map<String, Double> tilt = Map.of();

            private Builder(
                    double seconds, Pose2d truePose, String step, double[] wheelPowers, List<TelemetryPacket> packets) {
                this.seconds = seconds;
                this.truePose = truePose;
                this.step = step;
                this.wheelPowers = wheelPowers;
                this.packets = packets;
            }

            public Builder drivenBy(State gamepad1, State gamepad2) {
                this.gamepad1 = gamepad1;
                this.gamepad2 = gamepad2;
                return this;
            }

            public Builder withBalls(double[][] pieces, int held) {
                this.pieces = pieces;
                this.held = held;
                return this;
            }

            public Builder scoring(Map<String, Integer> scored) {
                this.scored = scored;
                return this;
            }

            public Builder tilted(Map<String, Double> tilt) {
                this.tilt = tilt;
                return this;
            }

            public Tick tick() {
                return new Tick(this);
            }
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
