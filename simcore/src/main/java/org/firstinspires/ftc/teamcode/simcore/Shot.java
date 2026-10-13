package org.firstinspires.ftc.teamcode.simcore;

import java.util.List;
import java.util.Optional;

public sealed interface Shot permits Shot.Flying, Shot.Finished {
    double REST_SECONDS = 0.25;
    double REST_SPEED_IN_PER_S = 0.5;
    double FRAME_SECONDS = 0.004;
    double TOUCHING_IN = 1e-6;
    double MOST_TRAVEL_IN = 0.1;
    int MOST_PIECES_PER_STEP = 1000;
    int MOST_EVENTS_PER_PIECE = 16;
    int BISECTIONS = 48;
    int PROJECTIONS = 4;
    long MOST_WORK_PER_SECOND = 3_000_000L;
    double LEAST_SECONDS_OF_WORK = 0.1;
    String LAUNCHED = "the launched pollen";

    Story story();

    enum Phase {
        FLYING,
        SLIDING,
        ROLLING,
        RESTING
    }

    record Moving(Vec3 at, Vec3 velocity, Vec3 spin, Turn turn, Phase phase) {}

    record Frame(Seconds t, Moving pollen) {}

    record Placed(Seconds t, Vec3 at, Turn turn) {}

    sealed interface Fate permits Fate.OnTheField, Fate.Gone {
        record OnTheField() implements Fate {}

        record Gone(Seconds t, Colliders.Side side) implements Fate {}
    }

    record Track(World.Piece piece, List<Placed> samples, Fate fate) {}

    record Impact(
            Seconds t,
            Vec3 at,
            Vec3 normal,
            Contacts.Hit hit,
            Vec3 velocityBefore,
            Vec3 spinBefore,
            Vec3 velocityAfter,
            Vec3 spinAfter) {}

    record Crossed(Seconds t, Colliders.Crossing crossing, double speed) {}

    record FirstBounce(Seconds t, Vec3 at, double range) {}

    record Aim(String cell, String alliance, boolean upturned, double yawDegrees, double distance, double height) {}

    record Preview(
            WheelPair.Launched launched,
            WheelPair.SpinKind kind,
            double spinRpm,
            double spinParameter,
            double ticksPerSecondOne,
            double ticksPerSecondTwo,
            List<Aim> aims) {}

    record Story(
            World world,
            Ball pollen,
            WheelPair.Launched launched,
            List<Frame> frames,
            List<Impact> impacts,
            List<Crossed> crossings,
            Frame apex,
            Optional<FirstBounce> firstBounce,
            List<Track> pieces,
            long work,
            Seconds simulated) {}

    sealed interface Resting permits Resting.InCell, Resting.InFlower, Resting.OnTheFloor, Resting.On {
        String kind();

        String says();

        record InCell(String cell, String alliance, boolean upturned) implements Resting {
            @Override
            public String kind() {
                return upturned ? "scored" : "cell";
            }

            @Override
            public String says() {
                return "in " + cell
                        + (upturned
                                ? ", which is upturned, so it scores"
                                : ", which is turned down, so it does not score");
            }
        }

        record InFlower(String flower) implements Resting {
            @Override
            public String kind() {
                return "flower";
            }

            @Override
            public String says() {
                return "in the bore of " + flower;
            }
        }

        record OnTheFloor() implements Resting {
            @Override
            public String kind() {
                return "floor";
            }

            @Override
            public String says() {
                return "on the floor";
            }
        }

        record On(String what) implements Resting {
            @Override
            public String kind() {
                return "rest";
            }

            @Override
            public String says() {
                return "on " + what;
            }
        }
    }

    sealed interface Finish permits Finish.AtRest, Finish.LeftTheField, Finish.OutOfTime, Finish.TooMuchWork {
        Seconds t();

        String kind();

        String says();

        record AtRest(Vec3 at, Seconds t, Resting where) implements Finish {
            @Override
            public String kind() {
                return where.kind();
            }

            @Override
            public String says() {
                return "came to rest " + where.says();
            }
        }

        record LeftTheField(Vec3 at, Seconds t, Colliders.Side side) implements Finish {
            @Override
            public String kind() {
                return "out";
            }

            @Override
            public String says() {
                return "left the field over " + side.says();
            }
        }

        record OutOfTime(Seconds t) implements Finish {
            @Override
            public String kind() {
                return "time";
            }

            @Override
            public String says() {
                return "was still moving after " + t;
            }
        }

        record TooMuchWork(Seconds t) implements Finish {
            @Override
            public String kind() {
                return "work";
            }

            @Override
            public String says() {
                return "was stopped after " + t + " for taking too much work: make the step longer";
            }
        }
    }

    static Checked<Flying> from(Field field, ShotSettings settings) {
        return ShotSetup.shot(field, settings);
    }

    static Checked<Preview> preview(Field field, ShotSettings settings) {
        return ShotSetup.preview(field, settings);
    }

    static Checked<Flying> launch(World world, Ball pollen, WheelPair.Launched launched) {
        return Flying.start(world, pollen, launched);
    }

    static boolean tooMuchWork(long work, Seconds simulated) {
        return work > MOST_WORK_PER_SECOND * StrictMath.max(simulated.value(), LEAST_SECONDS_OF_WORK);
    }

    final class Finished implements Shot {
        private final Story story;
        private final Finish finish;

        Finished(Story story, Finish finish) {
            this.story = story;
            this.finish = finish;
        }

        @Override
        public Story story() {
            return story;
        }

        public Finish finish() {
            return finish;
        }
    }

    final class Flying implements Shot {
        private final ShotRun frozen;

        Flying(ShotRun frozen) {
            this.frozen = frozen;
        }

        static Checked<Flying> start(World world, Ball pollen, WheelPair.Launched launched) {
            Colliders colliders = world.colliders();
            Length radius = pollen.radiusLength();
            Optional<Colliders.Side> beyond = colliders.over(launched.at(), radius);
            if (beyond.isPresent()) {
                return beyond.map(side -> Checked.<Flying>rejected("the pollen would start beyond " + side.says()))
                        .orElse(Checked.rejected("the pollen would start beyond the walls"));
            }
            for (Collider collider : colliders.all()) {
                if (colliders.gap(collider, launched.at(), radius) < 0) {
                    return Checked.rejected(
                            "the pollen would start inside " + collider.name() + ", at " + launched.at());
                }
            }
            for (World.Piece piece : world.pieces()) {
                Vec3 between = piece.at().minus(launched.at());
                double apart = StrictMath.sqrt(between.dot(between));
                if (apart < pollen.radius() + piece.shape().radius()) {
                    return Checked.rejected("the pollen would start inside " + piece.name() + ", at " + launched.at());
                }
            }
            return Checked.ok(new Flying(ShotRun.starting(world, pollen, launched)));
        }

        @Override
        public Story story() {
            return frozen.story();
        }

        public Shot advance(Seconds until) {
            return frozen.copy().until(until.value());
        }
    }
}
