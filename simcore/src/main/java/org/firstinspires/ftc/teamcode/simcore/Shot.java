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
    long MOST_WORK_PER_SECOND = 2_000_000L;
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

    record Track(World.Piece piece, List<Placed> samples) {}

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
            Pollen pollen,
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
        record InCell(String cell, String alliance, boolean upturned) implements Resting {}

        record InFlower(String flower) implements Resting {}

        record OnTheFloor() implements Resting {}

        record On(String what) implements Resting {}
    }

    sealed interface Finish permits Finish.AtRest, Finish.LeftTheField, Finish.OutOfTime, Finish.TooMuchWork {
        Seconds t();

        record AtRest(Vec3 at, Seconds t, Resting where) implements Finish {}

        record LeftTheField(Vec3 at, Seconds t, Colliders.Side side) implements Finish {}

        record OutOfTime(Seconds t) implements Finish {}

        record TooMuchWork(Seconds t) implements Finish {}
    }

    static Checked<Flying> from(Field field, ShotSettings settings) {
        return ShotSetup.shot(field, settings);
    }

    static Checked<Preview> preview(Field field, ShotSettings settings) {
        return ShotSetup.preview(field, settings);
    }

    static Checked<Flying> launch(World world, Pollen pollen, WheelPair.Launched launched) {
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

        static Checked<Flying> start(World world, Pollen pollen, WheelPair.Launched launched) {
            if (!(launched.at().isFinite()
                    && launched.velocity().isFinite()
                    && launched.spin().isFinite())) {
                return Checked.rejected("a pollen leaves the wheels from somewhere, at some speed and spin, not at "
                        + launched.at() + " going " + launched.velocity() + " spinning " + launched.spin());
            }
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
                double apart = piece.at().minus(launched.at()).length();
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
