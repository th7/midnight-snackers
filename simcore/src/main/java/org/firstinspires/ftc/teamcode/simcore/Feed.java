package org.firstinspires.ftc.teamcode.simcore;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class Feed {
    public static final double POLLEN_RADIUS_IN = Launcher.POLLEN_RADIUS_IN;
    static final String DEFAULTS_SOURCE = "doc/plans/feeder-sim.md, What the tab does: two pollen, one each side of"
            + " the arms against the wall, the spinner at full power, the ramp down at 0 and up at 35 degrees, over a run"
            + " of 3 s";
    static final double FIRST_POLLEN_DEGREES = 90;
    static final double SECOND_POLLEN_DEGREES = 270;
    static final double DOWN_DEGREES = 0;
    static final double UP_DEGREES = 35;
    static final String DECIDED_SOURCE = "issue 180, the session owner's decision of 21 November: the CAD's back"
            + " sill at z 43.12 holds a lifted pollen against the wheel, and 0.40 to 0.80 s misses both pollen, so the"
            + " default feed lowers the back sill to z 30 and raises the ramp from 0.30 s to 1.20 s";
    static final double BACK_SILL_MM = 30;
    static final double RAISED_AT_SECONDS = 0.3;
    static final double LOWERED_AT_SECONDS = 1.2;
    static final double RUN_SECONDS = 3;
    static final String FLYWHEEL_SOURCE = "doc/plans/feeder-sim.md, What the tab does: the flywheel at stage 4's"
            + " default, the code's close launch of 1050 ticks a second at the 24.2 ticks a turn the launcher is tested"
            + " at";
    static final double FLYWHEEL_TICKS_PER_SECOND = 1050;
    static final double FLYWHEEL_TICKS_PER_TURN = 24.2;
    static final String CONTACT_SOURCE = "doc/plans/feeder-sim.md, How it is simulated, Contact: a pollen meets the"
            + " feeder's faces at mu 0.30 and restitution 0.50, as Launcher uses";
    static final double FRICTION = 0.30;
    static final double RESTITUTION = 0.50;
    static final String RANGE_SOURCE = "doc/plans/feeder-sim.md, What the tab does: a run of 0.5 to 10 s, the ramp"
            + " moved within it; the back sill above the floor's top at z 15 and below the top plate at z 96.12";
    static final double SHORTEST_RUN_SECONDS = 0.5;
    static final double LONGEST_RUN_SECONDS = 10;
    static final double LOWEST_BACK_SILL_MM = 16;
    static final double HIGHEST_BACK_SILL_MM = 95;
    static final String JAM_SOURCE = "doc/plans/feeder-sim.md, Phase 3, 9: a pollen that stalls the spinner jams the"
            + " feed: turning under a tenth of its unloaded speed for a tenth of a second, at any power but none";
    static final double JAMMED_BELOW_UNLOADED = 0.1;
    static final double JAMMED_FOR_SECONDS = 0.1;
    static final String TICK_SOURCE =
            "the servos are commanded once a frame, every 4 ms, as the robot's loop does" + " about that often";

    private Feed() {}

    public static final class Pollen {
        private final double degreesFromTheBack;

        private Pollen(double degreesFromTheBack) {
            this.degreesFromTheBack = degreesFromTheBack;
        }

        public static Checked<Pollen> againstTheWall(double degreesFromTheBack) {
            if (!Double.isFinite(degreesFromTheBack)) {
                return Checked.rejected("a pollen starts a finite number of degrees clockwise from the back, not "
                        + degreesFromTheBack);
            }
            return Checked.ok(new Pollen(degreesFromTheBack));
        }

        public double degreesFromTheBack() {
            return degreesFromTheBack;
        }

        @Override
        public String toString() {
            return "a pollen against the wall " + degreesFromTheBack + " degrees clockwise from the back";
        }
    }

    public static final class RampSchedule {
        private final Feeder.RampAngle down;
        private final Feeder.RampAngle up;
        private final Seconds raisedAt;
        private final Seconds loweredAt;

        private RampSchedule(Feeder.RampAngle down, Feeder.RampAngle up, Seconds raisedAt, Seconds loweredAt) {
            this.down = down;
            this.up = up;
            this.raisedAt = raisedAt;
            this.loweredAt = loweredAt;
        }

        public static Checked<RampSchedule> of(
                Feeder.RampAngle down, Feeder.RampAngle up, Seconds raisedAt, Seconds loweredAt) {
            if (!(down.degrees() < up.degrees())) {
                return Checked.rejected(
                        "a ramp is down below where it is up, not down at " + down + " and up at " + up);
            }
            if (!(raisedAt.value() < loweredAt.value())) {
                return Checked.rejected("a ramp is raised before it is lowered, not raised at " + raisedAt
                        + " and lowered at " + loweredAt);
            }
            return Checked.ok(new RampSchedule(down, up, raisedAt, loweredAt));
        }

        Feeder.RampAngle commandedAt(double seconds) {
            return seconds >= raisedAt.value() && seconds < loweredAt.value() ? up : down;
        }

        boolean stillFrom(double seconds, Seconds runEnds) {
            return seconds >= loweredAt.value() || raisedAt.value() >= runEnds.value();
        }

        Feeder.RampAngle down() {
            return down;
        }

        @Override
        public String toString() {
            return "a ramp down at " + down + " and up at " + up + ", raised at " + raisedAt + " and lowered at "
                    + loweredAt;
        }
    }

    public static final class Budget {
        private final long workPerSecond;

        private Budget(long workPerSecond) {
            this.workPerSecond = workPerSecond;
        }

        public static Checked<Budget> ofWorkPerSecond(long workPerSecond) {
            if (!(workPerSecond > 0 && workPerSecond <= Shot.MOST_WORK_PER_SECOND)) {
                return Checked.rejected("a feed's budget is more than 0 and at most the engine's "
                        + Shot.MOST_WORK_PER_SECOND + " units of work a simulated second, not " + workPerSecond);
            }
            return Checked.ok(new Budget(workPerSecond));
        }

        public static Budget theEngines() {
            return new Budget(Shot.MOST_WORK_PER_SECOND);
        }

        boolean spentBy(long work, Seconds simulated) {
            return work > workPerSecond * StrictMath.max(simulated.value(), Shot.LEAST_SECONDS_OF_WORK);
        }

        @Override
        public String toString() {
            return workPerSecond + " units of work a simulated second";
        }
    }

    public static final class Plan {
        private final Feeder.Shape shape;
        private final Feeder.Spinner spinner;
        private final Power spinnerPower;
        private final Feeder.Ramp rampServo;
        private final RampSchedule ramp;
        private final Flywheel.AngularSpeed flywheel;
        private final List<Pollen> pollen;
        private final Seconds length;
        private final Budget budget;

        private Plan(
                Feeder.Shape shape,
                Feeder.Spinner spinner,
                Power spinnerPower,
                Feeder.Ramp rampServo,
                RampSchedule ramp,
                Flywheel.AngularSpeed flywheel,
                List<Pollen> pollen,
                Seconds length,
                Budget budget) {
            this.shape = shape;
            this.spinner = spinner;
            this.spinnerPower = spinnerPower;
            this.rampServo = rampServo;
            this.ramp = ramp;
            this.flywheel = flywheel;
            this.pollen = Collections.unmodifiableList(new ArrayList<>(pollen));
            this.length = length;
            this.budget = budget;
        }

        public static Checked<Plan> defaults() {
            return Feeder.Shape.cad()
                    .then(cad -> Feeder.Window.of(Feeder.Window.BACK_HALF_WIDTH_MM, BACK_SILL_MM)
                            .then(back -> cad.withBackWindow(back)))
                    .then(shape -> Feeder.RampAngle.ofDegrees(DOWN_DEGREES)
                            .then(down -> Feeder.RampAngle.ofDegrees(UP_DEGREES)
                                    .then(up -> Seconds.of(RAISED_AT_SECONDS)
                                            .then(raised -> Seconds.of(LOWERED_AT_SECONDS)
                                                    .then(lowered -> RampSchedule.of(down, up, raised, lowered))))
                                    .then(ramp -> Flywheel.TicksPerTurn.of(FLYWHEEL_TICKS_PER_TURN)
                                            .then(ticks -> Pollen.againstTheWall(FIRST_POLLEN_DEGREES)
                                                    .then(first -> Pollen.againstTheWall(SECOND_POLLEN_DEGREES)
                                                            .then(second -> Seconds.of(RUN_SECONDS)
                                                                    .map(length -> new Plan(
                                                                            shape,
                                                                            Feeder.Spinner.revSmartRobotServo(),
                                                                            Power.clamped(1),
                                                                            Feeder.Ramp.revSmartRobotServo(),
                                                                            ramp,
                                                                            Flywheel.AngularSpeed.from(
                                                                                    FLYWHEEL_TICKS_PER_SECOND, ticks),
                                                                            List.of(first, second),
                                                                            length,
                                                                            Budget.theEngines()))))))));
        }

        public Plan withShape(Feeder.Shape other) {
            return new Plan(other, spinner, spinnerPower, rampServo, ramp, flywheel, pollen, length, budget);
        }

        public Plan withSpinner(Feeder.Spinner other) {
            return new Plan(shape, other, spinnerPower, rampServo, ramp, flywheel, pollen, length, budget);
        }

        public Plan withSpinnerPower(Power other) {
            return new Plan(shape, spinner, other, rampServo, ramp, flywheel, pollen, length, budget);
        }

        public Plan withRampServo(Feeder.Ramp other) {
            return new Plan(shape, spinner, spinnerPower, other, ramp, flywheel, pollen, length, budget);
        }

        public Plan withRamp(RampSchedule other) {
            return new Plan(shape, spinner, spinnerPower, rampServo, other, flywheel, pollen, length, budget);
        }

        public Plan withFlywheel(Flywheel.AngularSpeed other) {
            return new Plan(shape, spinner, spinnerPower, rampServo, ramp, other, pollen, length, budget);
        }

        public Plan withPollen(List<Pollen> other) {
            return new Plan(shape, spinner, spinnerPower, rampServo, ramp, flywheel, other, length, budget);
        }

        public Plan withBudget(Budget other) {
            return new Plan(shape, spinner, spinnerPower, rampServo, ramp, flywheel, pollen, length, other);
        }

        public Checked<Plan> lasting(Seconds other) {
            if (!(other.value() > 0)) {
                return Checked.rejected("a feed lasts more than 0 s, not " + other);
            }
            return Checked.ok(new Plan(shape, spinner, spinnerPower, rampServo, ramp, flywheel, pollen, other, budget));
        }

        public Checked<Fed> run() {
            return FeedRun.starting(this).map(FeedRun::finished);
        }

        Feeder.Shape shape() {
            return shape;
        }

        Feeder.Spinner spinner() {
            return spinner;
        }

        Power spinnerPower() {
            return spinnerPower;
        }

        Feeder.Ramp rampServo() {
            return rampServo;
        }

        RampSchedule ramp() {
            return ramp;
        }

        Flywheel.AngularSpeed flywheel() {
            return flywheel;
        }

        List<Pollen> pollen() {
            return pollen;
        }

        Seconds length() {
            return length;
        }

        Budget budget() {
            return budget;
        }
    }

    public record Seen(String name, Vec3 at, Vec3 velocity, Vec3 spin, Turn turn) {}

    public record Frame(
            Seconds t,
            double spinnerDegrees,
            double spinnerRevolutionsPerSecond,
            double spinnerLoadNewtonMetres,
            double rampDegrees,
            double rampTorqueNewtonMetres,
            List<Seen> pollen) {}

    public sealed interface Event {
        Seconds at();

        record Thrown(String pollen, Seconds at, Vec3 from, Vec3 velocity, Vec3 spin) implements Event {}

        record FellOut(String pollen, Seconds at, Vec3 from) implements Event {}

        record Jammed(Seconds at, List<String> pollen) implements Event {}
    }

    public sealed interface Finish {
        Seconds at();

        record AllLeft(Seconds at) implements Finish {}

        record AtRest(Seconds at) implements Finish {}

        record OutOfTime(Seconds at) implements Finish {}

        record TooMuchWork(Seconds at) implements Finish {}
    }

    public record Fed(List<Frame> frames, List<Event> events, Finish finish, long work, Seconds simulated) {}
}
