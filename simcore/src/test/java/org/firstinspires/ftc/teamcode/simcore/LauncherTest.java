package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class LauncherTest {
    private static final String TESTED_AT_SOURCE = "doc/plans/runtime-flight.md, stage 4, Tests first: the core"
            + " launcher is tested at 1050 ticks a second and T 24.2, as in stage 4b, a wheel speed that exercises the"
            + " launcher, not Reginald's";
    private static final double TICKS_PER_TURN = 24.2;
    private static final double LAUNCH = 1050;
    private static final String THROW_SOURCE = "doc/plans/runtime-flight.md, stage 4, Tests first: the launcher"
            + " throws at 0.366 to 0.584 of u50, the surface speed at the 100 mm diameter, from hood drag spread over"
            + " the groove to a ball riding the groove's edges, at 70 +- 3 degrees, the first sample with no launcher"
            + " contact 0 to 10 degrees past the hood's upper end";
    private static final double SLOWEST_THROW = 0.366;
    private static final double FASTEST_THROW = 0.584;
    private static final double SURFACE_SPEED_RADIUS_MM = 50;
    private static final double THROWN_AT_DEGREES = 70;
    private static final double THROWN_WITHIN_DEGREES = 3;
    private static final double LEFT_WITHIN_DEGREES = 10;
    private static final String SPIN_SOURCE = "doc/plans/runtime-flight.md, stage 4, Tests first: backspin,"
            + " antiparallel to the wheel's, with 0.9 <= S <= 1.2, where S, the spin ratio, is how fast the ball's"
            + " surface goes about its centre over how fast its centre goes, |w| r / |v|";
    private static final double LEAST_SPIN_RATIO = 0.9;
    private static final double MOST_SPIN_RATIO = 1.2;
    private static final String ACROSS_SOURCE = "the launcher's rings lie evenly either side of the middle of its"
            + " groove, where a ball is fed, so it spins about the axle alone, to rounding";
    private static final double ACROSS_WITHIN = 1e-9;
    private static final String FRICTION_SOURCE = "doc/plans/runtime-flight.md, stage 4: drawn in at 0.3 on the wheel"
            + " and 0.5 on the hood, and a model of this launcher swept the hood's friction down to 0.15";
    private static final double LESS_GRIP_ON_THE_WHEEL = 0.3;
    private static final double MORE_GRIP_ON_THE_HOOD = 0.5;
    private static final double SMOOTHER_HOOD = 0.15;
    private static final double FASTER_LAUNCH = 1400;
    private static final String ROBOT_TOP_SOURCE =
            "doc/plans/runtime-flight.md, stage 4, What we change, the hood's upper end: the robot's top is 9.084 in"
                    + " after the trim";
    private static final double ROBOT_TOP_IN = 9.084;
    private static final double HELD_FOR_SECONDS = 1;
    private static final double THROWN_WITHIN_SECONDS = 0.2;
    private static final double STEP_SECONDS = Flight.STEP_NANOS / Flight.NANOS_PER_SECOND;
    private static final double COURSE_SECONDS = 0.005;
    private static final double DEGREE = StrictMath.PI / 180;
    private static final Length POLLEN = Valid.length(Launcher.POLLEN_RADIUS_IN);
    private static final ShotSettings DEFAULTS = ShotSettings.defaults();
    private static final Ball BALL =
            Valid.value(Ball.of(POLLEN, DEFAULTS.value(ShotSetting.MASS), DEFAULTS.value(ShotSetting.INERTIA)));
    private static final Field FIELD = CollidersTest.field(30);
    private static final Vec3 HUB = new Vec3(-13 / 25.4, 0, 180 / 25.4);
    private static final String ROUNDING_SOURCE = "a frame at a constant velocity, or turned about the upright,"
            + " throws in a vacuum, or in air that is still, as a still or unturned frame does with the frame's"
            + " velocity added or its turn applied, exactly but for where the two put the ball: that differs by the"
            + " rounding of a place on the field, an ulp of 100 in, which the squeeze, ringing at the root of its"
            + " stiffness over the pollen's mass, turns into a velocity each step, and even were every step's"
            + " rounding the same way the throw's steps add no more than that many of them";
    private static final double FIELD_IN = 100;
    private static final double SQUEEZING_N_PER_MM = 2 * Compliance.STIFFNESS_N_PER_MM;

    private static Air flightsAir() {
        return Valid.value(ShotSetup.air(DEFAULTS));
    }

    private static Flywheel.AngularSpeed wheel(double ticksPerSecond) {
        return wheel(ticksPerSecond, TICKS_PER_TURN);
    }

    private static Flywheel.AngularSpeed wheel(double ticksPerSecond, double ticksPerTurn) {
        return Flywheel.AngularSpeed.from(ticksPerSecond, Valid.value(Flywheel.TicksPerTurn.of(ticksPerTurn)));
    }

    private static Launcher.Frame frame(double x, double y, double yaw) {
        return Valid.value(Launcher.Frame.of(x, y, yaw));
    }

    private static Launcher.Course course(Launcher.Frame from, Launcher.Frame to) {
        return Valid.value(Launcher.Course.of(from, Valid.time(0), to, Valid.time(COURSE_SECONDS)));
    }

    private static Launcher.Course still() {
        return course(frame(-25, -35, 0), frame(-25, -35, 0));
    }

    private static Launcher.Faces faces() {
        return Valid.value(Launcher.Faces.built());
    }

    private static Launcher.Faces faces(double wheelFriction, double hoodFriction) {
        return Valid.value(Launcher.Faces.of(
                Valid.value(Compliance.of(
                        Compliance.STIFFNESS_N_PER_MM,
                        Compliance.RESTITUTION,
                        wheelFriction,
                        Compliance.REGULARISATION_M_PER_S)),
                Valid.value(Compliance.of(
                        Compliance.STIFFNESS_N_PER_MM,
                        Compliance.RESTITUTION,
                        hoodFriction,
                        Compliance.REGULARISATION_M_PER_S))));
    }

    private record Exit(double seconds, Vec3 at, Vec3 velocity, Vec3 spin, long work, double degreesAbout) {
        double speed() {
            return StrictMath.sqrt(velocity.dot(velocity));
        }

        double degreesUp() {
            return StrictMath.atan2(velocity.z(), StrictMath.hypot(velocity.x(), velocity.y())) / DEGREE;
        }

        double spinRatio() {
            return StrictMath.sqrt(spin.dot(spin)) * Launcher.POLLEN_RADIUS_IN / speed();
        }
    }

    private static final class Feeding {
        private final Bodies bodies;
        private Launcher launcher;
        private final Bodies.Allowance allowance =
                Valid.value(Bodies.Allowance.perWindow(Flight.WINDOW_STEPS, Flight.WINDOW_WORK));

        Feeding(Launcher launcher, Air air) {
            this.launcher = launcher;
            Hives hives = Valid.value(Hives.of(FIELD));
            this.bodies = Bodies.unwatched(
                    Valid.value(Surroundings.of(
                            Valid.value(Colliders.withoutHives(FIELD)), List.of(hives.part(), launcher))),
                    air,
                    Valid.value(ShotSetup.surfaces(DEFAULTS)),
                    Valid.value(Flight.stepping()));
        }

        static Feeding fed(Launcher launcher, Air air) {
            return new Feeding(launcher, air).feeding(launcher.feed(POLLEN));
        }

        Feeding feeding(Vec3 velocity) {
            Valid.value(bodies.add(
                    Valid.value(Bodies.Name.of("pollen")),
                    BALL,
                    Valid.value(Bodies.Motion.of(launcher.entry(POLLEN), velocity, Vec3.zero())),
                    new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING)));
            return this;
        }

        Bodies.Seen ball() {
            return bodies.body("pollen").orElseThrow(() -> new AssertionError("no ball"));
        }

        Launcher placed() {
            return launcher.at(bodies.time());
        }

        boolean inTheLauncher() {
            Launcher then = placed();
            Bodies.Seen ball = ball();
            return !then.squeezing(ball.at(), POLLEN).isEmpty()
                    || !then.near(ball.at(), POLLEN, Shot.TOUCHING_IN).isEmpty();
        }

        double degreesAbout() {
            Launcher.Placed where = placed().where();
            Vec3 off = ball().at().minus(where.origin());
            double cos = StrictMath.cos(where.yaw()), sin = StrictMath.sin(where.yaw());
            double back = -(cos * off.x() + sin * off.y() - HUB.x());
            return StrictMath.atan2(off.z() - HUB.z(), back) / DEGREE;
        }

        void step() {
            bodies.step(allowance);
        }

        void advance(double seconds) {
            long target = bodies.steps() + StrictMath.round(seconds / STEP_SECONDS);
            while (bodies.steps() < target) {
                if (!bodies.anyAwake() && bodies.idleThrough(target)) {
                    continue;
                }
                step();
            }
        }

        Optional<Exit> thrown() {
            long began = bodies.work();
            boolean squeezed = inTheLauncher();
            for (long i = 0; i < StrictMath.round(THROWN_WITHIN_SECONDS / STEP_SECONDS); i++) {
                step();
                boolean now = inTheLauncher();
                if (squeezed && !now) {
                    Bodies.Seen ball = ball();
                    return Optional.of(new Exit(
                            bodies.steps() * STEP_SECONDS,
                            ball.at(),
                            ball.velocity(),
                            ball.spin(),
                            bodies.work() - began,
                            degreesAbout()));
                }
                squeezed = squeezed || now;
            }
            return Optional.empty();
        }

        Exit throwsIt() {
            return thrown().orElseThrow(() -> new AssertionError("not thrown: " + ball()));
        }

        void spinUp(Launcher spinning) {
            launcher = spinning;
            bodies.surroundedBy(Valid.value(bodies.surroundings().replacing(spinning)));
        }
    }

    private static Exit fedAndThrown(Launcher.Faces faces, double ticksPerSecond) {
        return Feeding.fed(Launcher.placed(faces, still(), wheel(ticksPerSecond)), flightsAir())
                .throwsIt();
    }

    private static void assertThrownAsTheLauncherThrows(String what, Exit exit, Flywheel.AngularSpeed wheel) {
        double u50 = StrictMath.abs(wheel.radiansPerSecond()) * SURFACE_SPEED_RADIUS_MM / 25.4;
        assertTrue(
                THROW_SOURCE + ": " + what + " thrown at " + exit.speed() + " in/s, " + exit.speed() / u50 + " of "
                        + u50,
                exit.speed() >= SLOWEST_THROW * u50 && exit.speed() <= FASTEST_THROW * u50);
        assertEquals(THROW_SOURCE + ": " + what, THROWN_AT_DEGREES, exit.degreesUp(), THROWN_WITHIN_DEGREES);
        double past = exit.degreesAbout() - Launcher.HOOD_TO_DEGREES;
        assertTrue(
                THROW_SOURCE + ": " + what + " left " + past + " degrees past the hood's upper end",
                past >= 0 && past <= LEFT_WITHIN_DEGREES);
    }

    @Test
    public void aWheelAtTheLaunchItIsTestedAtThrowsAFedBallUpAtSeventyDegreesWithBackspinAsThePlanSays() {
        assertEquals(TESTED_AT_SOURCE, 24.2, TICKS_PER_TURN, 0);
        assertEquals(TESTED_AT_SOURCE, 1050, LAUNCH, 0);
        Exit exit = fedAndThrown(faces(), LAUNCH);

        assertThrownAsTheLauncherThrows("at the launch it is tested at", exit, wheel(LAUNCH));
        Vec3 axle = new Vec3(0, 1, 0);
        double about = exit.spin().dot(axle);
        Vec3 across = exit.spin().minus(axle.times(about));
        assertTrue(SPIN_SOURCE + ": spinning at " + exit.spin(), about < 0);
        assertTrue(
                ACROSS_SOURCE + ": spinning at " + exit.spin(),
                StrictMath.sqrt(across.dot(across)) <= ACROSS_WITHIN * StrictMath.abs(about));
        assertTrue(
                SPIN_SOURCE + ": a spin ratio of " + exit.spinRatio(),
                exit.spinRatio() >= LEAST_SPIN_RATIO && exit.spinRatio() <= MOST_SPIN_RATIO);
    }

    @Test
    public void aBallIsDrawnInFromAFeedOfNothingAndWithLessGripOnTheWheelAndMoreOnTheHood() {
        Launcher launcher = Launcher.placed(faces(), still(), wheel(LAUNCH));
        Exit unfed = new Feeding(launcher, flightsAir()).feeding(Vec3.zero()).throwsIt();
        assertThrownAsTheLauncherThrows("fed at nothing", unfed, wheel(LAUNCH));

        Exit slipping = fedAndThrown(faces(LESS_GRIP_ON_THE_WHEEL, MORE_GRIP_ON_THE_HOOD), LAUNCH);
        assertThrownAsTheLauncherThrows(FRICTION_SOURCE, slipping, wheel(LAUNCH));
    }

    @Test
    public void aFasterWheelThrowsStrictlyFasterWithMoreSpin() {
        Exit tested = fedAndThrown(faces(), LAUNCH);
        Exit faster = fedAndThrown(faces(), FASTER_LAUNCH);

        assertTrue(faster.speed() + " against " + tested.speed(), faster.speed() > tested.speed());
        assertTrue(
                faster.spin() + " against " + tested.spin(),
                faster.spin().dot(faster.spin()) > tested.spin().dot(tested.spin()));
    }

    private static Bodies.Seen straightBack(Launcher.Faces faces) {
        Feeding feeding = Feeding.fed(Launcher.placed(faces, still(), wheel(LAUNCH)), flightsAir());
        for (long i = 0;
                i < StrictMath.round(THROWN_WITHIN_SECONDS / STEP_SECONDS) && feeding.degreesAbout() < 0;
                i++) {
            feeding.step();
        }
        assertTrue(
                "carried straight back of the axle: " + feeding.ball(),
                feeding.inTheLauncher() && feeding.degreesAbout() >= 0);
        return feeding.ball();
    }

    private static double speed(Bodies.Seen ball) {
        return StrictMath.sqrt(ball.velocity().dot(ball.velocity()));
    }

    private static double spinRatio(Bodies.Seen ball) {
        return StrictMath.sqrt(ball.spin().dot(ball.spin())) * Launcher.POLLEN_RADIUS_IN / speed(ball);
    }

    @Test
    public void aSmootherHoodCarriesTheBallThroughItsGrooveFasterSpinningLessForItsSpeed() {
        Bodies.Seen built = straightBack(faces());
        Bodies.Seen smoother = straightBack(faces(Compliance.WHEEL_FRICTION, SMOOTHER_HOOD));

        assertTrue(
                FRICTION_SOURCE + ": " + speed(smoother) + " against " + speed(built), speed(smoother) > speed(built));
        assertTrue(
                SPIN_SOURCE + ": a spin ratio of " + spinRatio(smoother) + " against " + spinRatio(built),
                spinRatio(smoother) < spinRatio(built));
    }

    @Test
    public void
            withTheWheelStoppedTheBallStaysInTheLauncherInsideTheRobotAndOffTheFloorAndIsThrownOnceTheWheelSpinsUp() {
        Feeding launch = Feeding.fed(Launcher.placed(faces(), still(), wheel(0)), flightsAir());
        launch.advance(HELD_FOR_SECONDS);

        Bodies.Seen held = launch.ball();
        assertTrue(ROBOT_TOP_SOURCE + ": held at " + held, held.at().z() + Launcher.POLLEN_RADIUS_IN < ROBOT_TOP_IN);
        assertTrue("off the floor: " + held, held.at().z() - Launcher.POLLEN_RADIUS_IN > 0);
        assertTrue("in the launcher: " + held, launch.placed().reaches(held.at(), Launcher.POLLEN_RADIUS_IN));
        assertEquals("asleep in the launcher: " + held, Bodies.Life.ASLEEP, held.life());

        long now = launch.bodies.steps();
        Launcher.Course onwards = Valid.value(Launcher.Course.of(
                frame(-25, -35, 0),
                GridTime.at(now, Valid.value(Flight.stepping())),
                frame(-25, -35, 0),
                GridTime.at(now + 50, Valid.value(Flight.stepping()))));
        launch.spinUp(Launcher.placed(faces(), onwards, wheel(LAUNCH)));
        assertThrownAsTheLauncherThrows("once spun up", launch.throwsIt(), wheel(LAUNCH));
    }

    private static Launcher.Course onwardFrom(long steps, Launcher.Frame from, Launcher.Frame to) {
        Bodies.Step step = Valid.value(Flight.stepping());
        return Valid.value(Launcher.Course.of(
                from,
                GridTime.at(steps, step),
                to,
                GridTime.at(steps + StrictMath.round(COURSE_SECONDS / STEP_SECONDS), step)));
    }

    @Test
    public void
            anEngineIdlesThroughABallAsleepInAStillLauncherAsSteppingWouldAndNotOnceALauncherThatMovesTakesItsPlace() {
        Feeding held = Feeding.fed(Launcher.placed(faces(), still(), wheel(0)), flightsAir());
        held.advance(HELD_FOR_SECONDS);
        assertEquals(Bodies.Life.ASLEEP, held.ball().life());
        long now = held.bodies.steps(), later = now + StrictMath.round(HELD_FOR_SECONDS / STEP_SECONDS);

        Bodies jumping = held.bodies.copy();
        assertTrue("idles while the launcher is still", jumping.idleThrough(later));
        Bodies stepping = held.bodies.copy();
        while (stepping.steps() < later) {
            stepping.step();
        }
        assertEquals(stepping.body("pollen"), jumping.body("pollen"));
        assertEquals(stepping.work(), jumping.work());

        Launcher.Frame at = frame(-25, -35, 0);
        for (Launcher moving : List.of(
                Launcher.placed(faces(), onwardFrom(now, at, at), wheel(LAUNCH)),
                Launcher.placed(faces(), onwardFrom(now, at, frame(-24.9, -35, 0)), wheel(0)),
                Launcher.placed(faces(), onwardFrom(now, at, frame(-25, -35, 0.01)), wheel(0)))) {
            Bodies woken = held.bodies.copy();
            woken.surroundedBy(Valid.value(woken.surroundings().replacing(moving)));
            assertEquals(
                    moving.toString(),
                    Bodies.Life.AWAKE,
                    woken.body("pollen").map(Bodies.Seen::life).orElseThrow());
            assertTrue(moving.toString(), !woken.idleThrough(later));
        }
    }

    @Test
    public void aReversedWheelThrowsNothing() {
        Feeding launch = Feeding.fed(Launcher.placed(faces(), still(), wheel(-LAUNCH)), flightsAir());
        for (int i = 0; i < 100; i++) {
            launch.advance(0.005);
            Bodies.Seen ball = launch.ball();
            assertTrue(
                    "finite: " + ball, ball.at().isFinite() && ball.velocity().isFinite());
            assertTrue("in the launcher: " + ball, launch.placed().reaches(ball.at(), Launcher.POLLEN_RADIUS_IN));
            assertTrue(
                    "below the hood's upper end: " + launch.degreesAbout(),
                    launch.degreesAbout() < Launcher.HOOD_TO_DEGREES);
        }
    }

    private static Exit thrownBy(Launcher.Course course, Flywheel.AngularSpeed wheel, Air air) {
        return Feeding.fed(Launcher.placed(faces(), course, wheel), air).throwsIt();
    }

    private static Vec3 turned(Vec3 v, double radians) {
        double cos = StrictMath.cos(radians), sin = StrictMath.sin(radians);
        return new Vec3(cos * v.x() - sin * v.y(), sin * v.x() + cos * v.y(), v.z());
    }

    private static double rounding(Exit exit) {
        double ringing = StrictMath.sqrt(SQUEEZING_N_PER_MM * Compliance.MILLIMETRES_PER_METRE / BALL.kilograms());
        return StrictMath.ulp(FIELD_IN) * ringing * StrictMath.round(exit.seconds() / STEP_SECONDS);
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double within) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), within);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), within);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), within);
    }

    @Test
    public void theRobotsVelocityCarriesIntoTheThrowExactlyInAVacuum() {
        Vec3 robot = new Vec3(0.6, -0.8, 0).times(Chassis.MOST_ROBOT_SPEED_IN_PER_S);
        Exit still = thrownBy(still(), wheel(LAUNCH), Air.vacuum());
        Exit moving = thrownBy(
                course(
                        frame(-25, -35, 0),
                        frame(-25 + robot.x() * COURSE_SECONDS, -35 + robot.y() * COURSE_SECONDS, 0)),
                wheel(LAUNCH),
                Air.vacuum());

        assertEquals(still.seconds(), moving.seconds(), 0);
        assertVec(ROUNDING_SOURCE, still.velocity().plus(robot), moving.velocity(), rounding(still));
        assertVec(ROUNDING_SOURCE, still.spin(), moving.spin(), rounding(still) / Launcher.POLLEN_RADIUS_IN);
    }

    @Test
    public void turningTheFrameTurnsTheThrowAsMuchAboutTheUpright() {
        Exit facing = thrownBy(still(), wheel(LAUNCH), flightsAir());
        for (double yaw : new double[] {0.7, -2.5, StrictMath.PI}) {
            Exit turned = thrownBy(course(frame(-25, -35, yaw), frame(-25, -35, yaw)), wheel(LAUNCH), flightsAir());
            assertEquals(facing.seconds(), turned.seconds(), 0);
            assertVec(
                    ROUNDING_SOURCE + ", turned by " + yaw,
                    turned(facing.velocity(), yaw),
                    turned.velocity(),
                    rounding(facing));
            assertVec(
                    ROUNDING_SOURCE + ", turned by " + yaw,
                    turned(facing.spin(), yaw),
                    turned.spin(),
                    rounding(facing) / Launcher.POLLEN_RADIUS_IN);
        }
    }

    @Test
    public void aFrameWhoseYawCrossesHalfATurnThrowsAsOneTurningAsFastThatDoesNot() {
        double half = StrictMath.PI, turn = 0.01;
        Exit crossing = thrownBy(
                course(frame(-25, -35, half - turn), frame(-25, -35, -half + turn)), wheel(LAUNCH), flightsAir());
        Exit quarter = thrownBy(
                course(frame(-25, -35, half / 2 - turn), frame(-25, -35, half / 2 + turn)),
                wheel(LAUNCH),
                flightsAir());

        assertEquals(quarter.seconds(), crossing.seconds(), 0);
        assertVec(ROUNDING_SOURCE, turned(quarter.velocity(), half / 2), crossing.velocity(), rounding(quarter));
        assertVec(
                ROUNDING_SOURCE,
                turned(quarter.spin(), half / 2),
                crossing.spin(),
                rounding(quarter) / Launcher.POLLEN_RADIUS_IN);
    }

    @Test
    public void aCommandThatIsNoNumberLeavesEveryBallFinite() {
        Feeding launch = Feeding.fed(Launcher.placed(faces(), still(), wheel(Double.NaN)), flightsAir());
        for (int i = 0; i < 40; i++) {
            launch.advance(0.025);
            Bodies.Seen ball = launch.ball();
            assertTrue(
                    "finite: " + ball,
                    ball.at().isFinite()
                            && ball.velocity().isFinite()
                            && ball.spin().isFinite());
        }
        assertTrue(
                "in the launcher: " + launch.ball(),
                launch.placed().reaches(launch.ball().at(), Launcher.POLLEN_RADIUS_IN));
    }

    @Test
    public void atTheLeastTicksPerTurnTheMotorsTopSpeedAndTheFastestRobotTheBallLeavesNoFasterThanAFlightTakesOne() {
        double fastest = Chassis.MOST_ROBOT_SPEED_IN_PER_S;
        Launcher.Course along = course(frame(-25, -35, 0), frame(-25 + fastest * COURSE_SECONDS, -35, 0));
        Exit exit = thrownBy(along, wheel(Flywheel.TOP_TICKS_PER_SECOND, Flywheel.TicksPerTurn.LEAST), flightsAir());

        assertTrue("thrown at " + exit.velocity(), exit.speed() <= Flight.MOST_SPEED_IN_PER_S);
    }
}
