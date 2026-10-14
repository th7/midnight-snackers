package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.Test;

public class CompliantContactTest {
    private static final Ball POLLEN = Valid.value(Ball.of(2.78, 24.9, 0.634));
    private static final double R = POLLEN.radius();
    private static final double MILLIMETRE = 1 / 25.4;
    private static final double STEP = 1e-4;
    private static final double DEGREE = StrictMath.PI / 180;
    private static final Bodies.Start AWAKE = new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING);
    private static final Colliders FIELD = Valid.value(Colliders.of(
            Valid.value(Field.of(
                    Valid.length(141.17),
                    Valid.length(11.64),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of())),
            List.of()));
    private static final Compliance WHEEL = Valid.value(Compliance.of(
            Compliance.STIFFNESS_N_PER_MM,
            Compliance.RESTITUTION,
            Compliance.WHEEL_FRICTION,
            Compliance.REGULARISATION_M_PER_S));
    private static final Compliance HOOD = Valid.value(Compliance.of(
            Compliance.STIFFNESS_N_PER_MM,
            Compliance.RESTITUTION,
            Compliance.HOOD_FRICTION,
            Compliance.REGULARISATION_M_PER_S));
    private static final Vec3 AXLE = new Vec3(0, 1, 0);
    private static final Vec3 HUB = new Vec3(0, 0, 30);
    private static final double WHEEL_RADIUS = 39.729 * MILLIMETRE;
    private static final double FULL_SQUEEZE = 2.0 * MILLIMETRE;
    private static final double HOOD_RADIUS = WHEEL_RADIUS + 2 * R - FULL_SQUEEZE;
    private static final double HOOD_FROM = -64.8 * DEGREE;
    private static final double HOOD_TO = 20 * DEGREE;
    private static final double WHEEL_SPEED = 1050 * 2 * StrictMath.PI / 24.2;
    private static final double FEED = 0.1 / Length.METRES_PER_INCH;
    private static final double HELD_AT = -20 * DEGREE;

    private static Surfaces surfaces() {
        Bounce bounce = Valid.value(Bounce.of(0.55, 0.5, 0.2));
        return Valid.value(Surfaces.of(bounce, bounce, bounce, bounce, bounce, bounce, 0, 4, 0, 0, 0));
    }

    private static Bodies.Step stepping(double step) {
        return Valid.value(Bodies.Step.of(Valid.value(Seconds.of(step))));
    }

    private static Bodies among(double step, Surroundings.Part... parts) {
        return Bodies.watching(
                Valid.value(Surroundings.of(FIELD, List.of(parts))),
                Air.vacuum(),
                surfaces(),
                stepping(step),
                Valid.value(Bodies.Name.of("ball")));
    }

    private static void add(Bodies bodies, Vec3 at, Vec3 velocity) {
        Valid.value(bodies.add(
                Valid.value(Bodies.Name.of("ball")),
                POLLEN,
                Valid.value(Bodies.Motion.of(at, velocity, Vec3.zero())),
                AWAKE));
    }

    private static Bodies.Seen ball(Bodies bodies) {
        return bodies.body("ball").orElseThrow(() -> new AssertionError("no ball"));
    }

    private static List<Bodies.Span> stepFor(Bodies bodies, double seconds, double step) {
        List<Bodies.Span> spans = new ArrayList<>();
        long steps = StrictMath.round(seconds / step);
        for (long i = 0; i < steps; i++) {
            spans.addAll(bodies.step());
        }
        return spans;
    }

    private static Vec3 around(double angle, double out) {
        return HUB.plus(new Vec3(-StrictMath.cos(angle), 0, StrictMath.sin(angle)).times(out));
    }

    private static Vec3 alongThePath(double angle, double speed) {
        return new Vec3(StrictMath.sin(angle), 0, StrictMath.cos(angle)).times(speed);
    }

    private static CompliantArc hood() {
        return CompliantArc.still(
                "hood",
                Valid.value(Arc.of(HUB, AXLE, HOOD_RADIUS, Valid.value(Arc.Span.of(HOOD_FROM, HOOD_TO - HOOD_FROM)))),
                HOOD);
    }

    private static Bodies launcher(double step, double wheelSpeed) {
        return among(step, CompliantArc.spinning("wheel", HUB, AXLE, WHEEL_RADIUS, WHEEL, wheelSpeed), hood());
    }

    private static Bodies fedAt(Bodies launcher) {
        add(launcher, around(HOOD_FROM, WHEEL_RADIUS + R), alongThePath(HOOD_FROM, FEED));
        return launcher;
    }

    private static Bodies heldAt(Bodies launcher) {
        return heldAt(launcher, Vec3.zero());
    }

    private static Bodies heldAt(Bodies launcher, Vec3 velocity) {
        add(launcher, around(HELD_AT, (WHEEL_RADIUS + HOOD_RADIUS) / 2), velocity);
        return launcher;
    }

    private static List<Surroundings.Squeeze> squeezing(Bodies bodies) {
        return bodies.surroundings().squeezing(ball(bodies).at(), POLLEN.radiusLength());
    }

    private static double joules(Bodies bodies) {
        Bodies.Seen ball = ball(bodies);
        double m = POLLEN.kilograms(), inch = Length.METRES_PER_INCH;
        double spring = 0;
        for (Surroundings.Squeeze squeeze : squeezing(bodies)) {
            double depth = squeeze.depth() * inch;
            spring += squeeze.compliance().newtonsPerMetre() * depth * depth / 2;
        }
        return m * ball.velocity().dot(ball.velocity()) * inch * inch / 2
                + POLLEN.inertiaFactor() * m * R * R * inch * inch * ball.spin().dot(ball.spin()) / 2
                + m * Flight.GRAVITY_IN_PER_S2 * inch * ball.at().z() * inch
                + spring;
    }

    private static double speed(Vec3 v) {
        return StrictMath.sqrt(v.dot(v));
    }

    private record Exit(double seconds, Vec3 at, Vec3 velocity, Vec3 spin) {
        double speed() {
            return CompliantContactTest.speed(velocity);
        }

        double degreesUp() {
            return StrictMath.atan2(velocity.z(), StrictMath.hypot(velocity.x(), velocity.y())) / DEGREE;
        }
    }

    private static Exit thrown(Bodies launcher, double step) {
        boolean squeezed = false;
        for (long i = 1; i <= StrictMath.round(0.2 / step); i++) {
            launcher.step();
            boolean now = !squeezing(launcher).isEmpty();
            if (squeezed && !now) {
                Bodies.Seen ball = ball(launcher);
                return new Exit(i * step, ball.at(), ball.velocity(), ball.spin());
            }
            squeezed = squeezed || now;
        }
        throw new AssertionError("the ball was not thrown: " + ball(launcher) + ", squeezed " + squeezed);
    }

    @Test
    public void aBallPinchedBetweenAStillPlateAndASupportSlidingAtSomeSpeedRollsAtHalfThatSpeedOnceItGrips() {
        double u = 100;
        double floor = 20, squeeze = 1.0 * MILLIMETRE;
        Bodies pinch = among(
                STEP,
                CompliantPlate.still("plate", new Vec3(0, 0, floor), new Vec3(0, 0, 1), HOOD),
                CompliantPlate.sliding(
                        "belt", new Vec3(0, 0, floor + 2 * R - squeeze), new Vec3(0, 0, -1), WHEEL, new Vec3(u, 0, 0)));
        add(pinch, new Vec3(-30, 0, floor + R - squeeze / 2), Vec3.zero());

        stepFor(pinch, 0.05, STEP);
        for (int sample = 0; sample < 10; sample++) {
            stepFor(pinch, 0.01, STEP);
            Bodies.Seen ball = ball(pinch);
            assertEquals(
                    "rolling along at " + ball.velocity(),
                    u / 2,
                    ball.velocity().x(),
                    0.02 * u / 2);
            assertEquals("spinning at " + ball.spin(), u / 2 / R, ball.spin().y(), 0.02 * u / 2 / R);
        }
    }

    @Test
    public void aBallHeldInAStillSqueezeIsHeldThereWithoutAHitAPressOrATouch() {
        Bodies held = heldAt(launcher(STEP, 0));
        Vec3 was = ball(held).at();

        List<Bodies.Span> spans = stepFor(held, 0.1, STEP);

        Vec3 moved = ball(held).at().minus(was);
        assertTrue("moved " + moved, speed(moved) < 0.01);
        assertTrue("still squeezed", !squeezing(held).isEmpty());
        assertTrue("stepped", !spans.isEmpty());
        for (Bodies.Span span : spans) {
            assertEquals("pressing at " + span.now(), List.of(), span.pressing());
            assertTrue("hit at " + span.now(), span.resolved().isEmpty());
        }
        for (Bodies.Touched touched : held.touched()) {
            assertEquals(Set.of(), touched.colliders());
        }
    }

    @Test
    public void aBallAgainstStillSupportsNeverGainsEnergyOverASecondHeldOrThrownIntoThem() {
        for (Vec3 thrown : List.of(Vec3.zero(), alongThePath(HELD_AT, -20), alongThePath(HELD_AT, 40))) {
            Bodies held = heldAt(launcher(STEP, 0), thrown);
            double before = joules(held), gained = 0;
            for (int i = 0; i < 10_000; i++) {
                held.step();
                double now = joules(held);
                gained = StrictMath.max(gained, now - before);
                before = now;
            }
            assertTrue("thrown at " + thrown + ", a step gained " + gained + " J", gained <= 1e-12);
        }
    }

    @Test
    public void aSpinningWheelThrowsABallFedIntoItsHoodUpwardAndHalvingTheStepChangesTheThrowByLessThanAHalfPercent() {
        Exit coarse = thrown(fedAt(launcher(STEP, WHEEL_SPEED)), STEP);
        Exit fine = thrown(fedAt(launcher(STEP / 2, WHEEL_SPEED)), STEP / 2);
        double rim = WHEEL_SPEED * WHEEL_RADIUS;

        assertTrue("thrown at " + coarse.speed() + " in/s", coarse.speed() > 0.3 * rim);
        assertTrue("thrown " + coarse.degreesUp() + " degrees up", coarse.degreesUp() > 45);
        assertEquals(
                "thrown at " + coarse.speed() + " and then " + fine.speed() + " in/s",
                0,
                (fine.speed() - coarse.speed()) / coarse.speed(),
                0.005);
        assertEquals(
                "thrown " + coarse.degreesUp() + " and then " + fine.degreesUp() + " degrees up",
                coarse.degreesUp(),
                fine.degreesUp(),
                0.2);
    }

    private static final class Counted implements Surroundings.Part {
        private final Surroundings.Part inner;
        private final List<Double> asked;
        private final double seconds;

        private Counted(Surroundings.Part inner, List<Double> asked, double seconds) {
            this.inner = inner;
            this.asked = asked;
            this.seconds = seconds;
        }

        @Override
        public List<String> ids() {
            return inner.ids();
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            return inner.reaches(centre, by);
        }

        @Override
        public double sweep() {
            return inner.sweep();
        }

        @Override
        public boolean moving() {
            return inner.moving();
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            return inner.near(centre, radius, within);
        }

        @Override
        public boolean squeezes() {
            return inner.squeezes();
        }

        @Override
        public List<Surroundings.Squeeze> squeezing(Vec3 centre, Length radius) {
            asked.add(seconds);
            return inner.squeezing(centre, radius);
        }

        @Override
        public Surroundings.Part at(GridTime time) {
            return new Counted(inner.at(time), asked, time.seconds());
        }
    }

    private static Surroundings.Squeeze pressing(Compliance compliance, Vec3 normal, double depth, Vec3 velocity) {
        return Valid.value(Surroundings.Squeeze.of("plate", compliance, normal, depth, velocity));
    }

    private static CompliantContact.Push push(Vec3 velocity, Vec3 spin, Surroundings.Squeeze... squeezes) {
        return CompliantContact.push(POLLEN, velocity, spin, List.of(squeezes), stepping(STEP));
    }

    private static double newtons(double inchesPerSecondSquared) {
        return inchesPerSecondSquared * POLLEN.kilograms() * Length.METRES_PER_INCH;
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double within) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), within);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), within);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), within);
    }

    @Test
    public void aSqueezePushesTheBallAlongItsNormalAsItsDepthAndHowFastItClosesSayAndNeverPullsIt() {
        Vec3 up = new Vec3(0, 0, 1);
        double depth = 0.5 * MILLIMETRE;
        double k = HOOD.newtonsPerMetre(), c = HOOD.dampingFor(POLLEN.kilograms());
        double spring = k * depth * Length.METRES_PER_INCH;

        CompliantContact.Push resting = push(Vec3.zero(), Vec3.zero(), pressing(HOOD, up, depth, Vec3.zero()));
        assertVec("resting", new Vec3(0, 0, spring), resting.acceleration().times(newtons(1)), 1e-9);
        assertVec("resting", Vec3.zero(), resting.spinRate(), 0);

        double closing = 10;
        CompliantContact.Push falling =
                push(new Vec3(0, 0, -closing), Vec3.zero(), pressing(HOOD, up, depth, Vec3.zero()));
        double damped = spring + c * closing * Length.METRES_PER_INCH;
        assertVec("closing", new Vec3(0, 0, damped), falling.acceleration().times(newtons(1)), 1e-9);
        CompliantContact.Push pushedInto =
                push(Vec3.zero(), Vec3.zero(), pressing(HOOD, up, depth, new Vec3(0, 0, closing)));
        assertVec(
                "pushed into", new Vec3(0, 0, damped), pushedInto.acceleration().times(newtons(1)), 1e-9);

        CompliantContact.Push leaving =
                push(new Vec3(0, 3, 1000), new Vec3(5, 0, 0), pressing(HOOD, up, depth, Vec3.zero()));
        assertEquals(Vec3.zero(), leaving.acceleration());
        assertEquals(Vec3.zero(), leaving.spinRate());
    }

    @Test
    public void aSqueezeGripsAgainstTheSlipOfTheBallsSurfaceOverWhatSqueezesItAndTurnsTheBallAboutItsCentre() {
        Vec3 up = new Vec3(0, 0, 1);
        double depth = 0.5 * MILLIMETRE;
        double normal = HOOD.newtonsPerMetre() * depth * Length.METRES_PER_INCH;
        double grip = HOOD.friction() * normal;
        double inertia =
                POLLEN.inertiaFactor() * POLLEN.kilograms() * R * R * Length.METRES_PER_INCH * Length.METRES_PER_INCH;
        double lever = (R - depth / 2) * Length.METRES_PER_INCH;

        CompliantContact.Push sliding = push(new Vec3(30, 0, 0), Vec3.zero(), pressing(HOOD, up, depth, Vec3.zero()));
        assertVec("sliding", new Vec3(-grip, 0, normal), sliding.acceleration().times(newtons(1)), 1e-9);
        assertVec("sliding", new Vec3(0, lever * grip / inertia, 0), sliding.spinRate(), 1e-6);

        CompliantContact.Push rolling =
                push(new Vec3(30, 0, 0), new Vec3(0, 30 / (R - depth / 2), 0), pressing(HOOD, up, depth, Vec3.zero()));
        assertVec("rolling", new Vec3(0, 0, normal), rolling.acceleration().times(newtons(1)), 1e-9);
        assertVec("rolling", Vec3.zero(), rolling.spinRate(), 1e-9);

        CompliantContact.Push carried =
                push(new Vec3(30, -4, 0), Vec3.zero(), pressing(HOOD, up, depth, new Vec3(30, -4, 0)));
        assertVec("carried", new Vec3(0, 0, normal), carried.acceleration().times(newtons(1)), 1e-9);

        double light = 0.1 * MILLIMETRE;
        double lightly = HOOD.newtonsPerMetre() * light * Length.METRES_PER_INCH;
        double slow = 0.25 * Compliance.REGULARISATION_M_PER_S / Length.METRES_PER_INCH;
        CompliantContact.Push creeping =
                push(new Vec3(0, slow, 0), Vec3.zero(), pressing(HOOD, up, light, Vec3.zero()));
        assertVec(
                "creeping",
                new Vec3(0, -HOOD.friction() * lightly / 4, lightly),
                creeping.acceleration().times(newtons(1)),
                1e-9);
    }

    @Test
    public void rk4OnADecayIsStableWhileItsRateTimesTheStepIsUnderTheBoundTheGripIsHeldTo() {
        double under = CompliantContact.STICK_STABLE_WITHIN - 0.002,
                over = CompliantContact.STICK_STABLE_WITHIN + 0.002;

        assertTrue(CompliantContact.STICK_SOURCE, StrictMath.abs(rk4Growth(under)) < 1);
        assertTrue(CompliantContact.STICK_SOURCE, StrictMath.abs(rk4Growth(over)) > 1);
    }

    private static double rk4Growth(double rateTimesStep) {
        double z = -rateTimesStep;
        return 1 + z + z * z / 2 + z * z * z / 6 + z * z * z * z / 24;
    }

    @Test
    public void
            aBallsRegularisationSpeedIsNeverBelowWhatHoldsItsGripStableAtTheEnginesStepAndIsTheSurfacesOwnWhereThatHolds() {
        Vec3 up = new Vec3(0, 0, 1), down = new Vec3(0, 0, -1);
        double slip = 1e-4 / Length.METRES_PER_INCH;
        double perKilogram = (1 + 1 / POLLEN.inertiaFactor()) / POLLEN.kilograms();

        double light = 0.01 * MILLIMETRE;
        CompliantContact.Push touching =
                push(new Vec3(slip, 0, 0), Vec3.zero(), pressing(HOOD, up, light, Vec3.zero()));
        double lightly = HOOD.newtonsPerMetre() * light * Length.METRES_PER_INCH;
        assertEquals(
                Compliance.REGULARISATION_M_PER_S,
                HOOD.friction()
                        * lightly
                        * 1e-4
                        / -newtons(touching.acceleration().x()),
                1e-12 * Compliance.REGULARISATION_M_PER_S);

        double full = FULL_SQUEEZE / 2;
        double hard = HOOD.newtonsPerMetre() * full * Length.METRES_PER_INCH;
        double grips = (HOOD.friction() + WHEEL.friction()) * hard;
        for (double step : new double[] {STEP, STEP / 2}) {
            CompliantContact.Push held = CompliantContact.push(
                    POLLEN,
                    new Vec3(slip, 0, 0),
                    Vec3.zero(),
                    List.of(pressing(HOOD, up, full, Vec3.zero()), pressing(WHEEL, down, full, Vec3.zero())),
                    stepping(step));
            double floor = 2 * step * grips * perKilogram / CompliantContact.STICK_STABLE_WITHIN;
            assertTrue("a floor of " + floor + " m/s", floor > Compliance.REGULARISATION_M_PER_S);
            assertEquals(
                    "at a step of " + step,
                    floor,
                    grips * 1e-4 / -newtons(held.acceleration().x()),
                    1e-9 * floor);
        }
    }

    @Test
    public void aBallBeyondEveryCompliantPartsReachMovesAndCostsExactlyAsItWouldAmongPartsThatSqueezeNothing() {
        Vec3 high = HUB.plus(new Vec3(3, 20, 15)), throwing = new Vec3(5, -3, 40);
        Bodies amongCompliant =
                among(STEP, CompliantArc.spinning("wheel", HUB, AXLE, WHEEL_RADIUS, WHEEL, 300), hood());
        Bodies amongRigid = among(
                STEP,
                Carried.sliding(Carried.box("block", new Vec3(-1, -1, 1), new Vec3(1, 1, 2)), Vec3.zero(), 0),
                Carried.sliding(Carried.box("brick", new Vec3(-1, 3, 1), new Vec3(1, 4, 2)), Vec3.zero(), 0));
        add(amongCompliant, high, throwing);
        add(amongRigid, high, throwing);

        for (int i = 0; i < 5000; i++) {
            amongCompliant.step();
            amongRigid.step();
        }

        Bodies.Seen compliant = ball(amongCompliant), rigid = ball(amongRigid);
        assertTrue("fell to " + compliant.at(), compliant.at().z() < high.z() - 5);
        for (double[] same : new double[][] {
            {compliant.at().x(), rigid.at().x()},
            {compliant.at().y(), rigid.at().y()},
            {compliant.at().z(), rigid.at().z()},
            {compliant.velocity().x(), rigid.velocity().x()},
            {compliant.velocity().y(), rigid.velocity().y()},
            {compliant.velocity().z(), rigid.velocity().z()},
            {compliant.spin().x(), rigid.spin().x()},
            {compliant.spin().y(), rigid.spin().y()},
            {compliant.spin().z(), rigid.spin().z()}
        }) {
            assertEquals(Double.doubleToRawLongBits(same[1]), Double.doubleToRawLongBits(same[0]));
        }
        assertEquals(amongRigid.work(), amongCompliant.work());
    }

    @Test
    public void aSqueezedBallsIntegrationCostsAUnitForEachOfItsFourEvaluationsAndOneForEachAskingOfWhatSqueezesIt() {
        List<Double> asked = new ArrayList<>();
        Surroundings.Part squeezing = new Counted(
                Joined.of(CompliantArc.spinning("wheel", HUB, AXLE, WHEEL_RADIUS, WHEEL, 0), hood()), asked, 0);
        Bodies free = among(STEP, squeezing), held = heldAt(among(STEP, squeezing));
        add(free, HUB.plus(new Vec3(0, 20, 0)), Vec3.zero());

        free.step();
        held.step();

        assertTrue("asked " + asked.size() + " times", asked.size() >= 4);
        assertEquals("asked " + asked.size() + " times", free.work() + 3 + asked.size(), held.work());
    }

    @Test
    public void eachStageOfAStepAsksWhatSqueezesTheBallWithThePartsPlacedAtThatStagesOwnTime() {
        List<Double> asked = new ArrayList<>();
        Bodies held = heldAt(among(
                STEP,
                new Counted(
                        Joined.of(CompliantArc.spinning("wheel", HUB, AXLE, WHEEL_RADIUS, WHEEL, 0), hood()),
                        asked,
                        0)));
        stepFor(held, 7 * STEP, STEP);
        asked.clear();

        held.step();

        assertTrue("asked at " + asked, asked.size() >= 4);
        assertEquals(List.of(7 * STEP, 7.5 * STEP, 7.5 * STEP, 8 * STEP), asked.subList(0, 4));
        for (double seconds : asked.subList(4, asked.size())) {
            assertEquals("asked at " + asked, 8 * STEP, seconds, 0);
        }
    }

    @Test
    public void aBallSqueezedBetweenStillSupportsAtTheFullSqueezeFallsAsleepAndThenStaysPutExactly() {
        Bodies held = heldAt(launcher(STEP, 0));

        double asleepAfter = 0;
        while (ball(held).life() == Bodies.Life.AWAKE && asleepAfter < 1) {
            held.step();
            asleepAfter += STEP;
        }
        Bodies.Seen asleep = ball(held);
        stepFor(held, 0.5, STEP);

        assertEquals("after " + asleepAfter + " s: " + asleep, Bodies.Life.ASLEEP, asleep.life());
        assertTrue("squeezed asleep", !squeezing(held).isEmpty());
        assertEquals(asleep, ball(held));
        assertEquals(
                Double.doubleToRawLongBits(asleep.at().x()),
                Double.doubleToRawLongBits(ball(held).at().x()));
        assertEquals(
                Double.doubleToRawLongBits(asleep.at().z()),
                Double.doubleToRawLongBits(ball(held).at().z()));
    }

    @Test
    public void
            aBallSqueezedByASupportThatMovesNeverFallsAsleepEvenSlowerThanTheRestSpeedAndRestsOnlyFromWhenItStops() {
        double u = 0.8 * Shot.REST_SPEED_IN_PER_S, stops = 1;
        double floor = 20, squeeze = 1.0 * MILLIMETRE;
        Bodies pinch = among(
                STEP,
                CompliantPlate.still("plate", new Vec3(0, 0, floor), new Vec3(0, 0, 1), HOOD),
                CompliantPlate.slidingUntil(
                        "belt",
                        new Vec3(0, 0, floor + 2 * R - squeeze),
                        new Vec3(0, 0, -1),
                        WHEEL,
                        new Vec3(u, 0, 0),
                        stops));
        add(pinch, new Vec3(-30, 0, floor + R - squeeze / 2), Vec3.zero());

        stepFor(pinch, stops - 0.05, STEP);
        Bodies.Seen rolling = ball(pinch);
        stepFor(pinch, 0.05 + 0.8 * Shot.REST_SECONDS, STEP);
        Bodies.Seen stopping = ball(pinch);
        stepFor(pinch, 0.4 * Shot.REST_SECONDS, STEP);

        assertEquals(Bodies.Life.AWAKE, rolling.life());
        assertTrue(
                "slower than the rest speed: " + rolling,
                speed(rolling.velocity()) + R * speed(rolling.spin()) < Shot.REST_SPEED_IN_PER_S);
        assertEquals(
                "rolling along at " + rolling.velocity(),
                u / 2,
                rolling.velocity().x(),
                0.02 * u / 2);
        assertEquals("resting since it stopped: " + stopping, Bodies.Life.AWAKE, stopping.life());
        assertEquals("rested since it stopped", Bodies.Life.ASLEEP, ball(pinch).life());
    }

    @Test
    public void spinningTheWheelUpWakesABallAsleepInItsSqueezeAndThrowsItUpward() {
        double starts = 0.6;
        Bodies held = heldAt(among(
                STEP,
                CompliantArc.spinningUp("wheel", HUB, AXLE, WHEEL_RADIUS, WHEEL, starts, 0.05, WHEEL_SPEED),
                hood()));

        stepFor(held, starts - 0.05, STEP);
        assertEquals("before the wheel turns", Bodies.Life.ASLEEP, ball(held).life());
        stepFor(held, 0.05 + 2 * STEP, STEP);
        assertEquals("once the wheel turns", Bodies.Life.AWAKE, ball(held).life());
        Exit exit = thrown(held, STEP);

        double rim = WHEEL_SPEED * WHEEL_RADIUS;
        assertTrue("thrown at " + exit.speed() + " in/s", exit.speed() > 0.3 * rim);
        assertTrue("thrown " + exit.degreesUp() + " degrees up", exit.degreesUp() > 45);
    }
}
