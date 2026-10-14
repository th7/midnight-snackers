package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.Test;

public class MovingSurroundingsTest {
    private static final Ball POLLEN = Valid.value(Ball.of(2.8, 24.9, 0.634));
    private static final double R = POLLEN.radius();
    private static final double STEP = 1e-4;
    private static final Bodies.Start AWAKE = new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING);
    private static final Bodies.Start ASLEEP = new Bodies.Start.Asleep();
    private static final Colliders WALLS = Valid.value(Colliders.of(
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
    private static final Collider TRAY = Carried.box("tray", new Vec3(-30, -30, 10), new Vec3(30, 30, 11));
    private static final double ON_THE_TRAY = 11 + R;

    private record Unfiltered(boolean moving, Vec3 velocity, List<String> names) implements Surroundings.Part {
        @Override
        public List<String> ids() {
            return List.of("plate");
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            return StrictMath.abs(centre.x()) <= 10 + by && StrictMath.abs(centre.y()) <= 10 + by;
        }

        @Override
        public double sweep() {
            return 0;
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            List<Surroundings.Touch> touches = new ArrayList<>();
            for (String name : names) {
                touches.add(Valid.value(Surroundings.Touch.of(
                        name,
                        Surface.FRAME,
                        new Vec3(0, 0, 1),
                        centre.z() - 10 - radius.inches(),
                        velocity,
                        Vec3.zero())));
            }
            return touches;
        }

        @Override
        public Surroundings.Part at(double seconds) {
            return this;
        }
    }

    private static Surfaces surfaces() {
        Bounce bounce = Valid.value(Bounce.of(0.55, 0.5, 0.2));
        return Valid.value(Surfaces.of(bounce, bounce, bounce, bounce, bounce, bounce, 0, 4, 0, 0, 0));
    }

    private static Bodies.Step stepping(double step) {
        return Valid.value(Bodies.Step.of(Valid.value(Seconds.of(step))));
    }

    private static Surroundings around(Surroundings.Part... parts) {
        return Valid.value(Surroundings.of(WALLS, List.of(parts)));
    }

    private static Bodies among(double step, Surroundings.Part... parts) {
        return Bodies.unwatched(around(parts), Air.vacuum(), surfaces(), stepping(step));
    }

    private static Bodies watching(String name, double step, Surroundings.Part... parts) {
        return Bodies.watching(
                around(parts), Air.vacuum(), surfaces(), stepping(step), Valid.value(Bodies.Name.of(name)));
    }

    private static void add(Bodies bodies, String name, Vec3 at, Vec3 velocity, Bodies.Start start) {
        Valid.value(bodies.add(
                Valid.value(Bodies.Name.of(name)),
                POLLEN,
                Valid.value(Bodies.Motion.of(at, velocity, Vec3.zero())),
                start));
    }

    private static Bodies.Seen seen(Bodies bodies, String name) {
        return bodies.body(name).orElseThrow(() -> new AssertionError("no body " + name));
    }

    private static List<Bodies.Span> stepFor(Bodies bodies, double seconds, double step) {
        List<Bodies.Span> spans = new ArrayList<>();
        long steps = StrictMath.round(seconds / step);
        for (long i = 0; i < steps; i++) {
            spans.addAll(bodies.step());
        }
        return spans;
    }

    private static List<Contacts.Hit> hits(List<Bodies.Span> spans, String against) {
        List<Contacts.Hit> hits = new ArrayList<>();
        for (Bodies.Span span : spans) {
            span.resolved().ifPresent(resolved -> {
                for (Contacts.Hit hit : resolved.hits()) {
                    if (hit.against().equals(against)) {
                        hits.add(hit);
                    }
                }
            });
        }
        return hits;
    }

    private static Collider paddle() {
        return Carried.box("paddle", new Vec3(-39, -10, 0.5), new Vec3(-38, 10, 8));
    }

    private static double speed(Vec3 v) {
        return StrictMath.sqrt(v.dot(v));
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual, double within) {
        assertEquals(what + " x of " + actual, expected.x(), actual.x(), within);
        assertEquals(what + " y of " + actual, expected.y(), actual.y(), within);
        assertEquals(what + " z of " + actual, expected.z(), actual.z(), within);
    }

    @Test
    public void aBallRidingAPartAtThePartsOwnSpeedIsNeitherPushedAlongNorTurnedAndGoesWithIt() {
        Vec3 carried = new Vec3(12, 5, 0);
        Bodies bodies = watching("ball", STEP, Carried.sliding(TRAY, carried, 0));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), carried, AWAKE);

        List<Bodies.Span> spans = stepFor(bodies, 0.5, STEP);

        for (Bodies.Span span : spans) {
            assertTrue(
                    span.toString(),
                    span.resolved().map(resolved -> resolved.hits().isEmpty()).orElse(true));
        }
        Bodies.Seen ball = seen(bodies, "ball");
        assertVec("velocity", carried, ball.velocity(), 1e-9);
        assertVec("spin", Vec3.zero(), ball.spin(), 1e-9);
        assertVec("place", new Vec3(6, 2.5, ON_THE_TRAY), ball.at(), 1e-6);
        assertEquals(Bodies.Life.AWAKE, ball.life());
    }

    @Test
    public void aBallAsleepOnAPartWakesWhenThePartStartsToMoveAndGoesUpWithItWithoutBouncing() {
        Bodies bodies = watching("ball", STEP, Carried.sliding(TRAY, new Vec3(0, 0, 6), 0.1));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), ASLEEP);

        stepFor(bodies, 0.05, STEP);
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "ball").life());
        List<Bodies.Span> spans = stepFor(bodies, 0.55, STEP);

        List<Bodies.Seen> woken = new ArrayList<>();
        for (Bodies.Span span : spans) {
            woken.addAll(span.woken());
            assertTrue(
                    span.toString(),
                    span.resolved().map(resolved -> resolved.hits().isEmpty()).orElse(true));
        }
        assertEquals(1, woken.size());
        assertEquals(new Vec3(0, 0, ON_THE_TRAY), woken.get(0).at());
        assertEquals(Bodies.Life.AWAKE, woken.get(0).life());
        Bodies.Seen ball = seen(bodies, "ball");
        assertEquals(Bodies.Life.AWAKE, ball.life());
        assertVec("place", new Vec3(0, 0, ON_THE_TRAY + 6 * 0.5), ball.at(), 1e-3);
        assertVec("velocity", new Vec3(0, 0, 6), ball.velocity(), 1e-6);
    }

    @Test
    public void aBallAsleepOnABallAsleepOnAPartWakesWithItWhenThePartStartsToMoveAndBothGoUp() {
        Bodies bodies = watching("under", STEP, Carried.sliding(TRAY, new Vec3(0, 0, 6), 0.5));
        add(bodies, "under", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), AWAKE);
        add(bodies, "over", new Vec3(0, 0, ON_THE_TRAY + 2 * R), Vec3.zero(), AWAKE);
        stepFor(bodies, 0.4, STEP);
        for (String name : List.of("under", "over")) {
            assertEquals(name, Bodies.Life.ASLEEP, seen(bodies, name).life());
        }

        List<Bodies.Span> spans = stepFor(bodies, 0.6, STEP);

        List<String> woken = new ArrayList<>();
        List<Double> when = new ArrayList<>();
        for (Bodies.Span span : spans) {
            for (Bodies.Seen seen : span.woken()) {
                woken.add(seen.name());
                when.add(span.now());
            }
        }
        assertEquals(List.of("under", "over"), woken);
        assertEquals(when.get(0), when.get(1));

        for (String name : List.of("under", "over")) {
            assertEquals(name, Bodies.Life.AWAKE, seen(bodies, name).life());
        }
        assertEquals(ON_THE_TRAY + 3, seen(bodies, "under").at().z(), 1e-3);
        assertEquals(ON_THE_TRAY + 3 + 2 * R, seen(bodies, "over").at().z(), 1e-2);
        assertEquals(6, seen(bodies, "over").velocity().z(), 1e-2);
    }

    @Test
    public void aBallOnAPartMovingMoreSlowlyThanABallAtRestNeverFallsAsleepOnIt() {
        Vec3 creeping = new Vec3(0.2, 0, 0);
        assertTrue(speed(creeping) < Shot.REST_SPEED_IN_PER_S);
        Bodies bodies = watching("ball", STEP, Carried.sliding(TRAY, creeping, 0));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), creeping, AWAKE);

        List<Bodies.Span> spans = stepFor(bodies, 4 * Shot.REST_SECONDS, STEP);

        for (Bodies.Span span : spans) {
            assertTrue("asleep at " + span.now(), span.slept().isEmpty());
        }
        Bodies.Seen ball = seen(bodies, "ball");
        assertEquals(Bodies.Life.AWAKE, ball.life());
        assertEquals(0, ball.restFor(), 0);
        assertVec("place", new Vec3(0.2, 0, ON_THE_TRAY), ball.at(), 1e-6);
    }

    @Test
    public void aBallStackedOnABallOnAPartMovingMoreSlowlyThanABallAtRestRidesAlongAndNeitherFallsAsleep() {
        Vec3 creeping = new Vec3(0.2, 0, 0);
        Bodies bodies = watching("over", STEP, Carried.sliding(TRAY, creeping, 0));
        add(bodies, "over", new Vec3(0, 0, ON_THE_TRAY + 2 * R), creeping, AWAKE);
        add(bodies, "under", new Vec3(0, 0, ON_THE_TRAY), creeping, AWAKE);

        List<Bodies.Span> spans = stepFor(bodies, 2, STEP);

        for (Bodies.Span span : spans) {
            assertTrue(
                    "asleep at " + span.now() + ": " + span.slept(),
                    span.slept().isEmpty());
        }
        assertVec("under", new Vec3(0.4, 0, ON_THE_TRAY), seen(bodies, "under").at(), 1e-6);
        assertVec(
                "over",
                new Vec3(0.4, 0, ON_THE_TRAY + 2 * R),
                seen(bodies, "over").at(),
                1e-6);
    }

    @Test
    public void aPyramidOfBallsOnAPartMovingMoreSlowlyThanABallAtRestKeepsThePartsVelocityAndNoneFallsAsleep() {
        Vec3 creeping = new Vec3(0.2, 0, 0);
        Bodies bodies = watching("top", STEP, Carried.sliding(TRAY, creeping, 0));
        add(bodies, "left", new Vec3(-R, 0, ON_THE_TRAY), creeping, AWAKE);
        add(bodies, "right", new Vec3(R, 0, ON_THE_TRAY), creeping, AWAKE);
        add(bodies, "top", new Vec3(0, 0, ON_THE_TRAY + StrictMath.sqrt(3) * R), creeping, AWAKE);

        List<Bodies.Span> spans = stepFor(bodies, 3, STEP);

        for (Bodies.Span span : spans) {
            assertTrue(
                    "asleep at " + span.now() + ": " + span.slept(),
                    span.slept().isEmpty());
        }
        for (String name : List.of("left", "right", "top")) {
            assertEquals(name, Bodies.Life.AWAKE, seen(bodies, name).life());
            assertVec(name, creeping, seen(bodies, name).velocity(), 1e-4);
        }
        assertVec(
                "top",
                new Vec3(0.6, 0, ON_THE_TRAY + StrictMath.sqrt(3) * R),
                seen(bodies, "top").at(),
                1e-2);
    }

    @Test
    public void aBallRestingOnAMovingPartAndAgainstAStillPartListedAfterItNeverFallsAsleep() {
        Vec3 creeping = new Vec3(0, 0.2, 0);
        Collider post = Carried.box("post", new Vec3(R, -30, 11), new Vec3(R + 1, 30, 20));
        Bodies bodies =
                watching("ball", STEP, Carried.sliding(TRAY, creeping, 0), Carried.sliding(post, Vec3.zero(), 0));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), creeping, AWAKE);
        assertEquals(
                Set.of("tray", "post"), bodies.touching("ball").orElseThrow().colliders());

        List<Bodies.Span> spans = stepFor(bodies, 4 * Shot.REST_SECONDS, STEP);

        for (Bodies.Span span : spans) {
            assertTrue("asleep at " + span.now(), span.slept().isEmpty());
        }
        assertEquals(Bodies.Life.AWAKE, seen(bodies, "ball").life());
        assertEquals(0, seen(bodies, "ball").restFor(), 0);
    }

    @Test
    public void aPartSweepingFurtherInAStepThanABallIsWideMeetsABallAtRestWhereItsFaceReachesIt() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600;
        assertTrue(speed * step > 2 * R + 1);
        Collider paddle = Carried.box("paddle", new Vec3(-39, -10, 0.5), new Vec3(-38, 10, 8));
        Bodies bodies = watching("ball", step, Carried.sliding(paddle, new Vec3(speed, 0, 0), 0));
        add(bodies, "ball", new Vec3(0.05, 0, R), Vec3.zero(), ASLEEP);

        List<Bodies.Span> spans = stepFor(bodies, 0.1, step);

        List<Bodies.Span> struck = new ArrayList<>();
        for (Bodies.Span span : spans) {
            span.resolved().ifPresent(resolved -> {
                for (Contacts.Hit hit : resolved.hits()) {
                    if (hit.against().equals("paddle")) {
                        struck.add(span);
                    }
                }
            });
        }
        assertEquals(1, struck.size());
        Bodies.Span hit = struck.get(0);
        assertEquals("met at", -38 + speed * hit.now() + R, hit.atResolve().at().x(), 1e-6);
        Bodies.Seen ball = seen(bodies, "ball");
        assertTrue(ball.at() + " is not ahead of the paddle", ball.at().x() > -38 + speed * bodies.now() + R);
        assertTrue(
                ball.velocity() + " is not going faster than the paddle",
                ball.velocity().x() > speed);
    }

    @Test
    public void aPartThatStartsMovingInsideAStepMeetsAnAwakeBallAtItsFace() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600, starts = 1e-4;
        Bodies bodies = watching("ball", step, Carried.sliding(paddle(), new Vec3(speed, 0, 0), starts));
        add(bodies, "ball", new Vec3(-38 + 2 + R, 0, R), Vec3.zero(), new Bodies.Start.Awake(Bodies.Sleep.NEVER));

        List<Bodies.Span> spans = stepFor(bodies, 0.03, step);

        List<Contacts.Hit> struck = hits(spans, "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
        Bodies.Seen ball = seen(bodies, "ball");
        assertTrue(
                ball.at() + " is not ahead of the paddle", ball.at().x() > -38 + speed * (bodies.now() - starts) + R);
        assertTrue(
                ball.velocity() + " is not going faster than the paddle",
                ball.velocity().x() > speed);
    }

    @Test
    public void aPartThatStartsMovingInsideAStepWakesASleepingBallItWillReachBeforeTheStepMovesAndMeetsItAtItsFace() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600, starts = 0.00105;
        Vec3 resting = new Vec3(-38 + 0.05 + R, 0, R);
        Bodies bodies = watching("ball", step, Carried.sliding(paddle(), new Vec3(speed, 0, 0), starts));
        add(bodies, "ball", resting, Vec3.zero(), ASLEEP);

        List<Bodies.Span> spans = stepFor(bodies, 0.03, step);

        List<Bodies.Span> waking = new ArrayList<>();
        for (Bodies.Span span : spans) {
            if (!span.woken().isEmpty()) {
                waking.add(span);
            }
        }
        List<Contacts.Hit> struck = hits(spans, "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(1, waking.size());
        assertEquals(0, waking.get(0).began(), 0);
        assertEquals(resting, waking.get(0).woken().get(0).at());
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
        Bodies.Seen ball = seen(bodies, "ball");
        assertTrue(
                ball.at() + " is not ahead of the paddle", ball.at().x() > -38 + speed * (bodies.now() - starts) + R);
        assertTrue(
                ball.velocity() + " is not going faster than the paddle",
                ball.velocity().x() > speed);
    }

    @Test
    public void aPartThatStopsMovingInsideAStepWakesASleepingBallItWillReachBeforeTheStepMovesAndMeetsItAtItsFace() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600, stops = 0.009;
        Bodies bodies = watching("ball", step, Carried.slidingUntil(paddle(), new Vec3(speed, 0, 0), 0, stops));
        add(bodies, "ball", new Vec3(-38 + 0.05 + R, 0, R), Vec3.zero(), ASLEEP);

        List<Bodies.Span> spans = stepFor(bodies, 0.03, step);

        List<Contacts.Hit> struck = hits(spans, "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
        assertEquals(1, spans.get(0).woken().size());
    }

    @Test
    public void aPartMovingOnlyBetweenTheEndsOfAStepCutFinerThanItsMotionWakesASleepingBallAtAPiecesEndAndMeetsIt() {
        double step = World.LONGEST_STEP_SECONDS, speed = 100;
        Bodies bodies = watching("ball", step, Carried.slidingUntil(paddle(), new Vec3(speed, 0, 0), 0.0021, 0.0079));
        add(bodies, "ball", new Vec3(-38 + 0.2 + R, 0, R), Vec3.zero(), ASLEEP);
        add(bodies, "beside", new Vec3(-38 + 0.2 + 3 * R, 0, R), Vec3.zero(), ASLEEP);
        add(bodies, "fast", new Vec3(0, 40, 20), new Vec3(600, 0, 0), new Bodies.Start.Awake(Bodies.Sleep.NEVER));

        List<Bodies.Span> spans = stepFor(bodies, 0.03, step);

        List<Contacts.Hit> struck = hits(spans, "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
        List<String> woken = new ArrayList<>();
        List<Double> when = new ArrayList<>();
        for (Bodies.Span span : spans) {
            for (Bodies.Seen seen : span.woken()) {
                woken.add(seen.name());
                when.add(span.now());
            }
        }
        assertEquals(List.of("ball", "beside"), woken);
        assertEquals(when.get(0), when.get(1));
    }

    @Test
    public void aPartThatStopsMovingInsideAStepMeetsAnAwakeBallAtItsFace() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600, stops = 0.009;
        assertTrue(speed * stops > 0.5 + 2 * R + 1);
        Bodies bodies = watching("ball", step, Carried.slidingUntil(paddle(), new Vec3(speed, 0, 0), 0, stops));
        add(bodies, "ball", new Vec3(-38 + 0.5 + R, 0, R), Vec3.zero(), new Bodies.Start.Awake(Bodies.Sleep.NEVER));

        List<Bodies.Span> spans = stepFor(bodies, 0.03, step);

        List<Contacts.Hit> struck = hits(spans, "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
        assertTrue(
                seen(bodies, "ball").velocity().toString(),
                seen(bodies, "ball").velocity().x() > speed);
    }

    @Test
    public void aBallThatComesToRestWithinAStepsReachOfAnApproachingPartStaysAwakeAndIsStruckNotCarried() {
        double speed = 40, gapWhenRested = 0.002, face = -gapWhenRested - speed * Shot.REST_SECONDS;
        assertTrue(gapWhenRested < speed * STEP);
        Collider closing = Carried.box("paddle", new Vec3(face - 1, -10, 0.5), new Vec3(face, 10, 8));
        Bodies bodies = watching("ball", STEP, Carried.sliding(closing, new Vec3(speed, 0, 0), 0));
        add(bodies, "ball", new Vec3(R, 0, R), Vec3.zero(), AWAKE);

        List<Bodies.Span> spans = stepFor(bodies, 0.3, STEP);

        List<Vec3> thrown = new ArrayList<>();
        for (Bodies.Span span : spans) {
            assertTrue("asleep at " + span.now(), span.slept().isEmpty());
            if (!hits(List.of(span), "paddle").isEmpty()) {
                thrown.add(span.atResolve().velocity());
            }
        }
        List<Contacts.Hit> struck = hits(spans, "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
        assertTrue(thrown.toString(), thrown.get(0).x() > speed);
    }

    @Test
    public void aPartThatOnlySpinsAddsNoPiecesToAStepAndKeepsWhatRestsOnItAwake() {
        Bodies bodies =
                watching("ball", STEP, Carried.spinningInPlace(TRAY, new Vec3(0, 0, 11), new Vec3(0, 0, 100), 0));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), AWAKE);

        for (int i = 0; i < 4 * Shot.REST_SECONDS / STEP; i++) {
            List<Bodies.Span> spans = bodies.step();

            assertEquals("pieces of step " + i, 1, spans.size());
            assertTrue("asleep at step " + i, spans.get(0).slept().isEmpty());
        }
        assertEquals(Bodies.Life.AWAKE, seen(bodies, "ball").life());
    }

    @Test
    public void aBallAsleepFarFromAMovingPartCostsAnEngineWithNothingAwakeNothingAndOneWithABallAwakeItsBoundsChecks() {
        Surroundings.Part sliding = Carried.sliding(
                Carried.box("far", new Vec3(-60, -60, 10), new Vec3(-50, -50, 11)), new Vec3(10, 0, 0), 0);
        Vec3 far = new Vec3(40, 40, R), lane = new Vec3(0, -40, R), rolling = new Vec3(20, 0, 0);
        Bodies alone = among(STEP, sliding);
        add(alone, "far", far, Vec3.zero(), ASLEEP);
        Bodies moving = among(STEP, sliding);
        add(moving, "far", far, Vec3.zero(), ASLEEP);
        add(moving, "rolling", lane, rolling, AWAKE);
        Bodies still = among(STEP);
        add(still, "far", far, Vec3.zero(), ASLEEP);
        add(still, "rolling", lane, rolling, AWAKE);

        stepFor(alone, 0.1, STEP);
        stepFor(moving, 0.1, STEP);
        stepFor(still, 0.1, STEP);

        int boundsChecks = 3, placements = 2, placementsForTheRollingBallsTrial = 1;
        assertEquals(Bodies.Life.ASLEEP, seen(alone, "far").life());
        assertEquals(0, alone.work());
        assertEquals(Bodies.Life.ASLEEP, seen(moving, "far").life());
        assertEquals(
                still.work() + moving.steps() * (boundsChecks + placements + placementsForTheRollingBallsTrial),
                moving.work());
        assertEquals(still.seen(), moving.seen());
    }

    @Test
    public void aBallAsleepOnATrayThatTurnsOverAboutAPivotWakesFallsOffAndComesDownOnTheFloor() {
        Collider tray = Carried.box("tray", new Vec3(-3, -3, 10), new Vec3(3, 3, 10.5));
        Surroundings.Part tipping = Carried.turning(tray, new Vec3(0, 0, 10.5), new Vec3(0, 3, 0), 0.1);
        Surroundings surroundings = around(tipping);
        Bodies bodies = Bodies.unwatched(surroundings, Air.vacuum(), surfaces(), stepping(STEP));
        add(bodies, "ball", new Vec3(0, 0, 10.5 + R), Vec3.zero(), ASLEEP);

        double deepest = 0;
        for (int i = 0; i < 3 / STEP; i++) {
            bodies.step();
            Vec3 at = seen(bodies, "ball").at();
            for (Surroundings.Touch touch : surroundings.at(bodies.now()).near(at, POLLEN.radiusLength(), 1)) {
                if (touch.id().equals("tray")) {
                    deepest = StrictMath.min(deepest, touch.gap());
                }
            }
            if (i == (int) (0.15 / STEP)) {
                assertEquals(Bodies.Life.AWAKE, seen(bodies, "ball").life());
            }
        }

        assertTrue("the tray went " + -deepest + " in into the ball", deepest > -2e-3);
        assertEquals(R, seen(bodies, "ball").at().z(), 1e-3);
    }

    @Test
    public void aBallAsleepOnAPlatterWakesWhenItStartsToSpinAndIsDraggedTheWayItsSurfaceGoes() {
        Bodies bodies = among(STEP, Carried.spinningInPlace(TRAY, new Vec3(0, 0, 11), new Vec3(0, 0, 2), 0.1));
        add(bodies, "ball", new Vec3(10, 0, ON_THE_TRAY), Vec3.zero(), ASLEEP);

        stepFor(bodies, 0.05, STEP);
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "ball").life());
        stepFor(bodies, 0.45, STEP);

        Bodies.Seen ball = seen(bodies, "ball");
        assertEquals(Bodies.Life.AWAKE, ball.life());
        assertTrue(ball.velocity().toString(), ball.velocity().y() > 1);
        assertTrue(ball.at().toString(), ball.at().y() > 0.1);
    }

    @Test
    public void aBallLetGoAboveAPartThatAnswersBeyondTheReachAskedForFallsUntilItRestsOnIt() {
        Bodies bodies = among(STEP, new Unfiltered(false, Vec3.zero(), List.of("plate")));
        add(bodies, "ball", new Vec3(0, 0, 10 + R + 5), Vec3.zero(), AWAKE);

        stepFor(bodies, 1.5, STEP);

        Bodies.Seen ball = seen(bodies, "ball");
        assertEquals(10 + R, ball.at().z(), 1e-3);
        assertEquals(Bodies.Life.ASLEEP, ball.life());
    }

    @Test
    public void aBallAsleepIsNotWokenByAMovingPartsTouchBeyondItsReachOrUnderANameThePartNeverGave() {
        Vec3 rising = new Vec3(0, 0, 3);
        Bodies beyond = among(STEP, new Unfiltered(true, rising, List.of("plate")));
        add(beyond, "ball", new Vec3(0, 0, 10 + R + 5), Vec3.zero(), ASLEEP);
        Bodies misnamed = among(STEP, new Unfiltered(true, rising, List.of("Floor")));
        add(misnamed, "ball", new Vec3(0, 0, 10 + R), Vec3.zero(), ASLEEP);

        stepFor(beyond, 0.1, STEP);
        stepFor(misnamed, 0.1, STEP);

        assertEquals(Bodies.Life.ASLEEP, seen(beyond, "ball").life());
        assertEquals(Bodies.Life.ASLEEP, seen(misnamed, "ball").life());
    }

    @Test
    public void aBallAsleepOnAStillFaceOfAPartThatMovesElsewhereSleepsOnWhileTheBallOnTheMovingFaceWakes() {
        Collider blue = Carried.box("blue", new Vec3(-33, -3, 10), new Vec3(-27, 3, 10.5));
        Collider red = Carried.box("red", new Vec3(27, -3, 10), new Vec3(33, 3, 10.5));
        Surroundings.Part hives = Joined.of(
                Carried.turning(blue, new Vec3(-30, 0, 10.5), new Vec3(0, 1.645, 0), 0.1),
                Carried.sliding(red, Vec3.zero(), 0));
        Bodies bodies = watching("resting", STEP, hives);
        add(bodies, "resting", new Vec3(30, 0, 10.5 + R), Vec3.zero(), ASLEEP);
        add(bodies, "tipped", new Vec3(-30, 0, 10.5 + R), Vec3.zero(), ASLEEP);

        List<Bodies.Span> spans = stepFor(bodies, 1.1, STEP);

        List<String> woken = new ArrayList<>(), slept = new ArrayList<>();
        for (Bodies.Span span : spans) {
            for (Bodies.Seen seen : span.woken()) {
                woken.add(seen.name());
            }
            for (Bodies.Seen seen : span.slept()) {
                slept.add(seen.name());
            }
        }
        assertEquals(List.of("tipped"), woken);
        assertTrue(slept.toString(), !slept.contains("resting"));
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "resting").life());
    }

    @Test
    public void surroundingsGivenToAnEngineAreWhereTheyAreAtTheEnginesTime() {
        Bodies bodies = among(STEP);
        stepFor(bodies, 0.2, STEP);

        bodies.surroundedBy(around(Carried.sliding(TRAY, new Vec3(0, 0, 6), 0)));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY + 6 * 0.2), Vec3.zero(), AWAKE);

        assertEquals(Set.of("tray"), bodies.touching("ball").orElseThrow().colliders());
    }

    @Test
    public void surroundingsGivenWithAPartAlreadyMovingWakeAtOnceWhatItCouldReachWithinTheComingStepAndStrikeIt() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600;
        Collider paddle = Carried.box("paddle", new Vec3(-1, -10, 0.5), new Vec3(0, 10, 8));
        Bodies bodies = watching("ball", step, Carried.sliding(paddle, Vec3.zero(), 0));
        add(bodies, "ball", new Vec3(0.05 + R, 0, R), Vec3.zero(), ASLEEP);
        stepFor(bodies, 0.05, step);
        Bodies.Seen resting = seen(bodies, "ball");

        List<Bodies.Seen> woken =
                bodies.surroundedBy(around(Carried.sliding(paddle, new Vec3(speed, 0, 0), bodies.now())));

        assertEquals(Bodies.Life.AWAKE, seen(bodies, "ball").life());
        assertEquals(List.of("ball"), List.of(woken.get(0).name()));
        assertEquals(resting.at(), woken.get(0).at());
        assertEquals(Set.of("Floor"), bodies.touching("ball").orElseThrow().colliders());
        List<Contacts.Hit> struck = hits(stepFor(bodies, 0.03, step), "paddle");
        assertEquals(struck.toString(), 1, struck.size());
        assertEquals(speed, struck.get(0).approach(), 1e-6);
    }

    @Test
    public void surroundingsGivenToAnEngineAreWhatItsAwakeBodiesKnowTheyTouchAtOnce() {
        Collider plate = Carried.box("plate", new Vec3(-30, -30, 10), new Vec3(30, 30, 11));
        Bodies bodies = among(STEP, Carried.sliding(TRAY, Vec3.zero(), 0));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), AWAKE);
        assertEquals(Set.of("tray"), bodies.touching("ball").orElseThrow().colliders());

        List<Bodies.Seen> woken = bodies.surroundedBy(around(Carried.sliding(plate, Vec3.zero(), 0)));

        assertTrue(woken.isEmpty());
        assertEquals(Set.of("plate"), bodies.touching("ball").orElseThrow().colliders());
    }

    @Test
    public void anEngineWithNothingAwakeIdlesThroughToALaterStepExactlyWhereSteppingWouldLeaveItAndNeitherIsWork() {
        Surroundings.Part far = Carried.sliding(
                Carried.box("far", new Vec3(-60, -60, 10), new Vec3(-50, -50, 11)), new Vec3(10, 0, 0), 0);
        Bodies stepped = among(STEP, far, Carried.sliding(TRAY, Vec3.zero(), 0));
        add(stepped, "resting", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), ASLEEP);
        add(stepped, "lying", new Vec3(40, 40, R), Vec3.zero(), ASLEEP);
        add(stepped, "held", new Vec3(40, 0, R), Vec3.zero(), new Bodies.Start.Fixed());
        stepFor(stepped, 0.01, STEP);
        Bodies idled = stepped.copy();
        long target = stepped.steps() + 2000, before = stepped.work();

        stepFor(stepped, 0.2, STEP);
        boolean idle = idled.idleThrough(target);

        assertTrue(idle);
        assertEquals(stepped.seen(), idled.seen());
        assertEquals(stepped.steps(), idled.steps());
        assertEquals(Double.doubleToRawLongBits(stepped.now()), Double.doubleToRawLongBits(idled.now()));
        for (String name : List.of("resting", "lying", "held")) {
            assertEquals(name, stepped.touching(name), idled.touching(name));
        }
        assertEquals("jumping", before, idled.work());
        assertEquals("stepping", before, stepped.work());
        stepFor(stepped, 0.01, STEP);
        stepFor(idled, 0.01, STEP);
        assertEquals(stepped.seen(), idled.seen());
    }

    @Test
    public void anEngineWithABodyAwakeDoesNotIdleAndIsLeftAsItWas() {
        Bodies bodies = among(STEP);
        add(bodies, "rolling", new Vec3(0, 0, R), new Vec3(1, 0, 0), AWAKE);
        add(bodies, "lying", new Vec3(40, 40, R), Vec3.zero(), ASLEEP);
        List<Bodies.Seen> before = bodies.seen();

        assertTrue(!bodies.idleThrough(bodies.steps() + 10));
        assertEquals(before, bodies.seen());
        assertEquals(0, bodies.steps());
        assertEquals(0, bodies.now(), 0);
    }

    @Test
    public void anEngineDoesNotIdleThroughStepsInWhichAMovingFaceCouldReachASleepingBodyAndAskingIsNoWork() {
        double speed = 10, gap = 1;
        Collider closing = Carried.box("paddle", new Vec3(-gap - 1, -10, 0.5), new Vec3(-gap, 10, 8));
        Bodies bodies = among(STEP, Carried.sliding(closing, new Vec3(speed, 0, 0), 0));
        add(bodies, "ball", new Vec3(R, 0, R), Vec3.zero(), ASLEEP);

        assertTrue(bodies.idleThrough((long) (0.4 * gap / speed / STEP)));
        long reached = bodies.steps();
        assertTrue(!bodies.idleThrough(reached + (long) (0.7 * gap / speed / STEP)));
        assertTrue(!bodies.idleThrough(reached + (long) ((gap + 2 * R + 1) / speed / STEP)));
        assertEquals(reached, bodies.steps());
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "ball").life());
        assertEquals("idling or refusing to is no work", 0, bodies.work());
    }

    @Test
    public void anEngineDoesNotIdleThroughStepsInWhichAPartStartsOrStopsMovingWithinReachOfASleepingBody() {
        Collider near = Carried.box("paddle", new Vec3(-1.1, -10, 0.5), new Vec3(-0.1, 10, 8));
        Bodies starting = among(STEP, Carried.sliding(near, new Vec3(10, 0, 0), 0.05));
        add(starting, "ball", new Vec3(R, 0, R), Vec3.zero(), ASLEEP);
        Bodies stopping = among(STEP, Carried.slidingUntil(near, new Vec3(-10, 0, 0), 0, 0.05));
        add(stopping, "ball", new Vec3(R, 0, R), Vec3.zero(), ASLEEP);

        assertTrue(!starting.idleThrough(1000));
        assertTrue(!stopping.idleThrough(1000));
        assertTrue(starting.idleThrough(400));
    }

    @Test
    public void anEngineIdlesThroughABallAsleepOnAStillFaceOfAMovingPart() {
        Collider blue = Carried.box("blue", new Vec3(-33, -3, 10), new Vec3(-27, 3, 10.5));
        Collider red = Carried.box("red", new Vec3(27, -3, 10), new Vec3(33, 3, 10.5));
        Bodies bodies = among(
                STEP,
                Joined.of(
                        Carried.turning(blue, new Vec3(-30, 0, 10.5), new Vec3(0, 1.645, 0), 0),
                        Carried.sliding(red, Vec3.zero(), 0)));
        add(bodies, "resting", new Vec3(30, 0, 10.5 + R), Vec3.zero(), ASLEEP);

        assertTrue(bodies.idleThrough(1000));
        assertEquals(1000, bodies.steps());
    }

    @Test
    public void anEngineIdlesThroughToWhereItIsAlreadyAndNeverBack() {
        Bodies bodies = among(STEP);
        add(bodies, "rolling", new Vec3(0, 0, R), new Vec3(1, 0, 0), AWAKE);
        stepFor(bodies, 0.001, STEP);
        long steps = bodies.steps();
        double now = bodies.now();

        assertTrue(bodies.idleThrough(steps));
        assertTrue(!bodies.idleThrough(steps - 1));
        assertEquals(steps, bodies.steps());
        assertEquals(now, bodies.now(), 0);
    }

    @Test
    public void aCopyAmongMovingPartsGoesOnExactlyAsTheOriginal() {
        Collider tray = Carried.box("tray", new Vec3(-3, -3, 10), new Vec3(3, 3, 10.5));
        Bodies bodies = among(STEP, Carried.turning(tray, new Vec3(0, 0, 10.5), new Vec3(0, 3, 0), 0.1));
        add(bodies, "ball", new Vec3(0, 0, 10.5 + R), Vec3.zero(), ASLEEP);
        stepFor(bodies, 0.2, STEP);

        Bodies copy = bodies.copy();
        stepFor(bodies, 0.3, STEP);
        stepFor(copy, 0.3, STEP);

        assertEquals(bodies.seen(), copy.seen());
        assertEquals(bodies.work(), copy.work());
        assertTrue(seen(bodies, "ball").at().z() < 10.5 + R - 0.1);
    }
}
