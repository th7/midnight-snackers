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
        return Bodies.watching(around(parts), Air.vacuum(), surfaces(), stepping(step), name);
    }

    private static void add(Bodies bodies, String name, Vec3 at, Vec3 velocity, Bodies.Life life) {
        Valid.value(bodies.add(name, POLLEN, at, velocity, Vec3.zero(), life, Bodies.Sleep.WHEN_RESTING));
        bodies.touchingAtTheStart();
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
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), carried, Bodies.Life.AWAKE);

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
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), Bodies.Life.ASLEEP);

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
        Bodies bodies = among(STEP, Carried.sliding(TRAY, new Vec3(0, 0, 6), 0.1));
        add(bodies, "under", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), Bodies.Life.ASLEEP);
        add(bodies, "over", new Vec3(0, 0, ON_THE_TRAY + 2 * R), Vec3.zero(), Bodies.Life.ASLEEP);

        stepFor(bodies, 0.6, STEP);

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
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), creeping, Bodies.Life.AWAKE);

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
    public void aPartSweepingFurtherInAStepThanABallIsWideMeetsABallAtRestWhereItsFaceReachesIt() {
        double step = World.LONGEST_STEP_SECONDS, speed = 600;
        assertTrue(speed * step > 2 * R + 1);
        Collider paddle = Carried.box("paddle", new Vec3(-39, -10, 0.5), new Vec3(-38, 10, 8));
        Bodies bodies = watching("ball", step, Carried.sliding(paddle, new Vec3(speed, 0, 0), 0));
        add(bodies, "ball", new Vec3(0.05, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);

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
    public void aPartThatOnlySpinsAddsNoPiecesToAStepAndKeepsWhatRestsOnItAwake() {
        Bodies bodies =
                watching("ball", STEP, Carried.spinningInPlace(TRAY, new Vec3(0, 0, 11), new Vec3(0, 0, 100), 0));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY), Vec3.zero(), Bodies.Life.AWAKE);

        for (int i = 0; i < 4 * Shot.REST_SECONDS / STEP; i++) {
            List<Bodies.Span> spans = bodies.step();

            assertEquals("pieces of step " + i, 1, spans.size());
            assertTrue("asleep at step " + i, spans.get(0).slept().isEmpty());
        }
        assertEquals(Bodies.Life.AWAKE, seen(bodies, "ball").life());
    }

    @Test
    public void aBallAsleepFarFromAMovingPartStaysAsleepAndCostsTheEngineNoQuery() {
        Surroundings.Part sliding = Carried.sliding(
                Carried.box("far", new Vec3(-60, -60, 10), new Vec3(-50, -50, 11)), new Vec3(10, 0, 0), 0);
        Vec3 far = new Vec3(40, 40, R), lane = new Vec3(0, -40, R), rolling = new Vec3(20, 0, 0);
        Bodies alone = among(STEP, sliding);
        add(alone, "far", far, Vec3.zero(), Bodies.Life.ASLEEP);
        Bodies moving = among(STEP, sliding);
        add(moving, "far", far, Vec3.zero(), Bodies.Life.ASLEEP);
        add(moving, "rolling", lane, rolling, Bodies.Life.AWAKE);
        Bodies still = among(STEP);
        add(still, "far", far, Vec3.zero(), Bodies.Life.ASLEEP);
        add(still, "rolling", lane, rolling, Bodies.Life.AWAKE);

        stepFor(alone, 0.1, STEP);
        stepFor(moving, 0.1, STEP);
        stepFor(still, 0.1, STEP);

        assertEquals(Bodies.Life.ASLEEP, seen(alone, "far").life());
        assertEquals(0, alone.work());
        assertEquals(Bodies.Life.ASLEEP, seen(moving, "far").life());
        assertEquals(still.work(), moving.work());
        assertEquals(still.seen(), moving.seen());
    }

    @Test
    public void aBallAsleepOnATrayThatTurnsOverAboutAPivotWakesFallsOffAndComesDownOnTheFloor() {
        Collider tray = Carried.box("tray", new Vec3(-3, -3, 10), new Vec3(3, 3, 10.5));
        Surroundings.Part tipping = Carried.turning(tray, new Vec3(0, 0, 10.5), new Vec3(0, 3, 0), 0.1);
        Surroundings surroundings = around(tipping);
        Bodies bodies = Bodies.unwatched(surroundings, Air.vacuum(), surfaces(), stepping(STEP));
        add(bodies, "ball", new Vec3(0, 0, 10.5 + R), Vec3.zero(), Bodies.Life.ASLEEP);

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
        add(bodies, "ball", new Vec3(10, 0, ON_THE_TRAY), Vec3.zero(), Bodies.Life.ASLEEP);

        stepFor(bodies, 0.05, STEP);
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "ball").life());
        stepFor(bodies, 0.45, STEP);

        Bodies.Seen ball = seen(bodies, "ball");
        assertEquals(Bodies.Life.AWAKE, ball.life());
        assertTrue(ball.velocity().toString(), ball.velocity().y() > 1);
        assertTrue(ball.at().toString(), ball.at().y() > 0.1);
    }

    @Test
    public void surroundingsGivenToAnEngineAreWhereTheyAreAtTheEnginesTime() {
        Bodies bodies = among(STEP);
        stepFor(bodies, 0.2, STEP);

        bodies.surroundedBy(around(Carried.sliding(TRAY, new Vec3(0, 0, 6), 0)));
        add(bodies, "ball", new Vec3(0, 0, ON_THE_TRAY + 6 * 0.2), Vec3.zero(), Bodies.Life.AWAKE);

        assertEquals(Set.of("tray"), bodies.touching("ball").orElseThrow().colliders());
    }

    @Test
    public void aCopyAmongMovingPartsGoesOnExactlyAsTheOriginal() {
        Collider tray = Carried.box("tray", new Vec3(-3, -3, 10), new Vec3(3, 3, 10.5));
        Bodies bodies = among(STEP, Carried.turning(tray, new Vec3(0, 0, 10.5), new Vec3(0, 3, 0), 0.1));
        add(bodies, "ball", new Vec3(0, 0, 10.5 + R), Vec3.zero(), Bodies.Life.ASLEEP);
        stepFor(bodies, 0.2, STEP);

        Bodies copy = bodies.copy();
        stepFor(bodies, 0.3, STEP);
        stepFor(copy, 0.3, STEP);

        assertEquals(bodies.seen(), copy.seen());
        assertEquals(bodies.work(), copy.work());
        assertTrue(seen(bodies, "ball").at().z() < 10.5 + R - 0.1);
    }
}
