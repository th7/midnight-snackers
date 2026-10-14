package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class BodiesTest {
    private static final Ball POLLEN = Valid.value(Ball.of(2.8, 24.9, 0.634));
    private static final double R = POLLEN.radius();
    private static final double STEP = 1e-4;
    private static final World WORLD = Valid.value(World.of(
            Valid.value(Colliders.of(
                    Valid.value(Field.of(
                            Valid.length(141.17),
                            Valid.length(11.64),
                            List.of(),
                            List.of(),
                            List.of(),
                            List.of(),
                            List.of(),
                            List.of())),
                    List.of())),
            Air.vacuum(),
            surfaces(),
            Valid.value(Seconds.of(STEP)),
            Valid.value(Seconds.of(10)),
            List.of()));

    private static Surfaces surfaces() {
        Bounce bounce = Valid.value(Bounce.of(0.55, 0.5, 0.2));
        return Valid.value(Surfaces.of(bounce, bounce, bounce, bounce, bounce, bounce, 0, 4, 0, 0, 0));
    }

    private static Bodies with(Bodies bodies, String name, Vec3 at, Vec3 velocity, Bodies.Life life) {
        return Valid.value(bodies.add(name, POLLEN, at, velocity, Vec3.zero(), life, Bodies.Sleep.WHEN_RESTING));
    }

    private static Bodies.Seen seen(Bodies bodies, String name) {
        return bodies.body(name).orElseThrow(() -> new AssertionError("no body " + name));
    }

    private static List<String> names(Bodies bodies) {
        List<String> names = new ArrayList<>();
        for (Bodies.Seen seen : bodies.seen()) {
            names.add(seen.name());
        }
        return names;
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted", rule -> rule);
    }

    private static void stepFor(Bodies bodies, int steps) {
        for (int i = 0; i < steps; i++) {
            bodies.step();
        }
    }

    @Test
    public void aBodyIsAddedOnlyUnderANameOfItsOwn() {
        Bodies bodies = with(Bodies.unwatched(WORLD), "one", new Vec3(0, 0, R), Vec3.zero(), Bodies.Life.AWAKE);

        String blank = rejection(bodies.add(
                " ", POLLEN, new Vec3(10, 0, R), Vec3.zero(), Vec3.zero(), Bodies.Life.AWAKE, Bodies.Sleep.NEVER));
        String twice = rejection(bodies.add(
                "one", POLLEN, new Vec3(10, 0, R), Vec3.zero(), Vec3.zero(), Bodies.Life.AWAKE, Bodies.Sleep.NEVER));

        assertTrue(blank, blank.contains("name of its own"));
        assertTrue(twice, twice.contains("'one'"));
        assertEquals(List.of("one"), names(bodies));
    }

    @Test
    public void aBodyIsAddedOnlyAtAFinitePlaceMovingAndSpinningAtFiniteRates() {
        Bodies bodies = Bodies.unwatched(WORLD);
        Vec3 nowhere = new Vec3(0, Double.NaN, R), endless = new Vec3(Double.POSITIVE_INFINITY, 0, 0);

        List<String> said = List.of(
                rejection(bodies.add(
                        "at", POLLEN, nowhere, Vec3.zero(), Vec3.zero(), Bodies.Life.AWAKE, Bodies.Sleep.NEVER)),
                rejection(bodies.add(
                        "moving", POLLEN, Vec3.zero(), endless, Vec3.zero(), Bodies.Life.AWAKE, Bodies.Sleep.NEVER)),
                rejection(bodies.add(
                        "spinning", POLLEN, Vec3.zero(), Vec3.zero(), endless, Bodies.Life.AWAKE, Bodies.Sleep.NEVER)));

        for (String rule : said) {
            assertTrue(rule, rule.contains("finite"));
        }
        assertTrue(bodies.seen().isEmpty());
    }

    @Test
    public void aBodyComesInAwakeAsleepOrFixedButNotGone() {
        Bodies bodies = Bodies.unwatched(WORLD);

        String said = rejection(bodies.add(
                "ghost", POLLEN, new Vec3(0, 0, R), Vec3.zero(), Vec3.zero(), Bodies.Life.GONE, Bodies.Sleep.NEVER));

        assertTrue(said, said.contains("not gone"));
        assertTrue(bodies.seen().isEmpty());
    }

    @Test
    public void aBodyIsReadByItsNameAndANameNoBodyHasReadsNothing() {
        Vec3 at = new Vec3(3, 4, 5), velocity = new Vec3(6, 7, 8);
        Bodies bodies = with(Bodies.unwatched(WORLD), "one", at, velocity, Bodies.Life.ASLEEP);

        Bodies.Seen one = seen(bodies, "one");

        assertEquals(new Bodies.Seen("one", at, velocity, Vec3.zero(), Turn.identity(), Bodies.Life.ASLEEP, 0), one);
        assertTrue(bodies.body("two").isEmpty());
        assertTrue(bodies.touching("two").isEmpty());
    }

    @Test
    public void bodiesAreKeptInTheOrderTheyCameIn() {
        Bodies bodies = Bodies.unwatched(WORLD);
        for (String name : List.of("c", "a", "b")) {
            bodies = with(bodies, name, new Vec3(names(bodies).size() * 10, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);
        }

        bodies.remove("a");

        assertEquals(List.of("c", "b"), names(bodies));
    }

    @Test
    public void removingANameNoBodyHasChangesNothing() {
        Bodies bodies = with(Bodies.unwatched(WORLD), "one", new Vec3(0, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);
        List<Bodies.Seen> before = bodies.seen();

        bodies.remove("two");

        assertEquals(before, bodies.seen());
    }

    @Test
    public void removingABodyWakesWhatRestedAgainstItAndForgetsItsName() {
        Bodies bodies = Bodies.unwatched(WORLD);
        bodies = with(bodies, "held", new Vec3(0, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);
        bodies = with(bodies, "resting", new Vec3(2 * R, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);
        bodies = with(bodies, "leaning", new Vec3(-2 * R, 0, R), Vec3.zero(), Bodies.Life.AWAKE);
        bodies = with(bodies, "far", new Vec3(30, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);
        bodies.touchingAtTheStart();
        assertTrue(bodies.touching("leaning").orElseThrow().bodies().contains("held"));

        bodies.remove("held");

        assertEquals(Bodies.Life.AWAKE, seen(bodies, "resting").life());
        assertEquals(0, seen(bodies, "resting").restFor(), 0);
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "far").life());
        assertFalse(bodies.touching("leaning").orElseThrow().bodies().contains("held"));
        assertTrue(bodies.touching("leaning").orElseThrow().colliders().contains("Floor"));
    }

    @Test
    public void aBodyThatMayNeverSleepStaysAwakeAtRestWhileOneThatMayFallsAsleep() {
        Bodies bodies = Valid.value(Bodies.unwatched(WORLD)
                .add(
                        "restless",
                        POLLEN,
                        new Vec3(0, 0, R),
                        Vec3.zero(),
                        Vec3.zero(),
                        Bodies.Life.AWAKE,
                        Bodies.Sleep.NEVER)
                .then(more -> more.add(
                        "sleepy",
                        POLLEN,
                        new Vec3(20, 0, R),
                        Vec3.zero(),
                        Vec3.zero(),
                        Bodies.Life.AWAKE,
                        Bodies.Sleep.WHEN_RESTING)));
        bodies.touchingAtTheStart();
        assertTrue(bodies.anyAwakeBut("restless"));

        stepFor(bodies, (int) (2 * Shot.REST_SECONDS / STEP));

        assertEquals(Bodies.Life.AWAKE, seen(bodies, "restless").life());
        assertTrue(seen(bodies, "restless").restFor() >= Shot.REST_SECONDS);
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "sleepy").life());
        assertEquals(Vec3.zero(), seen(bodies, "sleepy").velocity());
        assertFalse(bodies.anyAwakeBut("restless"));
        assertTrue(bodies.anyAwakeBut("sleepy"));
    }

    @Test
    public void aCopyGoesOnExactlyAsTheOriginalAndLeavesItWhereItWas() {
        Bodies bodies = Bodies.unwatched(WORLD);
        bodies = with(bodies, "thrown", new Vec3(-20, 0, 6), new Vec3(150, 0, 20), Bodies.Life.AWAKE);
        bodies = with(bodies, "struck", new Vec3(0, 0, R), Vec3.zero(), Bodies.Life.AWAKE);
        bodies = with(bodies, "behind", new Vec3(2 * R, 0, R), Vec3.zero(), Bodies.Life.AWAKE);
        bodies = with(bodies, "beyond", new Vec3(4 * R, 0, R), Vec3.zero(), Bodies.Life.ASLEEP);
        bodies.touchingAtTheStart();
        stepFor(bodies, 1000);
        Bodies copy = bodies.copy();
        List<Bodies.Seen> then = copy.seen();
        long steps = copy.steps(), work = copy.work();
        for (String name : List.of("thrown", "struck", "behind", "beyond")) {
            assertEquals(name, bodies.touching(name), copy.touching(name));
        }
        assertTrue(copy.touching("struck").orElseThrow().bodies().contains("behind"));

        stepFor(bodies, 3000);
        Bodies later = copy.copy();
        stepFor(copy, 3000);

        assertEquals(bodies.seen(), copy.seen());
        assertEquals(bodies.steps(), copy.steps());
        assertEquals(Double.doubleToRawLongBits(bodies.now()), Double.doubleToRawLongBits(copy.now()));
        assertEquals(bodies.work(), copy.work());
        assertEquals(bodies.touching("struck"), copy.touching("struck"));
        assertEquals(then, later.seen());
        assertEquals(steps, later.steps());
        assertEquals(work, later.work());
        assertTrue(bodies.work() > work);
    }

    @Test
    public void onlyAwakeBodiesBeyondAWallAreGoneAndTheOneSparedIsNot() {
        Bodies bodies = Bodies.unwatched(WORLD);
        bodies = with(bodies, "over", new Vec3(80, 0, 20), Vec3.zero(), Bodies.Life.AWAKE);
        bodies = with(bodies, "spared", new Vec3(0, 80, 20), Vec3.zero(), Bodies.Life.AWAKE);
        bodies = with(bodies, "sleeping", new Vec3(-80, 0, 20), Vec3.zero(), Bodies.Life.ASLEEP);
        bodies = with(bodies, "inside", new Vec3(0, 0, 20), Vec3.zero(), Bodies.Life.AWAKE);

        List<Bodies.Gone> gone = bodies.goneOver("spared");

        assertEquals(List.of(new Bodies.Gone("over", Colliders.Side.FAR, new Vec3(80, 0, 20), Turn.identity())), gone);
        assertEquals(Bodies.Life.GONE, seen(bodies, "over").life());
        assertEquals(Bodies.Life.AWAKE, seen(bodies, "spared").life());
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "sleeping").life());
        assertEquals(Bodies.Life.AWAKE, seen(bodies, "inside").life());
        assertTrue(bodies.goneOver("spared").isEmpty());
    }

    @Test
    public void aStepSaysHowTheWatchedBodyCameThroughEachStretchOfItInTurn() {
        Bodies bodies = Valid.value(Bodies.watching(WORLD, "ball")
                .add(
                        "ball",
                        POLLEN,
                        new Vec3(0, 0, R + 0.002),
                        new Vec3(10, 0, -100),
                        Vec3.zero(),
                        Bodies.Life.AWAKE,
                        Bodies.Sleep.NEVER));
        bodies.touchingAtTheStart();
        Bodies.Seen start = seen(bodies, "ball");

        List<Bodies.Span> spans = bodies.step();

        assertTrue(spans.toString(), spans.size() >= 2);
        Bodies.Span first = spans.get(0);
        assertEquals(0, first.began(), 0);
        assertTrue(first.took() < STEP);
        assertEquals(start.at(), first.from());
        assertTrue(first.before().velocity().z() < 0);
        Contacts.Resolved resolved = first.resolved().orElseThrow();
        assertEquals("Floor", resolved.hits().get(0).against());
        assertTrue(first.atResolve().velocity().z() > 0);
        double began = 0;
        for (Bodies.Span span : spans) {
            assertEquals(began, span.began(), 0);
            assertEquals(span.began() + span.took(), span.now(), Math.ulp(span.now()));
            began = span.now();
        }
        assertEquals(bodies.now(), began, 0);
        assertEquals(seen(bodies, "ball"), spans.get(spans.size() - 1).after());
    }

    @Test
    public void aStepWatchingNothingSaysNothing() {
        Bodies bodies = with(Bodies.unwatched(WORLD), "ball", new Vec3(0, 0, 5), Vec3.zero(), Bodies.Life.AWAKE);

        assertTrue(bodies.step().isEmpty());
        assertEquals(1, bodies.steps());
        assertEquals(STEP, bodies.now(), 0);
        assertTrue(seen(bodies, "ball").at().z() < 5);
    }
}
