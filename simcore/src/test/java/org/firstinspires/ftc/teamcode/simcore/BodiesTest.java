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
    private static final Bodies.Start AWAKE = new Bodies.Start.Awake(Bodies.Sleep.WHEN_RESTING);
    private static final Bodies.Start ASLEEP = new Bodies.Start.Asleep();
    private static final Bodies.Step STEPPING = Valid.value(Bodies.Step.of(Valid.value(Seconds.of(STEP))));

    private static Surfaces surfaces() {
        Bounce bounce = Valid.value(Bounce.of(0.55, 0.5, 0.2));
        return Valid.value(Surfaces.of(bounce, bounce, bounce, bounce, bounce, bounce, 0, 4, 0, 0, 0));
    }

    private static Bodies unwatched() {
        return Bodies.unwatched(Surroundings.still(WALLS), Air.vacuum(), surfaces(), STEPPING);
    }

    private static Bodies watching(String name) {
        return Bodies.watching(Surroundings.still(WALLS), Air.vacuum(), surfaces(), STEPPING, name(name));
    }

    private static Bodies.Name name(String name) {
        return Valid.value(Bodies.Name.of(name));
    }

    private static Bodies.Motion motion(Vec3 at, Vec3 velocity) {
        return Valid.value(Bodies.Motion.of(at, velocity, Vec3.zero()));
    }

    private static Bodies.Seen add(Bodies bodies, String name, Vec3 at, Vec3 velocity, Bodies.Start start) {
        return Valid.value(bodies.add(name(name), POLLEN, motion(at, velocity), start));
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
    public void anEnginesStepIsMoreThanNothingAndAtMostTheLongestAShotMayTake() {
        List<String> said = List.of(
                rejection(Bodies.Step.of(Seconds.zero())),
                rejection(Bodies.Step.of(Valid.value(Seconds.of(Math.nextUp(World.LONGEST_STEP_SECONDS))))));

        for (String rule : said) {
            assertTrue(rule, rule.contains("more than 0 and at most " + World.LONGEST_STEP_SECONDS + " s"));
        }
        assertEquals(
                World.LONGEST_STEP_SECONDS,
                Valid.value(Bodies.Step.of(Valid.value(Seconds.of(World.LONGEST_STEP_SECONDS))))
                        .seconds()
                        .value(),
                0);
    }

    @Test
    public void aBodyHasANameThatIsNotBlank() {
        String blank = rejection(Bodies.Name.of(" "));

        assertTrue(blank, blank.contains("a body has a name, not ' '"));
        assertEquals("one", name("one").text());
    }

    @Test
    public void aBodyIsAddedOnlyUnderANameOfItsOwn() {
        Bodies bodies = unwatched();
        add(bodies, "one", new Vec3(0, 0, R), Vec3.zero(), AWAKE);

        String twice = rejection(bodies.add(name("one"), POLLEN, motion(new Vec3(10, 0, R), Vec3.zero()), AWAKE));

        assertTrue(twice, twice.contains("every body has a name of its own, and 'one' is not one"));
        assertEquals(List.of("one"), names(bodies));
    }

    @Test
    public void aBodyIsAtAFinitePlaceMovingAndSpinningAtFiniteRates() {
        Vec3 nowhere = new Vec3(0, Double.NaN, R), endless = new Vec3(Double.POSITIVE_INFINITY, 0, 0);

        List<String> said = List.of(
                rejection(Bodies.Motion.of(nowhere, Vec3.zero(), Vec3.zero())),
                rejection(Bodies.Motion.of(Vec3.zero(), endless, Vec3.zero())),
                rejection(Bodies.Motion.of(Vec3.zero(), Vec3.zero(), endless)));

        for (String rule : said) {
            assertTrue(rule, rule.contains("a body is at a finite place, moving and spinning at finite rates"));
        }
    }

    @Test
    public void aBodyComesInAwakeAsleepOrFixed() {
        Bodies bodies = unwatched();

        add(bodies, "awake", new Vec3(0, 0, R), Vec3.zero(), AWAKE);
        add(bodies, "asleep", new Vec3(10, 0, R), Vec3.zero(), ASLEEP);
        add(bodies, "fixed", new Vec3(20, 0, R), Vec3.zero(), new Bodies.Start.Fixed());

        assertEquals(Bodies.Life.AWAKE, seen(bodies, "awake").life());
        assertEquals(Bodies.Life.ASLEEP, seen(bodies, "asleep").life());
        assertEquals(Bodies.Life.FIXED, seen(bodies, "fixed").life());
    }

    @Test
    public void thePiecesABallIsLaunchedAmongAreEachNamedApartFromEveryOtherAndFromTheLaunchedBall() {
        Bodies.Entry one = new Bodies.Entry(name("one"), POLLEN, motion(new Vec3(0, 0, R), Vec3.zero()), ASLEEP);
        Bodies.Pieces pieces = Valid.value(Bodies.Pieces.none().with(one));

        String twice = rejection(pieces.with(one));
        String launched = rejection(pieces.with(
                new Bodies.Entry(name(Shot.LAUNCHED), POLLEN, motion(new Vec3(10, 0, R), Vec3.zero()), ASLEEP)));

        assertTrue(twice, twice.contains("every body has a name of its own, and 'one' is not one"));
        assertTrue(launched, launched.contains("'" + Shot.LAUNCHED + "' is not one"));
    }

    @Test
    public void aBodyIsReadByItsNameAndANameNoBodyHasReadsNothing() {
        Vec3 at = new Vec3(3, 4, 5), velocity = new Vec3(6, 7, 8);
        Bodies bodies = unwatched();
        add(bodies, "one", at, velocity, ASLEEP);

        Bodies.Seen one = seen(bodies, "one");

        assertEquals(new Bodies.Seen("one", at, velocity, Vec3.zero(), Turn.identity(), Bodies.Life.ASLEEP, 0), one);
        assertTrue(bodies.body("two").isEmpty());
        assertTrue(bodies.touching("two").isEmpty());
    }

    @Test
    public void bodiesAreKeptInTheOrderTheyCameIn() {
        Bodies bodies = unwatched();
        for (String name : List.of("c", "a", "b")) {
            add(bodies, name, new Vec3(names(bodies).size() * 10, 0, R), Vec3.zero(), ASLEEP);
        }

        bodies.remove("a");

        assertEquals(List.of("c", "b"), names(bodies));
    }

    @Test
    public void removingANameNoBodyHasChangesNothing() {
        Bodies bodies = unwatched();
        add(bodies, "one", new Vec3(0, 0, R), Vec3.zero(), ASLEEP);
        List<Bodies.Seen> before = bodies.seen();

        assertTrue(bodies.remove("two").isEmpty());
        assertEquals(before, bodies.seen());
    }

    @Test
    public void addingAndRemovingSayWhatCameAndWhatWent() {
        Bodies bodies = unwatched();
        Vec3 at = new Vec3(3, 4, R), velocity = new Vec3(6, 7, 8);

        Bodies.Seen came = add(bodies, "one", at, velocity, AWAKE);
        Bodies.Seen there = seen(bodies, "one");
        Bodies.Seen went = bodies.remove("one").orElseThrow();

        assertEquals(new Bodies.Seen("one", at, velocity, Vec3.zero(), Turn.identity(), Bodies.Life.AWAKE, 0), came);
        assertEquals(there, went);
        assertTrue(bodies.seen().isEmpty());
    }

    @Test
    public void addingToOrRemovingFromACopyLeavesTheOriginalAsItWas() {
        Bodies bodies = unwatched();
        add(bodies, "one", new Vec3(0, 0, R), Vec3.zero(), ASLEEP);
        add(bodies, "two", new Vec3(2 * R, 0, R), Vec3.zero(), ASLEEP);
        List<Bodies.Seen> before = bodies.seen();
        Bodies copy = bodies.copy();

        add(copy, "three", new Vec3(-2 * R, 0, R), Vec3.zero(), AWAKE);
        copy.remove("one");

        assertEquals(List.of("two", "three"), names(copy));
        assertEquals(Bodies.Life.AWAKE, seen(copy, "two").life());
        assertEquals(before, bodies.seen());
    }

    @Test
    public void removingABodyWakesWhatRestedAgainstItAndForgetsItsName() {
        Bodies bodies = unwatched();
        add(bodies, "held", new Vec3(0, 0, R), Vec3.zero(), ASLEEP);
        add(bodies, "resting", new Vec3(2 * R, 0, R), Vec3.zero(), ASLEEP);
        add(bodies, "leaning", new Vec3(-2 * R, 0, R), Vec3.zero(), AWAKE);
        add(bodies, "far", new Vec3(30, 0, R), Vec3.zero(), ASLEEP);
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
        Bodies bodies = unwatched();
        add(bodies, "restless", new Vec3(0, 0, R), Vec3.zero(), new Bodies.Start.Awake(Bodies.Sleep.NEVER));
        add(bodies, "sleepy", new Vec3(20, 0, R), Vec3.zero(), AWAKE);
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
        Bodies bodies = unwatched();
        add(bodies, "thrown", new Vec3(-20, 0, 6), new Vec3(150, 0, 20), AWAKE);
        add(bodies, "struck", new Vec3(0, 0, R), Vec3.zero(), AWAKE);
        add(bodies, "behind", new Vec3(2 * R, 0, R), Vec3.zero(), AWAKE);
        add(bodies, "beyond", new Vec3(4 * R, 0, R), Vec3.zero(), ASLEEP);
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
        Bodies bodies = unwatched();
        add(bodies, "over", new Vec3(80, 0, 20), Vec3.zero(), AWAKE);
        add(bodies, "spared", new Vec3(0, 80, 20), Vec3.zero(), AWAKE);
        add(bodies, "sleeping", new Vec3(-80, 0, 20), Vec3.zero(), ASLEEP);
        add(bodies, "inside", new Vec3(0, 0, 20), Vec3.zero(), AWAKE);

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
        Bodies bodies = watching("ball");
        add(
                bodies,
                "ball",
                new Vec3(0, 0, R + 0.002),
                new Vec3(10, 0, -100),
                new Bodies.Start.Awake(Bodies.Sleep.NEVER));
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
        Bodies bodies = unwatched();
        add(bodies, "ball", new Vec3(0, 0, 5), Vec3.zero(), AWAKE);

        assertTrue(bodies.step().isEmpty());
        assertEquals(1, bodies.steps());
        assertEquals(STEP, bodies.now(), 0);
        assertTrue(seen(bodies, "ball").at().z() < 5);
    }
}
