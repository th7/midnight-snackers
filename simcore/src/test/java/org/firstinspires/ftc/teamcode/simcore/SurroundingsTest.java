package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class SurroundingsTest {
    private static final Length R = Valid.length(1.4);
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
    private static final Collider PLATE = Carried.box("plate", new Vec3(-10, -10, 10), new Vec3(10, 10, 11));

    private record Stated(String id, double sweep, boolean moving, List<Surroundings.Touch> answer)
            implements Surroundings.Part {
        Stated(String id, double sweep, boolean moving) {
            this(id, sweep, moving, List.of());
        }

        @Override
        public List<String> ids() {
            return List.of(id);
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            return false;
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            return answer;
        }

        @Override
        public boolean squeezes() {
            return false;
        }

        @Override
        public List<Surroundings.Squeeze> squeezing(Vec3 centre, Length radius) {
            return List.of();
        }

        @Override
        public Surroundings.Part at(GridTime time) {
            return this;
        }
    }

    private record Squeezer(String id, boolean squeezes, boolean reaching, List<Surroundings.Squeeze> answer)
            implements Surroundings.Part {
        @Override
        public List<String> ids() {
            return List.of(id);
        }

        @Override
        public boolean reaches(Vec3 centre, double by) {
            return reaching;
        }

        @Override
        public double sweep() {
            return 0;
        }

        @Override
        public boolean moving() {
            return false;
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            return List.of();
        }

        @Override
        public List<Surroundings.Squeeze> squeezing(Vec3 centre, Length radius) {
            return answer;
        }

        @Override
        public Surroundings.Part at(GridTime time) {
            return this;
        }
    }

    private record Answering(List<String> ids, List<Surroundings.Touch> answer, List<String> later)
            implements Surroundings.Part {
        @Override
        public boolean reaches(Vec3 centre, double by) {
            return true;
        }

        @Override
        public double sweep() {
            return 0;
        }

        @Override
        public boolean moving() {
            return false;
        }

        @Override
        public List<Surroundings.Touch> near(Vec3 centre, Length radius, double within) {
            return answer;
        }

        @Override
        public boolean squeezes() {
            return false;
        }

        @Override
        public List<Surroundings.Squeeze> squeezing(Vec3 centre, Length radius) {
            return List.of();
        }

        @Override
        public Surroundings.Part at(GridTime time) {
            return new Answering(later, answer, later);
        }
    }

    private static Surroundings.Touch touch(String id, double gap) {
        return Valid.value(Surroundings.Touch.of(id, Surface.FRAME, new Vec3(0, 0, 1), gap, Vec3.zero(), Vec3.zero()));
    }

    private static final Compliance SOFT = Valid.value(Compliance.of(40, 0.6, 0.6, 0.02));

    private static Surroundings.Squeeze squeeze(String id, double depth) {
        return Valid.value(Surroundings.Squeeze.of(
                id, SOFT, Vec3.zero(), R, new Vec3(0, 0, 1), depth, Surroundings.Support.still()));
    }

    private static Surroundings with(Surroundings.Part... parts) {
        return Valid.value(Surroundings.of(WALLS, List.of(parts)));
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted", rule -> rule);
    }

    private static List<Surroundings.Part> parts(List<Surroundings.Held> held) {
        List<Surroundings.Part> parts = new ArrayList<>();
        for (Surroundings.Held part : held) {
            parts.add(part.part());
        }
        return parts;
    }

    private static List<String> ids(List<Surroundings.Touch> touches) {
        List<String> ids = new ArrayList<>();
        for (Surroundings.Touch touch : touches) {
            ids.add(touch.id());
        }
        return ids;
    }

    @Test
    public void stillSurroundingsTouchWhatTheirCollidersDoAndNeitherMoveNorSpin() {
        Surroundings still = Surroundings.still(WALLS);
        List<Vec3> places = List.of(
                new Vec3(0, 0, 1.4),
                new Vec3(70, 0, 1),
                new Vec3(-70.2, 70.2, 5),
                new Vec3(10, 3, 12),
                new Vec3(0, 0, -1));

        for (Vec3 at : places) {
            for (double within : new double[] {0, 1e-6, 2}) {
                List<Surroundings.Touch> expected = new ArrayList<>();
                for (Colliders.Near near : WALLS.near(at, R, within)) {
                    expected.add(Valid.value(Surroundings.Touch.of(
                            near.collider().name(),
                            near.collider().surface(),
                            near.normal(),
                            near.gap(),
                            Vec3.zero(),
                            Vec3.zero())));
                }

                assertEquals(at + " within " + within, expected, still.near(at, R, within));
                assertEquals(at + " later", expected, still.at(Valid.time(5)).near(at, R, within));
            }
        }
        assertEquals(Double.doubleToRawLongBits(0.0), Double.doubleToRawLongBits(still.sweep()));
        assertTrue(still.moving().isEmpty());
    }

    @Test
    public void aPartIsAskedOnlyWhereItReachesAndItsTouchesComeAfterTheColliders() {
        Surroundings.Touch made = Valid.value(
                Surroundings.Touch.of("nowhere", Surface.FRAME, new Vec3(0, 0, 1), 0, Vec3.zero(), Vec3.zero()));
        Surroundings around =
                with(Carried.sliding(PLATE, new Vec3(3, 0, 0), 0), new Stated("liar", 0, true, List.of(made)));

        assertEquals(List.of("Floor", "plate"), ids(around.near(new Vec3(9, 0, 1.4), R, 9)));
        assertEquals(List.of("Floor"), ids(around.near(new Vec3(30, 0, 1.4), R, 9)));
        assertEquals(List.of("plate"), ids(around.near(new Vec3(0, 0, 12.4), R, 1e-6)));
    }

    @Test
    public void aTouchHasAUnitNormalAndAFiniteGapVelocityAndSpin() {
        Vec3 up = new Vec3(0, 0, 1),
                unknown = new Vec3(Double.NaN, 0, 0),
                endless = new Vec3(0, Double.POSITIVE_INFINITY, 0);

        List<String> said = List.of(
                rejection(
                        Surroundings.Touch.of("plate", Surface.FRAME, new Vec3(0, 0, 2), 0, Vec3.zero(), Vec3.zero())),
                rejection(Surroundings.Touch.of("plate", Surface.FRAME, Vec3.zero(), 0, Vec3.zero(), Vec3.zero())),
                rejection(Surroundings.Touch.of("plate", Surface.FRAME, unknown, 0, Vec3.zero(), Vec3.zero())),
                rejection(Surroundings.Touch.of("plate", Surface.FRAME, up, Double.NaN, Vec3.zero(), Vec3.zero())),
                rejection(Surroundings.Touch.of(
                        "plate", Surface.FRAME, up, Double.NEGATIVE_INFINITY, Vec3.zero(), Vec3.zero())),
                rejection(Surroundings.Touch.of("plate", Surface.FRAME, up, 0, unknown, Vec3.zero())),
                rejection(Surroundings.Touch.of("plate", Surface.FRAME, up, 0, Vec3.zero(), endless)));

        for (String rule : said) {
            assertTrue(rule, rule.contains("a touch has a unit normal and a finite gap, velocity and spin"));
        }
        Vec3 slanted = new Vec3(0.6, 0, 0.8), velocity = new Vec3(1, 2, 3), spin = new Vec3(0, 0, 4);
        Surroundings.Touch touch =
                Valid.value(Surroundings.Touch.of("plate", Surface.CELL, slanted, -0.5, velocity, spin));
        assertEquals("plate", touch.id());
        assertEquals(Surface.CELL, touch.surface());
        assertEquals(slanted, touch.normal());
        assertEquals(-0.5, touch.gap(), 0);
        assertEquals(velocity, touch.velocity());
        assertEquals(spin, touch.spin());
    }

    @Test
    public void aPartsTouchIsKeptOnlyWithinTheReachAskedForAndUnderANameThePartGaveWhenItCameIn() {
        Surroundings.Part answering = new Answering(
                List.of("plate", "rim"),
                List.of(
                        touch("plate", 0.5),
                        touch("rim", 0.5000001),
                        touch("Floor", 0),
                        touch("nowhere", 0),
                        touch("rim", -3),
                        touch("renamed", 0)),
                List.of("plate", "rim", "renamed"));
        Surroundings around = with(answering);
        Vec3 above = new Vec3(0, 0, 30);

        assertEquals(List.of(touch("plate", 0.5), touch("rim", -3)), around.near(above, R, 0.5));
        assertEquals(List.of(touch("rim", -3)), around.near(above, R, 0.4));
        assertEquals(
                List.of(touch("plate", 0.5), touch("rim", -3)),
                around.at(Valid.time(1)).near(above, R, 0.5));
        assertEquals(
                List.of(touch("plate", 0.5), touch("rim", -3)),
                around.at(Valid.time(1)).at(Valid.time(2)).near(above, R, 0.5));
    }

    @Test
    public void everyIdAmongTheCollidersAndThePartsIsItsOwn() {
        String floor = rejection(Surroundings.of(WALLS, List.of(new Stated("Floor", 0, false))));
        String twice =
                rejection(Surroundings.of(WALLS, List.of(new Stated("arm", 0, false), new Stated("arm", 1, true))));

        assertTrue(floor, floor.contains("every collider has a name of its own, and 'Floor' is not one"));
        assertTrue(twice, twice.contains("'arm'"));
        assertEquals("accepted", rejection(Surroundings.of(WALLS, List.of(new Stated("arm", 0, false)))));
    }

    @Test
    public void aPartIsReplacedByOneUnderTheSameNamesInAnyOrder() {
        Surroundings.Part was =
                new Answering(List.of("arm", "hand"), List.of(touch("arm", 0.5)), List.of("arm", "hand"));
        Surroundings around = with(new Stated("post", 0, false), was);

        for (List<String> names : List.of(List.of("arm", "hand"), List.of("hand", "arm"))) {
            Surroundings replaced =
                    Valid.value(around.replacing(new Answering(names, List.of(touch("hand", -0.5)), names)));

            assertEquals(names.toString(), List.of("Floor", "hand"), ids(replaced.near(Vec3.zero(), R, 1)));
        }
    }

    @Test
    public void aPartWhoseNamesAreNotExactlyThoseOfAPartTheSurroundingsHoldIsRefusedNamingTheRule() {
        Surroundings.Part was =
                new Answering(List.of("arm", "hand"), List.of(touch("arm", 0.5)), List.of("arm", "hand"));
        Surroundings around = with(new Stated("post", 0, false), was);

        for (List<String> names :
                List.of(List.of("arm"), List.of("arm", "hand", "finger"), List.of("hand", "post"), List.of("leg"))) {
            String said = rejection(around.replacing(new Answering(names, List.of(touch("arm", -1)), names)));

            assertTrue(
                    said,
                    said.contains("a part takes the place of one the surroundings hold under exactly its names, and "
                            + names + " are not the names of any"));
        }
        String twice = rejection(around.replacing(new Answering(
                List.of("arm", "hand", "arm"), List.of(touch("arm", -1)), List.of("arm", "hand", "arm"))));
        assertTrue(twice, twice.contains("every collider has a name of its own, and 'arm' is not one"));
    }

    @Test
    public void theSweepIsTheFastestMovingPartsAndAPartThatIsStillAddsNone() {
        Stated slow = new Stated("slow", 3, true),
                still = new Stated("still", 7, false),
                fast = new Stated("fast", 5, true);

        Surroundings around = with(slow, still, fast);

        assertEquals(5, around.sweep(), 0);
        assertEquals(List.of(slow, fast), parts(around.moving()));
        assertEquals(
                Double.doubleToRawLongBits(0.0),
                Double.doubleToRawLongBits(with(still).sweep()));
        assertEquals(0, with(new Stated("spinning", 0, true)).sweep(), 0);
    }

    @Test
    public void aPartThatCannotSayHowFastItSweepsIsTakenToSweepAsFastAsCanBe() {
        for (double said : new double[] {Double.NaN, -1}) {
            Stated unsure = new Stated("unsure", said, true);

            assertEquals(Double.POSITIVE_INFINITY, with(unsure).sweep(), 0);
            for (Surroundings.Held held : with(unsure).moving()) {
                assertEquals(Double.POSITIVE_INFINITY, held.sweep(), 0);
            }
        }
    }

    @Test
    public void theSurroundingsAtATimeHaveEveryPartWhereItIsThenWhateverTimeTheyWereAt() {
        Surroundings around = with(Carried.sliding(PLATE, new Vec3(0, 0, 2), 1));

        double top = 11 + 1.4;
        assertEquals(0, around.near(new Vec3(0, 0, top), R, 1e-6).get(0).gap(), 1e-12);
        assertEquals(
                0,
                around.at(Valid.time(0.5))
                        .near(new Vec3(0, 0, top), R, 1e-6)
                        .get(0)
                        .gap(),
                1e-12);
        assertEquals(
                0,
                around.at(Valid.time(3))
                        .near(new Vec3(0, 0, top + 4), R, 1e-6)
                        .get(0)
                        .gap(),
                1e-12);
        assertEquals(
                around.at(Valid.time(3)).near(new Vec3(0, 0, top + 4), R, 1e-6),
                around.at(Valid.time(2)).at(Valid.time(3)).near(new Vec3(0, 0, top + 4), R, 1e-6));
        assertEquals(List.of(), around.at(Valid.time(0.5)).moving());
        assertEquals(1, around.at(Valid.time(3)).moving().size());
        assertEquals(
                new Vec3(0, 0, 2),
                around.at(Valid.time(3))
                        .near(new Vec3(0, 0, top + 4), R, 1e-6)
                        .get(0)
                        .velocity());
    }

    @Test
    public void theSurroundingsAskOnlyThePartsThatSqueezeAndOnlyWhereTheyReachAndKeepOnlyTheirOwnNames() {
        Squeezer wheel = new Squeezer(
                "wheel",
                true,
                true,
                List.of(squeeze("wheel", 0.1), squeeze("Floor", 0.2), squeeze("nowhere", 0.3), squeeze("wheel", 2)));
        Squeezer liar = new Squeezer("liar", false, true, List.of(squeeze("liar", 0.1)));
        Squeezer far = new Squeezer("far", true, false, List.of(squeeze("far", 0.1)));
        Vec3 here = new Vec3(0, 0, 30);

        assertEquals(
                List.of(squeeze("wheel", 0.1), squeeze("wheel", 2)),
                with(wheel, liar, far).squeezing(here, R));
        assertEquals(List.of(), with(liar, far).squeezing(here, R));
        assertEquals(List.of(), with(liar).near(here, R, 5));
        assertTrue(with(wheel, liar).squeezes());
        assertTrue(with(far).squeezes());
        assertEquals(false, with(liar).squeezes());
        assertEquals(false, Surroundings.still(WALLS).squeezes());
        assertTrue(with(liar, wheel).squeezeReaches(here, 1));
        assertEquals(false, with(liar, far).squeezeReaches(here, 1));
        assertEquals(List.of(), Surroundings.still(WALLS).squeezing(new Vec3(0, 0, 0.5), R));
    }

    @Test
    public void whatSqueezesABallAtATimeIsAnsweredByThePartsThatSqueezePlacedThenWhateverTimeTheyWereAt() {
        CompliantArc wheel = CompliantArc.spinningUp("wheel", new Vec3(0, 0, 20), new Vec3(0, 1, 0), 4, SOFT, 0, 1, 50);
        Surroundings around = with(Carried.sliding(PLATE, new Vec3(0, 0, 2), 1), wheel);
        Vec3 beside = new Vec3(-5.2, 0, 20);

        for (double seconds : new double[] {0, 0.25, 0.5, 2}) {
            GridTime then = Valid.time(seconds);
            assertEquals("at " + seconds, wheel.at(then).squeezing(beside, R), around.squeezingAt(then, beside, R));
            assertEquals(
                    "at " + seconds + " from later",
                    wheel.at(then).squeezing(beside, R),
                    around.at(Valid.time(3)).squeezingAt(then, beside, R));
            assertEquals("at " + seconds, around.at(then).squeezing(beside, R), around.squeezingAt(then, beside, R));
        }
        Vec3 rim = around.squeezingAt(Valid.time(0.25), beside, R).get(0).velocity();
        double depth = R.inches() - (5.2 - 4);
        assertEquals(0, rim.x(), 0);
        assertEquals(0, rim.y(), 0);
        assertEquals((5.2 - R.inches() + depth / 2) * 50 * 0.25, rim.z(), 1e-12);
    }

    private static Checked<Surroundings.Squeeze> squeezeOf(Vec3 normal, double depth, Surroundings.Support support) {
        return Surroundings.Squeeze.of("wheel", SOFT, new Vec3(0, 0, 30), R, normal, depth, support);
    }

    private static void assertSameVec(String what, Vec3 expected, Vec3 actual) {
        assertEquals(what + ": x of " + actual, expected.x(), actual.x(), 1e-12);
        assertEquals(what + ": y of " + actual, expected.y(), actual.y(), 1e-12);
        assertEquals(what + ": z of " + actual, expected.z(), actual.z(), 1e-12);
    }

    @Test
    public void aSqueezeHasAUnitNormalAFiniteDepthMoreThanNothingAndAFiniteVelocityWhereTheBallMeetsWhatSqueezesIt() {
        Vec3 up = new Vec3(0, 0, 1), unknown = new Vec3(Double.NaN, 0, 0);
        Surroundings.Support still = Surroundings.Support.still();
        Surroundings.Support huge = Valid.value(Surroundings.Support.of(
                Vec3.zero(), new Vec3(Double.MAX_VALUE, 0, 0), new Vec3(0, Double.MAX_VALUE, 0)));
        String rule =
                "a squeeze has a unit normal, a finite depth more than 0 and a finite velocity where the ball meets"
                        + " what squeezes it, not ";

        for (Checked<Surroundings.Squeeze> refused : List.of(
                squeezeOf(new Vec3(0, 0, 2), 0.1, still),
                squeezeOf(Vec3.zero(), 0.1, still),
                squeezeOf(unknown, 0.1, still),
                squeezeOf(up, 0, still),
                squeezeOf(up, -0.1, still),
                squeezeOf(up, Double.NaN, still),
                squeezeOf(up, Double.POSITIVE_INFINITY, still),
                squeezeOf(up, 0.1, huge),
                Surroundings.Squeeze.of("wheel", SOFT, unknown, R, up, 0.1, still))) {
            String said = rejection(refused);
            assertTrue(said, said.startsWith(rule));
        }
        Vec3 slanted = new Vec3(0.6, 0, 0.8), velocity = new Vec3(1, 2, 3);
        Surroundings.Squeeze squeeze = Valid.value(
                squeezeOf(slanted, 0.05, Valid.value(Surroundings.Support.of(Vec3.zero(), velocity, Vec3.zero()))));
        assertEquals("wheel", squeeze.id());
        assertEquals(SOFT, squeeze.compliance());
        assertEquals(slanted, squeeze.normal());
        assertEquals(0.05, squeeze.depth(), 0);
        assertEquals(velocity, squeeze.velocity());
        assertTrue(squeeze.moves());
        assertEquals(false, squeeze("wheel", 0.1).moves());
    }

    @Test
    public void aSupportMovesAsAFinitePointOfItGoesAndAsItSpins() {
        Vec3 unknown = new Vec3(Double.NaN, 0, 0), endless = new Vec3(0, Double.POSITIVE_INFINITY, 0);
        String rule = "a support moves as a point of it goes and as it spins, each finite, not ";

        for (Checked<Surroundings.Support> refused : List.of(
                Surroundings.Support.of(unknown, Vec3.zero(), Vec3.zero()),
                Surroundings.Support.of(Vec3.zero(), endless, Vec3.zero()),
                Surroundings.Support.of(Vec3.zero(), Vec3.zero(), unknown))) {
            String said = rejection(refused);
            assertTrue(said, said.startsWith(rule));
        }
        Vec3 pivot = new Vec3(1, 2, 3), going = new Vec3(4, 0, -1), spin = new Vec3(0, 2, 0);
        Surroundings.Support support = Valid.value(Surroundings.Support.of(pivot, going, spin));
        Vec3 elsewhere = new Vec3(-2, 7, 5);
        assertSameVec("going", going.plus(spin.cross(elsewhere.minus(pivot))), support.velocityAt(elsewhere));
        assertSameVec("at its point", going, support.velocityAt(pivot));
        assertSameVec("still", Vec3.zero(), Surroundings.Support.still().velocityAt(elsewhere));
    }

    @Test
    public void
            aSqueezesVelocityIsItsSupportsWhereTheBallMeetsItMidwayThroughTheirOverlapAndItMovesWhileItsSupportGoesOrTurns() {
        Vec3 up = new Vec3(0, 0, 1), centre = new Vec3(0, 0, 30), turning = new Vec3(0, 0, 4);
        double depth = 0.2;
        Vec3 arm = up.times(-(R.inches() - depth / 2));
        Vec3 meets = centre.plus(arm), hub = meets.plus(new Vec3(-3, 0, 0));
        Surroundings.Squeeze wheel = Valid.value(
                squeezeOf(up, depth, Valid.value(Surroundings.Support.of(hub, new Vec3(0, 5, 0), turning))));
        Surroundings.Squeeze aboutWhereItMeets =
                Valid.value(squeezeOf(up, depth, Valid.value(Surroundings.Support.of(meets, Vec3.zero(), turning))));

        assertSameVec("the arm", arm, wheel.arm());
        assertSameVec("the wheel", new Vec3(0, 5 + 4 * 3, 0), wheel.velocity());
        assertSameVec("the wheel's spin", turning, wheel.spin());
        assertTrue(wheel.moves());
        assertSameVec("turning about where it meets", Vec3.zero(), aboutWhereItMeets.velocity());
        assertTrue(aboutWhereItMeets.moves());
        assertEquals(
                false,
                Valid.value(squeezeOf(up, depth, Surroundings.Support.still())).moves());
    }
}
