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
        public Surroundings.Part at(GridTime time) {
            return new Answering(later, answer, later);
        }
    }

    private static Surroundings.Touch touch(String id, double gap) {
        return Valid.value(Surroundings.Touch.of(id, Surface.FRAME, new Vec3(0, 0, 1), gap, Vec3.zero(), Vec3.zero()));
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
    public void aPartIsReplacedByOneUnderTheSameNamesAndAPartUnderNamesTheSurroundingsDoNotHoldChangesNothing() {
        Surroundings.Part was =
                new Answering(List.of("arm", "hand"), List.of(touch("arm", 0.5)), List.of("arm", "hand"));
        Surroundings.Part now =
                new Answering(List.of("arm", "hand"), List.of(touch("hand", -0.5)), List.of("arm", "hand"));
        Surroundings around = with(new Stated("post", 0, false), was);

        Surroundings replaced = around.replacing(now);
        Surroundings unchanged =
                around.replacing(new Answering(List.of("arm"), List.of(touch("arm", -1)), List.of("arm")));

        assertEquals(List.of("Floor", "hand"), ids(replaced.near(Vec3.zero(), R, 1)));
        assertEquals(List.of("Floor", "arm"), ids(unchanged.near(Vec3.zero(), R, 1)));
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
}
