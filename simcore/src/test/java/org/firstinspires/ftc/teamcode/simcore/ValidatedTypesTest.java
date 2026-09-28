package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class ValidatedTypesTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static final List<Vec2> ANTICLOCKWISE_SQUARE =
            List.of(new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 1), new Vec2(0, 1));

    @Test
    public void aLengthIsPositiveAndFinite() {
        assertEquals(18, Valid.length(18).inches(), 0);
        assertRejected("positive", Length.of(0));
        assertRejected("positive", Length.of(-1));
        assertRejected("finite", Length.of(Double.POSITIVE_INFINITY));
        assertRejected("positive", Length.of(Double.NaN));
    }

    @Test
    public void aHeadingIsAUnitVector() {
        assertEquals(1, Valid.value(Heading.of(0, 1)).sin(), 0);
        assertRejected("unit", Heading.of(2, 0));
        assertRejected("unit", Heading.of(Double.NaN, 0));
        assertRejected("finite", Heading.ofRadians(Double.POSITIVE_INFINITY));
    }

    @Test
    public void aPoseIsAtAFinitePosition() {
        Heading ahead = Valid.heading(0);
        assertEquals(new Vec2(1, 2), Valid.value(Pose.of(1, 2, ahead)).position());
        assertRejected("finite", Pose.of(Double.NaN, 0, ahead));
        assertRejected("finite", Pose.of(0, Double.NEGATIVE_INFINITY, ahead));
    }

    @Test
    public void aConvexPolygonTurnsLeftAtEveryCornerAndGoesRoundOnce() {
        assertEquals(ANTICLOCKWISE_SQUARE, Valid.polygon(ANTICLOCKWISE_SQUARE).corners());
        assertRejected("three corners", ConvexPolygon.of(List.of(new Vec2(0, 0), new Vec2(1, 0))));
        assertRejected("finite", ConvexPolygon.of(List.of(new Vec2(0, 0), new Vec2(1, 0), new Vec2(Double.NaN, 1))));
        assertRejected(
                "turns left",
                ConvexPolygon.of(List.of(new Vec2(0, 1), new Vec2(1, 1), new Vec2(1, 0), new Vec2(0, 0))));
        assertRejected(
                "turns left",
                ConvexPolygon.of(
                        List.of(new Vec2(0, 0), new Vec2(2, 0), new Vec2(1, 0.5), new Vec2(2, 1), new Vec2(0, 1))));
        assertRejected(
                "turns left",
                ConvexPolygon.of(List.of(new Vec2(0, 0), new Vec2(1, 0), new Vec2(2, 0), new Vec2(1, 1))));
        assertRejected("round once", ConvexPolygon.of(pentagram()));
    }

    private static List<Vec2> pentagram() {
        List<Vec2> star = new java.util.ArrayList<>();
        for (int i = 0; i < 5; i++) {
            double angle = Math.PI / 2 + i * 4 * Math.PI / 5;
            star.add(new Vec2(Math.cos(angle), Math.sin(angle)));
        }
        return star;
    }

    @Test
    public void aRingHasAtLeastOneElementAndClosesOnItsFirst() {
        Ring<String> ring = Valid.value(Ring.of(List.of("a", "b", "c")));
        assertEquals(
                List.of(new Ring.Edge<>("a", "b"), new Ring.Edge<>("b", "c"), new Ring.Edge<>("c", "a")), ring.edges());
        assertEquals(List.of(new Ring.Edge<>("x", "x")), Ring.of("x", List.of()).edges());
        assertRejected("at least one", Ring.of(List.<String>of()));
    }

    @Test
    public void aCheckedValueIsFoldedMappedAndChained() {
        Checked<Integer> two = Checked.ok(2);
        assertEquals(Integer.valueOf(3), two.map(x -> x + 1).orElse(0));
        assertEquals(
                Integer.valueOf(0),
                two.then(x -> Checked.<Integer>rejected("no")).orElse(0));
        assertEquals("no", Checked.<Integer>rejected("no").map(x -> x + 1).fold(x -> "ok", rule -> rule));
    }
}
