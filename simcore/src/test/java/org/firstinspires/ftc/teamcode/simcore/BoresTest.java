package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class BoresTest {
    private static final Field FIELD = CollidersTest.fieldWithABasePlate();
    private static final Field.Flower FLOWER = FIELD.flowers().get(0);
    private static final Vec2 AXIS = FLOWER.axis();
    private static final double R = 1.39;
    private static final Length POLLEN = Valid.length(R);
    private static final double ON_THE_PLATE = CollidersTest.BASE_PLATE_TOP + R;
    private static final double SAME = 1e-12;
    private static final Vec2 LOOSE = new Vec2(30, 0);

    private static Bores holding(Stack<String> stack, List<Rolling<String>> rolling) {
        return Bores.of(FIELD).holding(List.of(stack), rolling);
    }

    private static Stack<String> secondOnTheBottom() {
        return Stack.<String>in(FLOWER).with("second", POLLEN, 3 * R);
    }

    private static Bores aStackOfTwo() {
        return holding(
                secondOnTheBottom(),
                List.of(new Rolling<>("bottom", AXIS, POLLEN), new Rolling<>("loose", LOOSE, POLLEN)));
    }

    private static Bores withTheBottomAt(Vec2 at) {
        return holding(
                secondOnTheBottom(),
                List.of(new Rolling<>("bottom", at, POLLEN), new Rolling<>("loose", LOOSE, POLLEN)));
    }

    @Test
    public void whatStandsInABoreStandsStillOnItsBasePlateWhereTheFloorHasItAndABallBesideItTouchesIt() {
        Bores bores = aStackOfTwo();

        List<Surroundings.Touch> beside =
                bores.near(new Vec3(AXIS.x() - 2 * R - 0.1, AXIS.y(), ON_THE_PLATE), POLLEN, 0.5);
        assertEquals(beside.toString(), 1, beside.size());
        Surroundings.Touch side = beside.get(0);
        assertEquals(Surface.PIECE, side.surface());
        assertEquals(new Vec3(-1, 0, 0), side.normal());
        assertEquals(0.1, side.gap(), SAME);
        assertFalse(side.moves());
        assertTrue(bores.ids().contains(side.id()));

        List<Surroundings.Touch> above =
                bores.near(new Vec3(AXIS.x(), AXIS.y(), ON_THE_PLATE + 4 * R + 0.05), POLLEN, 0.5);
        assertEquals("the second pollen stands on the bottom one: " + above, 1, above.size());
        assertEquals(new Vec3(0, 0, 1), above.get(0).normal());
        assertEquals(0.05, above.get(0).gap(), SAME);
        assertFalse("each ball under a name of its own", above.get(0).id().equals(side.id()));

        assertEquals(
                "a ball the floor has outside every bore is not there",
                List.of(),
                bores.near(new Vec3(LOOSE.x() + 2 * R + 0.1, LOOSE.y(), R), POLLEN, 0.5));
        assertEquals(2, bores.ids().size());
        assertFalse(bores.moving());
        assertEquals(0, bores.sweep(), 0);
        assertTrue(bores.reaches(new Vec3(AXIS.x() - 2 * R - 0.1, AXIS.y(), ON_THE_PLATE), R + 0.11));
        assertFalse(bores.reaches(new Vec3(AXIS.x() - 2 * R - 0.1, AXIS.y(), ON_THE_PLATE), R + 0.09));
    }

    @Test
    public void whatStandsInTheBoresKeepsThePartContract() {
        List<PartContract.Probe> probes = new ArrayList<>();
        for (double dx = -4; dx <= 4; dx += 0.7) {
            for (double dy = -4; dy <= 4; dy += 0.7) {
                for (double dz = -1; dz <= 8; dz += 0.9) {
                    probes.add(
                            PartContract.Probe.of(new Vec3(AXIS.x() + dx, AXIS.y() + dy, ON_THE_PLATE + dz), R, 0.5));
                }
            }
        }

        PartContract.assertKept(
                "standing in a bore", aStackOfTwo(), probes, List.of(0.0, 0.5, 1.0), PartContract.NO_FACE_STILL);
    }

    @Test
    public void aBoreTheFloorHasNothingInHoldsNothing() {
        assertTrue(Bores.of(FIELD).empty());
        assertTrue(holding(Stack.in(FLOWER), List.of(new Rolling<>("loose", LOOSE, POLLEN)))
                .empty());
        assertFalse(aStackOfTwo().empty());
        assertFalse(holding(Stack.in(FLOWER), List.of(new Rolling<>("bottom", AXIS, POLLEN)))
                .empty());
    }

    @Test
    public void whatStandsInTheBoresStandsAsItDidUntilABallInThemMovesFurtherThanItsStillnessAllows() {
        Bores before = aStackOfTwo();
        double still = Bores.STILL_WITHIN_IN;

        assertTrue(withTheBottomAt(new Vec2(AXIS.x() + still / 2, AXIS.y())).standsAsIn(before));
        assertFalse(withTheBottomAt(new Vec2(AXIS.x() + 2 * still, AXIS.y())).standsAsIn(before));
        assertFalse(
                "a ball come into the bore",
                holding(
                                secondOnTheBottom(),
                                List.of(
                                        new Rolling<>("bottom", AXIS, POLLEN),
                                        new Rolling<>("in", new Vec2(AXIS.x() + 1, AXIS.y()), POLLEN)))
                        .standsAsIn(before));
        assertFalse(
                "a ball gone from it",
                holding(Stack.in(FLOWER), List.of(new Rolling<>("bottom", AXIS, POLLEN)))
                        .standsAsIn(before));
        assertTrue(aStackOfTwo().standsAsIn(before));
    }
}
