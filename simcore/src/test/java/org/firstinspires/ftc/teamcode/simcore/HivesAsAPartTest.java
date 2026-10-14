package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class HivesAsAPartTest {
    private static final double REST_TILT = 30;
    private static final double SAME = 1e-9;
    private static final Length BALL = Valid.length(1.4);
    private static final double WITHIN = 0.5;
    private static final double STEP = Flight.STEP_NANOS / Flight.NANOS_PER_SECOND;
    private static final long TIPPED_AT_STEP = 2000;
    private static final double TIPPED_AT = TIPPED_AT_STEP * STEP;
    private static final long STEPS_OF_A_TIP = Math.round(Lean.TIP_SECONDS / STEP);
    private static final Field FIELD = CollidersTest.field(REST_TILT);
    private static final Hives AT_REST = Valid.value(Hives.of(FIELD));
    private static final Field.Hive BLUE = AT_REST.hiveOf("Blue").orElseThrow();
    private static final Field.Hive RED = AT_REST.hiveOf("Red").orElseThrow();

    private static GridTime onTheGrid(long steps) {
        return GridTime.at(steps, Valid.value(Flight.stepping()));
    }

    private static Hives tippingBlue() {
        return AT_REST.at(onTheGrid(TIPPED_AT_STEP)).tipped(BLUE);
    }

    private static List<String> panels(String cell) {
        return List.of(
                cell + " / wall <1>",
                cell + " / wall <2>",
                cell + " / wall <3>",
                cell + " / wall <4>",
                cell + " / back");
    }

    private static List<Vec3> around(Field.Hive hive, double step) {
        List<Vec3> points = new ArrayList<>();
        Vec3 pivot = hive.pivot();
        for (double dx = -12; dx <= 12; dx += step) {
            for (double dy = -5; dy <= 5; dy += step) {
                for (double dz = -7; dz <= 8; dz += step) {
                    points.add(pivot.plus(new Vec3(dx, dy, dz)));
                }
            }
        }
        return points;
    }

    private static List<Colliders.Near> hivesIn(Colliders colliders, Vec3 at) {
        List<Colliders.Near> near = new ArrayList<>();
        for (Colliders.Near one : colliders.near(at, BALL, WITHIN)) {
            if (AT_REST.part().ids().contains(one.collider().name())) {
                near.add(one);
            }
        }
        return near;
    }

    private static int assertMeetsABallAsTheCollidersDo(Hives hives, Colliders colliders) {
        int compared = 0;
        for (Field.Hive hive : List.of(BLUE, RED)) {
            for (Vec3 at : around(hive, 0.7)) {
                List<Colliders.Near> expected = hivesIn(colliders, at);
                List<Surroundings.Touch> touches = hives.part().near(at, BALL, WITHIN);
                assertEquals("touches at " + at, expected.size(), touches.size());
                for (int i = 0; i < touches.size(); i++) {
                    Colliders.Near near = expected.get(i);
                    Surroundings.Touch touch = touches.get(i);
                    String what = near.collider().name() + " at " + at;
                    assertEquals(what, near.collider().name(), touch.id());
                    assertEquals(what, near.collider().surface(), touch.surface());
                    assertEquals(what, near.gap(), touch.gap(), SAME);
                    assertEquals(what, 0, near.normal().minus(touch.normal()).length(), SAME);
                    assertEquals(what, Vec3.zero(), touch.velocity());
                    assertEquals(what, Vec3.zero(), touch.spin());
                    compared++;
                }
            }
        }
        return compared;
    }

    @Test
    public void theHivesAnswerToTheNamesTheShotsCollidersGiveTheirPartsAndPanelsInTheirOrder() {
        List<String> expected = new ArrayList<>(List.of("Blue Hive / Bracket <1>"));
        expected.addAll(panels("Blue Cell (Scoring)"));
        expected.addAll(panels("Blue Cell (Audience)"));
        expected.add("Red Hive / Bracket <1>");
        expected.addAll(panels("Red Cell (Scoring)"));
        expected.addAll(panels("Red Cell (Audience)"));

        assertEquals(expected, AT_REST.part().ids());
        assertEquals(expected, tippingBlue().part().at(Valid.time(0.7)).ids());
    }

    @Test
    public void aHiveAtRestMeetsABallUnderEachNameExactlyAsTheShotsCollidersDo() {
        int compared = assertMeetsABallAsTheCollidersDo(AT_REST, Valid.value(Colliders.of(FIELD, List.of())));

        assertTrue("compared " + compared + " touches", compared > 200);
    }

    @Test
    public void aHiveThatHasTippedMeetsABallUnderEachNameExactlyAsTheShotsCollidersTippedDo() {
        Hives tipped = tippingBlue().at(Valid.time(TIPPED_AT + Lean.TIP_SECONDS + 0.01));

        assertFalse("the tip is over", tipped.part().moving());
        int compared = assertMeetsABallAsTheCollidersDo(tipped, Valid.value(Colliders.of(FIELD, List.of("Blue"))));

        assertTrue("compared " + compared + " touches", compared > 200);
    }

    @Test
    public void theHivesKeepThePartContractAtRestWhileOneTipsAndOnceItHasTipped() {
        List<PartContract.Probe> probes = new ArrayList<>();
        for (Vec3 at : around(BLUE, 1.3)) {
            probes.add(new PartContract.Probe(at, BALL, 0.3));
        }
        PartContract.Still redIsStill = (seconds, touch) -> touch.id().startsWith("Red");

        PartContract.assertKept("at rest", AT_REST.part(), probes, List.of(0.0, 0.5), PartContract.NO_FACE_STILL);
        PartContract.assertKept(
                "while blue tips",
                tippingBlue().part(),
                probes,
                List.of(TIPPED_AT, 0.25, 0.5, 0.7, 0.95, 1.15, TIPPED_AT + Lean.TIP_SECONDS, 1.5),
                redIsStill);
    }

    @Test
    public void aTippingHiveSpinsAboutItsAxleSoWhereItTouchesABallMovesAsThatPartOfTheHiveDoes() {
        Field.Cell up = AT_REST.upturnedCell(BLUE).orElseThrow();
        Vec3 inward = up.mouthCentre().minus(up.back().first()).unit();
        Vec3 local = up.back().first().along(inward, 1.0).plus(new Vec3(0, 1.5, 1.5));
        double t = TIPPED_AT + 0.4, dt = 1e-5;
        Hives then = tippingBlue().at(Valid.time(t));
        double tilt = then.tilt(BLUE);
        Vec3 at = BLUE.at(tilt, local);

        Surroundings.Touch back = null;
        for (Surroundings.Touch touch : then.part().near(at, BALL, WITHIN)) {
            if (touch.id().equals(up.name() + " / back")) {
                back = touch;
            }
        }

        assertTrue("it touches the back", back != null);
        double rate = StrictMath.toRadians(then.lean(BLUE).degreesPerSecond());
        assertEquals(new Vec3(0, -rate, 0), back.spin());
        Vec3 contact = at.along(back.normal(), -(back.gap() + BALL.inches()));
        Vec3 fixed = unturned(BLUE, tilt, contact);
        Vec3 moving = BLUE.at(tippingBlue().at(Valid.time(t + dt)).tilt(BLUE), fixed)
                .minus(BLUE.at(tippingBlue().at(Valid.time(t - dt)).tilt(BLUE), fixed))
                .times(1 / (2 * dt));
        assertEquals(
                "as fast as that point of the hive: " + moving,
                0,
                moving.minus(back.velocity()).length(),
                1e-4);
        assertTrue("which it does: " + moving, moving.length() > 1);
    }

    private static Vec3 unturned(Field.Hive hive, double tilt, Vec3 point) {
        double c = StrictMath.cos(StrictMath.toRadians(tilt)), s = StrictMath.sin(StrictMath.toRadians(tilt));
        Vec3 d = point.minus(hive.pivot());
        return new Vec3(d.x() * c + d.z() * s, d.y(), -d.x() * s + d.z() * c);
    }

    @Test
    public void aTippingHiveSweepsAtItsTipsFastestRateTimesItsFarthestReachFromItsAxleAndAHiveAtRestNotAtAll() {
        double farthest = 0;
        for (Collider collider : Valid.value(Colliders.of(FIELD, List.of())).all()) {
            if (collider.name().startsWith("Blue") && AT_REST.part().ids().contains(collider.name())) {
                for (Ring<Vec3> face : collider.faces()) {
                    for (Vec3 corner : face.all()) {
                        farthest = StrictMath.max(
                                farthest, corner.minus(BLUE.pivot()).length());
                    }
                }
            }
        }
        double fastest = StrictMath.toRadians(2 * REST_TILT * StrictMath.PI / (2 * Lean.TIP_SECONDS));

        for (double t : List.of(TIPPED_AT, 0.6, 1.1)) {
            Surroundings.Part part = tippingBlue().part().at(Valid.time(t));
            assertTrue("moving at " + t, part.moving());
            assertEquals("at " + t, fastest * farthest, part.sweep(), 1e-9);
        }
        assertFalse(AT_REST.part().moving());
        assertEquals(0, AT_REST.part().sweep(), 0);
        assertFalse(tippingBlue()
                .part()
                .at(onTheGrid(TIPPED_AT_STEP + STEPS_OF_A_TIP))
                .moving());
    }

    @Test
    public void aTipIsTimedFromWhenItBeganWhereverTheHivesArePlacedFrom() {
        Hives placedOnce = tippingBlue().at(onTheGrid(9000));
        Hives placedOften = tippingBlue()
                .at(onTheGrid(3000))
                .at(Valid.time(0.45))
                .at(onTheGrid(6100))
                .at(onTheGrid(9000));

        assertEquals(placedOnce.lean(BLUE), placedOften.lean(BLUE));
        assertEquals(new Lean.Resting(Valid.value(Tilt.of(-REST_TILT))), placedOften.lean(RED));
        Vec3 at = BLUE.pivot().plus(new Vec3(6, 0, 1));
        assertEquals(placedOnce.part().near(at, BALL, 3), placedOften.part().near(at, BALL, 3));
        assertEquals(
                new Lean.Tipping(
                        Valid.value(Tilt.of(REST_TILT)),
                        Valid.value(Tilt.of(-REST_TILT)),
                        Valid.value(Seconds.of((9000 - TIPPED_AT_STEP) * STEP))),
                placedOnce.lean(BLUE));
    }

    @Test
    public void aTipBegunAtAnyStepOnTheGridEndsExactlyTipSecondsOfStepsLater() {
        for (long began : List.of(0L, 45L, 46L, 66L, 187L, 1235L)) {
            Hives tipped = AT_REST.at(onTheGrid(began)).tipped(BLUE);

            assertTrue(
                    "a step short, begun at " + began,
                    tipped.at(onTheGrid(began + STEPS_OF_A_TIP - 1)).tipping(BLUE));
            assertFalse(
                    "over, begun at " + began,
                    tipped.at(onTheGrid(began + STEPS_OF_A_TIP)).tipping(BLUE));
            assertFalse(
                    "and still, begun at " + began,
                    tipped.at(onTheGrid(began + STEPS_OF_A_TIP)).part().moving());
        }
    }

    @Test
    public void aHivePlacedAtATimeBeforeItsTipBeganRestsAtTheTiltItTipsFromAndDoesNotMove() {
        Hives before = tippingBlue().at(onTheGrid(TIPPED_AT_STEP - 1));

        assertEquals(new Lean.Resting(Valid.value(Tilt.of(REST_TILT))), before.lean(BLUE));
        assertFalse(before.tipping(BLUE));
        assertFalse(before.part().moving());
        assertTrue(
                "and tips from the step it began on",
                before.at(onTheGrid(TIPPED_AT_STEP + 1)).tipping(BLUE));
    }

    @Test
    public void aHiveThatIsAlreadyTippingIsNotStartedAgain() {
        Hives tipping = tippingBlue().at(Valid.time(0.5));

        assertEquals(tipping.lean(BLUE), tipping.tipped(BLUE).lean(BLUE));
        assertEquals(
                tipping.at(Valid.time(0.8)).lean(BLUE),
                tipping.tipped(BLUE).at(Valid.time(0.8)).lean(BLUE));
    }

    @Test
    public void eachPanelSaysWhichCellItBelongsToAndAHivesOtherPartsBelongToNone() {
        for (Field.Cell cell : FIELD.cells()) {
            for (String panel : panels(cell.name())) {
                assertEquals(panel, Optional.of(cell), AT_REST.cellOf(panel));
            }
        }
        assertEquals(Optional.empty(), AT_REST.cellOf("Blue Hive / Bracket <1>"));
        assertEquals(Optional.empty(), AT_REST.cellOf("Floor"));
    }
}
