package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.simcore.Collider;
import org.firstinspires.ftc.teamcode.simcore.Colliders;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Lean;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Surface;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class SeasonCollidersTest {
    private static final int FLOOR = 1;
    private static final int WALLS = 4;
    private static final int ELEMENTS = 51;
    private static final int HIVE_PARTS = 2 * 10;
    private static final int CELL_PANELS = 2 * 2 * (10 + 1);
    private static final double WALL_REACH_TOLERANCE_IN = 0.1;
    private static final Length A_POINT = Valid.value(Length.of(1e-9));

    private final Field field = SimPlacement.FIELD;
    private final Colliders atRest = Valid.value(Colliders.of(field, List.of()));

    private static Colliders.Mouth mouth(Colliders colliders, String cell) {
        for (Colliders.Mouth mouth : colliders.mouths()) {
            if (mouth.cell().equals(cell)) {
                return mouth;
            }
        }
        throw new AssertionError("no mouth of " + cell + " among " + colliders.mouths());
    }

    @Test
    public void theSeasonsFieldIsTheFloorFourWallsItsElementsAndItsHivesPartsAndPanels() {
        assertEquals(ELEMENTS, field.elements().size());
        assertEquals(
                FLOOR + WALLS + ELEMENTS + HIVE_PARTS + CELL_PANELS,
                atRest.all().size());
        assertEquals(List.of("Floor", "Far wall", "Audience wall", "Red wall", "Blue wall"), firstNames(5));
        assertEquals(field.size(), atRest.size(), 0);
        assertEquals(field.wallHeight(), atRest.wallHeight(), 0);
    }

    @Test
    public void withoutItsHivesTheSeasonsFieldIsTheFloorFourWallsAndItsElementsInTheirOrder() {
        Colliders without = Valid.value(Colliders.withoutHives(field));

        assertEquals(FLOOR + WALLS + ELEMENTS, without.all().size());
        for (int i = 0; i < without.all().size(); i++) {
            assertEquals(atRest.all().get(i).name(), without.all().get(i).name());
        }
    }

    private List<String> firstNames(int count) {
        List<String> names = new ArrayList<>();
        for (Collider collider : atRest.all()) {
            if (names.size() < count) {
                names.add(collider.name());
            }
        }
        return names;
    }

    @Test
    public void everyElementIsMadeOfFlowerOrFrameByItsGroup() {
        Map<String, Surface> surfaces = new LinkedHashMap<>();
        for (Collider collider : atRest.all()) {
            surfaces.put(collider.name(), collider.surface());
        }
        Map<String, Integer> seen = new LinkedHashMap<>();
        for (Field.Element element : field.elements()) {
            String key = element.group() + " / " + element.name();
            int number = seen.getOrDefault(key, 0) + 1;
            seen.put(key, number);
            String name = key + " <" + number + ">";
            Surface expected = element.group().startsWith("Flower") ? Surface.FLOWER : Surface.FRAME;
            assertEquals(name, expected, surfaces.get(name));
        }
        for (Field.Hive hive : field.hives()) {
            for (Field.Cell cell : hive.cells()) {
                assertEquals(Surface.CELL, surfaces.get(cell.name() + " / back"));
                assertEquals(Surface.CELL, surfaces.get(cell.name() + " / wall <10>"));
            }
        }
    }

    @Test
    public void theNectarInEachHiveIsHeldByItsOwnCell() {
        assertFalse(field.cellPieces().isEmpty());
        for (Field.Piece piece : field.cellPieces()) {
            Field.Cell cell = ((Field.Place.InCell) piece.place()).cell();
            Optional<Colliders.InCell> holding = atRest.cellHolding(piece.at());
            assertEquals(
                    piece.name() + " at " + piece.at(), Optional.of(cell.name()), holding.map(Colliders.InCell::cell));
            assertEquals(Optional.of(cell.alliance()), holding.map(Colliders.InCell::alliance));
            assertEquals(
                    "a hive is set up with its nectar in its upturned cell",
                    Optional.of(true),
                    holding.map(Colliders.InCell::upturned));
        }
    }

    @Test
    public void thePollenInEachFlowerIsHeldByThatFlower() {
        assertFalse(field.flowerPieces().isEmpty());
        for (Field.Piece piece : field.flowerPieces()) {
            Field.Flower flower = ((Field.Place.InFlower) piece.place()).flower();
            assertEquals(
                    piece.name() + " at " + piece.at(), Optional.of(flower.name()), atRest.flowerHolding(piece.at()));
        }
        for (Field.Piece piece : field.loosePieces()) {
            assertEquals(piece.name() + " at " + piece.at(), Optional.empty(), atRest.flowerHolding(piece.at()));
        }
    }

    @Test
    public void blueScoresInItsScoringCellUntilItsHiveTipsAndThenInItsAudienceCell() {
        Colliders tipped = Valid.value(Colliders.of(field, List.of("Blue")));

        assertTrue(mouth(atRest, "Blue Cell (Scoring) <1>").upturned());
        assertFalse(mouth(atRest, "Blue Cell (Audience) <1>").upturned());
        assertFalse(mouth(tipped, "Blue Cell (Scoring) <1>").upturned());
        assertTrue(mouth(tipped, "Blue Cell (Audience) <1>").upturned());
        assertEquals(-atRest.tilt("Blue"), tipped.tilt("Blue"), 0);
        assertEquals(atRest.tilt("Red"), tipped.tilt("Red"), 0);
        assertTrue(mouth(tipped, "Red Cell (Audience) <1>").upturned());
    }

    @Test
    public void noElementReachesIntoAWall() {
        List<String> walls = List.of("Far wall", "Audience wall", "Red wall", "Blue wall");
        for (Collider wall : atRest.all()) {
            if (!walls.contains(wall.name())) {
                continue;
            }
            for (Field.Element element : field.elements()) {
                double deepest = 0;
                for (Vec3 vertex : element.vertices()) {
                    deepest = Math.max(deepest, -atRest.gap(wall, vertex, A_POINT));
                }
                assertTrue(
                        element.group() + " / " + element.name() + " reaches " + deepest + " in into " + wall.name(),
                        deepest < WALL_REACH_TOLERANCE_IN);
            }
        }
    }

    private static final Length A_BALL = Valid.value(Length.of(1.4));
    private static final double WITHIN = 0.02;
    private static final double SAME = 1e-6;

    private static List<Vec3> around(Field.Hive hive) {
        List<Vec3> points = new ArrayList<>();
        for (double dx = -26; dx <= 26; dx += 1.7) {
            for (double dy = -12; dy <= 12; dy += 2.1) {
                for (double dz = -16; dz <= 18; dz += 1.5) {
                    points.add(hive.pivot().plus(new Vec3(dx, dy, dz)));
                }
            }
        }
        return points;
    }

    private List<String> partsAndPanelsOf(Field.Hive hive) {
        List<String> mine = new ArrayList<>(List.of(hive.name() + " / "));
        for (Field.Cell cell : hive.cells()) {
            mine.add(cell.name() + " / ");
        }
        return mine;
    }

    private Map<String, Colliders.Near> hivesIn(Colliders colliders, Vec3 at) {
        Map<String, Colliders.Near> near = new LinkedHashMap<>();
        for (Colliders.Near one : colliders.near(at, A_BALL, WITHIN)) {
            for (Field.Hive hive : field.hives()) {
                if (partsAndPanelsOf(hive).stream().anyMatch(one.collider().name()::startsWith)) {
                    near.put(one.collider().name(), one);
                }
            }
        }
        return near;
    }

    private static Hives tipped(Hives hives, Field field) {
        Hives tipped = hives;
        for (Field.Hive hive : field.hives()) {
            tipped = tipped.tipped(hive);
        }
        return tipped.after(Valid.value(Seconds.of(Lean.TIP_SECONDS + 0.01)));
    }

    private int assertMeetsABallAsTheCollidersDo(Hives hives, Colliders colliders) {
        int compared = 0;
        for (Field.Hive hive : field.hives()) {
            for (Vec3 at : around(hive)) {
                Map<String, Colliders.Near> expected = hivesIn(colliders, at);
                List<String> named = new ArrayList<>();
                for (Hives.Touch touch : hives.near(at, A_BALL, WITHIN)) {
                    named.add(touch.id());
                    Colliders.Near near = expected.get(touch.id());
                    assertTrue(touch.id() + " at " + at + " is not among " + expected.keySet(), near != null);
                    assertEquals(touch.id() + " at " + at, near.gap(), touch.gap(), SAME);
                    assertEquals(
                            touch.id() + " at " + at,
                            0,
                            near.normal().minus(touch.normal()).length(),
                            SAME);
                    compared++;
                }
                assertEquals("at " + at, new ArrayList<>(expected.keySet()), named);
            }
        }
        return compared;
    }

    @Test
    public void eachHiveMeetsABallUnderEachNameExactlyAsTheShotsCollidersDoAtRestAndTipped() {
        Hives hives = Valid.value(Hives.of(field));
        List<String> both = new ArrayList<>();
        for (Field.Hive hive : field.hives()) {
            both.add(hive.alliance());
        }

        int still = assertMeetsABallAsTheCollidersDo(hives, atRest);
        int over = assertMeetsABallAsTheCollidersDo(tipped(hives, field), Valid.value(Colliders.of(field, both)));

        assertTrue("compared " + still + " touches at rest", still > 500);
        assertTrue("compared " + over + " touches tipped", over > 500);
    }

    @Test
    public void aHivePartWayThroughATipMeetsABallAsItWouldAtRestWithTheBallTurnedBackAboutItsAxle() {
        Hives hives = Valid.value(Hives.of(field));
        Field.Hive blue = hives.hiveOf("Blue").orElseThrow();
        double rest = hives.tilt(blue);
        int compared = 0;
        for (double seconds : List.of(0.2, 0.5, 0.8)) {
            Hives tipping = hives.tipped(blue).after(Valid.value(Seconds.of(seconds)));
            double tilt = tipping.tilt(blue);
            for (Vec3 at : around(blue)) {
                Vec3 local = unturned(blue, tilt, at);
                List<Hives.Touch> expected = hives.near(blue.at(rest, local), A_BALL, WITHIN);
                List<Hives.Touch> touches = tipping.near(at, A_BALL, WITHIN);
                assertEquals("at " + at + " tilted " + tilt, expected.size(), touches.size());
                for (int i = 0; i < touches.size(); i++) {
                    Hives.Touch was = expected.get(i), is = touches.get(i);
                    String what = is.id() + " at " + at + " tilted " + tilt;
                    assertEquals(what, was.id(), is.id());
                    assertEquals(what, was.gap(), is.gap(), SAME);
                    Vec3 turned = blue.direction(tilt, unturnedDirection(rest, was.normal()));
                    assertEquals(what, 0, turned.minus(is.normal()).length(), SAME);
                    compared++;
                }
            }
        }
        assertTrue("compared " + compared + " touches", compared > 500);
    }

    private static Vec3 unturned(Field.Hive hive, double tilt, Vec3 point) {
        return unturnedDirection(tilt, point.minus(hive.pivot()));
    }

    private static Vec3 unturnedDirection(double tilt, Vec3 direction) {
        double c = Math.cos(Math.toRadians(tilt)), s = Math.sin(Math.toRadians(tilt));
        return new Vec3(direction.x() * c + direction.z() * s, direction.y(), -direction.x() * s + direction.z() * c);
    }
}
