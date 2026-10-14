package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.Test;

public class CollidersTest {
    private static final double DELTA = 1e-9;
    private static final double SIZE = 100;
    private static final double WALL_HEIGHT = 10;
    private static final double HALF = SIZE / 2;
    private static final double REST_TILT = 30;
    private static final Vec3 BLUE_PIVOT = new Vec3(0, -20, 30);
    private static final Vec3 RED_PIVOT = new Vec3(0, 20, 30);
    private static final Length HALF_INCH = Valid.length(0.5);
    private static final Length ONE_INCH = Valid.length(1);
    private static final double ROOT_HALF = StrictMath.sqrt(0.5);
    private static final double ROOT_THIRD = StrictMath.sqrt(1.0 / 3);

    private static final List<List<Integer>> BOX_FACES = List.of(
            List.of(0, 3, 2, 1),
            List.of(4, 5, 6, 7),
            List.of(0, 1, 5, 4),
            List.of(2, 3, 7, 6),
            List.of(1, 2, 6, 5),
            List.of(0, 4, 7, 3));

    private static List<Vec3> boxCorners(Vec3 centre, Vec3 half) {
        List<Vec3> corners = new ArrayList<>();
        for (double z : List.of(-1.0, 1.0)) {
            corners.add(new Vec3(centre.x() - half.x(), centre.y() - half.y(), centre.z() + z * half.z()));
            corners.add(new Vec3(centre.x() + half.x(), centre.y() - half.y(), centre.z() + z * half.z()));
            corners.add(new Vec3(centre.x() + half.x(), centre.y() + half.y(), centre.z() + z * half.z()));
            corners.add(new Vec3(centre.x() - half.x(), centre.y() + half.y(), centre.z() + z * half.z()));
        }
        return corners;
    }

    private static Field.Element box(String group, String name, Vec3 centre, Vec3 half) {
        return Valid.value(Field.Element.of(group, name, "#888888", boxCorners(centre, half), BOX_FACES));
    }

    private static Field.Element block(Vec3 centre) {
        return box("Frame <1>", "Block", centre, new Vec3(1, 1, 1));
    }

    private static Field.Element insideOut(Field.Element element) {
        List<List<Integer>> reversed = new ArrayList<>();
        for (List<Integer> face : BOX_FACES) {
            List<Integer> backwards = new ArrayList<>(face);
            Collections.reverse(backwards);
            reversed.add(backwards);
        }
        return Valid.value(
                Field.Element.of(element.group(), element.name(), element.colour(), element.vertices(), reversed));
    }

    private static List<Vec3> section(double x) {
        return List.of(new Vec3(x, -3, -3), new Vec3(x, 3, -3), new Vec3(x, 3, 3), new Vec3(x, -3, 3));
    }

    private static List<List<Vec3>> wallsBetween(double back, double mouth) {
        return List.of(
                List.of(new Vec3(back, -3, -3), new Vec3(mouth, -3, -3), new Vec3(mouth, 3, -3), new Vec3(back, 3, -3)),
                List.of(new Vec3(back, 3, -3), new Vec3(mouth, 3, -3), new Vec3(mouth, 3, 3), new Vec3(back, 3, 3)),
                List.of(new Vec3(back, 3, 3), new Vec3(mouth, 3, 3), new Vec3(mouth, -3, 3), new Vec3(back, -3, 3)),
                List.of(
                        new Vec3(back, -3, 3),
                        new Vec3(mouth, -3, 3),
                        new Vec3(mouth, -3, -3),
                        new Vec3(back, -3, -3)));
    }

    private static Field.CellShape cell(String alliance, String side, double toward) {
        return new Field.CellShape(
                alliance + " Cell (" + side + ")",
                side,
                section(10 * toward),
                section(2 * toward),
                wallsBetween(2 * toward, 10 * toward));
    }

    private static Field.Element bracket(String alliance) {
        return box(alliance + " Hive", "Bracket", new Vec3(0, 0, 4), new Vec3(0.5, 0.5, 0.5));
    }

    private static Field.Hive hive(String alliance, String colour, Vec3 pivot, double tilt, Field.CellShape scoring) {
        return Valid.value(Field.Hive.of(
                alliance + " Hive",
                alliance,
                colour,
                pivot,
                Valid.value(Tilt.of(tilt)),
                List.of(scoring, cell(alliance, "Audience", -1)),
                List.of(bracket(alliance))));
    }

    private static Field.Hive blueHive(double tilt) {
        return hive("Blue", "#0000ff", BLUE_PIVOT, tilt, cell("Blue", "Scoring", 1));
    }

    private static Field.Hive redHive() {
        return hive("Red", "#ff0000", RED_PIVOT, -REST_TILT, cell("Red", "Scoring", 1));
    }

    private static final Field.Flower FLOWER =
            Valid.value(Field.Flower.of("Flower Assembly <1>", new Vec2(40, 0), 2, 2.4, 4.25, 0.35));

    private static Field.Element pipe() {
        return box("Flower Assembly <1>", "Flower HIPS Pipe", new Vec3(42.5, 0, 10), new Vec3(0.5, 0.5, 10));
    }

    private static List<Field.Mark> tape() {
        return List.of(
                new Field.Mark(
                        "#0000ff",
                        List.of(new Vec2(-20, -60), new Vec2(20, -60), new Vec2(20, -55), new Vec2(-20, -55))),
                new Field.Mark(
                        "#ff0000", List.of(new Vec2(-20, 55), new Vec2(20, 55), new Vec2(20, 60), new Vec2(-20, 60))));
    }

    private static Field fieldOf(List<Field.Element> elements, Field.Hive blue) {
        return Valid.value(Field.of(
                Valid.length(SIZE),
                Valid.length(WALL_HEIGHT),
                elements,
                List.of(),
                List.of(blue, redHive()),
                List.of(FLOWER),
                List.of(),
                tape()));
    }

    static Field field(double blueTilt) {
        return fieldOf(List.of(block(new Vec3(20, 0, 5)), block(new Vec3(-20, 0, 5)), pipe()), blueHive(blueTilt));
    }

    private static Colliders colliders(double blueTilt) {
        return Valid.value(Colliders.of(field(blueTilt), List.of()));
    }

    private static Colliders atRest() {
        return colliders(REST_TILT);
    }

    private static Collider named(Colliders colliders, String name) {
        for (Collider collider : colliders.all()) {
            if (collider.name().equals(name)) {
                return collider;
            }
        }
        throw new AssertionError("no collider named " + name + " among " + colliders.all());
    }

    private static Colliders.Mouth mouth(Colliders colliders, String cell) {
        for (Colliders.Mouth mouth : colliders.mouths()) {
            if (mouth.cell().equals(cell)) {
                return mouth;
            }
        }
        throw new AssertionError("no mouth of " + cell + " among " + colliders.mouths());
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    private static void assertVec(String what, Vec3 expected, Vec3 actual) {
        assertEquals(what + " x of " + actual, expected.x(), actual.x(), DELTA);
        assertEquals(what + " y of " + actual, expected.y(), actual.y(), DELTA);
        assertEquals(what + " z of " + actual, expected.z(), actual.z(), DELTA);
    }

    private static void assertNear(
            Colliders colliders, String name, Vec3 centre, Length radius, Vec3 normal, double gap) {
        Collider collider = named(colliders, name);
        Colliders.Near near = colliders.near(collider, centre, radius);
        assertEquals(name + " at " + centre, gap, colliders.gap(collider, centre, radius), DELTA);
        assertEquals(name + " at " + centre, gap, near.gap(), DELTA);
        assertVec(name + "'s normal at " + centre, normal, near.normal());
        assertTrue(near.collider() == collider);
    }

    private static List<String> names(Colliders colliders) {
        List<String> names = new ArrayList<>();
        for (Collider collider : colliders.all()) {
            names.add(collider.name());
        }
        return names;
    }

    private static List<String> panels(String cell) {
        return List.of(
                cell + " / wall <1>",
                cell + " / wall <2>",
                cell + " / wall <3>",
                cell + " / wall <4>",
                cell + " / back");
    }

    @Test
    public void theCollidersAreTheFloorTheWallsTheElementsThenEachHivesPartsAndPanels() {
        List<String> expected = new ArrayList<>(List.of(
                "Floor",
                "Far wall",
                "Audience wall",
                "Red wall",
                "Blue wall",
                "Frame <1> / Block <1>",
                "Frame <1> / Block <2>",
                "Flower Assembly <1> / Flower HIPS Pipe <1>",
                "Blue Hive / Bracket <1>"));
        expected.addAll(panels("Blue Cell (Scoring)"));
        expected.addAll(panels("Blue Cell (Audience)"));
        expected.add("Red Hive / Bracket <1>");
        expected.addAll(panels("Red Cell (Scoring)"));
        expected.addAll(panels("Red Cell (Audience)"));

        assertEquals(expected, names(atRest()));
    }

    @Test
    public void withoutItsHivesTheFieldIsTheFloorTheWallsAndTheElementsAsTheyAreWithThem() {
        Colliders without = Valid.value(Colliders.withoutHives(field(REST_TILT)));

        assertEquals(
                List.of(
                        "Floor",
                        "Far wall",
                        "Audience wall",
                        "Red wall",
                        "Blue wall",
                        "Frame <1> / Block <1>",
                        "Frame <1> / Block <2>",
                        "Flower Assembly <1> / Flower HIPS Pipe <1>"),
                names(without));
        for (Collider collider : without.all()) {
            for (Vec3 at :
                    List.of(new Vec3(23, 0, 5), new Vec3(41, 0.4, 3), new Vec3(49.2, 3, 2), new Vec3(1, 2, 0.4))) {
                assertNear(
                        without,
                        collider.name(),
                        at,
                        HALF_INCH,
                        atRest().near(named(atRest(), collider.name()), at, HALF_INCH)
                                .normal(),
                        atRest().gap(named(atRest(), collider.name()), at, HALF_INCH));
            }
        }
        assertTrue("no cell has a mouth", without.mouths().isEmpty());
        assertEquals(
                Optional.empty(),
                without.cellHolding(mouth(atRest(), "Blue Cell (Scoring)")
                        .centre()
                        .along(mouth(atRest(), "Blue Cell (Scoring)").normal(), -1)));
        assertEquals(Optional.of("Flower Assembly <1>"), without.flowerHolding(new Vec3(40.5, 0.5, 5)));
        assertEquals(Optional.of(Colliders.Side.FAR), without.over(new Vec3(HALF + 2.6, 0, 1), HALF_INCH));
        assertEquals(field(REST_TILT).size(), without.size(), 0);
        assertEquals(field(REST_TILT).wallHeight(), without.wallHeight(), 0);
    }

    @Test
    public void eachColliderIsTheKindAndSurfaceOfWhatItIsMadeOf() {
        Colliders colliders = atRest();

        assertEquals(Surface.FLOOR, named(colliders, "Floor").surface());
        assertEquals(Collider.Kind.FLOOR, named(colliders, "Floor").kind());
        assertTrue(named(colliders, "Floor").faces().isEmpty());
        for (String wall : List.of("Far wall", "Audience wall", "Red wall", "Blue wall")) {
            assertEquals(wall, Surface.WALL, named(colliders, wall).surface());
            assertEquals(wall, Collider.Kind.SOLID, named(colliders, wall).kind());
            assertEquals(wall, 6, named(colliders, wall).faces().size());
        }
        assertEquals(Surface.FRAME, named(colliders, "Frame <1> / Block <1>").surface());
        assertEquals(
                Surface.FLOWER,
                named(colliders, "Flower Assembly <1> / Flower HIPS Pipe <1>").surface());
        assertEquals(Surface.FRAME, named(colliders, "Blue Hive / Bracket <1>").surface());
        assertEquals(
                Collider.Kind.SOLID, named(colliders, "Blue Hive / Bracket <1>").kind());
        Collider back = named(colliders, "Red Cell (Audience) / back");
        assertEquals(Surface.CELL, back.surface());
        assertEquals(Collider.Kind.PLATE, back.kind());
        assertEquals(1, back.faces().size());
        assertEquals("plate", back.kind().asked());
    }

    @Test
    public void aBallOffAFaceIsItsDistanceFromTheFaceAwayAlongTheFacesNormal() {
        assertNear(atRest(), "Frame <1> / Block <1>", new Vec3(23, 0, 5), HALF_INCH, new Vec3(1, 0, 0), 1.5);
        assertNear(atRest(), "Frame <1> / Block <1>", new Vec3(20.2, -0.3, 2), HALF_INCH, new Vec3(0, 0, -1), 1.5);
    }

    @Test
    public void aBallOffAnEdgeIsPushedAlongTheBisectorOfItsFaces() {
        assertNear(
                atRest(),
                "Frame <1> / Block <1>",
                new Vec3(22, 0.25, 7),
                HALF_INCH,
                new Vec3(ROOT_HALF, 0, ROOT_HALF),
                StrictMath.sqrt(2) - 0.5);
    }

    @Test
    public void aBallOffACornerIsPushedStraightAwayFromIt() {
        assertNear(
                atRest(),
                "Frame <1> / Block <1>",
                new Vec3(22, 2, 7),
                HALF_INCH,
                new Vec3(ROOT_THIRD, ROOT_THIRD, ROOT_THIRD),
                StrictMath.sqrt(3) - 0.5);
    }

    @Test
    public void aCentreInsideASolidIsPushedOutOfTheFaceItIsLeastDeepBehind() {
        assertNear(atRest(), "Frame <1> / Block <1>", new Vec3(20.75, 0, 5.5), HALF_INCH, new Vec3(1, 0, 0), -0.75);
        assertNear(atRest(), "Frame <1> / Block <1>", new Vec3(19.5, 0.9, 4.4), HALF_INCH, new Vec3(0, 1, 0), -0.6);
    }

    @Test
    public void theGapRunsOnWithoutAJumpAsACentreCrossesAFace() {
        Colliders colliders = atRest();
        Collider block = named(colliders, "Frame <1> / Block <1>");

        double outside = colliders.gap(block, new Vec3(21 + 1e-9, 0.3, 5.2), HALF_INCH);
        double inside = colliders.gap(block, new Vec3(21 - 1e-9, 0.3, 5.2), HALF_INCH);

        assertEquals(-0.5, outside, 1e-8);
        assertEquals(-0.5, inside, 1e-8);
    }

    @Test
    public void aSolidWoundInsideOutStillFacesOutAndIsDrawnFromOutside() {
        Field.Element inward = insideOut(block(new Vec3(20, 0, 5)));
        Colliders colliders = Valid.value(Colliders.of(fieldOf(List.of(inward), blueHive(REST_TILT)), List.of()));
        Collider block = named(colliders, "Frame <1> / Block <1>");

        assertNear(colliders, block.name(), new Vec3(23, 0, 5), HALF_INCH, new Vec3(1, 0, 0), 1.5);
        assertNear(colliders, block.name(), new Vec3(20.75, 0, 5.5), HALF_INCH, new Vec3(1, 0, 0), -0.75);
        Vec3 middle = new Vec3(20, 0, 5);
        for (Ring<Vec3> face : block.faces()) {
            Vec3 normal = Vec3.newellNormalOf(face);
            assertTrue(
                    face + " is wound counter-clockwise seen from outside",
                    face.first().minus(middle).dot(normal) > 0);
        }
    }

    @Test
    public void aPlateIsMetFromEitherSide() {
        Colliders colliders = colliders(0);
        String back = "Blue Cell (Scoring) / back";

        assertNear(colliders, back, new Vec3(0, -20, 30), ONE_INCH, new Vec3(-1, 0, 0), 1);
        assertNear(colliders, back, new Vec3(5, -19, 31), ONE_INCH, new Vec3(1, 0, 0), 2);
    }

    @Test
    public void aPlateIsMetEdgeOn() {
        assertNear(colliders(0), "Blue Cell (Scoring) / back", new Vec3(2, -20, 35), ONE_INCH, new Vec3(0, 0, 1), 1);
        assertNear(
                colliders(0),
                "Blue Cell (Scoring) / back",
                new Vec3(2, -25, 36),
                ONE_INCH,
                new Vec3(0, -2 / StrictMath.sqrt(13), 3 / StrictMath.sqrt(13)),
                StrictMath.sqrt(13) - 1);
    }

    @Test
    public void aCentreOnAPlateIsPushedAlongThePlatesNormalByItsRadius() {
        Colliders colliders = colliders(0);
        Collider back = named(colliders, "Blue Cell (Scoring) / back");

        Colliders.Near near = colliders.near(back, new Vec3(2, -19.5, 30.5), ONE_INCH);

        assertEquals(-1, near.gap(), DELTA);
        assertEquals(1, StrictMath.abs(near.normal().x()), DELTA);
        assertVec(
                "along the plate's normal",
                Vec3.newellNormalOf(back.faces().get(0)).unit(),
                near.normal());
    }

    @Test
    public void theFloorIsEverythingBelowZero() {
        assertNear(atRest(), "Floor", new Vec3(3, 4, 2.5), ONE_INCH, new Vec3(0, 0, 1), 1.5);
        assertNear(atRest(), "Floor", new Vec3(-300, 4000, 0.5), ONE_INCH, new Vec3(0, 0, 1), -0.5);
    }

    @Test
    public void theWallsAreAnInchThickJustOutsideTheFieldAndAsHighAsItsWalls() {
        Colliders colliders = atRest();

        assertNear(colliders, "Far wall", new Vec3(HALF - 1, 0, 5), ONE_INCH, new Vec3(-1, 0, 0), 0);
        assertNear(colliders, "Audience wall", new Vec3(-HALF + 3, 7, 5), ONE_INCH, new Vec3(1, 0, 0), 2);
        assertNear(colliders, "Red wall", new Vec3(-3, HALF - 2, 9), ONE_INCH, new Vec3(0, -1, 0), 1);
        assertNear(colliders, "Blue wall", new Vec3(-3, -HALF + 2, 1), ONE_INCH, new Vec3(0, 1, 0), 1);
        assertNear(colliders, "Far wall", new Vec3(HALF + 0.5, 0, WALL_HEIGHT + 2), ONE_INCH, new Vec3(0, 0, 1), 1);
        assertNear(colliders, "Far wall", new Vec3(HALF + 3, 0, 5), ONE_INCH, new Vec3(1, 0, 0), 1);
        assertEquals(new Vec3(HALF, -HALF - 1, 0), named(colliders, "Far wall").least());
        assertEquals(
                new Vec3(HALF + 1, HALF + 1, WALL_HEIGHT),
                named(colliders, "Far wall").most());
    }

    @Test
    public void theWallsMeetAtTheCorners() {
        Colliders colliders = atRest();
        Vec3 corner = new Vec3(HALF + 0.5, HALF + 0.5, 5);

        for (String wall : List.of("Far wall", "Red wall")) {
            assertTrue(wall, colliders.gap(named(colliders, wall), corner, HALF_INCH) < 0);
        }
        Vec3 otherCorner = new Vec3(-HALF - 0.5, -HALF - 0.5, 5);
        for (String wall : List.of("Audience wall", "Blue wall")) {
            assertTrue(wall, colliders.gap(named(colliders, wall), otherCorner, HALF_INCH) < 0);
        }
    }

    @Test
    public void culledNearAnswersAsEveryColliderAskedInTurnWould() {
        for (double blueTilt : List.of(REST_TILT, 0.0)) {
            Colliders colliders = colliders(blueTilt);
            int found = 0;
            for (double x = -HALF - 2; x <= HALF + 2; x += 2.3) {
                for (double y = -HALF - 2; y <= HALF + 2; y += 2.9) {
                    for (double z = -1; z <= 42; z += 3.1) {
                        Vec3 centre = new Vec3(x, y, z);
                        for (double within : List.of(0.0, 0.4, 2.5)) {
                            List<String> expected = new ArrayList<>();
                            for (Collider collider : colliders.all()) {
                                Colliders.Near near = colliders.near(collider, centre, ONE_INCH);
                                if (near.gap() <= within) {
                                    expected.add(collider.name() + " " + near.gap() + " " + near.normal());
                                }
                            }
                            List<String> culled = new ArrayList<>();
                            for (Colliders.Near near : colliders.near(centre, ONE_INCH, within)) {
                                culled.add(near.collider().name() + " " + near.gap() + " " + near.normal());
                            }
                            assertEquals("near " + centre + " within " + within, expected, culled);
                            found += culled.size();
                        }
                    }
                }
            }
            assertTrue("the points come near something: " + found, found > 1000);
        }
    }

    @Test
    public void aSolidThatIsNotConvexIsRefusedByName() {
        List<Vec3> corners = new ArrayList<>(boxCorners(new Vec3(0, 0, 1), new Vec3(1, 1, 1)));
        corners.add(new Vec3(0, 0, 1.2));
        List<List<Integer>> faces = new ArrayList<>(BOX_FACES.subList(0, 1));
        faces.addAll(BOX_FACES.subList(2, 6));
        faces.addAll(List.of(List.of(4, 5, 8), List.of(5, 6, 8), List.of(6, 7, 8), List.of(7, 4, 8)));
        Field.Element dented = Valid.value(Field.Element.of("Frame <1>", "Dented", "#888888", corners, faces));

        String said = rejection(Colliders.of(fieldOf(List.of(dented), blueHive(REST_TILT)), List.of()));

        assertTrue(said, said.contains("Frame <1> / Dented <1>"));
        assertTrue(said, said.contains("convex"));
    }

    @Test
    public void aSolidWithAnOpenEdgeIsRefusedByName() {
        Field.Element open = Valid.value(Field.Element.of(
                "Frame <1>",
                "Open",
                "#888888",
                boxCorners(new Vec3(0, 0, 1), new Vec3(1, 1, 1)),
                BOX_FACES.subList(0, 5)));

        String said = rejection(Colliders.of(fieldOf(List.of(open), blueHive(REST_TILT)), List.of()));

        assertTrue(said, said.contains("Frame <1> / Open <1>"));
        assertTrue(said, said.contains("closed"));
    }

    @Test
    public void aPanelWithNoAreaIsRefusedByName() {
        Field.CellShape shape = cell("Blue", "Scoring", 1);
        List<List<Vec3>> walls = new ArrayList<>(shape.walls());
        walls.set(0, List.of(new Vec3(2, -3, -3), new Vec3(6, -3, -3), new Vec3(10, -3, -3)));
        Field.Hive blue = hive(
                "Blue",
                "#0000ff",
                BLUE_PIVOT,
                REST_TILT,
                new Field.CellShape(shape.name(), shape.side(), shape.mouth(), shape.back(), walls));

        String said = rejection(Colliders.of(fieldOf(List.of(), blue), List.of()));

        assertTrue(said, said.contains("Blue Cell (Scoring) / wall <1>"));
        assertTrue(said, said.contains("area"));
    }

    @Test
    public void twoCellsOfOneNameAreRefusedByTheNameTheirPanelsWouldShare() {
        Field.Hive blue = hive(
                "Blue",
                "#0000ff",
                BLUE_PIVOT,
                REST_TILT,
                new Field.CellShape("Blue Cell (Audience)", "Scoring", section(10), section(2), wallsBetween(2, 10)));

        String said = rejection(Colliders.of(fieldOf(List.of(), blue), List.of()));

        assertTrue(said, said.contains("'Blue Cell (Audience) / wall <1>'"));
        assertTrue(said, said.contains("name of its own"));
    }

    @Test
    public void anElementNamedAsACellsPanelIsRefusedByThatName() {
        Field.Element impostor = box("Blue Cell (Scoring)", "wall", new Vec3(20, 0, 5), new Vec3(1, 1, 1));

        String said = rejection(Colliders.of(fieldOf(List.of(impostor), blueHive(REST_TILT)), List.of()));

        assertTrue(said, said.contains("'Blue Cell (Scoring) / wall <1>'"));
        assertTrue(said, said.contains("name of its own"));
    }

    @Test
    public void onlyAnAllianceWithAHiveCanHaveItTipped() {
        String said = rejection(Colliders.of(field(REST_TILT), List.of("Blue", "Green")));

        assertTrue(said, said.contains("Green"));
        assertTrue(said, said.contains("Blue"));
        assertTrue(said, said.contains("Red"));
    }

    @Test
    public void aTippedHiveLeansAsFarTheOtherWayAndItsOtherCellIsUpturned() {
        Colliders rest = atRest();
        Colliders tipped = Valid.value(Colliders.of(field(REST_TILT), List.of("Blue")));

        assertEquals(REST_TILT, rest.tilt("Blue"), 0);
        assertEquals(-REST_TILT, tipped.tilt("Blue"), 0);
        assertEquals(-REST_TILT, tipped.tilt("Red"), 0);
        assertEquals(List.of("Blue", "Red"), new ArrayList<>(tipped.tilts().keySet()));
        assertEquals(-REST_TILT, tipped.tilts().get("Blue"), 0);
        assertTrue(mouth(rest, "Blue Cell (Scoring)").upturned());
        assertFalse(mouth(rest, "Blue Cell (Audience)").upturned());
        assertFalse(mouth(tipped, "Blue Cell (Scoring)").upturned());
        assertTrue(mouth(tipped, "Blue Cell (Audience)").upturned());
        assertTrue(mouth(tipped, "Red Cell (Audience)").upturned());
        assertFalse(mouth(tipped, "Red Cell (Scoring)").upturned());
    }

    @Test
    public void aHivesCollidersAreTurnedAboutItsPivotWithStrictMathToTheBit() {
        double radians = StrictMath.toRadians(-REST_TILT);
        double c = StrictMath.cos(radians), s = StrictMath.sin(radians);
        Set<Vec3> expected = new HashSet<>();
        for (Vec3 local : bracket("Blue").vertices()) {
            Vec3 turned = new Vec3(local.x() * c - local.z() * s, local.y(), local.x() * s + local.z() * c);
            expected.add(
                    new Vec3(BLUE_PIVOT.x() + turned.x(), BLUE_PIVOT.y() + turned.y(), BLUE_PIVOT.z() + turned.z()));
        }
        Colliders tipped = Valid.value(Colliders.of(field(REST_TILT), List.of("Blue")));

        Set<Vec3> corners = new HashSet<>();
        for (Ring<Vec3> face : named(tipped, "Blue Hive / Bracket <1>").faces()) {
            corners.addAll(face.all());
        }

        assertEquals(expected, corners);
    }

    @Test
    public void aCellsMouthIsWhereItsHiveLeansItFacingOutOfTheCell() {
        Colliders level = colliders(0);
        Colliders.Mouth flat = mouth(level, "Blue Cell (Scoring)");

        assertEquals("Blue", flat.alliance());
        assertVec("the mouth's centre", new Vec3(10, -20, 30), flat.centre());
        assertVec("facing out", new Vec3(1, 0, 0), flat.normal());
        assertEquals(4, flat.corners().size());
        assertFalse(flat.upturned());
        assertVec(
                "facing out",
                new Vec3(-1, 0, 0),
                mouth(level, "Blue Cell (Audience)").normal());

        Colliders.Mouth leaning = mouth(atRest(), "Blue Cell (Scoring)");
        double c = StrictMath.cos(StrictMath.toRadians(REST_TILT)), s = StrictMath.sin(StrictMath.toRadians(REST_TILT));
        assertVec("turned with the hive", new Vec3(10 * c, -20, 30 + 10 * s), leaning.centre());
        assertVec("turned with the hive", new Vec3(c, 0, s), leaning.normal());
        assertTrue(leaning.upturned());
    }

    @Test
    public void aCellHoldsWhatIsBehindItsMouthAndNothingJustOutsideIt() {
        Colliders colliders = atRest();
        Colliders.Mouth mouth = mouth(colliders, "Blue Cell (Scoring)");

        assertEquals(
                Optional.of(new Colliders.InCell("Blue Cell (Scoring)", "Blue", true)),
                colliders.cellHolding(mouth.centre().along(mouth.normal(), -0.01)));
        assertEquals(Optional.empty(), colliders.cellHolding(mouth.centre().along(mouth.normal(), 0.01)));
        Colliders.Mouth audience = mouth(colliders, "Red Cell (Audience)");
        assertEquals(
                Optional.of(new Colliders.InCell("Red Cell (Audience)", "Red", true)),
                colliders.cellHolding(audience.centre().along(audience.normal(), -4)));
        assertEquals(
                Optional.of(new Colliders.InCell("Red Cell (Scoring)", "Red", false)),
                colliders.cellHolding(
                        mouth(colliders, "Red Cell (Scoring)").centre().along(new Vec3(-1, 0, 0), 0.5)));
        assertEquals(Optional.empty(), colliders.cellHolding(BLUE_PIVOT));
    }

    @Test
    public void aFlowerHoldsWhatIsInItsBoreAndBelowItsTop() {
        Colliders colliders = atRest();

        assertEquals(Optional.of("Flower Assembly <1>"), colliders.flowerHolding(new Vec3(40.5, 0.5, 5)));
        assertEquals(Optional.of("Flower Assembly <1>"), colliders.flowerHolding(new Vec3(40, 0, 19.5)));
        assertEquals(Optional.empty(), colliders.flowerHolding(new Vec3(40, 0, 20.5)));
        assertEquals(Optional.empty(), colliders.flowerHolding(new Vec3(42.1, 0, 5)));
    }

    @Test
    public void aCentreIsOverTheWallItIsBeyondByMoreThanItsRadius() {
        Colliders colliders = atRest();

        assertEquals(Optional.of(Colliders.Side.FAR), colliders.over(new Vec3(HALF + 2.5, 0, 5), ONE_INCH));
        assertEquals(Optional.empty(), colliders.over(new Vec3(HALF + 1.5, 0, 5), ONE_INCH));
        assertEquals(Optional.of(Colliders.Side.AUDIENCE), colliders.over(new Vec3(-HALF - 3, 10, 1), ONE_INCH));
        assertEquals(Optional.of(Colliders.Side.BLUE), colliders.over(new Vec3(0, -HALF - 3, 40), ONE_INCH));
        assertEquals(Optional.of(Colliders.Side.RED), colliders.over(new Vec3(-5, HALF + 3, 0), ONE_INCH));
        assertEquals(Optional.of(Colliders.Side.RED), colliders.over(new Vec3(HALF + 3, HALF + 4, 5), ONE_INCH));
        assertEquals(Optional.empty(), colliders.over(new Vec3(0, 0, 5), ONE_INCH));
        assertEquals(Optional.empty(), colliders.over(new Vec3(HALF - 1, -HALF + 1, 50), ONE_INCH));
    }

    @Test
    public void eachSideSaysWhichWallItIs() {
        assertEquals("the audience's wall", Colliders.Side.AUDIENCE.says());
        assertEquals("the far wall", Colliders.Side.FAR.says());
        assertEquals("blue's wall", Colliders.Side.BLUE.says());
        assertEquals("red's wall", Colliders.Side.RED.says());
    }

    @Test
    public void aCentreCrossingIntoAMouthSaysWhereAndAtWhatAngle() {
        Colliders colliders = colliders(0);

        Colliders.Crossing straight =
                colliders.crossed(new Vec3(12, -20, 30), new Vec3(8, -20, 30)).orElseThrow();

        assertEquals("Blue Cell (Scoring)", straight.cell());
        assertEquals("Blue", straight.alliance());
        assertFalse(straight.upturned());
        assertTrue(straight.into());
        assertEquals(0.5, straight.fraction(), DELTA);
        assertVec("where it crossed", new Vec3(10, -20, 30), straight.at());
        assertEquals(0, straight.offset(), DELTA);
        assertEquals(3, straight.rim(), DELTA);
        assertEquals(0, straight.angleDegrees(), DELTA);

        Colliders.Crossing slanted =
                colliders.crossed(new Vec3(11, -20, 30), new Vec3(9, -20, 32)).orElseThrow();

        assertVec("where it crossed", new Vec3(10, -20, 31), slanted.at());
        assertEquals(1, slanted.offset(), DELTA);
        assertEquals(2, slanted.rim(), DELTA);
        assertEquals(45, slanted.angleDegrees(), DELTA);
    }

    @Test
    public void aCentreCrossingOutOfAMouthSaysSo() {
        Colliders.Crossing out = colliders(0)
                .crossed(new Vec3(9, -21, 30), new Vec3(10.5, -21, 30))
                .orElseThrow();

        assertFalse(out.into());
        assertEquals(1.0 / 1.5, out.fraction(), DELTA);
        assertVec("where it crossed", new Vec3(10, -21, 30), out.at());
        assertEquals(1, out.offset(), DELTA);
        assertEquals(2, out.rim(), DELTA);
        assertEquals(0, out.angleDegrees(), DELTA);
    }

    @Test
    public void aCentreThatMissesTheMouthOrDoesNotReachItCrossesNothing() {
        Colliders colliders = colliders(0);

        assertEquals(Optional.empty(), colliders.crossed(new Vec3(12, -20, 34), new Vec3(8, -20, 34)));
        assertEquals(Optional.empty(), colliders.crossed(new Vec3(12, -20, 30), new Vec3(10.5, -20, 30)));
        assertEquals(Optional.empty(), colliders.crossed(new Vec3(12, -20, 30), new Vec3(12, -20, 30)));
        assertEquals(Optional.empty(), colliders.crossed(new Vec3(9, -20, 30), new Vec3(8, -20, 30)));
    }

    @Test
    public void theyKeepTheFieldsSize() {
        assertEquals(SIZE, atRest().size(), 0);
        assertEquals(WALL_HEIGHT, atRest().wallHeight(), 0);
    }
}
