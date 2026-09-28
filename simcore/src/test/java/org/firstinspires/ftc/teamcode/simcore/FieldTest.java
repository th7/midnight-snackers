package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.Test;

public class FieldTest {
    private static final double DELTA = 1e-12;

    private static final List<Vec3> TETRAHEDRON =
            List.of(new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1));
    private static final List<List<Integer>> TETRAHEDRON_FACES =
            List.of(List.of(0, 2, 1), List.of(0, 1, 3), List.of(0, 3, 2), List.of(1, 2, 3));

    private static List<Vec3> square(double x) {
        return List.of(new Vec3(x, -1, -1), new Vec3(x, 1, -1), new Vec3(x, 1, 1), new Vec3(x, -1, 1));
    }

    private static Field.CellShape box(String name) {
        List<Vec3> mouth = square(10), back = square(0);
        return new Field.CellShape(
                name,
                "Scoring",
                mouth,
                back,
                List.of(
                        List.of(new Vec3(0, -1, -1), new Vec3(10, -1, -1), new Vec3(10, 1, -1), new Vec3(0, 1, -1)),
                        List.of(new Vec3(0, 1, -1), new Vec3(10, 1, -1), new Vec3(10, 1, 1), new Vec3(0, 1, 1)),
                        List.of(new Vec3(0, 1, 1), new Vec3(10, 1, 1), new Vec3(10, -1, 1), new Vec3(0, -1, 1)),
                        List.of(new Vec3(0, -1, 1), new Vec3(10, -1, 1), new Vec3(10, -1, -1), new Vec3(0, -1, -1))));
    }

    private static Field.Hive hive(String alliance, String colour, Vec3 pivot) {
        return Valid.value(Field.Hive.of(
                alliance + " Hive",
                alliance,
                colour,
                pivot,
                Valid.value(Tilt.of(30)),
                List.of(box(alliance + " Cell")),
                List.of()));
    }

    private static String rejection(Checked<?> checked) {
        return checked.fold(value -> "accepted " + value, rule -> rule);
    }

    @Test
    public void anElementsFacesAreItsOwnVerticesInOrder() {
        Field.Element element =
                Valid.value(Field.Element.of("Frame", "Tetrahedron", "#ffffff", TETRAHEDRON, TETRAHEDRON_FACES));

        assertEquals(4, element.faces().size());
        assertEquals(
                List.of(new Vec3(0, 0, 0), new Vec3(0, 1, 0), new Vec3(1, 0, 0)),
                element.faces().get(0).all());
    }

    @Test
    public void anElementIsRefusedAFaceNamingAVertexItHasNot() {
        String said = rejection(Field.Element.of(
                "Frame",
                "Broken",
                "#ffffff",
                TETRAHEDRON,
                List.of(List.of(0, 2, 1), List.of(0, 1, 3), List.of(0, 3, 2), List.of(1, 2, 4))));

        assertTrue(said, said.contains("no vertex 4"));
    }

    @Test
    public void anElementIsAClosedShapeOfFlatFacesWithArea() {
        assertTrue(rejection(Field.Element.of("F", "Open", "#fff", TETRAHEDRON, TETRAHEDRON_FACES.subList(0, 3)))
                .contains("at least 4 faces"));
        assertTrue(rejection(Field.Element.of(
                        "F",
                        "Sliver",
                        "#fff",
                        TETRAHEDRON,
                        List.of(List.of(0, 1), List.of(0, 1, 3), List.of(0, 3, 2), List.of(1, 2, 3))))
                .contains("at least 3 corners"));
        assertTrue(rejection(Field.Element.of(
                        "F",
                        "Flat",
                        "#fff",
                        List.of(new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(2, 0, 0), new Vec3(0, 0, 1)),
                        TETRAHEDRON_FACES))
                .contains("has none"));
    }

    @Test
    public void aHiveTurnsWhatIsInItsFrameAboutItsPivotByItsTilt() {
        Field.Hive hive = hive("Blue", "#0000ff", new Vec3(1, 2, 3));

        Vec3 up = hive.direction(90, new Vec3(1, 0, 0));
        assertEquals(0, up.x(), DELTA);
        assertEquals(1, up.z(), DELTA);
        assertEquals(new Vec3(1, 2 + 5, 3), hive.at(0, new Vec3(0, 5, 0)));
        Vec3 turned = hive.at(90, new Vec3(1, 0, 0));
        assertEquals(1, turned.x(), DELTA);
        assertEquals(4, turned.z(), DELTA);
    }

    @Test
    public void aCellsMouthFacesOutOfItAndIsUpturnedWhileItFacesUp() {
        Field.Cell cell = hive("Blue", "#0000ff", Vec3.zero()).cells().get(0);

        assertEquals(new Vec3(5, 0, 0), cell.centre());
        assertEquals(new Vec3(10, 0, 0), cell.mouthCentre());
        assertEquals(new Vec3(1, 0, 0), cell.mouthNormal());
        assertTrue(cell.upturnedAt(30));
        assertFalse(cell.upturnedAt(-30));
        assertEquals(5, cell.panels().size());
        assertEquals(cell.back(), cell.panels().get(4));
    }

    @Test
    public void aHiveIsRefusedACellWithoutAMouth() {
        Field.CellShape shape = box("Blue Cell");
        String said = rejection(Field.Hive.of(
                "Blue Hive",
                "Blue",
                "#0000ff",
                Vec3.zero(),
                Valid.value(Tilt.of(30)),
                List.of(new Field.CellShape(shape.name(), shape.side(), List.of(), shape.back(), shape.walls())),
                List.of()));

        assertTrue(said, said.contains("at least 3 corners"));
    }

    @Test
    public void anObstacleStandsAboveWhereItClearsTheFloor() {
        ConvexPolygon square = Valid.polygon(List.of(new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 1), new Vec2(0, 1)));

        assertEquals(4, Valid.value(Field.Obstacle.of("Post", square, 0, 4)).stands(), 0);
        assertTrue(rejection(Field.Obstacle.of("Post", square, 4, 4)).contains("stands above"));
    }

    @Test
    public void aFlowersBoreHoldsWhatStandsWithinItOfItsAxis() {
        Field.Flower flower = Valid.value(Field.Flower.of("Flower", new Vec2(10, 0), 2, 2.4, 4.25, 0.35));

        assertTrue(flower.standsIn(11.9, 0));
        assertFalse(flower.standsIn(12.1, 0));
        assertTrue(rejection(Field.Flower.of("Flower", new Vec2(10, 0), 0, 2.4, 4.25, 0.35))
                .contains("positive"));
    }

    @Test
    public void anAllianceStandsInTheBoxItsHivesTapeMarksBeyondTheWalls() {
        Field.Hive blue = hive("Blue", "#0000FF", new Vec3(0, -10, 40));
        List<Field.Mark> tape = List.of(
                new Field.Mark(
                        "#0000ff",
                        List.of(new Vec2(-20, -80), new Vec2(-19, -80), new Vec2(-19, -72), new Vec2(-20, -72))),
                new Field.Mark(
                        "#0000ff", List.of(new Vec2(19, -80), new Vec2(20, -80), new Vec2(20, -72), new Vec2(19, -72))),
                new Field.Mark("#0000ff", List.of(new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 1), new Vec2(0, 1))));

        Field field = Valid.value(Field.of(
                Valid.length(141), Valid.length(12), List.of(), List.of(), List.of(blue), List.of(), List.of(), tape));

        Field.AllianceArea area = Valid.value(field.allianceArea("Blue"));
        assertEquals(-20, area.minX(), 0);
        assertEquals(20, area.maxX(), 0);
        assertEquals(-80, area.minY(), 0);
        assertEquals(-72, area.maxY(), 0);
        assertEquals(new Vec3(0, -76, Field.AllianceArea.EYE_IN), area.eye());
        assertTrue(rejection(field.allianceArea("Green")).contains("nowhere for Green"));
    }

    @Test
    public void aFieldWithNowhereForAHivesDriversIsRefused() {
        String said = rejection(Field.of(
                Valid.length(141),
                Valid.length(12),
                List.of(),
                List.of(),
                List.of(hive("Red", "#ff0000", Vec3.zero())),
                List.of(),
                List.of(),
                List.of()));

        assertTrue(said, said.contains("nowhere for Red's drivers to stand"));
    }

    @Test
    public void theMovedPiecesAreTheLooseThenThoseInCellsThenThoseInFlowers() {
        Field.Hive blue = hive("Blue", "#0000ff", Vec3.zero());
        Field.Flower flower = Valid.value(Field.Flower.of("Flower", new Vec2(10, 0), 2, 2.4, 4.25, 0.35));
        Field.Piece inAFlower = piece("A", new Field.Place.InFlower(flower));
        Field.Piece inACell =
                piece("Blue B", new Field.Place.InCell(blue.cells().get(0)));
        Field.Piece loose = piece("C", new Field.Place.Loose());
        List<Field.Mark> tape = List.of(new Field.Mark(
                "#0000ff", List.of(new Vec2(-1, -80), new Vec2(1, -80), new Vec2(1, -79), new Vec2(-1, -79))));

        Field field = Valid.value(Field.of(
                Valid.length(141),
                Valid.length(12),
                List.of(),
                List.of(),
                List.of(blue),
                List.of(flower),
                List.of(inAFlower, inACell, loose),
                tape));

        assertEquals(List.of(loose, inACell, inAFlower), field.movedPieces());
        assertEquals(Optional.of("Blue"), inACell.alliance());
        assertEquals(Optional.empty(), loose.alliance());
    }

    private static Field.Piece piece(String name, Field.Place place) {
        return Valid.value(Field.Piece.of(name, Field.Kind.POLLEN, place, new Vec3(0, 0, 1.39), Valid.length(1.39)));
    }

    @Test
    public void aPieceIsAPollenOrANectarByName() {
        assertEquals(Field.Kind.NECTAR, Valid.value(Field.Kind.named("Nectar")));
        assertTrue(rejection(Field.Kind.named("Artifact")).contains("Pollen or a Nectar"));
    }

    @Test
    public void aTiltIsWithinAQuarterTurnOfLevel() {
        assertEquals(-90, Valid.value(Tilt.of(-90)).degrees(), 0);
        assertTrue(rejection(Tilt.of(90.5)).contains("within 90.0 degrees"));
        assertTrue(rejection(Tilt.of(Double.NaN)).contains("within 90.0 degrees"));
    }
}
