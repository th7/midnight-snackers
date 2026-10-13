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
import org.firstinspires.ftc.teamcode.simcore.Length;
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
}
