package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.Test;

/**
 * The field model is what the season's CAD says, reduced: the walls where the perimeter stands,
 * the elements as convex shapes, the game pieces, the tape, and what the robot runs into.
 */
public class SimFieldTest {
    private final SimField field = SimField.load();

    @Test
    public void theWallsStandWhereTheCadsPerimeterDoes() {
        assertTrue(
                "a competition field is 141 in between the walls, give or take an inch: " + field.size,
                field.size > 140 && field.size < 142);
        assertTrue(
                "the perimeter is a foot high, give or take: " + field.wallHeight,
                field.wallHeight > 11 && field.wallHeight < 13);
    }

    @Test
    public void theFieldHasThisSeasonsElements() {
        Set<String> groups = new HashSet<>();
        for (SimField.Element element : field.elements) {
            groups.add(element.group);
        }
        assertTrue(groups.toString(), groups.contains("Blue Hive <1>"));
        assertTrue(groups.toString(), groups.contains("Red Hive <1>"));
        assertTrue(groups.toString(), groups.contains("Frame <1>"));
        int flowers = 0;
        for (String group : groups) {
            if (group.startsWith("Flower Assembly")) {
                flowers++;
            }
        }
        assertEquals("four flowers, one at each wall", 4, flowers);
        assertTrue(
                "the field is set up with game pieces",
                field.json().getAsJsonArray("pieces").size() > 0);
    }

    @Test
    public void everyElementIsAClosedShapeOfItsOwnVertices() {
        for (SimField.Element element : field.elements) {
            assertTrue(element.name + " has a body", element.surface || element.faces.length >= 4);
            for (int[] face : element.faces) {
                assertTrue(element.name + " has a face of " + face.length + " vertices", face.length >= 3);
                for (int index : face) {
                    assertTrue(
                            element.name + " names vertex " + index + " of " + element.vertices.length,
                            index >= 0 && index < element.vertices.length);
                }
            }
        }
    }

    /**
     * What the robot collides with: the flowers and the frame's legs and feet, each a convex
     * polygon inside the walls, wound counter-clockwise; nothing that hangs above the robot.
     */
    @Test
    public void theObstaclesAreConvexPolygonsInsideTheWalls() {
        assertTrue(field.obstacles.size() >= 4);
        double half = field.size / 2;
        for (SimField.Obstacle obstacle : field.obstacles) {
            double[][] ring = obstacle.footprint;
            assertTrue(obstacle.name + " has " + ring.length + " corners", ring.length >= 3);
            for (int i = 0; i < ring.length; i++) {
                double[] a = ring[i], b = ring[(i + 1) % ring.length], c = ring[(i + 2) % ring.length];
                double turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
                assertTrue(obstacle.name + " turns left at every corner", turn > 0);
                assertTrue(obstacle.name + " stands inside the walls", Math.abs(a[0]) < half && Math.abs(a[1]) < half);
            }
        }
        assertNotNull(field.obstacle("Flower Assembly <1>"));
        assertNotNull(field.obstacle("Flower Assembly <4>"));
        for (SimField.Obstacle obstacle : field.obstacles) {
            assertTrue(obstacle.name + " hangs above the robot and is no obstacle", !obstacle.name.contains("Hive"));
        }
    }

    @Test
    public void theTapeMarksBothAlliancesOnTheFloor() {
        JsonArray tape = field.json().getAsJsonArray("tape");
        Set<String> colours = new HashSet<>();
        for (int i = 0; i < tape.size(); i++) {
            JsonObject mark = tape.get(i).getAsJsonObject();
            colours.add(mark.get("colour").getAsString());
            assertTrue(
                    "a mark is a strip of four corners",
                    mark.getAsJsonArray("footprint").size() == 4);
        }
        assertEquals("red and blue", 2, colours.size());
    }

    /**
     * A hive cell is its six flat panels, seen through and outlined in its alliance's colour: two
     * sides, a bottom, two tops and a back, each one flat polygon.
     */
    @Test
    public void aHiveCellIsSixFlatPanelsInItsAlliancesColour() {
        for (String hive : List.of("Blue Hive <1>", "Red Hive <1>")) {
            for (String cell : List.of("(Audience)", "(Scoring)")) {
                List<SimField.Element> panels = new ArrayList<>();
                for (SimField.Element element : field.elements) {
                    if (element.group.equals(hive) && element.surface && element.name.contains(cell)) {
                        panels.add(element);
                    }
                }
                assertEquals(hive + " " + cell + " panels: " + panels.size(), 6, panels.size());
                for (SimField.Element panel : panels) {
                    assertEquals(hive.startsWith("Blue") ? "#1651b0" : "#c62828", panel.colour);
                    assertEquals(panel.name + " is one polygon", 1, panel.faces.length);
                    assertTrue(panel.name + " has at least three corners", panel.vertices.length >= 3);
                    assertFlat(panel);
                }
            }
        }
        for (SimField.Element element : field.elements) {
            assertTrue(
                    "only the hives' panels are seen through: " + element.name,
                    !element.surface || element.group.contains("Hive"));
        }
    }

    /**
     * A cell is what the simulator scores in: its six panels, one of which is the <b>mouth</b> the
     * ball comes in through, the rib at the lower, open end, facing out of the cell and down.
     */
    @Test
    public void eachHiveHasTwoCellsAndEachCellAMouthFacingOutAndDown() {
        assertEquals(4, field.cells.size());
        int blue = 0, red = 0;
        for (SimField.Cell cell : field.cells) {
            assertTrue(cell.name, cell.name.startsWith(cell.alliance + " Cell"));
            assertEquals(cell.name + " panels", 6, cell.panels.size());
            assertTrue(cell.name + "'s mouth is one of its panels", cell.panels.contains(cell.mouth));
            assertTrue(cell.name + "'s mouth is a rib", cell.mouthName.endsWith("Goal Rib"));
            assertTrue(cell.name + "'s mouth is below its centre", cell.mouthCentre[2] < cell.centre[2]);
            assertTrue(cell.name + "'s mouth faces down: " + cell.mouthNormal[2], cell.mouthNormal[2] < -0.3);
            assertEquals(cell.name + "'s mouth faces along x", 0, cell.mouthNormal[1], 0.05);
            assertEquals(
                    "a unit normal",
                    1,
                    Math.sqrt(cell.mouthNormal[0] * cell.mouthNormal[0]
                            + cell.mouthNormal[1] * cell.mouthNormal[1]
                            + cell.mouthNormal[2] * cell.mouthNormal[2]),
                    1e-6);
            if (cell.alliance.equals("Blue")) {
                blue++;
                assertTrue("a blue mouth faces the audience: " + cell.mouthNormal[0], cell.mouthNormal[0] < 0);
            } else {
                red++;
                assertEquals("Red", cell.alliance);
                assertTrue("a red mouth faces away from the audience: " + cell.mouthNormal[0], cell.mouthNormal[0] > 0);
            }
        }
        assertEquals(2, blue);
        assertEquals(2, red);
        assertNotNull(field.cell("Blue Cell (Audience) <1>"));
        assertNull(field.cell("Green Cell"));
    }

    /** The frame's top bar joins the two hives, and each hive hangs from it by its pivot brackets. */
    @Test
    public void theHivesHangFromTheBarBetweenThem() {
        Set<String> names = new HashSet<>();
        for (SimField.Element element : field.elements) {
            names.add(element.group + " / " + element.name);
        }
        assertTrue(names.toString(), names.contains("Frame <1> / A-Frame Top Bar"));
        assertTrue(names.toString(), names.contains("Blue Hive <1> / Goal Pivot Bracket"));
        assertTrue(names.toString(), names.contains("Red Hive <1> / Goal Pivot Bracket"));
    }

    /**
     * The pollen on the floor in the open is loose, for the robot to push; pollen held in a flower
     * or lying outside the walls, and the nectar in the hives, stay where they are.
     */
    @Test
    public void thePollenOnTheOpenFloorIsLoose() {
        assertEquals("two rows of four in the corners", 8, field.loosePieces.size());
        double half = field.size / 2;
        for (SimField.Piece piece : field.loosePieces) {
            assertEquals("Pollen", piece.name);
            assertEquals(piece.name + " rests on the floor", piece.radius, piece.z, 0.25);
            assertTrue(
                    piece.name + " is inside the walls",
                    Math.abs(piece.x) < half - piece.radius / 2 && Math.abs(piece.y) < half - piece.radius / 2);
            for (SimField.Obstacle obstacle : field.obstacles) {
                assertTrue(
                        piece.name + " is not held in " + obstacle.name, !inside(obstacle.footprint, piece.x, piece.y));
            }
        }
        JsonArray pieces = field.json().getAsJsonArray("pieces");
        int held = 0;
        for (int i = 0; i < pieces.size(); i++) {
            if (!pieces.get(i).getAsJsonObject().get("loose").getAsBoolean()) {
                held++;
            }
        }
        assertTrue("the flowers' stacks, the rows outside and the nectar are held: " + held, held > 30);
    }

    private static boolean inside(double[][] ring, double x, double y) {
        for (int i = 0; i < ring.length; i++) {
            double[] a = ring[i], b = ring[(i + 1) % ring.length];
            if ((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) < 0) {
                return false;
            }
        }
        return true;
    }

    /** Every vertex lies on the polygon's own plane (its normal by Newell's method, robust to near-collinear corners). */
    private static void assertFlat(SimField.Element panel) {
        double[] n = new double[3];
        double[][] ring = panel.vertices;
        for (int i = 0; i < ring.length; i++) {
            double[] a = ring[i], b = ring[(i + 1) % ring.length];
            n[0] += (a[1] - b[1]) * (a[2] + b[2]);
            n[1] += (a[2] - b[2]) * (a[0] + b[0]);
            n[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        double length = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        double[] a = ring[0];
        for (double[] p : ring) {
            double off = ((p[0] - a[0]) * n[0] + (p[1] - a[1]) * n[1] + (p[2] - a[2]) * n[2]) / length;
            assertEquals(panel.name + " is flat", 0, off, 0.05);
        }
    }

    /** The page draws what the simulator loaded: one model, read once. */
    @Test
    public void thePageReadsTheModelTheSimulatorCollides() {
        JsonObject json = field.json();
        for (String key : List.of("size", "wallHeight", "elements", "obstacles", "pieces", "tape")) {
            assertTrue(key, json.has(key));
        }
        assertEquals(field.size, json.get("size").getAsDouble(), 0);
        assertEquals(field.obstacles.size(), json.getAsJsonArray("obstacles").size());
    }
}
