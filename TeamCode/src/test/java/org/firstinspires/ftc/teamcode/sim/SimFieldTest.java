package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import org.junit.Test;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The field model is what the season's CAD says, reduced: the walls where the perimeter stands,
 * the elements as convex shapes, the game pieces, the tape, and what the robot runs into.
 */
public class SimFieldTest {
    private final SimField field = SimField.load();

    @Test
    public void theWallsStandWhereTheCadsPerimeterDoes() {
        assertTrue("a competition field is 141 in between the walls, give or take an inch: " + field.size,
                field.size > 140 && field.size < 142);
        assertTrue("the perimeter is a foot high, give or take: " + field.wallHeight,
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
        assertTrue("the field is set up with game pieces", field.json().getAsJsonArray("pieces").size() > 0);
    }

    @Test
    public void everyElementIsAClosedShapeOfItsOwnVertices() {
        for (SimField.Element element : field.elements) {
            assertTrue(element.name + " has a body", element.faces.length >= 4);
            for (int[] face : element.faces) {
                assertTrue(element.name + " has a face of " + face.length + " vertices", face.length >= 3);
                for (int index : face) {
                    assertTrue(element.name + " names vertex " + index + " of " + element.vertices.length,
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
            assertTrue("a mark is a strip of four corners", mark.getAsJsonArray("footprint").size() == 4);
        }
        assertEquals("red and blue", 2, colours.size());
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
