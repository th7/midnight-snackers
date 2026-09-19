package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.Test;

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
        assertTrue(
                "the hives are not elements: they hang from the frame and turn",
                groups.stream().noneMatch(group -> group.contains("Hive")));
        assertEquals("one hive each", 2, field.hives.size());
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
            assertTrue(element.name + " has a body", element.faces.length >= 4);
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
            assertTrue(obstacle.name + " is one part of one element", obstacle.name.matches(".+ / .+ <\\d+>"));
        }
        assertNotNull(field.obstacle("Flower Assembly <1> / Flower HIPS Pipe <1>"));
        assertNotNull(field.obstacle("Frame <1> / A-Frame Leg <1>"));
        for (SimField.Obstacle obstacle : field.obstacles) {
            assertTrue(obstacle.name + " hangs above the robot and is no obstacle", !obstacle.name.contains("Hive"));
        }
    }

    @Test
    public void everyObstacleSaysHowHighItStandsAndHowFarItClearsTheFloor() {
        double pollen = 2 * pollenRadius();
        for (SimField.Obstacle obstacle : field.obstacles) {
            assertTrue(
                    obstacle.name + " stands " + obstacle.stands + " above its underside at " + obstacle.clears,
                    obstacle.stands > obstacle.clears);
            assertTrue(
                    obstacle.name + " stands " + obstacle.stands + ", which is the floor's own lip",
                    obstacle.stands > 0.5);
            assertTrue(
                    obstacle.name + " clears the floor by " + obstacle.clears + ", which is above the robot",
                    obstacle.clears < 18);
        }
        for (SimField.Flower flower : field.flowers) {
            for (SimField.Obstacle obstacle : partsOf(flower.name)) {
                assertTrue(
                        obstacle.name + " overhangs, so a pollen rolls under it: " + obstacle.clears,
                        obstacle.clears > pollen);
            }
        }
        assertTrue(
                "the frame's feet stand on the floor, so a ball meets them",
                field.obstacle("Frame <1> / Sheet Metal Foot Bar <1>").clears < pollen);
    }

    @Test
    public void eachWallHasAFlowerWhoseBoreHoldsAStackOfPollen() {
        assertEquals("one flower at each wall", 4, field.flowers.size());
        double radius = pollenRadius();
        double half = field.size / 2;
        Set<String> walls = new HashSet<>();
        for (SimField.Flower flower : field.flowers) {
            assertTrue(flower.name, flower.name.startsWith("Flower Assembly"));
            assertTrue(flower.name + "'s bore is wider than a pollen: " + flower.bore, flower.bore > radius);
            assertTrue(
                    flower.name + "'s bore holds a pollen in: gap " + flower.gap + " vs " + 2 * radius,
                    flower.gap < 2 * radius);
            assertTrue(
                    flower.name + "'s lip is above the pollen standing on the floor: " + flower.lip,
                    flower.lip > 2 * radius);
            assertTrue(
                    flower.name + "'s nest is a ring a pollen can be rolled up over: " + flower.nest,
                    flower.nest > 0 && flower.nest < radius);
            assertTrue(
                    flower.name + " stands at a wall: " + flower.axis[0] + ", " + flower.axis[1],
                    Math.max(Math.abs(flower.axis[0]), Math.abs(flower.axis[1])) > half - 6);
            walls.add(
                    Math.abs(flower.axis[0]) > Math.abs(flower.axis[1])
                            ? (flower.axis[0] > 0 ? "+x" : "-x")
                            : (flower.axis[1] > 0 ? "+y" : "-y"));
            List<SimField.Obstacle> pipes = partsOf(flower.name);
            assertEquals(flower.name + " is four pipes", 4, pipes.size());
            for (SimField.Obstacle pipe : pipes) {
                assertTrue(flower.name + " is made of pipes: " + pipe.name, pipe.name.contains("Pipe"));
                assertEquals(flower.name + "'s pipes begin at its lip", flower.lip, pipe.clears, 0.01);
                for (double[] corner : pipe.footprint) {
                    assertTrue(
                            pipe.name + " stands clear of the bore",
                            Math.hypot(corner[0] - flower.axis[0], corner[1] - flower.axis[1]) >= flower.bore - 0.01);
                }
            }
        }
        assertEquals("one at each of the four walls", 4, walls.size());
        assertNotNull(field.flower("Flower Assembly <1>"));
        assertNull(field.flower("Flower Assembly <9>"));
    }

    @Test
    public void theCadStacksFourPollenInEachFlowerOneRestingOnAnother() {
        assertEquals("four in each of the four flowers", 16, field.flowerPieces.size());
        Map<String, List<SimField.Piece>> stacks = new HashMap<>();
        for (SimField.Piece piece : field.flowerPieces) {
            assertEquals(SimField.POLLEN, piece.kind);
            assertNull("in a flower, so in no cell", piece.cell);
            SimField.Flower flower = field.flower(piece.flower);
            assertNotNull(piece.flower, flower);
            assertTrue(
                    piece.name + " at " + piece.x + ", " + piece.y + " stands in " + flower.name + "'s bore",
                    flower.standsIn(piece.x, piece.y));
            stacks.computeIfAbsent(piece.flower, name -> new ArrayList<>()).add(piece);
        }
        assertEquals("one stack per flower", 4, stacks.size());
        for (Map.Entry<String, List<SimField.Piece>> stack : stacks.entrySet()) {
            List<SimField.Piece> pollen = new ArrayList<>(stack.getValue());
            pollen.sort((a, b) -> Double.compare(a.z, b.z));
            assertEquals(stack.getKey() + " holds four", 4, pollen.size());
            SimField.Flower flower = field.flower(stack.getKey());
            double resting = pollen.get(0).radius;
            for (SimField.Piece piece : pollen) {
                assertEquals(
                        stack.getKey() + ": a pollen rests on what is under it, at " + piece.z, resting, piece.z, 0.15);
                assertEquals(
                        stack.getKey() + " stacks them on the bore's axis",
                        0,
                        Math.hypot(piece.x - flower.axis[0], piece.y - flower.axis[1]),
                        0.1);
                resting = piece.z + 2 * piece.radius;
            }
            assertTrue(
                    stack.getKey() + "'s bottom pollen stands wholly below the lip",
                    pollen.get(0).z + pollen.get(0).radius < flower.lip);
            assertTrue(
                    stack.getKey() + "'s next pollen up reaches the lip, so the bore holds it",
                    pollen.get(1).z + pollen.get(1).radius > flower.lip);
        }
    }

    private List<SimField.Obstacle> partsOf(String element) {
        List<SimField.Obstacle> parts = new ArrayList<>();
        for (SimField.Obstacle obstacle : field.obstacles) {
            if (obstacle.name.startsWith(element + " / ")) {
                parts.add(obstacle);
            }
        }
        return parts;
    }

    private double pollenRadius() {
        for (SimField.Piece piece : field.loosePieces) {
            if (SimField.POLLEN.equals(piece.kind)) {
                return piece.radius;
            }
        }
        throw new AssertionError("the field has no pollen");
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

    @Test
    public void eachAllianceHasAHiveOnTheFramesAxleLeaningItsOwnWay() {
        assertEquals(2, field.hives.size());
        SimField.Hive blue = field.hive("Blue Hive <1>");
        SimField.Hive red = field.hive("Red Hive <1>");
        assertNotNull(blue);
        assertNotNull(red);
        assertNull(field.hive("Green Hive"));
        assertEquals("Blue", blue.alliance);
        assertEquals("Red", red.alliance);
        for (SimField.Hive hive : field.hives) {
            assertEquals(hive.name + " turns over the middle of the field", 0, hive.pivot[0], 0.5);
            assertEquals(hive.name + " hangs from the frame's top bar", 44, hive.pivot[2], 1.5);
            assertEquals(hive.name + " leans 30 degrees", 30, Math.abs(hive.tilt), 1);
            assertEquals(hive.name + " has a cell at each end", 2, hive.cells.size());
            assertTrue(hive.name + " is drawn", hive.parts.size() > 0);
            for (SimField.Element part : hive.parts) {
                assertEquals(hive.name, part.group);
            }
        }
        assertEquals("the hives hang either side of the bar", blue.pivot[1], -red.pivot[1], 0.1);
        assertEquals("and lean opposite ways", blue.tilt, -red.tilt, 0.1);
        assertSameShape("one shape built twice", extentOf(blue), extentOf(red), 0.2);
    }

    @Test
    public void blueIsTheHiveAtNegativeYAndTheTraysStandOffTheYAxis() {
        SimField.Hive blue = field.hive("Blue Hive <1>");
        SimField.Hive red = field.hive("Red Hive <1>");
        assertTrue("the blue hive is the one at negative y: " + blue.pivot[1], blue.pivot[1] < 0);
        assertTrue("the red hive is the one at positive y: " + red.pivot[1], red.pivot[1] > 0);

        double nearest = Double.POSITIVE_INFINITY;
        double furthest = Double.NEGATIVE_INFINITY;
        for (SimField.Element element : field.elements) {
            if (element.name.contains("Artifact Tray")) {
                for (double[] vertex : element.vertices) {
                    nearest = Math.min(nearest, vertex[1]);
                    furthest = Math.max(furthest, vertex[1]);
                }
            }
        }
        assertTrue("a tray stands off each end of the y axis", nearest < -60 && furthest > 60);
    }

    @Test
    public void eachHiveCarriesItsTwoGoalAprilTagsInItsOwnFrame() {
        for (SimField.Hive hive : field.hives) {
            Set<String> sides = new HashSet<>();
            for (SimField.Element part : hive.parts) {
                if (part.name.contains("April Tag")) {
                    assertTrue(part.name + " is the hive's alliance's", part.name.startsWith(hive.alliance));
                    assertTrue(part.name + " stands somewhere", part.vertices.length > 0);
                    sides.add(part.name.contains("(Scoring)") ? "Scoring" : "Audience");
                }
            }
            assertEquals(hive.name + " carries a goal tag at each end: " + sides, Set.of("Audience", "Scoring"), sides);
        }
    }

    @Test
    public void everythingDrawnHasAColour() {
        List<SimField.Element> drawn = new ArrayList<>(field.elements);
        for (SimField.Hive hive : field.hives) {
            drawn.addAll(hive.parts);
        }
        for (SimField.Element element : drawn) {
            assertNotNull(element.name + " has no colour", element.colour);
            assertTrue(element.name + " has colour " + element.colour, element.colour.matches("#[0-9a-fA-F]{6}"));
        }
    }

    @Test
    public void aCellIsAMouthAWallAllRoundAndABack() {
        assertEquals(4, field.cells.size());
        for (SimField.Cell cell : field.cells) {
            assertTrue(cell.name, cell.name.startsWith(cell.alliance + " Cell"));
            assertTrue(cell.name, cell.name.contains("(" + cell.side + ")"));
            assertEquals(cell.name + " belongs to its hive", cell.alliance, cell.hive.alliance);
            assertTrue(cell.name + " is one of its hive's cells", cell.hive.cells.contains(cell));
            assertTrue(cell.name + "'s mouth has corners", cell.mouth.length >= 4);
            assertFlatRing(cell.name + "'s mouth", cell.mouth);
            assertFlatRing(cell.name + "'s back", cell.back);
            assertEquals(cell.name + " is the same ring at both ends", cell.mouth.length, cell.back.length);
            assertEquals(cell.name + " has a wall between every pair of corners", cell.mouth.length, cell.walls.size());
            assertEquals(cell.name + "'s panels are its walls and its back", cell.walls.size() + 1, cell.panels.size());
            double[] mouth = centreOf(cell.mouth), back = centreOf(cell.back);
            assertEquals(cell.name + " is 12 inches deep", 12, distance(mouth, back), 1);
            double[] size = extentOf(List.<double[][]>of(cell.mouth));
            assertEquals(cell.name + " is 20 inches across", 20, size[1], 1.5);
            assertEquals(cell.name + " is 14 inches high", 14, size[2], 1.5);
        }
        assertNotNull(field.cell("Blue Cell (Audience) <1>"));
        assertNull(field.cell("Green Cell"));
    }

    @Test
    public void everyCellIsClosedButForItsMouth() {
        for (SimField.Cell cell : field.cells) {
            List<double[][]> rings = new ArrayList<>(cell.panels);
            rings.add(cell.mouth);
            Map<String, Integer> edges = new HashMap<>();
            for (double[][] ring : rings) {
                for (int i = 0; i < ring.length; i++) {
                    edges.merge(edge(ring[i], ring[(i + 1) % ring.length]), 1, Integer::sum);
                }
            }
            for (Map.Entry<String, Integer> entry : edges.entrySet()) {
                assertEquals(cell.name + " has a gap at " + entry.getKey(), 2, (int) entry.getValue());
            }
        }
    }

    @Test
    public void oneCellOfEachHiveIsUpturnedAndTheOtherDownturned() {
        for (SimField.Hive hive : field.hives) {
            int upturned = 0;
            for (SimField.Cell cell : hive.cells) {
                double[] mouth = cell.mouthCentreAt(hive.tilt);
                double[] normal = cell.mouthNormalAt(hive.tilt);
                double[] back = hive.at(hive.tilt, centreOf(cell.back));
                assertEquals(cell.name + "'s mouth faces along x", 0, normal[1], 0.05);
                assertEquals("a unit normal", 1, length(normal), 1e-6);
                if (cell.upturnedAt(hive.tilt)) {
                    upturned++;
                    assertTrue(cell.name + "'s mouth is above its back", mouth[2] > back[2]);
                    assertTrue(cell.name + "'s mouth faces up: " + normal[2], normal[2] > 0.3);
                } else {
                    assertTrue(cell.name + "'s mouth is below its back", mouth[2] < back[2]);
                    assertTrue(cell.name + "'s mouth faces down: " + normal[2], normal[2] < -0.3);
                }
                assertTrue(
                        cell.name + " turns over when the hive tips",
                        cell.upturnedAt(hive.tilt) != cell.upturnedAt(-hive.tilt));
            }
            assertEquals(hive.name + " holds at one end at a time", 1, upturned);
        }
    }

    @Test
    public void theNectarTheFieldIsSetUpWithRestsInEachHivesUpturnedCell() {
        assertEquals("three in each hive", 6, field.cellPieces.size());
        Set<String> cells = new HashSet<>();
        for (SimField.Piece piece : field.cellPieces) {
            assertEquals(SimField.NECTAR, piece.kind);
            SimField.Cell cell = field.cell(piece.cell);
            assertNotNull(piece.cell, cell);
            assertEquals("a hive is set up with its own alliance's nectar", cell.alliance, piece.alliance);
            assertTrue(cell.name + " holds what is in it", cell.upturnedAt(cell.hive.tilt));
            assertTrue(
                    piece.name + " at " + piece.x + ", " + piece.y + ", " + piece.z + " is in " + cell.name,
                    holds(cell, cell.hive.tilt, new double[] {piece.x, piece.y, piece.z}));
            cells.add(cell.name);
        }
        assertEquals("the upturned cell of each hive", 2, cells.size());
    }

    @Test
    public void theHivesHangFromTheBarBetweenThem() {
        Set<String> names = new HashSet<>();
        for (SimField.Element element : field.elements) {
            names.add(element.group + " / " + element.name);
        }
        assertTrue(names.toString(), names.contains("Frame <1> / A-Frame Top Bar"));
        for (SimField.Hive hive : field.hives) {
            Set<String> parts = new HashSet<>();
            for (SimField.Element part : hive.parts) {
                parts.add(part.name);
            }
            assertTrue(hive.name + " " + parts, parts.contains("Goal Pivot Bracket"));
        }
    }

    @Test
    public void thePollenOnTheOpenFloorIsLoose() {
        assertEquals("two rows of four in the corners", 8, field.loosePieces.size());
        double half = field.size / 2;
        for (SimField.Piece piece : field.loosePieces) {
            assertEquals("Pollen", piece.name);
            assertEquals(SimField.POLLEN, piece.kind);
            assertNull("loose, so in no cell", piece.cell);
            assertNull("loose, so in no flower", piece.flower);
            assertTrue("pollen is smaller than nectar", piece.radius < 1.6);
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

        for (int i = 0; i < pieces.size(); i++) {
            JsonObject piece = pieces.get(i).getAsJsonObject();
            JsonArray centre = piece.getAsJsonArray("centre");
            boolean modelled = piece.get("loose").getAsBoolean() || piece.has("cell") || piece.has("flower");
            boolean insideTheWalls = Math.abs(centre.get(0).getAsDouble()) < half
                    && Math.abs(centre.get(1).getAsDouble()) < half;
            assertTrue(
                    piece.get("name").getAsString() + " at " + centre + " is inside the walls and in no model",
                    modelled || !insideTheWalls);
        }
    }

    private static double[] centreOf(double[][] ring) {
        double[] sum = new double[3];
        for (double[] v : ring) {
            for (int axis = 0; axis < 3; axis++) {
                sum[axis] += v[axis] / ring.length;
            }
        }
        return sum;
    }

    private static double[] extentOf(List<double[][]> rings) {
        double[] min = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        double[] max = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (double[][] ring : rings) {
            for (double[] v : ring) {
                for (int axis = 0; axis < 3; axis++) {
                    min[axis] = Math.min(min[axis], v[axis]);
                    max[axis] = Math.max(max[axis], v[axis]);
                }
            }
        }
        return new double[] {max[0] - min[0], max[1] - min[1], max[2] - min[2]};
    }

    private static double[] extentOf(SimField.Hive hive) {
        List<Double> out = new ArrayList<>();
        for (String side : List.of("Audience", "Scoring")) {
            for (SimField.Cell cell : hive.cells) {
                if (!cell.side.equals(side)) {
                    continue;
                }
                for (double[] point : List.of(centreOf(cell.mouth), centreOf(cell.back))) {
                    out.add(point[0]);
                    out.add(Math.abs(point[1]));
                    out.add(point[2]);
                }
                for (double size : extentOf(List.<double[][]>of(cell.mouth))) {
                    out.add(size);
                }
            }
        }
        double[] array = new double[out.size()];
        for (int i = 0; i < array.length; i++) {
            array[i] = out.get(i);
        }
        return array;
    }

    private static void assertSameShape(String message, double[] expected, double[] actual, double delta) {
        assertEquals(message + ": measures", expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertEquals(message + " at " + i, expected[i], actual[i], delta);
        }
    }

    private static void assertFlatRing(String name, double[][] ring) {
        double[] n = SimField.normal(ring);
        for (double[] p : ring) {
            double off = (p[0] - ring[0][0]) * n[0] + (p[1] - ring[0][1]) * n[1] + (p[2] - ring[0][2]) * n[2];
            assertEquals(name + " is flat", 0, off, 0.05);
        }
    }

    private static double distance(double[] a, double[] b) {
        return Math.sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
    }

    private static double length(double[] v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    private static String edge(double[] a, double[] b) {
        String one = String.format("%.2f,%.2f,%.2f", a[0], a[1], a[2]);
        String other = String.format("%.2f,%.2f,%.2f", b[0], b[1], b[2]);
        return one.compareTo(other) < 0 ? one + " - " + other : other + " - " + one;
    }

    private static boolean holds(SimField.Cell cell, double tilt, double[] point) {
        double[] inside = cell.centreAt(tilt);
        List<double[][]> rings = new ArrayList<>(cell.panelsAt(tilt));
        rings.add(cell.mouthAt(tilt));
        for (double[][] ring : rings) {
            double[] n = SimField.normal(ring);
            double toInside = 0, toPoint = 0;
            for (int axis = 0; axis < 3; axis++) {
                toInside += (inside[axis] - ring[0][axis]) * n[axis];
                toPoint += (point[axis] - ring[0][axis]) * n[axis];
            }
            if (Math.signum(toInside) != Math.signum(toPoint)) {
                return false;
            }
        }
        return true;
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

    @Test
    public void theOrderATickListsTheBallsInIsServedRatherThanWorkedOutAgainByWhoeverReadsIt() {
        JsonArray moved = field.json().getAsJsonArray("moved");

        assertEquals(field.movedPieces.size(), moved.size());
        for (int i = 0; i < moved.size(); i++) {
            assertEquals(
                    field.movedPieces.get(i).name,
                    moved.get(i).getAsJsonObject().get("name").getAsString());
        }
        assertEquals(field.loosePieces.size() + field.cellPieces.size() + field.flowerPieces.size(), moved.size());
        for (int i = 0; i < field.loosePieces.size(); i++) {
            assertEquals(field.loosePieces.get(i).name, field.movedPieces.get(i).name);
        }
    }

    @Test
    public void thePageReadsTheModelTheSimulatorCollides() {
        JsonObject json = field.json();
        for (String key :
                List.of("size", "wallHeight", "elements", "obstacles", "flowers", "pieces", "tape", "moved")) {
            assertTrue(key, json.has(key));
        }
        assertEquals(field.size, json.get("size").getAsDouble(), 0);
        assertEquals(field.obstacles.size(), json.getAsJsonArray("obstacles").size());
    }
}
