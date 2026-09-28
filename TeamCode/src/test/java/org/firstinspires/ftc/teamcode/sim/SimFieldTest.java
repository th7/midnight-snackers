package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Ring;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.junit.Test;

public class SimFieldTest {
    private final SimField.Loaded loaded = SimField.load();
    private final Field field = loaded.field();

    @Test
    public void theWallsStandWhereTheCadsPerimeterDoes() {
        assertTrue(
                "a competition field is 141 in between the walls, give or take an inch: " + field.size(),
                field.size() > 140 && field.size() < 142);
        assertTrue(
                "the perimeter is a foot high, give or take: " + field.wallHeight(),
                field.wallHeight() > 11 && field.wallHeight() < 13);
    }

    @Test
    public void theFieldHasThisSeasonsElements() {
        Set<String> groups = new HashSet<>();
        for (Field.Element element : field.elements()) {
            groups.add(element.group());
        }
        assertTrue(
                "the hives are not elements: they hang from the frame and turn",
                groups.stream().noneMatch(group -> group.contains("Hive")));
        assertEquals("one hive each", 2, field.hives().size());
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
                loaded.page().getAsJsonArray("pieces").size() > 0);
    }

    @Test
    public void everyElementIsAClosedShapeOfItsOwnVertices() {
        for (Field.Element element : field.elements()) {
            assertTrue(element.name() + " has a body", element.faces().size() >= 4);
            for (Ring<Vec3> face : element.faces()) {
                assertTrue(element.name() + " has a face of " + face.size() + " vertices", face.size() >= 3);
                for (Vec3 corner : face.all()) {
                    assertTrue(
                            element.name() + " names its own vertex " + corner,
                            element.vertices().contains(corner));
                }
            }
        }
    }

    @Test
    public void theObstaclesAreConvexPolygonsInsideTheWalls() {
        assertTrue(field.obstacles().size() >= 4);
        double half = field.size() / 2;
        for (Field.Obstacle obstacle : field.obstacles()) {
            double[][] ring = Points.footprint(obstacle);
            assertTrue(obstacle.name() + " has " + ring.length + " corners", ring.length >= 3);
            for (int i = 0; i < ring.length; i++) {
                double[] a = ring[i], b = ring[(i + 1) % ring.length], c = ring[(i + 2) % ring.length];
                double turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
                assertTrue(obstacle.name() + " turns left at every corner", turn > 0);
                assertTrue(
                        obstacle.name() + " stands inside the walls", Math.abs(a[0]) < half && Math.abs(a[1]) < half);
            }
            assertTrue(
                    obstacle.name() + " is one part of one element",
                    obstacle.name().matches(".+ / .+ <\\d+>"));
        }
        assertNotNull(field.obstacle("Flower Assembly <1> / Flower HIPS Pipe <1>"));
        assertNotNull(field.obstacle("Frame <1> / A-Frame Leg <1>"));
        for (Field.Obstacle obstacle : field.obstacles()) {
            assertTrue(
                    obstacle.name() + " hangs above the robot and is no obstacle",
                    !obstacle.name().contains("Hive"));
        }
    }

    @Test
    public void everyObstacleSaysHowHighItStandsAndHowFarItClearsTheFloor() {
        double pollen = 2 * pollenRadius();
        for (Field.Obstacle obstacle : field.obstacles()) {
            assertTrue(
                    obstacle.name() + " stands " + obstacle.stands() + " above its underside at " + obstacle.clears(),
                    obstacle.stands() > obstacle.clears());
            assertTrue(
                    obstacle.name() + " stands " + obstacle.stands() + ", which is the floor's own lip",
                    obstacle.stands() > 0.5);
            assertTrue(
                    obstacle.name() + " clears the floor by " + obstacle.clears() + ", which is above the robot",
                    obstacle.clears() < 18);
        }
        for (Field.Flower flower : field.flowers()) {
            for (Field.Obstacle obstacle : partsOf(flower.name())) {
                assertTrue(
                        obstacle.name() + " overhangs, so a pollen rolls under it: " + obstacle.clears(),
                        obstacle.clears() > pollen);
            }
        }
        assertTrue(
                "the frame's feet stand on the floor, so a ball meets them",
                field.obstacle("Frame <1> / Sheet Metal Foot Bar <1>")
                                .orElseThrow()
                                .clears()
                        < pollen);
    }

    @Test
    public void eachWallHasAFlowerWhoseBoreHoldsAStackOfPollen() {
        assertEquals("one flower at each wall", 4, field.flowers().size());
        double radius = pollenRadius();
        double half = field.size() / 2;
        Set<String> walls = new HashSet<>();
        for (Field.Flower flower : field.flowers()) {
            assertTrue(flower.name(), flower.name().startsWith("Flower Assembly"));
            assertTrue(flower.name() + "'s bore is wider than a pollen: " + flower.bore(), flower.bore() > radius);
            assertTrue(
                    flower.name() + "'s bore holds a pollen in: gap " + flower.gap() + " vs " + 2 * radius,
                    flower.gap() < 2 * radius);
            assertTrue(
                    flower.name() + "'s lip is above the pollen standing on the floor: " + flower.lip(),
                    flower.lip() > 2 * radius);
            assertTrue(
                    flower.name() + "'s nest is a ring a pollen can be rolled up over: " + flower.nest(),
                    flower.nest() > 0 && flower.nest() < radius);
            assertTrue(
                    flower.name() + " stands at a wall: " + flower.axis().x() + ", "
                            + flower.axis().y(),
                    Math.max(Math.abs(flower.axis().x()), Math.abs(flower.axis().y())) > half - 6);
            walls.add(
                    Math.abs(flower.axis().x()) > Math.abs(flower.axis().y())
                            ? (flower.axis().x() > 0 ? "+x" : "-x")
                            : (flower.axis().y() > 0 ? "+y" : "-y"));
            List<Field.Obstacle> pipes = partsOf(flower.name());
            assertEquals(flower.name() + " is four pipes", 4, pipes.size());
            for (Field.Obstacle pipe : pipes) {
                assertTrue(
                        flower.name() + " is made of pipes: " + pipe.name(),
                        pipe.name().contains("Pipe"));
                assertEquals(flower.name() + "'s pipes begin at its lip", flower.lip(), pipe.clears(), 0.01);
                for (double[] corner : Points.footprint(pipe)) {
                    assertTrue(
                            pipe.name() + " stands clear of the bore",
                            Math.hypot(
                                            corner[0] - flower.axis().x(),
                                            corner[1] - flower.axis().y())
                                    >= flower.bore() - 0.01);
                }
            }
        }
        assertEquals("one at each of the four walls", 4, walls.size());
        assertTrue(field.flower("Flower Assembly <1>").isPresent());
        assertTrue(field.flower("Flower Assembly <9>").isEmpty());
    }

    @Test
    public void theCadStacksFourPollenInEachFlowerOneRestingOnAnother() {
        assertEquals(
                "four in each of the four flowers", 16, field.flowerPieces().size());
        Map<String, List<Field.Piece>> stacks = new HashMap<>();
        for (Field.Piece piece : field.flowerPieces()) {
            assertEquals(Field.Kind.POLLEN, piece.kind());
            Field.Flower flower = ((Field.Place.InFlower) piece.place()).flower();
            assertTrue(
                    piece.name() + " at " + piece.at().x() + ", " + piece.at().y() + " stands in " + flower.name()
                            + "'s bore",
                    flower.standsIn(piece.at().x(), piece.at().y()));
            stacks.computeIfAbsent(flower.name(), name -> new ArrayList<>()).add(piece);
        }
        assertEquals("one stack per flower", 4, stacks.size());
        for (Map.Entry<String, List<Field.Piece>> stack : stacks.entrySet()) {
            List<Field.Piece> pollen = new ArrayList<>(stack.getValue());
            pollen.sort((a, b) -> Double.compare(a.at().z(), b.at().z()));
            assertEquals(stack.getKey() + " holds four", 4, pollen.size());
            Field.Flower flower = field.flower(stack.getKey()).orElseThrow();
            double resting = pollen.get(0).radius();
            for (Field.Piece piece : pollen) {
                assertEquals(
                        stack.getKey() + ": a pollen rests on what is under it, at "
                                + piece.at().z(),
                        resting,
                        piece.at().z(),
                        0.15);
                assertEquals(
                        stack.getKey() + " stacks them on the bore's axis",
                        0,
                        Math.hypot(
                                piece.at().x() - flower.axis().x(),
                                piece.at().y() - flower.axis().y()),
                        0.1);
                resting = piece.at().z() + 2 * piece.radius();
            }
            assertTrue(
                    stack.getKey() + "'s bottom pollen stands wholly below the lip",
                    pollen.get(0).at().z() + pollen.get(0).radius() < flower.lip());
            assertTrue(
                    stack.getKey() + "'s next pollen up reaches the lip, so the bore holds it",
                    pollen.get(1).at().z() + pollen.get(1).radius() > flower.lip());
        }
    }

    private List<Field.Obstacle> partsOf(String element) {
        List<Field.Obstacle> parts = new ArrayList<>();
        for (Field.Obstacle obstacle : field.obstacles()) {
            if (obstacle.name().startsWith(element + " / ")) {
                parts.add(obstacle);
            }
        }
        return parts;
    }

    private double pollenRadius() {
        for (Field.Piece piece : field.loosePieces()) {
            if (Field.Kind.POLLEN.equals(piece.kind())) {
                return piece.radius();
            }
        }
        throw new AssertionError("the field has no pollen");
    }

    @Test
    public void theTapeMarksBothAlliancesOnTheFloor() {
        JsonArray tape = loaded.page().getAsJsonArray("tape");
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
    public void eachAllianceStandsOutsideTheWallOnItsOwnSideAcrossTheFieldFromTheOther() {
        Field.AllianceArea blue = Valid.value(field.allianceArea("Blue"));
        Field.AllianceArea red = Valid.value(field.allianceArea("Red"));
        double wall = field.size() / 2;

        assertEquals("Blue", blue.alliance());
        assertTrue("the blue drivers stand behind the -y wall, where the blue tape is: " + blue, blue.maxY() < -wall);
        assertTrue("the red drivers stand behind the +y wall, where the red tape is: " + red, red.minY() > wall);
        assertEquals("the one is the other seen in a mirror", -blue.maxY(), red.minY(), 0.01);
        assertEquals(-blue.minY(), red.maxY(), 0.01);
        assertEquals(blue.minX(), red.minX(), 0.01);
        assertEquals(blue.maxX(), red.maxX(), 0.01);
        assertTrue("an area is somewhere to stand, not a strip of tape: " + blue, blue.maxX() - blue.minX() > 48);
        assertTrue(blue.toString(), blue.maxY() - blue.minY() > 24);

        for (JsonObject piece : trayPieces()) {
            String name = piece.get("name").getAsString();
            JsonArray centre = piece.getAsJsonArray("centre");
            double x = centre.get(0).getAsDouble(), y = centre.get(1).getAsDouble();
            Field.AllianceArea own = name.startsWith("Blue") ? blue : red;
            assertTrue(
                    name + " at " + x + ", " + y + " waits in its tray, which is in " + own,
                    x > own.minX() && x < own.maxX() && y > own.minY() && y < own.maxY());
        }
    }

    @Test
    public void aDriverLooksFromTheMiddleOfTheirAreaFiveFeetUpTowardTheMiddleOfTheField() {
        Field.AllianceArea blue = Valid.value(field.allianceArea("Blue"));

        assertEquals(new Vec3((blue.minX() + blue.maxX()) / 2, (blue.minY() + blue.maxY()) / 2, 60), blue.eye());
        assertEquals(Vec3.zero(), blue.lookingAt());
        assertArrayEquals(
                "red's eye is blue's in the mirror",
                new double[] {blue.eye().x(), -blue.eye().y(), blue.eye().z()},
                Points.array(Valid.value(field.allianceArea("Red")).eye()),
                0.01);
    }

    @Test
    public void anAllianceTheFieldHasNoAreaForIsRefusedByName() {
        String refused = field.allianceArea("Green").fold(area -> "stands in " + area, rule -> rule);

        assertTrue(refused, refused.contains("nowhere for Green"));
    }

    /** The pieces that wait outside the walls for a human player: in a tray, and an alliance's own. */
    private List<JsonObject> trayPieces() {
        List<JsonObject> waiting = new ArrayList<>();
        JsonArray pieces = loaded.page().getAsJsonArray("pieces");
        for (int i = 0; i < pieces.size(); i++) {
            JsonObject piece = pieces.get(i).getAsJsonObject();
            String name = piece.get("name").getAsString();
            boolean outside = Math.abs(piece.getAsJsonArray("centre").get(1).getAsDouble()) > field.size() / 2;
            if (outside && (name.startsWith("Blue") || name.startsWith("Red"))) {
                waiting.add(piece);
            }
        }
        assertTrue("each alliance's tray holds its nectar", waiting.size() >= 2);
        return waiting;
    }

    @Test
    public void eachAllianceHasAHiveOnTheFramesAxleLeaningItsOwnWay() {
        assertEquals(2, field.hives().size());
        Field.Hive blue = field.hive("Blue Hive <1>").orElseThrow();
        Field.Hive red = field.hive("Red Hive <1>").orElseThrow();
        assertTrue(field.hive("Green Hive").isEmpty());
        assertEquals("Blue", blue.alliance());
        assertEquals("Red", red.alliance());
        for (Field.Hive hive : field.hives()) {
            assertEquals(
                    hive.name() + " turns over the middle of the field",
                    0,
                    hive.pivot().x(),
                    0.5);
            assertEquals(
                    hive.name() + " hangs from the frame's top bar",
                    44,
                    hive.pivot().z(),
                    1.5);
            assertEquals(
                    hive.name() + " leans 30 degrees", 30, Math.abs(hive.tilt().degrees()), 1);
            assertEquals(
                    hive.name() + " has a cell at each end", 2, hive.cells().size());
            assertTrue(hive.name() + " is drawn", hive.parts().size() > 0);
            for (Field.Element part : hive.parts()) {
                assertEquals(hive.name(), part.group());
            }
        }
        assertEquals(
                "the hives hang either side of the bar",
                blue.pivot().y(),
                -red.pivot().y(),
                0.1);
        assertEquals(
                "and lean opposite ways", blue.tilt().degrees(), -red.tilt().degrees(), 0.1);
        assertSameShape("one shape built twice", extentOf(blue), extentOf(red), 0.2);
    }

    @Test
    public void blueIsTheHiveAtNegativeYAndTheTraysStandOffTheYAxis() {
        Field.Hive blue = field.hive("Blue Hive <1>").orElseThrow();
        Field.Hive red = field.hive("Red Hive <1>").orElseThrow();
        assertTrue(
                "the blue hive is the one at negative y: " + blue.pivot().y(),
                blue.pivot().y() < 0);
        assertTrue(
                "the red hive is the one at positive y: " + red.pivot().y(),
                red.pivot().y() > 0);

        double nearest = Double.POSITIVE_INFINITY;
        double furthest = Double.NEGATIVE_INFINITY;
        for (Field.Element element : field.elements()) {
            if (element.name().contains("Artifact Tray")) {
                for (Vec3 vertex : element.vertices()) {
                    nearest = Math.min(nearest, vertex.y());
                    furthest = Math.max(furthest, vertex.y());
                }
            }
        }
        assertTrue("a tray stands off each end of the y axis", nearest < -60 && furthest > 60);
    }

    @Test
    public void eachHiveCarriesItsTwoGoalAprilTagsInItsOwnFrame() {
        for (Field.Hive hive : field.hives()) {
            Set<String> sides = new HashSet<>();
            for (Field.Element part : hive.parts()) {
                if (part.name().contains("April Tag")) {
                    assertTrue(
                            part.name() + " is the hive's alliance's",
                            part.name().startsWith(hive.alliance()));
                    assertTrue(
                            part.name() + " stands somewhere", !part.vertices().isEmpty());
                    sides.add(part.name().contains("(Scoring)") ? "Scoring" : "Audience");
                }
            }
            assertEquals(
                    hive.name() + " carries a goal tag at each end: " + sides, Set.of("Audience", "Scoring"), sides);
        }
    }

    @Test
    public void everythingDrawnHasAColour() {
        List<Field.Element> drawn = new ArrayList<>(field.elements());
        for (Field.Hive hive : field.hives()) {
            drawn.addAll(hive.parts());
        }
        for (Field.Element element : drawn) {
            assertNotNull(element.name() + " has no colour", element.colour());
            assertTrue(
                    element.name() + " has colour " + element.colour(),
                    element.colour().matches("#[0-9a-fA-F]{6}"));
        }
    }

    @Test
    public void aCellIsAMouthAWallAllRoundAndABack() {
        assertEquals(4, field.cells().size());
        for (Field.Cell cell : field.cells()) {
            assertTrue(cell.name(), cell.name().startsWith(cell.alliance() + " Cell"));
            assertTrue(cell.name(), cell.name().contains("(" + cell.side() + ")"));
            assertEquals(
                    cell.name() + " belongs to its hive",
                    cell.alliance(),
                    cell.hive().alliance());
            assertTrue(
                    cell.name() + " is one of its hive's cells",
                    cell.hive().cells().contains(cell));
            assertTrue(cell.name() + "'s mouth has corners", Points.arrays(cell.mouth()).length >= 4);
            assertFlatRing(cell.name() + "'s mouth", Points.arrays(cell.mouth()));
            assertFlatRing(cell.name() + "'s back", Points.arrays(cell.back()));
            assertEquals(
                    cell.name() + " is the same ring at both ends",
                    Points.arrays(cell.mouth()).length,
                    Points.arrays(cell.back()).length);
            assertEquals(
                    cell.name() + " has a wall between every pair of corners",
                    Points.arrays(cell.mouth()).length,
                    cell.walls().size());
            assertEquals(
                    cell.name() + "'s panels are its walls and its back",
                    cell.walls().size() + 1,
                    cell.panels().size());
            double[] mouth = centreOf(Points.arrays(cell.mouth())), back = centreOf(Points.arrays(cell.back()));
            assertEquals(cell.name() + " is 12 inches deep", 12, distance(mouth, back), 1);
            double[] size = extentOf(List.<double[][]>of(Points.arrays(cell.mouth())));
            assertEquals(cell.name() + " is 20 inches across", 20, size[1], 1.5);
            assertEquals(cell.name() + " is 14 inches high", 14, size[2], 1.5);
        }
        assertTrue(field.cell("Blue Cell (Audience) <1>").isPresent());
        assertTrue(field.cell("Green Cell").isEmpty());
    }

    @Test
    public void everyCellIsClosedButForItsMouth() {
        for (Field.Cell cell : field.cells()) {
            List<double[][]> rings = new ArrayList<>(Points.arrays(cell.panels()));
            rings.add(Points.arrays(cell.mouth()));
            Map<String, Integer> edges = new HashMap<>();
            for (double[][] ring : rings) {
                for (int i = 0; i < ring.length; i++) {
                    edges.merge(edge(ring[i], ring[(i + 1) % ring.length]), 1, Integer::sum);
                }
            }
            for (Map.Entry<String, Integer> entry : edges.entrySet()) {
                assertEquals(cell.name() + " has a gap at " + entry.getKey(), 2, (int) entry.getValue());
            }
        }
    }

    @Test
    public void oneCellOfEachHiveIsUpturnedAndTheOtherDownturned() {
        for (Field.Hive hive : field.hives()) {
            int upturned = 0;
            for (Field.Cell cell : hive.cells()) {
                double[] mouth = Points.array(cell.mouthCentreAt(hive.tilt().degrees()));
                double[] normal = Points.array(cell.mouthNormalAt(hive.tilt().degrees()));
                double[] back =
                        Points.array(hive.at(hive.tilt().degrees(), Points.vec(centreOf(Points.arrays(cell.back())))));
                assertEquals(cell.name() + "'s mouth faces along x", 0, normal[1], 0.05);
                assertEquals("a unit normal", 1, length(normal), 1e-6);
                if (cell.upturnedAt(hive.tilt().degrees())) {
                    upturned++;
                    assertTrue(cell.name() + "'s mouth is above its back", mouth[2] > back[2]);
                    assertTrue(cell.name() + "'s mouth faces up: " + normal[2], normal[2] > 0.3);
                } else {
                    assertTrue(cell.name() + "'s mouth is below its back", mouth[2] < back[2]);
                    assertTrue(cell.name() + "'s mouth faces down: " + normal[2], normal[2] < -0.3);
                }
                assertTrue(
                        cell.name() + " turns over when the hive tips",
                        cell.upturnedAt(hive.tilt().degrees())
                                != cell.upturnedAt(-hive.tilt().degrees()));
            }
            assertEquals(hive.name() + " holds at one end at a time", 1, upturned);
        }
    }

    @Test
    public void theNectarTheFieldIsSetUpWithRestsInEachHivesUpturnedCell() {
        assertEquals("three in each hive", 6, field.cellPieces().size());
        Set<String> cells = new HashSet<>();
        for (Field.Piece piece : field.cellPieces()) {
            assertEquals(Field.Kind.NECTAR, piece.kind());
            Field.Cell cell = ((Field.Place.InCell) piece.place()).cell();
            assertEquals(
                    "a hive is set up with its own alliance's nectar", Optional.of(cell.alliance()), piece.alliance());
            assertTrue(
                    cell.name() + " holds what is in it",
                    cell.upturnedAt(cell.hive().tilt().degrees()));
            assertTrue(
                    piece.name() + " at " + piece.at().x() + ", " + piece.at().y() + ", "
                            + piece.at().z() + " is in " + cell.name(),
                    holds(cell, cell.hive().tilt().degrees(), new double[] {
                        piece.at().x(), piece.at().y(), piece.at().z()
                    }));
            cells.add(cell.name());
        }
        assertEquals("the upturned cell of each hive", 2, cells.size());
    }

    @Test
    public void theHivesHangFromTheBarBetweenThem() {
        Set<String> names = new HashSet<>();
        for (Field.Element element : field.elements()) {
            names.add(element.group() + " / " + element.name());
        }
        assertTrue(names.toString(), names.contains("Frame <1> / A-Frame Top Bar"));
        for (Field.Hive hive : field.hives()) {
            Set<String> parts = new HashSet<>();
            for (Field.Element part : hive.parts()) {
                parts.add(part.name());
            }
            assertTrue(hive.name() + " " + parts, parts.contains("Goal Pivot Bracket"));
        }
    }

    @Test
    public void thePollenOnTheOpenFloorIsLoose() {
        assertEquals("two rows of four in the corners", 8, field.loosePieces().size());
        double half = field.size() / 2;
        for (Field.Piece piece : field.loosePieces()) {
            assertEquals("Pollen", piece.name());
            assertEquals(Field.Kind.POLLEN, piece.kind());
            assertEquals("loose, so in no cell and no flower", new Field.Place.Loose(), piece.place());
            assertTrue("pollen is smaller than nectar", piece.radius() < 1.6);
            assertEquals(
                    piece.name() + " rests on the floor",
                    piece.radius(),
                    piece.at().z(),
                    0.25);
            assertTrue(
                    piece.name() + " is inside the walls",
                    Math.abs(piece.at().x()) < half - piece.radius() / 2
                            && Math.abs(piece.at().y()) < half - piece.radius() / 2);
            for (Field.Obstacle obstacle : field.obstacles()) {
                assertTrue(
                        piece.name() + " is not held in " + obstacle.name(),
                        !inside(
                                Points.footprint(obstacle),
                                piece.at().x(),
                                piece.at().y()));
            }
        }
        JsonArray pieces = loaded.page().getAsJsonArray("pieces");
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

    private static double[] extentOf(Field.Hive hive) {
        List<Double> out = new ArrayList<>();
        for (String side : List.of("Audience", "Scoring")) {
            for (Field.Cell cell : hive.cells()) {
                if (!cell.side().equals(side)) {
                    continue;
                }
                for (double[] point :
                        List.of(centreOf(Points.arrays(cell.mouth())), centreOf(Points.arrays(cell.back())))) {
                    out.add(point[0]);
                    out.add(Math.abs(point[1]));
                    out.add(point[2]);
                }
                for (double size : extentOf(List.<double[][]>of(Points.arrays(cell.mouth())))) {
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
        double[] n = Points.normal(ring);
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

    private static boolean holds(Field.Cell cell, double tilt, double[] point) {
        double[] inside = Points.array(cell.centreAt(tilt));
        List<double[][]> rings = new ArrayList<>(Points.arrays(cell.panelsAt(tilt)));
        rings.add(Points.arrays(cell.mouthAt(tilt)));
        for (double[][] ring : rings) {
            double[] n = Points.normal(ring);
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

    @Test
    public void theOrderATickListsTheBallsInIsServedRatherThanWorkedOutAgainByWhoeverReadsIt() {
        JsonArray moved = loaded.page().getAsJsonArray("moved");

        assertEquals(field.movedPieces().size(), moved.size());
        for (int i = 0; i < moved.size(); i++) {
            assertEquals(
                    field.movedPieces().get(i).name(),
                    moved.get(i).getAsJsonObject().get("name").getAsString());
        }
        assertEquals(
                field.loosePieces().size()
                        + field.cellPieces().size()
                        + field.flowerPieces().size(),
                moved.size());
        for (int i = 0; i < field.loosePieces().size(); i++) {
            assertEquals(
                    field.loosePieces().get(i).name(),
                    field.movedPieces().get(i).name());
        }
    }

    @Test
    public void thePageReadsTheModelTheSimulatorCollides() {
        JsonObject json = loaded.page();
        for (String key :
                List.of("size", "wallHeight", "elements", "obstacles", "flowers", "pieces", "tape", "moved")) {
            assertTrue(key, json.has(key));
        }
        assertEquals(field.size(), json.get("size").getAsDouble(), 0);
        assertEquals(field.obstacles().size(), json.getAsJsonArray("obstacles").size());
    }
}
