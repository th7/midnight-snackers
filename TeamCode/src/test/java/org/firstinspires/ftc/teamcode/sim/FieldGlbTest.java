package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class FieldGlbTest {
    private static double[] triangleAt(double x, double y, double z, double size) {
        return new double[] {x, y, z, x + size, y, z, x, y + size, z};
    }

    private static Gltf.Part part(String name, double[] triangles) {
        return new Gltf.Part(name, List.of(name), triangles, "#b3b3b3");
    }

    @Test
    public void theAssemblysFrameBecomesTheFieldsInInches() {
        double[] metres = {0, 0, 0, Onshape.INCH_IN_METRES, 0, 0, 0, Onshape.INCH_IN_METRES, 0};

        double[] inches = FieldGlb.toField(metres);

        assertEquals("x comes from the assembly's y", 0.0, inches[0], 1e-9);
        assertEquals(0.0, inches[1], 1e-9);
        assertEquals("an inch of assembly y is an inch of field x", 0.0, inches[3], 1e-9);
        assertEquals("and turns a quarter", -1.0, inches[4], 1e-9);
        assertEquals(1.0, inches[6], 1e-9);
    }

    @Test
    public void pointsAreSnappedToTheGrid() {
        double[] snapped = FieldGlb.snap(new double[] {0.111, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0, 0.0}, null, 0.05)
                .triangles();

        assertEquals(9, snapped.length);
        assertEquals(0.1, snapped[0], 1e-9);
    }

    @Test
    public void aTriangleThatCollapsesOntoTheGridIsDropped() {
        double[] flat = triangleAt(0, 0, 0, 0.001);

        assertEquals(0, FieldGlb.snap(flat, null, 0.05).triangles().length);
    }

    @Test
    public void aTriangleWideEnoughToSurviveTheGridIsKept() {
        assertEquals(9, FieldGlb.snap(triangleAt(0, 0, 0, 10), null, 0.05).triangles().length);
    }

    @Test
    public void snappingRoundsHalvesTheWayPythonDoes() {
        double[] halves = FieldGlb.snap(new double[] {0.5, 2.5, 0, 10, 0, 0, 0, 10, 0}, null, 1.0)
                .triangles();

        assertEquals("a half goes to even, as Python's round() does, not up", 0.0, halves[0], 1e-9);
        assertEquals(2.0, halves[1], 1e-9);
    }

    @Test
    public void noGridMeansTheCadsOwnPointsAreKept() {
        double[] fine = {0.111, 0.222, 0.333, 10, 0, 0, 0, 10, 0};

        double[] kept = FieldGlb.snap(fine, null, FieldGlb.NO_GRID).triangles();

        assertEquals(0.111, kept[0], 1e-12);
        assertEquals(0.222, kept[1], 1e-12);
        assertEquals(0.333, kept[2], 1e-12);
    }

    @Test
    public void noGridStillDropsATriangleWithNoAreaAtAll() {
        double[] doubled = {1, 1, 1, 1, 1, 1, 2, 2, 2};

        assertEquals(0, FieldGlb.snap(doubled, null, FieldGlb.NO_GRID).triangles().length);
    }

    @Test
    public void theHardwareTheRulesDropIsDropped() {
        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(
                        part("am-1611 Socket Head Screw", triangleAt(0, 0, 0, 1)),
                        part("Blue Hive", triangleAt(0, 0, 0, 1))),
                FieldGlb.GRID_IN,
                FieldGlb.Keep.WHAT_WE_DRAW,
                FieldGlb.Shading.A_COLOUR_APIECE);

        assertEquals(1, kept.size());
        assertEquals("Blue Hive", kept.get(0).name);
    }

    @Test
    public void fullDetailKeepsTheHardwareTheFieldIsHeldTogetherWith() {
        List<Gltf.Part> both = List.of(
                part("am-1611 Socket Head Screw", triangleAt(0, 0, 0, 1)), part("Blue Hive", triangleAt(0, 0, 0, 1)));

        assertEquals(
                "the normal model is the field as it plays",
                1,
                FieldGlb.visualParts(
                                both, FieldGlb.GRID_IN, FieldGlb.Keep.WHAT_WE_DRAW, FieldGlb.Shading.A_COLOUR_APIECE)
                        .size());
        assertEquals(
                "full detail is the whole export, screws and all",
                2,
                FieldGlb.visualParts(
                                both, FieldGlb.NO_GRID, FieldGlb.Keep.EVERY_PART, FieldGlb.Shading.AS_THE_CAD_DREW_IT)
                        .size());
    }

    @Test
    public void fullDetailKeepsAPartTheNamesAboveItWouldHaveDropped() {
        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(new Gltf.Part("Bracket", List.of("Fastener Kit", "Bracket"), triangleAt(0, 0, 0, 1), null)),
                FieldGlb.NO_GRID,
                FieldGlb.Keep.EVERY_PART,
                FieldGlb.Shading.AS_THE_CAD_DREW_IT);

        assertEquals(1, kept.size());
    }

    @Test
    public void fullDetailStillKeepsNoTriangleWithNoArea() {
        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(part("Screw", new double[] {1, 1, 1, 1, 1, 1, 2, 2, 2})),
                FieldGlb.NO_GRID,
                FieldGlb.Keep.EVERY_PART,
                FieldGlb.Shading.AS_THE_CAD_DREW_IT);

        assertTrue("a part the export drew as nothing is not a part", kept.isEmpty());
    }

    @Test
    public void thePerimeterIsKeptThoughItIsNamedForWhatTheRulesDrop() {
        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(
                        part("FTC Rail with Rivet Holes", triangleAt(0, 0, 0, 1)),
                        part("Field Panel", triangleAt(0, 0, 0, 1)),
                        part("Side Glass", triangleAt(0, 0, 0, 1))),
                FieldGlb.GRID_IN,
                FieldGlb.Keep.WHAT_WE_DRAW,
                FieldGlb.Shading.A_COLOUR_APIECE);

        assertEquals("rail, rivet and side glass are all words the skip rule holds", 3, kept.size());
    }

    @Test
    public void aPartIsDroppedByTheNamesAboveItAsWellAsItsOwn() {
        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(new Gltf.Part("Bracket", List.of("Fastener Kit", "Bracket"), triangleAt(0, 0, 0, 1), null)),
                FieldGlb.GRID_IN,
                FieldGlb.Keep.WHAT_WE_DRAW,
                FieldGlb.Shading.A_COLOUR_APIECE);

        assertTrue(kept.toString(), kept.isEmpty());
    }

    @Test
    public void aPartThatSnapsAwayEntirelyIsNotKeptAsAnEmptyOne() {
        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(part("Tiny Thing", triangleAt(0, 0, 0, 0.0001))),
                FieldGlb.GRID_IN,
                FieldGlb.Keep.WHAT_WE_DRAW,
                FieldGlb.Shading.A_COLOUR_APIECE);

        assertTrue(kept.toString(), kept.isEmpty());
    }

    // The frame change turns the model a quarter and scales metres to inches. A normal is turned by
    // the same quarter and scaled by nothing: it says which way a face points, not where it is.
    @Test
    public void theFieldsFrameTurnsNormalsAsWellAsPoints() {
        double[] turned = FieldGlb.toFacing(new double[] {1, 0, 0, 0, 1, 0, 0, 0, 1});

        assertEquals("x turns into -y", 0.0, turned[0], 1e-9);
        assertEquals(-1.0, turned[1], 1e-9);
        assertEquals("y turns into x", 1.0, turned[3], 1e-9);
        assertEquals(0.0, turned[4], 1e-9);
        assertEquals("z stays z, and none of them grows by the inch", 1.0, turned[8], 1e-9);
    }

    @Test
    public void aTriangleDroppedBySnappingTakesItsNormalsWithIt() {
        double[] two = new double[18];
        System.arraycopy(triangleAt(0, 0, 0, 0.001), 0, two, 0, 9);
        System.arraycopy(triangleAt(0, 0, 0, 10), 0, two, 9, 9);
        double[] normals = new double[18];
        for (int i = 0; i < 9; i++) {
            normals[i] = i % 3 == 0 ? 1 : 0;
        }
        for (int i = 9; i < 18; i++) {
            normals[i] = i % 3 == 2 ? 1 : 0;
        }

        FieldGlb.Mesh kept = FieldGlb.snap(two, normals, FieldGlb.GRID_IN);

        assertEquals("the triangle that collapsed is gone", 9, kept.triangles().length);
        assertEquals("and so are its normals, not the survivor's", 9, kept.normals().length);
        assertEquals("what is left faces the way the survivor did", 1.0, kept.normals()[2], 1e-9);
    }

    @Test
    public void snappingWithoutNormalsStillHasNone() {
        FieldGlb.Mesh kept = FieldGlb.snap(triangleAt(0, 0, 0, 10), null, FieldGlb.GRID_IN);

        assertEquals(9, kept.triangles().length);
        assertNull("nothing invents a normal the export did not carry", kept.normals());
    }

    @Test
    public void theCheapModelGoesWithoutTheNormalsAndTheCadsMaterials() {
        double[] facing = {0, 0, 1, 0, 0, 1, 0, 0, 1};
        Gltf.Part part = new Gltf.Part(
                "Blue Hive", List.of("Blue Hive"), triangleAt(0, 0, 0, 1), facing, Gltf.colouredMaterial("#1651b0"));

        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(part), FieldGlb.GRID_IN, FieldGlb.Keep.WHAT_WE_DRAW, FieldGlb.Shading.A_COLOUR_APIECE);

        assertNull(
                "normals are a normal to a vertex, which is the model over again; the cheap one pays for "
                        + "neither them nor the CAD's own materials",
                kept.get(0).normals);
        assertEquals("#1651b0", kept.get(0).colour);
    }

    @Test
    public void aPartKeepsTheNormalsTheExportGaveIt() {
        double[] facing = {0, 0, 1, 0, 0, 1, 0, 0, 1};
        Gltf.Part part = new Gltf.Part("Blue Hive", List.of("Blue Hive"), triangleAt(0, 0, 0, 1), facing, null);

        List<Gltf.Part> kept = FieldGlb.visualParts(
                List.of(part), FieldGlb.NO_GRID, FieldGlb.Keep.EVERY_PART, FieldGlb.Shading.AS_THE_CAD_DREW_IT);

        assertEquals(1, kept.size());
        assertNotNull("the CAD said which way this faces; nothing here may forget it", kept.get(0).normals);
    }

    @Test
    public void theFieldTheWrongWayRoundIsRefused() {
        List<Gltf.Part> parts = new ArrayList<>();
        for (int i = 0; i < 12; i++) {
            parts.add(part("Blue Hive " + i, triangleAt(0, 60, 0, 10)));
            parts.add(part("Red Hive " + i, triangleAt(0, -60, 0, 10)));
        }

        List<String> off = FieldGlb.disagreements(parts, 141.17);

        assertFalse(off.isEmpty());
        assertTrue(off.toString(), off.get(0).contains("wrong way round"));
    }

    @Test
    public void tooFewPartsToJudgeSaysSoRatherThanPassing() {
        List<String> off = FieldGlb.disagreements(List.of(part("Blue Hive", triangleAt(0, -60, 0, 10))), 141.17);

        assertFalse(off.isEmpty());
        assertTrue(off.toString(), off.get(0).contains("could not judge"));
    }

    @Test
    public void aModelReadInMetresDoesNotReachTheWalls() {
        List<Gltf.Part> parts = new ArrayList<>();
        for (int i = 0; i < 12; i++) {
            parts.add(part("Blue Hive " + i, triangleAt(0, -1, 0, 0.1)));
            parts.add(part("Red Hive " + i, triangleAt(0, 1, 0, 0.1)));
        }

        List<String> off = FieldGlb.disagreements(parts, 141.17);

        assertFalse(off.isEmpty());
        assertTrue(off.toString(), off.toString().contains("frame or the units"));
    }

    @Test
    public void aFieldThatAgreesWithTheCollisionModelPassesQuietly() {
        List<Gltf.Part> parts = new ArrayList<>();
        for (int i = 0; i < 12; i++) {
            parts.add(part("Blue Hive " + i, triangleAt(0, -60, 0, 10)));
            parts.add(part("Red Hive " + i, triangleAt(0, 60, 0, 10)));
        }
        parts.add(part("Field Panel", triangleAt(-70, 0, 0, 10)));

        assertEquals(List.of(), FieldGlb.disagreements(parts, 141.17));
    }

    @Test
    public void buildingRefusesAnExportThatKeepsNothing() {
        byte[] export = Gltf.write(List.of(part("am-1611 Socket Head Screw", triangleAt(0, 0, 0, 10))));

        IllegalStateException refused = assertThrows(
                IllegalStateException.class,
                () -> FieldGlb.build(
                        export,
                        141.17,
                        FieldGlb.GRID_IN,
                        FieldGlb.Keep.WHAT_WE_DRAW,
                        FieldGlb.Shading.A_COLOUR_APIECE));

        assertTrue(refused.getMessage(), refused.getMessage().contains("nothing was kept"));
    }

    @Test
    public void buildingRefusesAFieldThatDisagreesWithTheCollisionModel() {
        List<Gltf.Part> parts = new ArrayList<>();
        for (int i = 0; i < 12; i++) {
            parts.add(part("Blue Hive " + i, inMetres(triangleAt(0, 60, 0, 10))));
            parts.add(part("Red Hive " + i, inMetres(triangleAt(0, -60, 0, 10))));
        }

        IllegalStateException refused = assertThrows(
                IllegalStateException.class,
                () -> FieldGlb.build(
                        Gltf.write(parts),
                        141.17,
                        FieldGlb.GRID_IN,
                        FieldGlb.Keep.WHAT_WE_DRAW,
                        FieldGlb.Shading.A_COLOUR_APIECE));

        assertTrue(refused.getMessage(), refused.getMessage().contains("disagree"));
    }

    private static double[] inMetres(double[] inches) {
        double[] out = new double[inches.length];
        for (int i = 0; i < inches.length; i += 3) {
            out[i] = -inches[i + 1] * Onshape.INCH_IN_METRES;
            out[i + 1] = inches[i] * Onshape.INCH_IN_METRES;
            out[i + 2] = inches[i + 2] * Onshape.INCH_IN_METRES;
        }
        return out;
    }
}
