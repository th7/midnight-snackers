package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.Test;

public class GltfTest {
    private static final int PARTS_IN_THE_FIELD = 305;
    private static final int TRIANGLES_IN_THE_FIELD = 315988;

    private static double[] triangle(double[]... points) {
        double[] out = new double[points.length * 3];
        for (int i = 0; i < points.length; i++) {
            System.arraycopy(points[i], 0, out, i * 3, 3);
        }
        return out;
    }

    private static final double[] ONE_TRIANGLE =
            triangle(new double[] {0, 0, 0}, new double[] {1, 0, 0}, new double[] {0, 1, 0});

    @Test
    public void aTriangleReadsBackAsItself() {
        List<Gltf.Part> parts =
                Gltf.read(Gltf.write(List.of(new Gltf.Part("Wedge", List.of("Wedge"), ONE_TRIANGLE, null))));

        assertEquals(1, parts.size());
        assertEquals("Wedge", parts.get(0).name);
        assertEquals(1, parts.get(0).count());
        assertArrayAlmostEquals(ONE_TRIANGLE, parts.get(0).triangles);
    }

    @Test
    public void bytesThatAreNotGlbOrJsonAreRefused() {
        Gltf.NotGltf refused = assertThrows(Gltf.NotGltf.class, () -> Gltf.read("nowhere near a model".getBytes()));

        assertTrue(refused.getMessage(), refused.getMessage().contains("not a glb"));
    }

    @Test
    public void aVersionWeDoNotReadIsRefused() {
        byte[] glb = Gltf.write(List.of(part(ONE_TRIANGLE)));
        ByteBuffer.wrap(glb).order(ByteOrder.LITTLE_ENDIAN).putInt(4, 3);

        Gltf.NotGltf refused = assertThrows(Gltf.NotGltf.class, () -> Gltf.read(glb));

        assertTrue(refused.getMessage(), refused.getMessage().contains("version 3"));
    }

    @Test
    public void aTruncatedContainerIsRefused() {
        byte[] glb = Gltf.write(List.of(part(ONE_TRIANGLE)));

        Gltf.NotGltf refused =
                assertThrows(Gltf.NotGltf.class, () -> Gltf.read(java.util.Arrays.copyOf(glb, glb.length - 40)));

        assertTrue(refused.getMessage(), refused.getMessage().contains("arrived"));
    }

    @Test
    public void tooShortToBeAGlbIsRefused() {
        Gltf.NotGltf refused = assertThrows(Gltf.NotGltf.class, () -> Gltf.read(new byte[] {'g', 'l', 'T', 'F'}));

        assertTrue(refused.getMessage(), refused.getMessage().contains("too short"));
    }

    @Test
    public void plainJsonGltfWithAnEmbeddedBufferReads() {
        byte[] glb = Gltf.write(List.of(part(ONE_TRIANGLE)));
        String json = jsonChunkOf(glb);
        byte[] binary = binaryChunkOf(glb);
        JsonObject document = new Gson().fromJson(json, JsonObject.class);
        document.getAsJsonArray("buffers")
                .get(0)
                .getAsJsonObject()
                .addProperty(
                        "uri",
                        "data:application/octet-stream;base64,"
                                + java.util.Base64.getEncoder().encodeToString(binary));

        List<Gltf.Part> parts = Gltf.read(new Gson().toJson(document).getBytes(StandardCharsets.UTF_8));

        assertEquals(1, parts.size());
        assertArrayAlmostEquals(ONE_TRIANGLE, parts.get(0).triangles);
    }

    @Test
    public void aBufferInASeparateFileIsRefusedNamingIt() {
        byte[] glb = Gltf.write(List.of(part(ONE_TRIANGLE)));
        JsonObject document = new Gson().fromJson(jsonChunkOf(glb), JsonObject.class);
        document.getAsJsonArray("buffers").get(0).getAsJsonObject().addProperty("uri", "field.bin");

        Gltf.NotGltf refused = assertThrows(
                Gltf.NotGltf.class, () -> Gltf.read(new Gson().toJson(document).getBytes(StandardCharsets.UTF_8)));

        assertTrue(refused.getMessage(), refused.getMessage().contains("field.bin"));
    }

    @Test
    public void aTranslationMovesTheTriangle() {
        JsonObject node = new JsonObject();
        JsonArray translation = new JsonArray();
        translation.add(10);
        translation.add(0);
        translation.add(0);
        node.add("translation", translation);

        double[] placed = Gltf.placement(node);

        assertEquals(10.0, placed[12], 1e-9);
        assertEquals(1.0, placed[0], 1e-9);
    }

    @Test
    public void aMatrixIsReadColumnMajor() {
        JsonObject node = new JsonObject();
        JsonArray matrix = new JsonArray();
        double[] given = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 7, 8, 9, 1};
        for (double value : given) {
            matrix.add(value);
        }
        node.add("matrix", matrix);

        double[] placed = Gltf.placement(node);

        assertEquals(7.0, placed[12], 1e-9);
        assertEquals(8.0, placed[13], 1e-9);
        assertEquals(9.0, placed[14], 1e-9);
    }

    @Test
    public void aMatrixThatIsNotSixteenNumbersIsRefused() {
        JsonObject node = new JsonObject();
        JsonArray matrix = new JsonArray();
        matrix.add(1);
        node.add("matrix", matrix);

        assertThrows(Gltf.NotGltf.class, () -> Gltf.placement(node));
    }

    @Test
    public void transformsComposeDownTheTree() {
        double[] outer = Gltf.fromTrs(new double[] {1, 0, 0}, new double[] {0, 0, 0, 1}, new double[] {2, 2, 2});
        double[] inner = Gltf.fromTrs(new double[] {0, 3, 0}, new double[] {0, 0, 0, 1}, new double[] {1, 1, 1});

        double[] both = Gltf.multiply(outer, inner);

        assertEquals("the outer scale reaches the inner translation", 1.0, both[12], 1e-9);
        assertEquals(6.0, both[13], 1e-9);
    }

    @Test
    public void aScaleAndARotationAreAppliedInTheOrderTheFormatSays() {
        double half = Math.sqrt(0.5);
        double[] placed = Gltf.fromTrs(new double[] {0, 0, 0}, new double[] {0, 0, half, half}, new double[] {2, 1, 1});

        assertEquals("x scaled then turned onto y", 0.0, placed[0], 1e-9);
        assertEquals(2.0, placed[1], 1e-9);
    }

    @Test
    public void aBaseColourBecomesAHexString() {
        Gltf.Part red = new Gltf.Part("Red", List.of("Red"), ONE_TRIANGLE, "#ff0000");

        List<Gltf.Part> back = Gltf.read(Gltf.write(List.of(red)));

        assertEquals("#ff0000", back.get(0).colour);
    }

    @Test
    public void aPartWithNoMaterialHasNoColour() {
        List<Gltf.Part> back = Gltf.read(Gltf.write(List.of(part(ONE_TRIANGLE))));

        assertNull(back.get(0).colour);
    }

    @Test
    public void twoPartsOfTheSameColourShareOneMaterial() {
        byte[] glb = Gltf.write(List.of(
                new Gltf.Part("A", List.of("A"), ONE_TRIANGLE, "#0000ff"),
                new Gltf.Part("B", List.of("B"), ONE_TRIANGLE, "#0000ff")));

        JsonObject document = new Gson().fromJson(jsonChunkOf(glb), JsonObject.class);

        assertEquals(1, document.getAsJsonArray("materials").size());
    }

    @Test
    public void partsKeepTheTreeTheirPathsDescribe() {
        byte[] glb = Gltf.write(List.of(
                new Gltf.Part("Leaf", List.of("Field", "Hive", "Leaf"), ONE_TRIANGLE, null),
                new Gltf.Part("Other", List.of("Field", "Hive", "Other"), ONE_TRIANGLE, null)));

        List<Gltf.Part> back = Gltf.read(glb);

        assertEquals(List.of("Field", "Hive", "Leaf"), back.get(0).path);
        assertEquals(List.of("Field", "Hive", "Other"), back.get(1).path);
    }

    @Test
    public void verticesSharedBetweenTrianglesAreWrittenOnce() {
        double[] two = new double[18];
        System.arraycopy(ONE_TRIANGLE, 0, two, 0, 9);
        System.arraycopy(ONE_TRIANGLE, 0, two, 9, 9);

        byte[] glb = Gltf.write(List.of(part(two)));
        JsonObject document = new Gson().fromJson(jsonChunkOf(glb), JsonObject.class);

        assertEquals(
                "three points for two triangles that share all of them",
                3,
                document.getAsJsonArray("accessors")
                        .get(0)
                        .getAsJsonObject()
                        .get("count")
                        .getAsInt());
        assertEquals(
                6,
                document.getAsJsonArray("accessors")
                        .get(1)
                        .getAsJsonObject()
                        .get("count")
                        .getAsInt());
    }

    @Test
    public void positionsCarryTheBoundsTheFormatRequires() {
        byte[] glb = Gltf.write(List.of(part(ONE_TRIANGLE)));
        JsonObject position = new Gson()
                .fromJson(jsonChunkOf(glb), JsonObject.class)
                .getAsJsonArray("accessors")
                .get(0)
                .getAsJsonObject();

        assertEquals(0.0, position.getAsJsonArray("min").get(0).getAsDouble(), 1e-9);
        assertEquals(1.0, position.getAsJsonArray("max").get(0).getAsDouble(), 1e-9);
    }

    @Test
    public void whatIsWrittenIsAGlb() {
        byte[] glb = Gltf.write(List.of(part(ONE_TRIANGLE)));

        assertEquals("glTF", new String(glb, 0, 4, StandardCharsets.UTF_8));
        assertEquals(
                glb.length, ByteBuffer.wrap(glb).order(ByteOrder.LITTLE_ENDIAN).getInt(8));
    }

    @Test
    public void aPartWithNoTrianglesIsNotWritten() {
        byte[] glb = Gltf.write(List.of(new Gltf.Part("Empty", List.of("Empty"), new double[0], null)));

        assertEquals(0, Gltf.read(glb).size());
    }

    @Test
    public void theFieldModelReadsAsThePythonReadIt() throws IOException {
        List<Gltf.Part> parts = Gltf.read(fieldModel());

        int triangles = 0;
        Set<String> colours = new HashSet<>();
        List<String> names = new ArrayList<>();
        for (Gltf.Part part : parts) {
            triangles += part.count();
            names.add(part.name);
            if (part.colour != null) {
                colours.add(part.colour);
            }
        }

        assertEquals(PARTS_IN_THE_FIELD, parts.size());
        assertEquals(TRIANGLES_IN_THE_FIELD, triangles);
        assertEquals("the CAD's materials", 11, colours.size());
        assertTrue("the assembly names survive", names.stream().anyMatch(name -> name.contains("Hive")));
        assertTrue(names.stream().anyMatch(name -> name.contains("Flower")));
    }

    @Test
    public void theFieldModelIsWhereTheCollisionModelSaysTheFieldIs() throws IOException {
        double[] lows = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        double[] highs = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for (Gltf.Part part : Gltf.read(fieldModel())) {
            for (int i = 0; i < part.triangles.length; i += 3) {
                for (int axis = 0; axis < 3; axis++) {
                    lows[axis] = Math.min(lows[axis], part.triangles[i + axis]);
                    highs[axis] = Math.max(highs[axis], part.triangles[i + axis]);
                }
            }
        }

        assertEquals(-71.8, lows[0], 0.1);
        assertEquals(71.8, highs[0], 0.1);
        assertEquals(-125.7, lows[1], 0.1);
        assertEquals(125.7, highs[1], 0.1);
        assertEquals("the floor", -1.7, lows[2], 0.1);
        assertEquals(66.0, highs[2], 0.1);
    }

    @Test
    public void theFieldModelSurvivesBeingWrittenBackOut() throws IOException {
        List<Gltf.Part> parts = Gltf.read(fieldModel());

        List<Gltf.Part> again = Gltf.read(Gltf.write(parts));

        assertEquals(parts.size(), again.size());
        int triangles = 0;
        for (int i = 0; i < parts.size(); i++) {
            assertEquals(parts.get(i).name, again.get(i).name);
            assertEquals(parts.get(i).path, again.get(i).path);
            assertEquals(parts.get(i).colour, again.get(i).colour);
            assertEquals(parts.get(i).count(), again.get(i).count());
            triangles += again.get(i).count();
        }
        assertEquals(TRIANGLES_IN_THE_FIELD, triangles);
    }

    private static Gltf.Part part(double[] triangles) {
        return new Gltf.Part("Part", List.of("Part"), triangles, null);
    }

    private static byte[] fieldModel() throws IOException {
        try (InputStream in = SimField.class.getResourceAsStream("field.glb")) {
            assertNotNull("field.glb is beside the simulator's other resources", in);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] chunk = new byte[1 << 16];
            int read;
            while ((read = in.read(chunk)) > 0) {
                out.write(chunk, 0, read);
            }
            return out.toByteArray();
        }
    }

    private static String jsonChunkOf(byte[] glb) {
        ByteBuffer bytes = ByteBuffer.wrap(glb).order(ByteOrder.LITTLE_ENDIAN);
        int length = bytes.getInt(12);
        return new String(glb, 20, length, StandardCharsets.UTF_8).trim();
    }

    private static byte[] binaryChunkOf(byte[] glb) {
        ByteBuffer bytes = ByteBuffer.wrap(glb).order(ByteOrder.LITTLE_ENDIAN);
        int json = bytes.getInt(12);
        int at = 20 + json;
        int length = bytes.getInt(at);
        return java.util.Arrays.copyOfRange(glb, at + 8, at + 8 + length);
    }

    private static void assertArrayAlmostEquals(double[] wanted, double[] got) {
        assertEquals(wanted.length, got.length);
        for (int i = 0; i < wanted.length; i++) {
            assertEquals("at " + i, wanted[i], got[i], 1e-6);
        }
    }

    @Test
    public void everyPartOfTheFieldCarriesTheNamesOfTheNodesAboveIt() throws IOException {
        List<Gltf.Part> parts = Gltf.read(fieldModel());

        assertFalse(parts.isEmpty());
        for (Gltf.Part part : parts) {
            assertFalse(part.name + " has no path", part.path.isEmpty());
            assertEquals("a part's path ends with its own name", part.name, part.path.get(part.path.size() - 1));
        }
    }
}
