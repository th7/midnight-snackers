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
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.Test;

public class FieldAssetsTest {
    private static final Path INTO = Path.of("assets");

    /** What a test that is not about which resolution is built asks for: the one a page draws. */
    private static final List<FieldAssets.Resolution> ONE_TO_DRAW = List.of(FieldAssets.Resolution.DEFAULT);

    private static final byte[] PNG = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n', 'p', 'i', 'x'};

    private static double[] wedge(double x, double y) {
        double metre = Onshape.INCH_IN_METRES;
        return new double[] {
            -y * metre, x * metre, 0,
            -y * metre, (x + 10) * metre, 0,
            -(y + 10) * metre, x * metre, 0
        };
    }

    private static final double[] FACING_UP = {0, 0, 1, 0, 0, 1, 0, 0, 1};

    private static Gltf.Part part(String name, double[] triangles, String colour) {
        return new Gltf.Part(name, List.of(name), triangles, FACING_UP, Gltf.colouredMaterial(colour));
    }

    private static byte[] anExportOfAField() {
        List<Gltf.Part> parts = new ArrayList<>();
        for (int i = 0; i < 12; i++) {
            parts.add(part("Blue Hive " + i, wedge(0, -60), "#0000ff"));
            parts.add(part("Red Hive " + i, wedge(0, 60), "#ff0000"));
        }
        parts.add(part("Field Panel", wedge(-70, 0), "#b3b3b3"));
        parts.add(part("am-1611 Socket Head Screw", wedge(10, 10), "#666666"));
        return Gltf.write(parts);
    }

    private static final class Asked implements Onshape.Calls {
        final List<String> urls = new ArrayList<>();
        final Map<String, String> blobs = new LinkedHashMap<>();
        byte[] export = anExportOfAField();
        byte[] image = PNG;

        Asked() {
            for (String name : FieldAssets.TEXTURES) {
                blobs.put(name, "id-" + name);
            }
        }

        @Override
        public Onshape.Answer get(String url, Map<String, String> headers) {
            urls.add(url);
            if (url.contains("/gltf")) {
                return new Onshape.Answer(200, export);
            }
            if (url.endsWith("/elements")) {
                JsonArray listed = new JsonArray();
                for (Map.Entry<String, String> blob : blobs.entrySet()) {
                    JsonObject one = new JsonObject();
                    one.addProperty("name", blob.getKey());
                    one.addProperty("id", blob.getValue());
                    one.addProperty("elementType", "BLOB");
                    listed.add(one);
                }
                JsonObject assembly = new JsonObject();
                assembly.addProperty("name", "Field");
                assembly.addProperty("id", "asm");
                assembly.addProperty("elementType", "ASSEMBLY");
                listed.add(assembly);
                return new Onshape.Answer(200, new Gson().toJson(listed).getBytes(StandardCharsets.UTF_8));
            }
            return new Onshape.Answer(200, image);
        }
    }

    private static Onshape onshapeOf(Asked asked) {
        return Onshape.configured(Map.of(), asked);
    }

    @Test
    public void aRefreshWritesTheModelAndEveryTexture() {
        Asked asked = new Asked();
        InMemoryStore store = new InMemoryStore();

        FieldAssets.Refreshed refreshed = FieldAssets.refresh(onshapeOf(asked), store, INTO, ONE_TO_DRAW);

        assertEquals(
                "the model, the textures, and the export they were built from",
                2 + FieldAssets.TEXTURES.size(),
                refreshed.written.size());
        assertTrue(store.isFile(INTO.resolve(FieldAssets.EXPORT_FILE)));
        assertTrue(store.isFile(INTO.resolve(FieldAssets.Resolution.DEFAULT.file)));
        for (String name : FieldAssets.TEXTURES) {
            assertTrue(
                    name, store.isFile(INTO.resolve(FieldAssets.TEXTURES_UNDER).resolve(name)));
        }
    }

    @Test
    public void theModelItWritesIsOneTheRendererCanRead() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(onshapeOf(new Asked()), store, INTO, ONE_TO_DRAW);

        List<Gltf.Part> parts = partsIn(store, FieldAssets.Resolution.DEFAULT);
        assertEquals(26, parts.size());
    }

    @Test
    public void theExportIsAskedForAsGltfAndTheBlobsByTheirOwnIds() {
        Asked asked = new Asked();

        FieldAssets.refresh(onshapeOf(asked), new InMemoryStore(), INTO, ONE_TO_DRAW);

        assertTrue(asked.urls.toString(), asked.urls.get(0).endsWith("/gltf"));
        assertTrue(asked.urls.toString(), asked.urls.stream().anyMatch(url -> url.endsWith("/elements")));
        for (String name : FieldAssets.TEXTURES) {
            assertTrue(name, asked.urls.stream().anyMatch(url -> url.endsWith("id-" + name)));
        }
    }

    @Test
    public void aBlobTheDocumentDoesNotHoldIsNamedRatherThanSkipped() {
        Asked asked = new Asked();
        asked.blobs.remove("GoalAprilTag_bluescoring.png");

        FieldAssets.NotAnAsset refused = assertThrows(
                FieldAssets.NotAnAsset.class,
                () -> FieldAssets.refresh(onshapeOf(asked), new InMemoryStore(), INTO, ONE_TO_DRAW));

        assertTrue(refused.getMessage(), refused.getMessage().contains("GoalAprilTag_bluescoring.png"));
        assertTrue("and says what it does hold", refused.getMessage().contains("BIOBUZZ_Panel_Resized.png"));
    }

    @Test
    public void somethingThatIsNotAPngIsRefused() {
        Asked asked = new Asked();
        asked.image = "<html>signed out</html>".getBytes(StandardCharsets.UTF_8);

        FieldAssets.NotAnAsset refused = assertThrows(
                FieldAssets.NotAnAsset.class,
                () -> FieldAssets.refresh(onshapeOf(asked), new InMemoryStore(), INTO, ONE_TO_DRAW));

        assertTrue(refused.getMessage(), refused.getMessage().contains("not a PNG"));
    }

    @Test
    public void nothingIsWrittenUntilEveryAssetHasArrived() {
        Asked asked = new Asked();
        asked.blobs.remove("GoalAprilTag_redscoring.png");
        InMemoryStore store = new InMemoryStore();

        assertThrows(
                FieldAssets.NotAnAsset.class, () -> FieldAssets.refresh(onshapeOf(asked), store, INTO, ONE_TO_DRAW));

        assertFalse(
                "a rename upstream must stop the run rather than leave half a refresh behind",
                store.isFile(INTO.resolve(FieldAssets.FIELD_GLB)));
    }

    @Test
    public void anExportThatIsNotAFieldIsRefusedBeforeAnythingIsWritten() {
        Asked asked = new Asked();
        asked.export = Gltf.write(List.of(new Gltf.Part("Blue Hive", List.of("Blue Hive"), wedge(0, -60), null)));
        InMemoryStore store = new InMemoryStore();

        assertThrows(
                IllegalStateException.class, () -> FieldAssets.refresh(onshapeOf(asked), store, INTO, ONE_TO_DRAW));

        assertFalse(store.isFile(INTO.resolve(FieldAssets.FIELD_GLB)));
    }

    @Test
    public void everyResolutionIsBuiltFromOneExport() {
        Asked asked = new Asked();
        InMemoryStore store = new InMemoryStore();

        FieldAssets.Refreshed refreshed = FieldAssets.refresh(
                onshapeOf(asked), store, INTO, List.of(FieldAssets.Resolution.LOW, FieldAssets.Resolution.HIGH));

        assertEquals(
                "the export is fetched once however many models are built",
                1,
                asked.urls.stream().filter(url -> url.endsWith("/gltf")).count());
        assertTrue(store.isFile(INTO.resolve(FieldAssets.Resolution.LOW.file)));
        assertTrue(store.isFile(INTO.resolve(FieldAssets.Resolution.HIGH.file)));
        assertEquals(
                "both models, the textures, and the one export they were both built from",
                3 + FieldAssets.TEXTURES.size(),
                refreshed.written.size());
    }

    @Test
    public void eachResolutionIsOneStepMoreOfTheCadThanTheOneBelow() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(
                onshapeOf(new Asked()),
                store,
                INTO,
                FieldAssets.resolutionsNamed("all", FieldAssets.Resolution.DEFAULT));

        assertEquals("low draws the field as it plays", 25, partsOf(store, FieldAssets.Resolution.LOW));
        assertEquals("and so does medium, at the CAD's own points", 25, partsOf(store, FieldAssets.Resolution.MEDIUM));
        assertEquals(
                "high is every part the export holds, hardware and all",
                26,
                partsOf(store, FieldAssets.Resolution.HIGH));
    }

    private static int partsOf(InMemoryStore store, FieldAssets.Resolution resolution) {
        return Gltf.read(store.readIfThere(INTO.resolve(resolution.file)).orElseThrow())
                .size();
    }

    // Resolution decides two things: which parts are in a model, and how much of their surface comes
    // with them. Normals are a normal to a vertex -- the model over again -- so the cheapest model
    // goes without them and the two above it pay.
    @Test
    public void lowGoesWithoutTheCadsNormalsAndTheOthersCarryThem() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(
                onshapeOf(new Asked()),
                store,
                INTO,
                FieldAssets.resolutionsNamed("all", FieldAssets.Resolution.DEFAULT));

        for (FieldAssets.Resolution carries : List.of(FieldAssets.Resolution.MEDIUM, FieldAssets.Resolution.HIGH)) {
            for (Gltf.Part part : partsIn(store, carries)) {
                assertNotNull(carries.asked + ": " + part.name, part.normals);
            }
        }
        for (Gltf.Part part : partsIn(store, FieldAssets.Resolution.LOW)) {
            assertNull("the cheapest model is the cheapest model: " + part.name, part.normals);
        }
    }

    private static List<Gltf.Part> partsIn(InMemoryStore store, FieldAssets.Resolution resolution) {
        return Gltf.read(store.readIfThere(INTO.resolve(resolution.file)).orElseThrow());
    }

    @Test
    public void aDownloadKeepsTheExportItFetched() {
        Asked asked = new Asked();
        InMemoryStore store = new InMemoryStore();

        FieldAssets.download(onshapeOf(asked), store, INTO);

        assertTrue(
                "the dear half is the download; keeping it is what lets a build be cheap",
                store.isFile(INTO.resolve(FieldAssets.EXPORT_FILE)));
        for (String name : FieldAssets.TEXTURES) {
            assertTrue(
                    name, store.isFile(INTO.resolve(FieldAssets.TEXTURES_UNDER).resolve(name)));
        }
        assertFalse(
                "a download is not a build: no model is made until one is asked for",
                store.isFile(INTO.resolve(FieldAssets.Resolution.LOW.file)));
    }

    @Test
    public void aBuildMakesEveryDetailFromTheExportOnDisk() {
        Asked asked = new Asked();
        InMemoryStore store = new InMemoryStore();
        FieldAssets.download(onshapeOf(asked), store, INTO);
        int fetches = asked.urls.size();

        FieldAssets.build(store, INTO, List.of(FieldAssets.Resolution.LOW, FieldAssets.Resolution.HIGH));

        assertEquals("a build fetches nothing; it cannot, it is given no Onshape", fetches, asked.urls.size());
        assertEquals(25, partsOf(store, FieldAssets.Resolution.LOW));
        assertEquals(26, partsOf(store, FieldAssets.Resolution.HIGH));
    }

    @Test
    public void aBuildWithNothingDownloadedSaysSoRatherThanFetching() {
        FieldAssets.NotAnAsset refused = assertThrows(
                FieldAssets.NotAnAsset.class,
                () -> FieldAssets.build(new InMemoryStore(), INTO, List.of(FieldAssets.Resolution.LOW)));

        assertTrue(refused.getMessage(), refused.getMessage().contains("nothing has been downloaded"));
    }

    @Test
    public void aRefreshIsADownloadAndThenABuild() {
        Asked asked = new Asked();
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(onshapeOf(asked), store, INTO, List.of(FieldAssets.Resolution.HIGH));

        assertTrue(store.isFile(INTO.resolve(FieldAssets.EXPORT_FILE)));
        assertTrue(store.isFile(INTO.resolve(FieldAssets.Resolution.HIGH.file)));
        assertEquals(
                "the export is fetched once",
                1,
                asked.urls.stream().filter(url -> url.endsWith("/gltf")).count());
    }

    @Test
    public void onlyTheResolutionAskedForIsBuilt() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(onshapeOf(new Asked()), store, INTO, List.of(FieldAssets.Resolution.HIGH));

        assertTrue(store.isFile(INTO.resolve(FieldAssets.Resolution.HIGH.file)));
        assertFalse(store.isFile(INTO.resolve(FieldAssets.Resolution.LOW.file)));
    }

    @Test
    public void theResolutionsAreNamedLowMediumHighOrAll() {
        FieldAssets.Resolution drawn = FieldAssets.Resolution.MEDIUM;
        assertEquals(List.of(FieldAssets.Resolution.LOW), FieldAssets.resolutionsNamed("low", drawn));
        assertEquals(List.of(FieldAssets.Resolution.MEDIUM), FieldAssets.resolutionsNamed("medium", drawn));
        assertEquals(List.of(FieldAssets.Resolution.HIGH), FieldAssets.resolutionsNamed("high", drawn));
        assertEquals(
                List.of(FieldAssets.Resolution.LOW, FieldAssets.Resolution.MEDIUM, FieldAssets.Resolution.HIGH),
                FieldAssets.resolutionsNamed("all", drawn));

        FieldAssets.NotAnAsset refused =
                assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.resolutionsNamed("finest", drawn));
        assertTrue(refused.getMessage(), refused.getMessage().contains("low, medium, high or all"));
    }

    /**
     * A build nobody gave a resolution builds the one the pages of whoever asked draw, which is a
     * setting on their admin page rather than anything this knows: it is named at the call, and the
     * built-in answer is what a server holds until an admin sets another.
     */
    @Test
    public void askingForNoResolutionInParticularBuildsTheOneThePagesDraw() {
        assertEquals(FieldAssets.Resolution.HIGH, FieldAssets.Resolution.DEFAULT);
        assertEquals(
                List.of(FieldAssets.Resolution.LOW), FieldAssets.resolutionsNamed(null, FieldAssets.Resolution.LOW));
        assertEquals(
                List.of(FieldAssets.Resolution.MEDIUM),
                FieldAssets.resolutionsNamed("", FieldAssets.Resolution.MEDIUM));
    }

    /** A page draws one field, so the name an admin sets is one of the three and never all of them. */
    @Test
    public void theResolutionAPageDrawsIsOneOfThemRatherThanAllOfThem() {
        assertEquals(FieldAssets.Resolution.LOW, FieldAssets.theOneNamed("low"));
        assertEquals(FieldAssets.Resolution.MEDIUM, FieldAssets.theOneNamed("medium"));
        assertEquals(FieldAssets.Resolution.HIGH, FieldAssets.theOneNamed("high"));

        for (String notOne : new String[] {"all", "finest", "", null}) {
            FieldAssets.NotAnAsset refused =
                    assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.theOneNamed(notOne));
            assertTrue(refused.getMessage(), refused.getMessage().contains("low, medium or high"));
        }
    }

    /**
     * Which field a page draws when it names none is said in one place and read in another -- a
     * server's setting, a line of JavaScript a page imports -- so the line is this server's to
     * write, and the copy committed beside the models is what it writes for the built-in answer.
     * Said twice by hand, the page and the server would drift apart without either saying so.
     */
    @Test
    public void theLineAPageReadsForTheDefaultIsTheServersToWrite() {
        assertTrue(
                FieldAssets.defaultResolutionModule(FieldAssets.Resolution.LOW),
                FieldAssets.defaultResolutionModule(FieldAssets.Resolution.LOW)
                        .endsWith("export const DEFAULT_RESOLUTION = 'low';\n"));
        assertEquals(
                "the committed " + FieldAssets.DEFAULT_RESOLUTION_FILE + " is not what this server writes for "
                        + FieldAssets.Resolution.DEFAULT.asked + "; write what this expects into it",
                FieldAssets.defaultResolutionModule(FieldAssets.Resolution.DEFAULT),
                SimAssets.page(FieldAssets.DEFAULT_RESOLUTION_FILE));
    }

    @Test
    public void anElementNamedLikeAPathIsRefused() {
        assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.safeName("../../etc/passwd"));
        assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.safeName(" padded.png"));
        assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.safeName(""));
        assertEquals("GoalAprilTag_redscoring.png", FieldAssets.safeName("GoalAprilTag_redscoring.png"));
    }

    @Test
    public void theTexturesAreTheOnesThePageAsksFor() {
        for (String name : FieldAssets.TEXTURES) {
            assertTrue(name + " is not beside the page", SimAssets.serve("textures/" + name).status == 200);
        }
    }
}
