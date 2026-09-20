package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
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
    private static final byte[] PNG = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n', 'p', 'i', 'x'};

    private static double[] wedge(double x, double y) {
        double metre = Onshape.INCH_IN_METRES;
        return new double[] {
            -y * metre, x * metre, 0,
            -y * metre, (x + 10) * metre, 0,
            -(y + 10) * metre, x * metre, 0
        };
    }

    private static byte[] anExportOfAField() {
        List<Gltf.Part> parts = new ArrayList<>();
        for (int i = 0; i < 12; i++) {
            parts.add(new Gltf.Part("Blue Hive " + i, List.of("Blue Hive " + i), wedge(0, -60), "#0000ff"));
            parts.add(new Gltf.Part("Red Hive " + i, List.of("Red Hive " + i), wedge(0, 60), "#ff0000"));
        }
        parts.add(new Gltf.Part("Field Panel", List.of("Field Panel"), wedge(-70, 0), "#b3b3b3"));
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

        FieldAssets.Refreshed refreshed = FieldAssets.refresh(onshapeOf(asked), store, INTO);

        assertEquals(1 + FieldAssets.TEXTURES.size(), refreshed.written.size());
        assertTrue(store.isFile(INTO.resolve(FieldAssets.FIELD_GLB)));
        for (String name : FieldAssets.TEXTURES) {
            assertTrue(
                    name, store.isFile(INTO.resolve(FieldAssets.TEXTURES_UNDER).resolve(name)));
        }
    }

    @Test
    public void theModelItWritesIsOneTheRendererCanRead() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(onshapeOf(new Asked()), store, INTO);

        List<Gltf.Part> parts =
                Gltf.read(store.readIfThere(INTO.resolve(FieldAssets.FIELD_GLB)).orElseThrow());
        assertEquals(25, parts.size());
    }

    @Test
    public void theExportIsAskedForAsGltfAndTheBlobsByTheirOwnIds() {
        Asked asked = new Asked();

        FieldAssets.refresh(onshapeOf(asked), new InMemoryStore(), INTO);

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
                FieldAssets.NotAnAsset.class, () -> FieldAssets.refresh(onshapeOf(asked), new InMemoryStore(), INTO));

        assertTrue(refused.getMessage(), refused.getMessage().contains("GoalAprilTag_bluescoring.png"));
        assertTrue("and says what it does hold", refused.getMessage().contains("BIOBUZZ_Panel_Resized.png"));
    }

    @Test
    public void somethingThatIsNotAPngIsRefused() {
        Asked asked = new Asked();
        asked.image = "<html>signed out</html>".getBytes(StandardCharsets.UTF_8);

        FieldAssets.NotAnAsset refused = assertThrows(
                FieldAssets.NotAnAsset.class, () -> FieldAssets.refresh(onshapeOf(asked), new InMemoryStore(), INTO));

        assertTrue(refused.getMessage(), refused.getMessage().contains("not a PNG"));
    }

    @Test
    public void nothingIsWrittenUntilEveryAssetHasArrived() {
        Asked asked = new Asked();
        asked.blobs.remove("GoalAprilTag_redscoring.png");
        InMemoryStore store = new InMemoryStore();

        assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.refresh(onshapeOf(asked), store, INTO));

        assertFalse(
                "a rename upstream must stop the run rather than leave half a refresh behind",
                store.isFile(INTO.resolve(FieldAssets.FIELD_GLB)));
    }

    @Test
    public void anExportThatIsNotAFieldIsRefusedBeforeAnythingIsWritten() {
        Asked asked = new Asked();
        asked.export = Gltf.write(List.of(new Gltf.Part("Blue Hive", List.of("Blue Hive"), wedge(0, -60), null)));
        InMemoryStore store = new InMemoryStore();

        assertThrows(IllegalStateException.class, () -> FieldAssets.refresh(onshapeOf(asked), store, INTO));

        assertFalse(store.isFile(INTO.resolve(FieldAssets.FIELD_GLB)));
    }

    @Test
    public void bothDetailsAreBuiltFromOneExport() {
        Asked asked = new Asked();
        InMemoryStore store = new InMemoryStore();

        FieldAssets.Refreshed refreshed = FieldAssets.refresh(
                onshapeOf(asked), store, INTO, List.of(FieldAssets.Detail.NORMAL, FieldAssets.Detail.FULL));

        assertEquals(
                "the export is fetched once however many models are built",
                1,
                asked.urls.stream().filter(url -> url.endsWith("/gltf")).count());
        assertTrue(store.isFile(INTO.resolve(FieldAssets.Detail.NORMAL.file)));
        assertTrue(store.isFile(INTO.resolve(FieldAssets.Detail.FULL.file)));
        assertEquals(2 + FieldAssets.TEXTURES.size(), refreshed.written.size());
    }

    @Test
    public void theFullModelKeepsMoreOfTheCadThanTheNormalOne() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(
                onshapeOf(new Asked()), store, INTO, List.of(FieldAssets.Detail.NORMAL, FieldAssets.Detail.FULL));

        assertTrue(
                "the same parts either way",
                Gltf.read(store.readIfThere(INTO.resolve(FieldAssets.Detail.FULL.file))
                                        .orElseThrow())
                                .size()
                        == Gltf.read(store.readIfThere(INTO.resolve(FieldAssets.Detail.NORMAL.file))
                                        .orElseThrow())
                                .size());
    }

    @Test
    public void onlyTheDetailAskedForIsBuilt() {
        InMemoryStore store = new InMemoryStore();

        FieldAssets.refresh(onshapeOf(new Asked()), store, INTO, List.of(FieldAssets.Detail.FULL));

        assertTrue(store.isFile(INTO.resolve(FieldAssets.Detail.FULL.file)));
        assertFalse(store.isFile(INTO.resolve(FieldAssets.Detail.NORMAL.file)));
    }

    @Test
    public void theDetailsAreNamedNormalFullOrBoth() {
        assertEquals(List.of(FieldAssets.Detail.NORMAL), FieldAssets.detailsNamed(null));
        assertEquals(List.of(FieldAssets.Detail.NORMAL), FieldAssets.detailsNamed("normal"));
        assertEquals(List.of(FieldAssets.Detail.FULL), FieldAssets.detailsNamed("full"));
        assertEquals(List.of(FieldAssets.Detail.NORMAL, FieldAssets.Detail.FULL), FieldAssets.detailsNamed("both"));

        FieldAssets.NotAnAsset refused =
                assertThrows(FieldAssets.NotAnAsset.class, () -> FieldAssets.detailsNamed("finest"));
        assertTrue(refused.getMessage(), refused.getMessage().contains("normal, full or both"));
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
