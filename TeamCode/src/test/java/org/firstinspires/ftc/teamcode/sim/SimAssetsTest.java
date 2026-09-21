package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.Test;

public class SimAssetsTest {
    private static byte[] onTheClasspath(String name) {
        try (InputStream in = SimField.class.getResourceAsStream(name)) {
            assertNotNull(name + " is not on the classpath", in);
            return in.readAllBytes();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Test
    public void theVisualModelIsServedWholeAndAsAModel() {
        Response response = SimAssets.serve("field.glb");

        assertEquals(200, response.status);
        assertEquals("model/gltf-binary", response.contentType);
        assertArrayEquals(onTheClasspath("field.glb"), SimAssets.body(response));
    }

    @Test
    public void theModelIsBigEnoughThatServingItAsTextWouldHaveRuinedIt() {
        byte[] model = SimAssets.body(SimAssets.serve("field.glb"));

        assertTrue("a real model, not a stub: " + model.length, model.length > 1_000_000);
        assertEquals("glTF", new String(model, 0, 4, java.nio.charset.StandardCharsets.US_ASCII));
    }

    @Test
    public void theGoalTagArtworkIsServedAsAnImage() {
        Response response = SimAssets.serve("textures/GoalAprilTag_bluescoring.png");

        assertEquals(200, response.status);
        assertEquals("image/png", response.contentType);
        assertArrayEquals(onTheClasspath("textures/GoalAprilTag_bluescoring.png"), SimAssets.body(response));
    }

    @Test
    public void theRendererItselfIsServedAsAScript() {
        Response response = SimAssets.serve("vendor/three.module.min.js");

        assertEquals(200, response.status);
        assertTrue(response.contentType, response.contentType.startsWith("text/javascript"));
        assertTrue(SimAssets.body(response).length > 100_000);
    }

    @Test
    public void aNameNobodyHasIsNotFound() {
        assertEquals(404, SimAssets.serve("no-such-asset.glb").status);
    }

    @Test
    public void aNameThatClimbsOutOfTheAssetsIsRefused() {
        for (String hostile : new String[] {
            "../field.json",
            "../../teamcode/Robot.class",
            "textures/../../field.json",
            "/etc/passwd",
            "..\\field.json",
            "",
            ".",
            ".."
        }) {
            assertEquals(hostile + " is not an asset", 404, SimAssets.serve(hostile).status);
        }
    }

    @Test
    public void everyAssetTheLiveViewAsksForIsOneTheBenchServes() throws IOException {
        Set<String> asked = new TreeSet<>();
        for (String module : List.of("fieldscene.js", "framecost.js", "webcam.js")) {
            assertEquals(module + " is what the live view loads", 200, SimAssets.serve(module).status);
            Matcher named = Pattern.compile("['\"]([A-Za-z0-9_./-]+\\.(?:glb|png|js))['\"]")
                    .matcher(SimAssets.page(module));
            while (named.find()) {
                // A module beside this one is asked for as "./name.js", and is the same asset.
                asked.add(named.group(1).replaceFirst("^\\./", ""));
            }
        }

        assertTrue("the modules name no assets at all: " + asked, asked.size() >= 5);
        // Every model above the lowest is built rather than committed, so the page names files that
        // are not here until an admin builds them. The lowest is the stand-in and must be served.
        Set<String> built = new java.util.HashSet<>();
        for (FieldAssets.Resolution one : FieldAssets.Resolution.values()) {
            if (one != FieldAssets.Resolution.LOW) {
                built.add(one.file);
            }
        }
        for (String name : asked) {
            if (name.startsWith("three") || name.contains("vendor") || built.contains(name)) {
                continue;
            }
            assertEquals(name + " is asked for by the live view and not served", 200, SimAssets.serve(name).status);
        }
        assertTrue(
                "the lowest model is among them, and it is the committed one",
                asked.contains(FieldAssets.Resolution.LOW.file));
        assertTrue("and the tag artwork", asked.stream().anyMatch(name -> name.startsWith("textures/")));
    }

    @Test
    public void theVendoredAddonsCanReachWhatTheyImport() {
        for (String name : new String[] {
            "vendor/three.module.min.js",
            "vendor/jsm/loaders/GLTFLoader.js",
            "vendor/jsm/controls/OrbitControls.js",
            "vendor/jsm/utils/BufferGeometryUtils.js"
        }) {
            assertEquals(name + " is not where the page expects it", 200, SimAssets.serve(name).status);
        }
    }

    @Test
    public void onlyTheKindsOfFileThePageLoadsAreServed() {
        assertEquals(
                "the collision model is read by the simulator, not fetched", 404, SimAssets.serve("field.json").status);
        assertEquals(404, SimAssets.serve("replay.html").status);
    }
}
