package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.Test;

/**
 * What the bench serves beside its pages: the field's visual model, the artwork printed on it, and
 * the renderer that draws them. All of it sits on the classpath next to {@link SimField}, and a
 * name that asks for anything else is refused rather than read.
 */
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
        // The guard against a quiet regression to String bodies: glTF begins with "glTF" and then
        // bytes that are not UTF-8, so a round trip through a String shows up here as a size that
        // no longer matches and a magic number that does not survive.
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

    /**
     * The name comes off the URL, so it must not be able to name anything but an asset. A walk up
     * out of the directory, an absolute path, or a backslash where a slash was expected: each is
     * refused before anything is opened.
     */
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

    /**
     * Only the kinds of file the page actually loads are served. Everything the simulator keeps on
     * that classpath sits in the same place -- the collision model, the page templates -- and a
     * route that served whatever it was asked for would hand out all of it.
     */
    /**
     * Every asset the field page names is one the bench will actually serve. A page that asks for
     * a file nobody has does not say so: it draws an empty canvas, or a field with a part missing,
     * and the only clue is in a console nobody has open. The names are in the page's import map
     * and in the one line that loads the model, and they are checked here against the route that
     * has to answer for them.
     */
    @Test
    public void everyAssetTheFieldPageAsksForIsOneTheBenchServes() {
        String page = SimAssets.page("field.html");
        Matcher named =
                Pattern.compile("[\"']" + "(assets/[A-Za-z0-9._/-]+)" + "[\"']").matcher(page);
        Set<String> asked = new TreeSet<>();
        while (named.find()) {
            asked.add(named.group(1));
        }

        assertTrue("the page names no assets at all: " + page.length() + " bytes", asked.size() >= 3);
        for (String url : asked) {
            String name = url.substring("assets/".length());
            // An import map may name a directory, which stands for the files under it.
            if (name.endsWith("/")) {
                assertNotNull(
                        name + " is a directory the page imports from, and nothing is under it",
                        SimField.class.getResource(name.substring(0, name.length() - 1)));
                continue;
            }
            assertEquals(url + " is asked for by the page and not served", 200, SimAssets.serve(name).status);
        }
    }

    /** The page loads the model through the addons, which reach for each other by relative path. */
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
