package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimFeedPageTest {
    private static final Path BROWSER_CHECKS = Paths.get("..", "tools", "browser");
    private static final Path THE_CHECK = BROWSER_CHECKS.resolve("feeder.mjs");
    private static final Path WHAT_THE_TAB_IS_ANSWERED = BROWSER_CHECKS.resolve("feeder-answers.json");
    private static final Pattern PLACEHOLDER = Pattern.compile("__[A-Z_]+__");

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private SimBench bench;

    @After
    public void stopBench() {
        if (bench != null) {
            bench.stop();
        }
    }

    private static String template() throws IOException {
        try (InputStream in = SimFeedPage.class.getResourceAsStream(SimFeedPage.TEMPLATE)) {
            assertTrue("feeder.html is beside SimFeedPage", in != null);
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    private static TreeSet<String> placeholdersIn(String text) {
        TreeSet<String> found = new TreeSet<>();
        Matcher matcher = PLACEHOLDER.matcher(text);
        while (matcher.find()) {
            found.add(matcher.group());
        }
        return found;
    }

    @Test
    public void thePageHoldsExactlyThePlaceholdersTheBenchFillsAndTheBrowserCheckFillsThemToo() throws IOException {
        TreeSet<String> holes = placeholdersIn(template());

        assertEquals(new TreeSet<>(List.of("__ASSETS__", "__IMPORTMAP__", "__TITLE__")), holes);
        assertTrue(placeholdersIn(SimFeedPage.viewer()).isEmpty());
        String check = new String(Files.readAllBytes(THE_CHECK), StandardCharsets.UTF_8);
        for (String hole : holes) {
            assertTrue(THE_CHECK + " fills " + hole, check.contains("'" + hole + "'"));
        }
    }

    @Test
    public void theViewerFetchesItsAssetsFromWhereTheBenchServesThem() {
        String page = SimFeedPage.viewer();

        assertTrue(page.contains("\"../assets/\""));
        assertTrue(page.contains("../assets/vendor/three.module.min.js"));
        assertTrue(page.contains("<title>" + SimFeedPage.TITLE + "</title>"));
    }

    private Router routes() {
        if (bench == null) {
            bench = new SimBench(
                    TeamRobot.REGINALD,
                    SimCatalog.of(HangingAuto.class),
                    null,
                    folder.getRoot().toPath().resolve("sim"),
                    SimBench.Waits.ofTheBench());
        }
        return bench.routes("ada");
    }

    private JsonElement answer(String method, String path, String body) {
        return new Gson()
                .fromJson(routes().handle(TinyHttpServer.Request.of(method, path, body)).body, JsonElement.class);
    }

    private String whatTheBenchAnswersTheTab() {
        JsonObject answers = new JsonObject();
        answers.add("settings", answer("GET", "/feed/settings", ""));
        JsonElement fed = answer("POST", "/feed", "{}");
        answers.add("fed", fed);
        String id = fed.getAsJsonObject().get("id").getAsString();
        answers.add("listing", answer("GET", "/feeds", ""));
        JsonElement feed = answer("GET", "/feeds/" + id + "/feed", "");
        answers.add("feed", feed);
        JsonArray chunks = new JsonArray();
        for (int chunk = 0; chunk < feed.getAsJsonObject().get("chunks").getAsInt(); chunk++) {
            chunks.add(answer("GET", "/feeds/" + id + "/frames/" + chunk, ""));
        }
        answers.add("frames", chunks);
        answers.add("refused", answer("POST", "/feed", "{\"back_sill\": 10}"));
        return new Gson().toJson(answers);
    }

    @Test
    public void whatTheFeederChecksBrowserIsAnsweredIsShapedAsTheBenchAnswersIt() throws IOException {
        String answered = whatTheBenchAnswersTheTab();

        if (Boolean.getBoolean(SimTrace.REGENERATE)) {
            RepoFile.write(WHAT_THE_TAB_IS_ANSWERED, answered.getBytes(StandardCharsets.UTF_8));
            throw new AssertionError("regenerated " + WHAT_THE_TAB_IS_ANSWERED.toAbsolutePath()
                    + ". A run that wrote the answers has not checked them: read the diff, then re-run without -D"
                    + SimTrace.REGENERATE + ".");
        }
        assertTrue(
                "no answers at " + WHAT_THE_TAB_IS_ANSWERED.toAbsolutePath() + " for the feeder's check. Write them"
                        + " with:\n  ./gradlew :TeamCode:testDebugUnitTest --rerun -D" + SimTrace.REGENERATE
                        + "=true --tests '*SimFeedPageTest*'",
                Files.isRegularFile(WHAT_THE_TAB_IS_ANSWERED));
        JsonElement kept = new Gson()
                .fromJson(
                        new String(Files.readAllBytes(WHAT_THE_TAB_IS_ANSWERED), StandardCharsets.UTF_8),
                        JsonElement.class);
        assertEquals(
                "the answers " + THE_CHECK + " gives the Feeder Sim are no longer shaped as the bench gives them;"
                        + " regenerate with -D" + SimTrace.REGENERATE + "=true --tests '*SimFeedPageTest*'",
                SimLaunchPageTest.shapeOf(new Gson().fromJson(answered, JsonElement.class)),
                SimLaunchPageTest.shapeOf(kept));
    }
}
