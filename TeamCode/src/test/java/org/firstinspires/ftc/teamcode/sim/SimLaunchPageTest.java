package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
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
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimLaunchPageTest {
    private static final Path BROWSER_CHECKS = Paths.get("..", "tools", "browser");
    private static final Path THE_CHECK = BROWSER_CHECKS.resolve("launch.mjs");
    private static final Path THE_SHOT_IT_PLAYS = BROWSER_CHECKS.resolve("launch-shot.json");
    private static final Path WHAT_THE_TAB_IS_ANSWERED = BROWSER_CHECKS.resolve("launch-answers.json");
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
        try (InputStream in = SimLaunchPage.class.getResourceAsStream(SimLaunchPage.TEMPLATE)) {
            assertTrue("launch.html is beside SimLaunchPage", in != null);
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

        assertEquals(
                new TreeSet<>(java.util.List.of("__ASSETS__", "__DATA__", "__FIELD__", "__IMPORTMAP__", "__TITLE__")),
                holes);
        assertTrue(placeholdersIn(SimLaunchPage.viewer()).isEmpty());
        String check = new String(Files.readAllBytes(THE_CHECK), StandardCharsets.UTF_8);
        for (String hole : holes) {
            assertTrue(THE_CHECK + " fills " + hole, check.contains("'" + hole + "'"));
        }
    }

    @Test
    public void theViewerFetchesItsAssetsFromWhereTheBenchServesThem() {
        String page = SimLaunchPage.viewer();

        assertTrue(page.contains("\"../assets/\""));
        assertTrue(page.contains("../assets/vendor/three.module.min.js"));
        assertTrue(page.contains(SimLaunchPage.TITLE));
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

    private String theBenchsDefaultShot() {
        Router routes = routes();
        TinyHttpServer.Response launched = routes.handle(TinyHttpServer.Request.of("POST", "/launch", "{}"));
        assertEquals(launched.body, 200, launched.status);
        int id = new Gson().fromJson(launched.body, JsonObject.class).get("id").getAsInt();
        return routes.handle(TinyHttpServer.Request.of("GET", "/launches/" + id + "/shot", "")).body;
    }

    static String shapeOf(JsonElement json) {
        if (json == null || json.isJsonNull()) {
            return "null";
        }
        if (json.isJsonPrimitive()) {
            return json.getAsJsonPrimitive().isNumber()
                    ? "number"
                    : json.getAsJsonPrimitive().isBoolean() ? "boolean" : "string";
        }
        if (json.isJsonArray()) {
            JsonArray array = json.getAsJsonArray();
            boolean row = array.size() > 0;
            for (JsonElement element : array) {
                row &= element.isJsonPrimitive();
            }
            if (row) {
                StringBuilder columns = new StringBuilder("(");
                for (JsonElement element : array) {
                    columns.append(shapeOf(element).charAt(0));
                }
                return columns.append(")").toString();
            }
            TreeSet<String> kinds = new TreeSet<>();
            for (JsonElement element : array) {
                kinds.add(shapeOf(element));
            }
            return "[" + String.join(" | ", kinds) + "]";
        }
        Map<String, String> fields = new TreeMap<>();
        for (Map.Entry<String, JsonElement> entry : json.getAsJsonObject().entrySet()) {
            fields.put(entry.getKey(), shapeOf(entry.getValue()));
        }
        return fields.toString();
    }

    @Test
    public void theShotTheBrowserCheckPlaysIsShapedAsTheBenchServesOne() throws IOException {
        String served = theBenchsDefaultShot();

        if (Boolean.getBoolean(SimTrace.REGENERATE)) {
            RepoFile.write(THE_SHOT_IT_PLAYS, served.getBytes(StandardCharsets.UTF_8));
            throw new AssertionError("regenerated " + THE_SHOT_IT_PLAYS.toAbsolutePath()
                    + ". A run that wrote the shot has not checked it: read the diff, then re-run without -D"
                    + SimTrace.REGENERATE + ".");
        }
        assertTrue(
                "no shot at " + THE_SHOT_IT_PLAYS.toAbsolutePath() + " for the browser check to play. Write it with:\n"
                        + "  ./gradlew :TeamCode:testDebugUnitTest --rerun -D" + SimTrace.REGENERATE
                        + "=true --tests '*SimLaunchPageTest*'",
                Files.isRegularFile(THE_SHOT_IT_PLAYS));
        JsonElement played = new Gson()
                .fromJson(new String(Files.readAllBytes(THE_SHOT_IT_PLAYS), StandardCharsets.UTF_8), JsonElement.class);
        assertEquals(
                "the shot " + THE_CHECK + " plays is no longer shaped as the bench serves one; regenerate it with"
                        + " -D" + SimTrace.REGENERATE + "=true --tests '*SimLaunchPageTest*' and read the diff",
                shapeOf(new Gson().fromJson(served, JsonElement.class)),
                shapeOf(played));
    }

    private String whatTheBenchAnswersTheTab() {
        Router routes = routes();
        Gson gson = new Gson();
        JsonObject answers = new JsonObject();
        answers.add(
                "settings",
                gson.fromJson(
                        routes.handle(TinyHttpServer.Request.of("GET", "/launch/settings", "")).body,
                        JsonElement.class));
        answers.add(
                "preview",
                gson.fromJson(
                        routes.handle(TinyHttpServer.Request.of("POST", "/launch/preview", "{}")).body,
                        JsonElement.class));
        routes.handle(TinyHttpServer.Request.of("POST", "/launch", "{\"pieces\": 0, \"longest\": 0.5}"));
        routes.handle(TinyHttpServer.Request.of("POST", "/launch", "{\"pieces\": 0, \"longest\": 0.5, \"pitch\": 60}"));
        answers.add(
                "launches",
                gson.fromJson(
                        routes.handle(TinyHttpServer.Request.of("GET", "/launches", "")).body, JsonElement.class));
        answers.add(
                "refused",
                gson.fromJson(
                        routes.handle(TinyHttpServer.Request.of("POST", "/launch", "{\"pitch\": 95}")).body,
                        JsonElement.class));
        return new com.google.gson.GsonBuilder()
                .serializeNulls()
                .setPrettyPrinting()
                .create()
                .toJson(answers);
    }

    @Test
    public void whatTheTabsBrowserCheckIsAnsweredIsShapedAsTheBenchAnswersIt() throws IOException {
        String answered = whatTheBenchAnswersTheTab();

        if (Boolean.getBoolean(SimTrace.REGENERATE)) {
            RepoFile.write(WHAT_THE_TAB_IS_ANSWERED, answered.getBytes(StandardCharsets.UTF_8));
            throw new AssertionError("regenerated " + WHAT_THE_TAB_IS_ANSWERED.toAbsolutePath()
                    + ". A run that wrote the answers has not checked them: read the diff, then re-run without -D"
                    + SimTrace.REGENERATE + ".");
        }
        assertTrue(
                "no answers at " + WHAT_THE_TAB_IS_ANSWERED.toAbsolutePath()
                        + " for the dashboard's check. Write them with:\n"
                        + "  ./gradlew :TeamCode:testDebugUnitTest --rerun -D" + SimTrace.REGENERATE
                        + "=true --tests '*SimLaunchPageTest*'",
                Files.isRegularFile(WHAT_THE_TAB_IS_ANSWERED));
        JsonElement kept = new Gson()
                .fromJson(
                        new String(Files.readAllBytes(WHAT_THE_TAB_IS_ANSWERED), StandardCharsets.UTF_8),
                        JsonElement.class);
        assertEquals(
                "the answers tools/browser/dashboard.mjs gives the Launch Sim tab are no longer shaped as the bench"
                        + " gives them; regenerate with -D" + SimTrace.REGENERATE
                        + "=true --tests '*SimLaunchPageTest*'",
                shapeOf(new Gson().fromJson(answered, JsonElement.class)),
                shapeOf(kept));
    }

    @Test
    public void aShapeSaysEveryKeyAndWhatKindOfValueItHolds() {
        Gson gson = new Gson();

        assertEquals(
                "{a=number, b=[{c=string}], d=(nns)}",
                shapeOf(gson.fromJson("{\"b\":[{\"c\":\"x\"}],\"a\":1,\"d\":[1,2,\"f\"]}", JsonElement.class)));
        assertEquals(
                shapeOf(gson.fromJson("{\"a\":[{\"b\":1},{\"b\":2}]}", JsonElement.class)),
                shapeOf(gson.fromJson("{\"a\":[{\"b\":3}]}", JsonElement.class)));
        assertFalse(shapeOf(gson.fromJson("{\"a\":[{\"b\":1}]}", JsonElement.class))
                .equals(shapeOf(gson.fromJson("{\"a\":[{\"c\":1}]}", JsonElement.class))));
    }
}
