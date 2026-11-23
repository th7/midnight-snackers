package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimRobotTabTest {
    private static final Path BROWSER_CHECKS = Paths.get("..", "tools", "browser");
    private static final Path THE_CHECK = BROWSER_CHECKS.resolve("robot.mjs");
    private static final Path WHAT_THE_TAB_IS_ANSWERED = BROWSER_CHECKS.resolve("robot-answers.json");

    static final String MOVED_WHEEL = "{\"constants\":{\"parallel_dead_wheel_y_in\":-1.5}}";
    static final String TOO_FLAT = "{\"constants\":{\"parallel_dead_wheel_y_in\":-1.5,\"flattest_volts\":14}}";
    static final String REWIRED = "{\"configuration\":{\"rightBack\":{\"encoder\":\"nothing\"}}}";
    static final String REVERSED = "{\"directions\":{\"intake\":\"pushes_out\"}}";
    static final String KEPT_BY_AN_OLDER_BUILD = "{\"constants\":{\"mass_kg\":40,\"launch_throw\":1.2}}";

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private final List<SimBench> benches = new ArrayList<>();

    @After
    public void stopBenches() {
        benches.forEach(SimBench::stop);
    }

    private SimBench aBench(String kept) throws IOException {
        Path outputDir = folder.newFolder().toPath().resolve("sim");
        if (kept != null) {
            Files.createDirectories(outputDir);
            Files.write(outputDir.resolve(SimOverrides.FILE), kept.getBytes(StandardCharsets.UTF_8));
        }
        SimBench bench = new SimBench(SimCatalog.of(HangingAuto.class), null, outputDir, SimBench.Waits.ofTheBench());
        benches.add(bench);
        return bench;
    }

    private static JsonElement answer(SimBench bench, String method, String body) {
        return new Gson()
                .fromJson(
                        bench.routes("ada").handle(TinyHttpServer.Request.of(method, "/robot", body)).body,
                        JsonElement.class);
    }

    private String whatTheBenchAnswersTheTab() throws IOException {
        SimBench bench = aBench(null);
        JsonObject answers = new JsonObject();
        answers.add("listing", answer(bench, "GET", ""));
        answers.add("saved", answer(bench, "PUT", MOVED_WHEEL));
        answers.add("refused", answer(bench, "PUT", TOO_FLAT));
        answers.add("reset", answer(bench, "PUT", "{}"));
        answers.add("rewired", answer(bench, "PUT", REWIRED));
        answers.add("reversed", answer(bench, "PUT", REVERSED));
        answers.add("recalled", answer(aBench(KEPT_BY_AN_OLDER_BUILD), "GET", ""));
        return new Gson().toJson(answers);
    }

    @Test
    public void whatTheRobotTabsBrowserCheckIsAnsweredIsShapedAsTheBenchAnswersIt() throws IOException {
        String answered = whatTheBenchAnswersTheTab();

        if (Boolean.getBoolean(SimTrace.REGENERATE)) {
            RepoFile.write(WHAT_THE_TAB_IS_ANSWERED, answered.getBytes(StandardCharsets.UTF_8));
            throw new AssertionError("regenerated " + WHAT_THE_TAB_IS_ANSWERED.toAbsolutePath()
                    + ". A run that wrote the answers has not checked them: read the diff, then re-run without -D"
                    + SimTrace.REGENERATE + ".");
        }
        assertTrue(
                "no answers at " + WHAT_THE_TAB_IS_ANSWERED.toAbsolutePath() + " for the Robot tab's check. Write"
                        + " them with:\n  ./gradlew :TeamCode:testDebugUnitTest --rerun -D" + SimTrace.REGENERATE
                        + "=true --tests '*SimRobotTabTest*'",
                Files.isRegularFile(WHAT_THE_TAB_IS_ANSWERED));
        JsonElement kept = new Gson()
                .fromJson(
                        new String(Files.readAllBytes(WHAT_THE_TAB_IS_ANSWERED), StandardCharsets.UTF_8),
                        JsonElement.class);
        assertEquals(
                "the answers " + THE_CHECK + " gives the Robot tab are no longer shaped as the bench gives them;"
                        + " regenerate with -D" + SimTrace.REGENERATE + "=true --tests '*SimRobotTabTest*'",
                SimLaunchPageTest.shapeOf(new Gson().fromJson(answered, JsonElement.class)),
                SimLaunchPageTest.shapeOf(kept));
    }

    @Test
    public void theBrowserCheckSendsTheEditsItsAnswersWereMadeFrom() throws IOException {
        String check = new String(Files.readAllBytes(THE_CHECK), StandardCharsets.UTF_8);

        for (String sent : List.of(MOVED_WHEEL, TOO_FLAT, REWIRED, REVERSED, KEPT_BY_AN_OLDER_BUILD)) {
            assertTrue(THE_CHECK + " expects the tab to send " + sent, check.contains("'" + sent + "'"));
        }
    }
}
