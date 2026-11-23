package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimOverridesTest {
    private static final String MOVED_WHEEL = "{\"constants\": {\"parallel_dead_wheel_y_in\": -1.5}}";

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private final List<SimBench> benches = new ArrayList<>();

    @After
    public void stopBenches() {
        benches.forEach(SimBench::stop);
    }

    private Path outputDir() {
        return folder.getRoot().toPath().resolve("sim");
    }

    private Path kept() {
        return outputDir().resolve(SimOverrides.FILE);
    }

    private SimBench aBench() {
        SimBench bench = new SimBench(SimCatalog.of(HangingAuto.class), null, outputDir(), SimBench.Waits.ofTheBench());
        benches.add(bench);
        return bench;
    }

    private static Response get(SimBench bench) {
        return bench.routes("ada").handle(TinyHttpServer.Request.of("GET", "/robot", ""));
    }

    private static Response put(SimBench bench, String edits) {
        return bench.routes("ada").handle(TinyHttpServer.Request.of("PUT", "/robot", edits));
    }

    private static JsonObject json(Response response) {
        return new Gson().fromJson(response.body, JsonObject.class);
    }

    private static JsonObject constant(JsonObject listing, String name) {
        for (JsonElement one : listing.getAsJsonArray("constants")) {
            if (one.getAsJsonObject().get("name").getAsString().equals(name)) {
                return one.getAsJsonObject();
            }
        }
        throw new AssertionError("no constant " + name + " in " + listing);
    }

    private static JsonObject direction(JsonObject listing, String name) {
        for (JsonElement one : listing.getAsJsonArray("directions")) {
            if (one.getAsJsonObject().get("name").getAsString().equals(name)) {
                return one.getAsJsonObject();
            }
        }
        throw new AssertionError("no direction " + name + " in " + listing);
    }

    private static List<String> notices(JsonObject listing) {
        List<String> said = new ArrayList<>();
        for (JsonElement notice : listing.getAsJsonArray("notices")) {
            said.add(notice.getAsString());
        }
        return said;
    }

    private void keep(String text) throws Exception {
        Files.createDirectories(outputDir());
        Files.write(kept(), text.getBytes(StandardCharsets.UTF_8));
    }

    private String keptText() throws Exception {
        return new String(Files.readAllBytes(kept()), StandardCharsets.UTF_8);
    }

    @Test
    public void withNothingKeptTheRobotIsTheDefaultsAndRunsAreMadeOnThem() {
        SimBench bench = aBench();

        JsonObject listing = json(get(bench));

        assertEquals(RobotDescription.reginalds(), bench.description());
        assertEquals(11, listing.getAsJsonArray("configuration").size());
        assertEquals(
                Constant.values().length, listing.getAsJsonArray("constants").size());
        assertEquals(new JsonObject(), listing.getAsJsonObject("edits"));
        assertEquals(List.of(), notices(listing));
        assertEquals("as built", listing.get("described").getAsString());
        assertFalse(Files.exists(kept()));
    }

    @Test
    public void everyValueIsListedWithItsGroupLabelUnitRangeSentenceDefaultAndValue() {
        JsonObject wheel = constant(json(get(aBench())), "parallel_dead_wheel_y_in");

        assertEquals("dead_wheels", wheel.get("group").getAsString());
        assertEquals("Parallel dead wheel, left of centre", wheel.get("label").getAsString());
        assertEquals("in", wheel.get("unit").getAsString());
        assertEquals(-9, wheel.get("least").getAsDouble(), 0);
        assertEquals(9, wheel.get("most").getAsDouble(), 0);
        assertTrue(wheel.get("says").getAsString().startsWith("How far left of the robot's centre"));
        assertEquals(-0.25, wheel.get("byDefault").getAsDouble(), 0);
        assertEquals(-0.25, wheel.get("value").getAsDouble(), 0);
    }

    @Test
    public void theConfigurationIsListedByDeviceWithWhatEachMotorPortsEncoderReads() {
        JsonArray devices = json(get(aBench())).getAsJsonArray("configuration");

        JsonObject rightBack = devices.get(3).getAsJsonObject();
        assertEquals("rightBack", rightBack.get("device").getAsString());
        assertEquals("motor", rightBack.get("kind").getAsString());
        assertEquals(
                "REV Robotics UltraPlanetary HD Hex Motor",
                rightBack.getAsJsonObject("value").get("type").getAsString());
        assertEquals(
                "parallel_dead_wheel",
                rightBack.getAsJsonObject("value").get("encoder").getAsString());
        JsonObject spinner = devices.get(8).getAsJsonObject();
        assertEquals("continuous_servo", spinner.get("kind").getAsString());
        assertFalse(spinner.getAsJsonObject("value").has("encoder"));
    }

    @Test
    public void aSavedEditIsWhatTheNextRunIsMadeOnAndOutlivesTheSimulator() throws Exception {
        SimBench bench = aBench();

        JsonObject saved = json(put(bench, MOVED_WHEEL));

        assertEquals(
                -1.5, constant(saved, "parallel_dead_wheel_y_in").get("value").getAsDouble(), 0);
        assertEquals(-1.5, bench.description().constants().value(Constant.PARALLEL_DEAD_WHEEL_Y_IN), 0);
        assertEquals(
                "Parallel dead wheel, left of centre -1.5 in (built -0.25)",
                saved.get("described").getAsString());
        assertEquals("{\"constants\":{\"parallel_dead_wheel_y_in\":-1.5}}", keptText());
        assertEquals(-1.5, aBench().description().constants().value(Constant.PARALLEL_DEAD_WHEEL_Y_IN), 0);
    }

    @Test
    public void savingNoEditsPutsEveryValueBackToItsDefault() throws Exception {
        SimBench bench = aBench();
        put(bench, MOVED_WHEEL);

        Response reset = put(bench, "{}");

        assertEquals(200, reset.status);
        assertEquals(RobotDescription.reginalds(), bench.description());
        assertEquals("as built", json(reset).get("described").getAsString());
        assertEquals("{}", keptText());
    }

    @Test
    public void aSetTheCoreRefusesIsRefusedNamingTheRuleAndNothingIsSaved() throws Exception {
        SimBench bench = aBench();
        put(bench, MOVED_WHEEL);

        Response refused = put(bench, "{\"constants\": {\"flattest_volts\": 14}}");

        assertEquals(400, refused.status);
        assertEquals(
                "Flattest battery may not be more than freshest battery, not 14.0 V against 13.8 V",
                json(refused).get("rule").getAsString());
        assertEquals("{\"constants\":{\"parallel_dead_wheel_y_in\":-1.5}}", keptText());
        assertEquals(-1.5, bench.description().constants().value(Constant.PARALLEL_DEAD_WHEEL_Y_IN), 0);
        assertEquals(12, bench.description().constants().value(Constant.FLATTEST_VOLTS), 0);
    }

    @Test
    public void aNameTheDescriptionDoesNotHaveIsRefusedWhenSaved() {
        Response refused = put(aBench(), "{\"constants\": {\"launch_throw\": 1}}");

        assertEquals(400, refused.status);
        assertEquals(
                "the robot description has no value named 'launch_throw'",
                json(refused).get("rule").getAsString());
    }

    @Test
    public void theHubAndTheConfigurationAreEditedAndKeptAsTheyAreAsked() throws Exception {
        SimBench bench = aBench();

        JsonObject saved = json(put(
                bench,
                "{\"hub\": {\"logo\": \"LEFT\"}, \"configuration\": {\"rightBack\": {\"encoder\": \"nothing\"},"
                        + " \"ramp\": {\"name\": \"lift\"}}}"));

        assertEquals("LEFT", saved.getAsJsonObject("hub").get("logo").getAsString());
        assertEquals(
                "RIGHT",
                saved.getAsJsonObject("hub")
                        .getAsJsonObject("byDefault")
                        .get("logo")
                        .getAsString());
        JsonObject rightBack = saved.getAsJsonArray("configuration").get(3).getAsJsonObject();
        assertEquals(
                "nothing", rightBack.getAsJsonObject("value").get("encoder").getAsString());
        assertEquals(
                "parallel_dead_wheel",
                rightBack.getAsJsonObject("byDefault").get("encoder").getAsString());
        assertEquals(
                "{\"hub\":{\"logo\":\"LEFT\"},\"configuration\":{\"rightBack\":{\"encoder\":\"nothing\"},"
                        + "\"ramp\":{\"name\":\"lift\"}}}",
                keptText());
        assertTrue(
                saved.get("described").getAsString(),
                saved.get("described").getAsString().contains("hub logo LEFT, USB UP (built logo RIGHT, USB UP)"));
    }

    @Test
    public void eachMechanismIsListedWithWhatPositiveDriveDoesEachWayAndItsDefault() {
        JsonObject intake = direction(json(get(aBench())), "intake");

        assertEquals("intake", intake.get("name").getAsString());
        assertEquals("Intake", intake.get("label").getAsString());
        assertEquals("takes_in", intake.get("value").getAsString());
        assertEquals("takes_in", intake.get("byDefault").getAsString());
        JsonArray ways = intake.getAsJsonArray("ways");
        assertEquals("takes_in", ways.get(0).getAsJsonObject().get("name").getAsString());
        assertEquals(
                "Positive power takes pollen in.",
                ways.get(0).getAsJsonObject().get("says").getAsString());
        assertEquals("pushes_out", ways.get(1).getAsJsonObject().get("name").getAsString());
        assertEquals(
                "Positive power pushes pollen out.",
                ways.get(1).getAsJsonObject().get("says").getAsString());
    }

    @Test
    public void eachDriveMotorIsListedWithWhichWayPositivePowerDrivesItsWheel() {
        JsonObject listing = json(get(aBench()));
        JsonObject leftFront = direction(listing, "left_front");
        JsonObject rightBack = direction(listing, "right_back");

        assertEquals("Left front", leftFront.get("label").getAsString());
        assertEquals("drives_forward", leftFront.get("byDefault").getAsString());
        assertEquals(
                "Positive power drives its wheel backward.",
                leftFront
                        .getAsJsonArray("ways")
                        .get(1)
                        .getAsJsonObject()
                        .get("says")
                        .getAsString());
        assertEquals("drives_backward", rightBack.get("value").getAsString());
        assertEquals(
                "left_front",
                listing.getAsJsonArray("directions")
                        .get(0)
                        .getAsJsonObject()
                        .get("name")
                        .getAsString());
    }

    @Test
    public void aReversedMechanismIsWhatTheNextRunIsMadeOnAndIsKept() throws Exception {
        SimBench bench = aBench();

        JsonObject saved = json(put(bench, "{\"directions\": {\"intake\": \"pushes_out\"}}"));

        assertTrue(bench.description().directions().reversed(Mechanism.INTAKE));
        assertEquals("pushes_out", direction(saved, "intake").get("value").getAsString());
        assertEquals("{\"directions\":{\"intake\":\"pushes_out\"}}", keptText());
        assertTrue(
                saved.get("described").getAsString(),
                saved.get("described").getAsString().contains("intake pushes out on positive power (built takes in)"));
    }

    @Test
    public void aKeptWayAMechanismDoesNotHaveGoesBackToItsDefaultAndTheTabSaysSo() throws Exception {
        keep("{\"directions\": {\"intake\": \"spins\"}}");

        SimBench bench = aBench();

        assertFalse(bench.description().directions().reversed(Mechanism.INTAKE));
        assertEquals(
                List.of("Intake's direction went back to its default:"
                        + " the intake takes_in or pushes_out on positive power, not 'spins'."),
                notices(json(get(bench))));
    }

    @Test
    public void aKeptValuePastItsRangeGoesBackToItsDefaultAndTheTabSaysSo() throws Exception {
        keep("{\"constants\": {\"mass_kg\": 40, \"track_width_in\": 14}}");

        SimBench bench = aBench();
        JsonObject listing = json(get(bench));

        assertEquals(15, bench.description().constants().value(Constant.MASS_KG), 0);
        assertEquals(14, bench.description().constants().value(Constant.TRACK_WIDTH_IN), 0);
        assertEquals(
                List.of("Mass went back to its default: Mass is from 1.0 to 30.0 kg, not 40.0."), notices(listing));
    }

    @Test
    public void aKeptNameTheDescriptionNoLongerHasIsDroppedOnTheNextSaveAndTheTabSaysSo() throws Exception {
        keep("{\"constants\": {\"launch_throw\": 1.2, \"mass_kg\": 16}}");
        SimBench bench = aBench();

        List<String> said = notices(json(get(bench)));
        String untilTheNextSave = keptText();
        double recalled = bench.description().constants().value(Constant.MASS_KG);
        JsonObject saved = json(put(bench, "{\"constants\": {\"mass_kg\": 17}}"));

        assertEquals(
                List.of("launch_throw is dropped on the next save: the robot description has no such value now."),
                said);
        assertTrue(untilTheNextSave.contains("launch_throw"));
        assertEquals(16, recalled, 0);
        assertEquals("{\"constants\":{\"mass_kg\":17.0}}", keptText());
        assertEquals(List.of(), notices(saved));
    }

    @Test
    public void aKeptFileThatIsNotJsonIsTheDefaultsAndTheTabSaysWhy() throws Exception {
        keep("{not json");

        SimBench bench = aBench();
        List<String> said = notices(json(get(bench)));

        assertEquals(RobotDescription.reginalds(), bench.description());
        assertEquals(1, said.size());
        assertTrue(said.get(0), said.get(0).startsWith("every edit went back to its default: "));
        assertTrue(said.get(0), said.get(0).contains("is not JSON"));
    }
}
