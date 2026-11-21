package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimFeedsTest {
    private static final String A_SECOND_AND_A_HALF = "{\"pollen\": 1, \"run\": 1.5}";
    private static final double POLLEN_AT_90_X_IN = 1.7439;
    private static final double POLLEN_AT_90_Y_IN = 1.849;
    private static final double POLLEN_ON_THE_FLOOR_Z_IN = 3.0325;

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private final List<SimBench> benches = new ArrayList<>();
    private Router routes;

    @After
    public void stopBenches() {
        for (SimBench bench : benches) {
            bench.stop();
        }
    }

    private Router restarted() {
        SimBench bench = new SimBench(
                SimCatalog.of(HangingAuto.class),
                null,
                folder.getRoot().toPath().resolve("sim"),
                SimBench.Waits.ofTheBench());
        benches.add(bench);
        routes = bench.routes("ada");
        return routes;
    }

    private Router routes() {
        return routes == null ? restarted() : routes;
    }

    private Response get(String path) {
        return routes().handle(TinyHttpServer.Request.of("GET", path, ""));
    }

    private Response post(String path, String body) {
        return routes().handle(TinyHttpServer.Request.of("POST", path, body));
    }

    private static JsonObject json(Response response) {
        return new Gson().fromJson(response.body, JsonObject.class);
    }

    private int feed(String body) {
        Response fed = post("/feed", body);
        assertEquals(fed.body, 200, fed.status);
        return json(fed).get("id").getAsInt();
    }

    private static JsonObject named(JsonArray settings, String name) {
        for (JsonElement setting : settings) {
            if (setting.getAsJsonObject().get("name").getAsString().equals(name)) {
                return setting.getAsJsonObject();
            }
        }
        throw new AssertionError("no setting " + name + " in " + settings);
    }

    private List<JsonObject> frames(int id) {
        List<JsonObject> frames = new ArrayList<>();
        JsonObject first = json(get("/feeds/" + id + "/frames/0"));
        int chunks = first.get("chunks").getAsInt();
        for (int chunk = 0; chunk < chunks; chunk++) {
            Response sent = get("/feeds/" + id + "/frames/" + chunk);
            assertEquals(sent.body, 200, sent.status);
            for (JsonElement frame : json(sent).getAsJsonArray("frames")) {
                frames.add(frame.getAsJsonObject());
            }
        }
        return frames;
    }

    @Test
    public void theDefaultsGiveAFeedThatThrowsBothPollen() {
        JsonObject summary = json(get("/feeds/" + feed("{}") + "/feed")).getAsJsonObject("summary");

        assertEquals(2, summary.get("thrown").getAsInt());
        assertEquals("all left", summary.getAsJsonObject("ending").get("kind").getAsString());
    }

    @Test
    public void aSettingOutOfRangeIsRefusedNamingItsRuleAndSoIsARampLoweredBeforeItIsRaised() {
        Response fast = post("/feed", "{\"spinner_speed\": 9}");
        Response early = post("/feed", "{\"ramp_raised_at\": 1, \"ramp_lowered_at\": 0.5}");
        Response unknown = post("/feed", "{\"speed\": 1}");

        assertEquals(400, fast.status);
        assertEquals("spinner_speed", json(fast).get("setting").getAsString());
        assertEquals(
                "Spinner speed is from 0.2 to 3.0 rev/s, not 9.0",
                json(fast).get("rule").getAsString());
        assertEquals(400, early.status);
        assertEquals(
                "a ramp is raised before it is lowered, not raised at 1.0 s and lowered at 0.5 s",
                json(early).get("rule").getAsString());
        assertEquals(400, unknown.status);
        assertEquals("speed", json(unknown).get("unknown").getAsString());
        assertEquals("{\"feeds\":[]}", get("/feeds").body);
    }

    @Test
    public void theLastSettingsFedAreRememberedAcrossARestartOfTheServer() {
        feed("{\"pollen\": 1, \"run\": 1.5, \"back_sill\": 43.12}");

        restarted();
        JsonObject listing = json(get("/feed/settings"));

        JsonArray settings = listing.getAsJsonArray("settings");
        assertEquals(1, named(settings, "pollen").get("value").getAsDouble(), 0);
        assertEquals(43.12, named(settings, "back_sill").get("value").getAsDouble(), 0);
        assertEquals(1.5, named(settings, "run").get("value").getAsDouble(), 0);
        assertEquals(35, named(settings, "ramp_up").get("value").getAsDouble(), 0);
        assertEquals("", listing.get("forgot").getAsString());
    }

    @Test
    public void aFeedsSummaryNamesHowManyWereThrownAndWhenAndEarlierFeedsAreKept() {
        int first = feed(A_SECOND_AND_A_HALF);
        int second = feed("{}");

        JsonArray kept = json(get("/feeds")).getAsJsonArray("feeds");
        assertEquals(2, kept.size());
        assertEquals(second, kept.get(0).getAsJsonObject().get("id").getAsInt());
        assertEquals(first, kept.get(1).getAsJsonObject().get("id").getAsInt());
        JsonObject summary = kept.get(0).getAsJsonObject();
        JsonArray thrown = summary.getAsJsonArray("thrownAt");
        assertEquals(2, thrown.size());
        assertEquals(0.416, thrown.get(0).getAsDouble(), 0.01);
        assertEquals(0.987, thrown.get(1).getAsDouble(), 0.01);
        assertEquals(1, kept.get(1).getAsJsonObject().get("thrown").getAsInt());
    }

    @Test
    public void aFeedsFramesReplayTheSpinnerAndRampAnglesAndEachPollensPlace() {
        List<JsonObject> frames = frames(feed("{}"));

        JsonObject start = frames.get(0);
        assertEquals(0, start.get("t").getAsDouble(), 0);
        assertEquals(0, start.getAsJsonArray("spinner").get(0).getAsDouble(), 0);
        assertEquals(0, start.getAsJsonArray("ramp").get(0).getAsDouble(), 0);
        JsonArray pollen = start.getAsJsonArray("pollen");
        assertEquals(2, pollen.size());
        JsonObject first = pollen.get(0).getAsJsonObject();
        assertEquals("Pollen 1", first.get("name").getAsString());
        assertEquals(POLLEN_AT_90_X_IN, first.getAsJsonArray("at").get(0).getAsDouble(), 1e-3);
        assertEquals(POLLEN_AT_90_Y_IN, first.getAsJsonArray("at").get(1).getAsDouble(), 1e-3);
        assertEquals(POLLEN_ON_THE_FLOOR_Z_IN, first.getAsJsonArray("at").get(2).getAsDouble(), 1e-3);
        assertEquals(4, first.getAsJsonArray("turn").size());
        JsonObject half = frames.get(125);
        assertEquals(0.5, half.get("t").getAsDouble(), 1e-9);
        assertEquals(35, half.getAsJsonArray("ramp").get(0).getAsDouble(), 1e-9);
        assertEquals(180, half.getAsJsonArray("spinner").get(0).getAsDouble(), 10);
        assertEquals(0, frames.get(frames.size() - 1).getAsJsonArray("pollen").size());
    }

    @Test
    public void theViewerIsServedAtFeedsAndAFeedsNumberLeadsToItsPlaceInTheViewer() {
        int id = feed(A_SECOND_AND_A_HALF);

        Response viewer = get("/feeds/");
        assertEquals(200, viewer.status);
        assertTrue(viewer.body, viewer.body.contains("<title>Feeder Sim</title>"));
        assertEquals(308, get("/feeds/" + id).status);
        assertEquals("./#" + id, get("/feeds/" + id).headers.get("Location"));
        assertEquals("../#" + id, get("/feeds/" + id + "/").headers.get("Location"));
        assertEquals(404, get("/feeds/" + (id + 1) + "/feed").status);
    }

    @Test
    public void aFeedCarriesTheDrawingOfTheFeederAndTheLauncherInInches() {
        JsonObject drawing =
                json(get("/feeds/" + feed(A_SECOND_AND_A_HALF) + "/feed")).getAsJsonObject("drawing");

        JsonObject feeder = drawing.getAsJsonObject("feeder");
        assertEquals(1.18268, feeder.getAsJsonArray("origin").get(0).getAsDouble(), 1e-4);
        assertEquals(0.95901, feeder.getAsJsonArray("z").get(2).getAsDouble(), 1e-4);
        assertEquals(-1, feeder.getAsJsonArray("x").get(1).getAsDouble(), 1e-4);
        assertEquals(3.23898, feeder.getAsJsonObject("inches").get("wallRadius").getAsDouble(), 1e-4);
        JsonObject launcher = drawing.getAsJsonObject("launcher");
        assertEquals(7.08661, launcher.getAsJsonArray("origin").get(2).getAsDouble(), 1e-4);
        assertEquals(
                -64.8, launcher.getAsJsonObject("inches").get("hoodFromDegrees").getAsDouble(), 0);
    }
}
