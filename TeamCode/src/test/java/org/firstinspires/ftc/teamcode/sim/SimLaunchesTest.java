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
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.teamcode.sim.TestAutos.HangingAuto;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.ShotSetting;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimLaunchesTest {
    private static final String QUICK = "{\"pieces\": 0, \"longest\": 0.5}";

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private SimBench bench;

    @After
    public void stopBench() {
        if (bench != null) {
            bench.stop();
        }
    }

    private Path outputDir() {
        return folder.getRoot().toPath().resolve("sim");
    }

    private Router routes() {
        if (bench == null) {
            bench = new SimBench(
                    TeamRobot.REGINALD,
                    SimCatalog.of(HangingAuto.class),
                    null,
                    outputDir(),
                    SimBench.Waits.ofTheBench());
        }
        return bench.routes("ada");
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

    private static JsonObject named(JsonArray settings, String name) {
        for (JsonElement setting : settings) {
            if (setting.getAsJsonObject().get("name").getAsString().equals(name)) {
                return setting.getAsJsonObject();
            }
        }
        throw new AssertionError("no setting " + name + " in " + settings);
    }

    private int launch(String body) {
        Response launched = post("/launch", body);
        assertEquals(launched.body, 200, launched.status);
        return json(launched).get("id").getAsInt();
    }

    @Test
    public void everySettingIsListedWithItsGroupRangeDefaultAndWhatItIsNow() {
        JsonObject listing = json(get("/launch/settings"));

        assertEquals(
                ShotSetting.Group.values().length,
                listing.getAsJsonArray("groups").size());
        JsonArray settings = listing.getAsJsonArray("settings");
        assertEquals(ShotSetting.values().length, settings.size());
        for (ShotSetting setting : ShotSetting.values()) {
            JsonObject one = named(settings, setting.asked());
            assertEquals(setting.least(), one.get("least").getAsDouble(), 0);
            assertEquals(setting.most(), one.get("most").getAsDouble(), 0);
            assertEquals(setting.byDefault(), one.get("byDefault").getAsDouble(), 0);
            assertEquals(setting.byDefault(), one.get("value").getAsDouble(), 0);
            assertFalse(one.get("label").getAsString().isBlank());
            assertFalse(one.get("says").getAsString().isBlank());
            assertTrue(one.has("switch"));
        }
        assertEquals(
                "floor",
                named(settings, ShotSetting.FLOOR_GRIP.asked()).get("surface").getAsString());
        assertEquals(
                "grip",
                named(settings, ShotSetting.FLOOR_GRIP.asked()).get("property").getAsString());
        assertEquals(6, listing.getAsJsonArray("surfaces").size());
        assertEquals("", listing.get("forgot").getAsString());
    }

    @Test
    public void aPreviewSaysHowTheWheelsWillThrowWithoutLaunchingAnything() {
        JsonObject preview = json(post("/launch/preview", "{}"));

        double one = Math.PI * 3.78 * 900 / 60, two = Math.PI * 3.78 * 1200 / 60;
        assertEquals((one + two) / 2, preview.get("speed").getAsDouble(), 1e-6);
        assertEquals("backspin", preview.get("spinKind").getAsString());
        assertEquals(one, preview.getAsJsonArray("surfaces").get(0).getAsDouble(), 1e-6);
        assertEquals(
                900 * 28 / 60.0, preview.getAsJsonArray("ticksPerSecond").get(0).getAsDouble(), 1e-9);
        boolean blue = false;
        for (JsonElement aim : preview.getAsJsonArray("aims")) {
            JsonObject cell = aim.getAsJsonObject();
            if (cell.get("cell").getAsString().equals("Blue Cell (Scoring) <1>")) {
                blue = true;
                assertTrue(cell.get("upturned").getAsBoolean());
                assertEquals(180, Math.abs(cell.get("yaw").getAsDouble()), 1);
            }
        }
        assertTrue(blue);
        assertEquals(0, json(get("/launches")).getAsJsonArray("launches").size());
    }

    @Test
    public void aLaunchIsSimulatedListedAndThereToReplay() {
        int id = launch("{}");

        JsonArray listed = json(get("/launches")).getAsJsonArray("launches");
        assertEquals(1, listed.size());
        JsonObject summary = listed.get(0).getAsJsonObject();
        assertEquals(id, summary.get("id").getAsInt());
        assertEquals("scored", summary.getAsJsonObject("ending").get("kind").getAsString());
        assertTrue(summary.get("computeSeconds").getAsDouble() >= 0);

        JsonObject shot = json(get("/launches/" + id + "/shot"));
        assertTrue(shot.getAsJsonArray("frames").size() > 100);
        assertTrue(shot.getAsJsonArray("colliders").size() > 100);
        assertEquals("scored", shot.getAsJsonObject("ending").get("kind").getAsString());
        assertEquals(15, shot.getAsJsonArray("frames").get(0).getAsJsonArray().size());

        String page = get("/launches/").body;
        assertFalse(page, page.contains("__"));
        assertEquals("./#" + id, get("/launches/" + id).headers.get("Location"));
        assertEquals("../#" + id, get("/launches/" + id + "/").headers.get("Location"));
        String csv = get("/launches/" + id + "/frames.csv").body;
        assertTrue(csv, csv.startsWith("t,x,y,z,vx,vy,vz,wx,wy,wz,phase\n"));
        assertEquals(shot.getAsJsonArray("frames").size() + 1, csv.split("\n").length);
        assertEquals(404, get("/launches/" + (id + 1) + "/shot").status);
    }

    @Test
    public void aSettingThatIsOutOfRangeIsRefusedNamingIt() {
        Response refused = post("/launch", "{\"pitch\": 95}");

        assertEquals(400, refused.status);
        JsonObject said = json(refused);
        assertEquals("pitch", said.get("setting").getAsString());
        assertTrue(
                said.get("rule").getAsString(), said.get("rule").getAsString().contains("95"));
        assertEquals(
                "nope", json(post("/launch", "{\"nope\": 1}")).get("setting").getAsString());
        assertEquals(400, post("/launch", "{\"pitch\": \"high\"}").status);
        assertEquals(400, post("/launch", "not json").status);
        assertEquals(400, post("/launch/preview", "{\"pitch\": 95}").status);
        assertEquals(0, json(get("/launches")).getAsJsonArray("launches").size());
    }

    @Test
    public void aPollenThatWouldStartInsideTheFieldIsRefusedNamingWhatItIsIn() {
        Response refused = post("/launch", "{\"x\": 68.04, \"y\": 23.39, \"height\": 4.5}");

        assertEquals(400, refused.status);
        assertTrue(
                json(refused).get("rule").getAsString(),
                json(refused).get("rule").getAsString().contains("Flower"));
    }

    @Test
    public void onlyTheNewestThirtyLaunchesAreKept() {
        for (int i = 0; i < SimLaunches.MOST_KEPT + 1; i++) {
            launch(QUICK);
        }

        JsonArray listed = json(get("/launches")).getAsJsonArray("launches");
        assertEquals(SimLaunches.MOST_KEPT, listed.size());
        assertEquals(
                SimLaunches.MOST_KEPT + 1,
                listed.get(0).getAsJsonObject().get("id").getAsInt());
        assertEquals(404, get("/launches/1/shot").status);
    }

    private static final class HeldClock implements Clock {
        private final CountDownLatch reading = new CountDownLatch(1);
        private final CountDownLatch go = new CountDownLatch(1);
        private boolean held;

        @Override
        public synchronized long nanos() {
            if (!held) {
                held = true;
                reading.countDown();
                try {
                    go.await();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            return 0;
        }

        @Override
        public long millisSinceEpoch() {
            return 1_700_000_000_000L;
        }

        @Override
        public void sleep(double seconds) {}
    }

    @Test
    public void aSecondLaunchWhileOneIsBeingSimulatedIsRefused() throws Exception {
        HeldClock clock = new HeldClock();
        bench = new SimBench(
                TeamRobot.REGINALD,
                SimSources.ofThisClasspath(SimCatalog.of(HangingAuto.class)),
                outputDir(),
                SimBench.Waits.ofTheBench(),
                new JvmChild(),
                clock);
        AtomicReference<Response> first = new AtomicReference<>();
        Thread launching = new Thread(() -> first.set(post("/launch", QUICK)));
        launching.start();
        clock.reading.await();

        Response second = post("/launch", QUICK);
        clock.go.countDown();
        launching.join();

        assertEquals(409, second.status);
        assertEquals(200, first.get().status);
        assertEquals(1, json(get("/launches")).getAsJsonArray("launches").size());
    }

    @Test
    public void theSettingsLastLaunchedAreWhatTheFormComesBackWith() {
        launch("{\"pitch\": 60, \"pieces\": 0, \"longest\": 0.5}");
        bench.stop();
        bench = null;

        JsonArray settings = json(get("/launch/settings")).getAsJsonArray("settings");
        assertEquals(60, named(settings, "pitch").get("value").getAsDouble(), 0);
        assertEquals(0, named(settings, "pieces").get("value").getAsDouble(), 0);
        assertEquals(
                ShotSetting.YAW.byDefault(), named(settings, "yaw").get("value").getAsDouble(), 0);
    }

    @Test
    public void aRememberedSettingThatNoLongerFitsIsForgottenAndSaidSo() throws Exception {
        Files.createDirectories(outputDir());
        Files.write(
                outputDir().resolve(SimLaunches.REMEMBERED_FILE),
                "{\"pitch\": 500, \"nope\": 3, \"x\": 10}".getBytes(StandardCharsets.UTF_8));

        JsonObject listing = json(get("/launch/settings"));
        assertEquals(
                10, named(listing.getAsJsonArray("settings"), "x").get("value").getAsDouble(), 0);
        assertEquals(
                ShotSetting.PITCH.byDefault(),
                named(listing.getAsJsonArray("settings"), "pitch").get("value").getAsDouble(),
                0);
        String forgot = listing.get("forgot").getAsString();
        assertTrue(forgot, forgot.contains("pitch") && forgot.contains("nope"));
        assertEquals(200, get("/status").status);
    }

    @Test
    public void aRememberedFileThatIsNotSettingsLeavesTheBenchWorking() throws Exception {
        Files.createDirectories(outputDir());
        Files.write(outputDir().resolve(SimLaunches.REMEMBERED_FILE), "[1, 2".getBytes(StandardCharsets.UTF_8));

        assertEquals(200, get("/status").status);
        JsonObject listing = json(get("/launch/settings"));
        assertFalse(listing.get("forgot").getAsString().isBlank());
        assertEquals(
                ShotSetting.X.byDefault(),
                named(listing.getAsJsonArray("settings"), "x").get("value").getAsDouble(),
                0);
    }
}
