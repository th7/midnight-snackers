package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation.State;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class SimReplayPageTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    @Test
    public void writesASelfContainedPageCarryingEveryRecordedTick() throws IOException {
        SimRecording recording = new SimRecording("SquareAuto");
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#4CAF50").strokePolyline(new double[] {0, 24}, new double[] {0, 24});
        packet.put("xError", 0.25);
        recording.add(
                SimRecording.Tick.at(0.0, new Pose2d(0, 0, 0), "1. waitFor 5.0", new double[] {0, 0, 0, 0}, List.of())
                        .tick());
        recording.add(SimRecording.Tick.at(
                        0.5,
                        new Pose2d(12.5, -3, Math.PI / 2),
                        "2. driveTo 24, 0, 0",
                        new double[] {1, 0.75, -0.5, 0.25},
                        List.of(packet))
                .tick());
        recording.add(SimRecording.Tick.at(
                        1.0 / 3,
                        new Pose2d(1.0 / 3, 0, 0),
                        "3. rounding",
                        new double[] {1, 0.75, -0.5, 0.25},
                        List.of(packet))
                .tick());
        recording.finish("done");
        Path page = folder.getRoot().toPath().resolve("SquareAuto.html");

        SimReplayPage.write(recording, page);

        String html = new String(Files.readAllBytes(page), StandardCharsets.UTF_8);
        assertTrue("has a canvas", html.contains("<canvas"));
        assertTrue("titled after the recording", html.contains("<title>SquareAuto"));
        assertTrue("carries the step names", html.contains("2. driveTo 24, 0, 0"));
        assertTrue("carries the true pose", html.contains("12.5"));
        assertTrue("rounds numbers to three decimals", html.contains("0.333"));
        assertFalse("does not carry full precision", html.contains("0.3333"));
        assertTrue("carries the dashboard drawing ops", html.contains("POLYLINE"));
        assertTrue("carries the dashboard data", html.contains("xError"));
        assertTrue("carries the outcome", html.contains("done"));
        assertTrue("says what kind of run it was", html.contains("\"kind\":\"auto\""));
        assertFalse("loads nothing from the network", html.matches("(?s).*(src|href)=\"http.*"));
    }

    @Test
    public void thePageDrawsTheFieldAndTheRobotAtTheSimulatorsSizes() {
        String html = SimReplayPage.page(new SimRecording("SquareAuto"), false);

        assertTrue(html, html.contains("FIELD_IN = " + SimPlacement.FIELD_SIZE_IN));
        assertTrue(html, html.contains("ROBOT_IN = " + SimPlacement.ROBOT_SIZE_IN));
        assertTrue(html, html.contains("WALL_IN = " + SimPlacement.WALL_HEIGHT_IN));
        assertFalse(
                "no placeholder is left behind",
                html.contains("__FIELD_IN__") || html.contains("__ROBOT_IN__") || html.contains("__WALL_IN__"));
    }

    @Test
    public void thePageCarriesTheFieldModelTheSimulatorCollides() {
        String html = SimReplayPage.page(new SimRecording("SquareAuto"), false);

        assertTrue(html, html.contains("FIELD = " + new Gson().toJson(SimRobot.FIELD.json())));
        assertTrue(html, html.contains("Flower Assembly"));
        assertFalse("no placeholder is left behind", html.contains("__FIELD__"));
    }

    @Test
    public void thePageDrawsEachHiveWhereItLeans() {
        SimRecording recording = new SimRecording("TipAuto");
        SimField.Hive blue = SimRobot.FIELD.hive("Blue Hive <1>");
        recording.add(
                SimRecording.Tick.at(0.0, new Pose2d(0, 0, 0), "1. fill the hive", new double[] {0, 0, 0, 0}, List.of())
                        .tilted(Map.of(blue.alliance, -blue.tilt))
                        .tick());
        recording.finish("done");

        String html = SimReplayPage.page(recording, false);

        assertTrue("the hive's tilt reaches the page", html.contains("\"tilt\":{\"Blue\":" + -blue.tilt));
        assertTrue("and the page draws the hives with it", html.contains("hiveFaces(hive,"));
        assertTrue("from the model's own pivot and cells", html.contains("hive.pivot") && html.contains("hive.cells"));
    }

    @Test
    public void thePageDrawsAHiveThatTipsBackWhereTheFieldWasSetUpAgain() throws Exception {
        SimField.Hive blue = SimRobot.FIELD.hive("Blue Hive <1>");

        String ticks = "[{}, {\"tilt\":{\"Blue\":" + -blue.tilt + "}}, {}]";

        double[] leaning = tiltOnThePage(ticks, blue);

        assertEquals("as the field was set up", blue.tilt, leaning[0], 0);
        assertEquals("tipped", -blue.tilt, leaning[1], 0);
        assertEquals("and tipped back, not left tipped", blue.tilt, leaning[2], 0);
    }

    private double[] tiltOnThePage(String ticks, SimField.Hive hive) throws Exception {
        String html = SimReplayPage.page(new SimRecording("TiltRule"), false);
        Matcher rule = Pattern.compile("\n  function tiltAt\\(hive, upTo\\) \\{.*?\n  \\}", Pattern.DOTALL)
                .matcher(html);
        assertTrue("the page works each hive's tilt out in tiltAt(hive, upTo)", rule.find());
        Path script = folder.newFile("tiltAt.js").toPath();
        Files.write(
                script,
                ("const ticks = " + ticks + ";\n"
                                + "const hive = "
                                + new Gson().toJson(Map.of("alliance", hive.alliance, "tilt", hive.tilt)) + ";\n"
                                + rule.group() + "\n"
                                + "console.log(JSON.stringify(ticks.map((_, i) => tiltAt(hive, i))));\n")
                        .getBytes(StandardCharsets.UTF_8));
        Process node = new ProcessBuilder("node", script.toString())
                .redirectErrorStream(true)
                .start();
        String out = new String(node.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
        assertEquals("node ran the page's rule: " + out, 0, node.waitFor());
        return new Gson().fromJson(out.trim(), double[].class);
    }

    @Test
    public void aTickCarriesWhereTheLoosePiecesAre() {
        SimRecording recording = new SimRecording("PushAuto");
        recording.add(SimRecording.Tick.at(0.0, new Pose2d(0, 0, 0), "1. push", new double[] {1, 1, 1, 1}, List.of())
                .withBalls(new double[][] {{69.27, -60.6, 1.39}, {10.5, -40.1234, 20}, null}, 2)
                .scoring(java.util.Map.of("Blue", 1))
                .tick());
        recording.add(SimRecording.Tick.at(0.5, new Pose2d(1, 0, 0), "1. push", new double[] {1, 1, 1, 1}, List.of())
                .tick());
        recording.finish("done");

        String html = SimReplayPage.page(recording, false);

        assertTrue(html, html.contains("\"pieces\":[[69.27,-60.6,1.39],[10.5,-40.123,20.0],null]"));
        assertTrue(
                "carries what the robot holds and the score",
                html.contains("\"held\":2") && html.contains("\"scored\":{\"Blue\":1}"));
        assertTrue("shows the balls in the side panel", html.contains("id=\"balls\""));
        assertEquals(
                "a tick without them carries no pieces key",
                1,
                html.split("\"pieces\"", -1).length - 1 - templateMentions("\"pieces\""));
    }

    @Test
    public void thePlacementPageCarriesTheStartPoseAndTheInputsToMoveIt() {
        String html = SimReplayPage.placement("SquareAuto", "auto", new Pose2d(12.5, -3, 0.5));

        assertTrue(html, html.contains("<canvas"));
        assertTrue(html, html.contains("\"name\":\"SquareAuto\""));
        assertTrue(html, html.contains("\"placing\":{\"x\":12.5,\"y\":-3.0,\"heading\":0.5}"));
        for (String id : List.of("start-x", "start-y", "start-heading")) {
            assertTrue(id, html.contains("id=\"" + id + "\""));
        }
        assertTrue("saves the pose to the bench", html.contains("'start?opmode='"));
        assertTrue(html, html.contains("FIELD_IN = " + SimPlacement.FIELD_SIZE_IN));
        assertFalse("loads nothing from the network", html.matches("(?s).*(src|href)=\"http.*"));
        assertFalse(
                "a replay is not a placement",
                SimReplayPage.page(new SimRecording("SquareAuto"), false).contains("\"placing\":{"));
    }

    @Test
    public void aTeleOpPageCarriesTheDriversInputsTickByTick() {
        SimRecording recording = new SimRecording("StickTeleOp", "teleop");
        State driving =
                State.fromJson(new Gson().fromJson("{\"cross\": true, \"left_stick_y\": -1}", JsonObject.class));
        recording.add(SimRecording.Tick.at(0.0, new Pose2d(0, 0, 0), "", new double[] {0, 0, 0, 0}, List.of())
                .drivenBy(State.NEUTRAL, State.NEUTRAL)
                .tick());
        recording.add(SimRecording.Tick.at(0.5, new Pose2d(3, 0, 0), "", new double[] {1, 1, 1, 1}, List.of())
                .drivenBy(driving, State.NEUTRAL)
                .tick());
        recording.finish("stopped");

        String html = SimReplayPage.page(recording, false);

        assertTrue(html, html.contains("\"kind\":\"teleop\""));
        assertTrue(html, html.contains("\"gamepads\":{\"1\":{\"cross\":true,\"left_stick_y\":-1.0}}"));
        assertFalse("a neutral gamepad is not carried", html.contains("\"2\":{}"));
        assertEquals(
                "only the ticks with input carry a gamepads key",
                1,
                html.split("\"gamepads\"", -1).length - 1 - templateMentions("\"gamepads\""));
    }

    @Test
    public void theControllerOffersEveryGamepadInputWithItsOwnKeyboardShortcut() {
        String html = SimReplayPage.page(new SimRecording("StickTeleOp", "teleop"), true);

        assertTrue(html, html.contains("id=\"controller\""));
        Set<String> keys = new HashSet<>();
        List<String> inputs = new ArrayList<>(State.BUTTONS);
        inputs.addAll(State.TRIGGERS);
        for (String input : inputs) {
            Matcher clickable = Pattern.compile("data-button=\"" + input + "\"[^>]*data-key=\"([A-Za-z0-9]+)\"")
                    .matcher(html);
            assertTrue(input + " is clickable and has a keyboard shortcut", clickable.find());
            assertTrue(input + " shares its key " + clickable.group(1), keys.add(clickable.group(1)));
        }
        for (String stick : List.of("left", "right")) {
            Matcher draggable = Pattern.compile("data-stick=\"" + stick
                            + "\"[^>]*data-keys=\"([A-Za-z0-9]+) ([A-Za-z0-9]+) ([A-Za-z0-9]+) ([A-Za-z0-9]+)\"")
                    .matcher(html);
            assertTrue(stick + " stick is draggable and has four keys, up down left right", draggable.find());
            for (int i = 1; i <= 4; i++) {
                assertTrue(stick + " stick shares its key " + draggable.group(i), keys.add(draggable.group(i)));
            }
        }
        for (String gamepad : List.of("1", "2")) {
            Matcher selectable = Pattern.compile("data-gamepad=\"" + gamepad + "\"[^>]*data-key=\"([A-Za-z0-9]+)\"")
                    .matcher(html);
            assertTrue("gamepad " + gamepad + " can be selected by key", selectable.find());
            assertTrue(keys.add(selectable.group(1)));
        }
        assertTrue("Stop is a control, not an input", html.contains("id=\"stop\""));
    }

    @Test
    public void aRunKnownOnlyByItsChildsLinesIsTheSamePageAsTheRecordingItCameFrom() {
        SimRecording recording = new SimRecording("StickTeleOp", "teleop");
        recording.add(SimRecording.Tick.at(0.0, new Pose2d(0, 0, 0), "", new double[] {0, 0, 0, 0}, List.of())
                .drivenBy(State.NEUTRAL, State.NEUTRAL)
                .tick());
        recording.add(SimRecording.Tick.at(0.5, new Pose2d(1.0 / 3, 0, 0), "", new double[] {1, 1, 1, 1}, List.of())
                .drivenBy(State.NEUTRAL, State.NEUTRAL)
                .tick());
        recording.finish("stopped");
        JsonArray streamed = new JsonArray();
        String[] outcome = {null};
        SimRunStream.Listener parent = new SimRunStream.Listener() {
            @Override
            public void started() {}

            @Override
            public void tick(JsonObject tick) {
                streamed.add(tick);
            }

            @Override
            public void finished(String how) {
                outcome[0] = how;
            }
        };
        for (SimRecording.Tick tick : recording.ticks()) {
            SimRunStream.accept(SimRunStream.tick(tick), parent);
        }
        SimRunStream.accept(SimRunStream.finished(recording.outcome()), parent);
        SimReplayPage.Source fromTheChild = new SimReplayPage.Source() {
            @Override
            public String name() {
                return "StickTeleOp";
            }

            @Override
            public String kind() {
                return "teleop";
            }

            @Override
            public JsonArray ticksJson(int from) {
                JsonArray rest = new JsonArray();
                for (int i = from; i < streamed.size(); i++) {
                    rest.add(streamed.get(i));
                }
                return rest;
            }

            @Override
            public String outcome() {
                return outcome[0];
            }
        };

        assertEquals(SimReplayPage.page(recording, false), SimReplayPage.page(fromTheChild, false));
        assertEquals(SimReplayPage.page(recording, true), SimReplayPage.page(fromTheChild, true));
        assertEquals(SimReplayPage.update(recording, 1), SimReplayPage.update(fromTheChild, 1));
    }

    private static int templateMentions(String text) {
        String empty = SimReplayPage.page(new SimRecording("Empty", "teleop"), false);
        return empty.split(Pattern.quote(text), -1).length - 1;
    }
}
