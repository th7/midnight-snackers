package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.sim.SimRunStream.Outcome;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.junit.Test;

public class SimRunStreamTest {
    private static final class Heard implements SimRunStream.Listener {
        final List<String> events = new ArrayList<>();
        final List<SimRunStream.TickLine> lines = new ArrayList<>();
        final List<JsonObject> ticks = new ArrayList<>();
        String outcome;

        @Override
        public void started() {
            events.add("started");
        }

        @Override
        public void tick(SimRunStream.TickLine tick) {
            events.add("tick");
            lines.add(tick);
            ticks.add(tick.json());
        }

        @Override
        public void finished(String outcome) {
            events.add("finished");
            this.outcome = outcome;
        }
    }

    private static SimRecording.Tick tick(double seconds, String step) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("xError", 0.25);
        return SimRecording.Tick.at(
                        seconds,
                        new Pose2d(12.5, -3, Math.PI / 2),
                        step,
                        new double[] {1, 0.75, -0.5, 0.25},
                        List.of(packet))
                .tick();
    }

    @Test
    public void startedEachTickAndTheOutcomeAreOneLineEachAndReadBackAsWhatTheySaid() {
        Heard heard = new Heard();

        SimRunStream.accept(SimRunStream.started(), heard);
        SimRunStream.accept(SimRunStream.tick(tick(0.5, "2. driveTo 24, 0, 0")), heard);
        SimRunStream.accept(SimRunStream.tick(tick(1.0 / 3, "3. rounding")), heard);
        SimRunStream.accept(SimRunStream.finished(Outcome.done()), heard);

        assertEquals(List.of("started", "tick", "tick", "finished"), heard.events);
        JsonObject first = heard.ticks.get(0);
        assertEquals(0.5, heard.lines.get(0).seconds(), 0);
        assertEquals(12.5, first.get("x").getAsDouble(), 0);
        assertEquals(-3, first.get("y").getAsDouble(), 0);
        assertEquals(Math.PI / 2, first.get("heading").getAsDouble(), 0.001);
        assertEquals("2. driveTo 24, 0, 0", first.get("step").getAsString());
        assertEquals(4, first.getAsJsonArray("powers").size());
        assertEquals(
                0.25,
                first.getAsJsonArray("packets")
                        .get(0)
                        .getAsJsonObject()
                        .getAsJsonObject("data")
                        .get("xError")
                        .getAsDouble(),
                0);
        assertEquals(
                "rounded to three decimals on the wire",
                0.333,
                heard.lines.get(1).seconds(),
                0);
        assertEquals("done", heard.outcome);
    }

    @Test
    public void aTickCarriesTheBallsWhatTheRobotHoldsAndTheScore() {
        Heard heard = new Heard();
        SimRecording.Tick balls = SimRecording.Tick.at(1, new Pose2d(0, 0, 0), "", new double[] {0, 0, 0, 0}, List.of())
                .withBalls(new double[][] {{1, 2, 1.39}, null, {3, 4, 30.5}}, 1)
                .scoring(java.util.Map.of("Red", 2))
                .tick();

        SimRunStream.accept(SimRunStream.tick(balls), heard);
        SimRunStream.accept(SimRunStream.tick(tick(2, "quiet")), heard);

        JsonObject first = heard.ticks.get(0);
        assertEquals("[[1.0,2.0,1.39],null,[3.0,4.0,30.5]]", first.get("pieces").toString());
        assertEquals(1, first.get("held").getAsInt());
        assertEquals(2, first.getAsJsonObject("scored").get("Red").getAsInt());
        JsonObject quiet = heard.ticks.get(1);
        assertFalse("nothing held, nothing scored: nothing said", quiet.has("held") || quiet.has("scored"));
    }

    @Test
    public void aTickSaysAHivesTiltOnlyWhileItIsNotLeaningTheWayTheFieldWasSetUp() {
        Heard heard = new Heard();
        Field.Hive blue = SimRobot.FIELD.hive("Blue Hive <1>").orElseThrow();
        Field.Hive red = SimRobot.FIELD.hive("Red Hive <1>").orElseThrow();
        SimRecording.Tick asSetUp = tilted(
                1,
                java.util.Map.of(
                        blue.alliance(),
                        blue.tilt().degrees(),
                        red.alliance(),
                        red.tilt().degrees()));
        SimRecording.Tick tipped = tilted(
                2,
                java.util.Map.of(
                        blue.alliance(),
                        -blue.tilt().degrees(),
                        red.alliance(),
                        red.tilt().degrees()));
        SimRecording.Tick tippedBack = tilted(
                3,
                java.util.Map.of(
                        blue.alliance(),
                        blue.tilt().degrees(),
                        red.alliance(),
                        red.tilt().degrees()));

        SimRunStream.accept(SimRunStream.tick(asSetUp), heard);
        SimRunStream.accept(SimRunStream.tick(tipped), heard);
        SimRunStream.accept(SimRunStream.tick(tippedBack), heard);

        assertFalse(
                "every hive leans as the field was set up: nothing said",
                heard.ticks.get(0).has("tilt"));
        JsonObject tilt = heard.ticks.get(1).getAsJsonObject("tilt");
        assertEquals(-blue.tilt().degrees(), tilt.get("Blue").getAsDouble(), 0);
        assertFalse("the hive that has not tipped says nothing", tilt.has("Red"));
        assertFalse(
                "the hive that has tipped back says nothing again",
                heard.ticks.get(2).has("tilt"));
    }

    private static SimRecording.Tick tilted(double seconds, java.util.Map<String, Double> tilt) {
        return SimRecording.Tick.at(seconds, new Pose2d(0, 0, 0), "", new double[] {0, 0, 0, 0}, List.of())
                .tilted(tilt)
                .tick();
    }

    @Test
    public void everyLineIsOneLine() {
        assertTrue(SimRunStream.started().indexOf('\n') < 0);
        assertTrue(SimRunStream.tick(tick(0, "a\nstep")).indexOf('\n') < 0);
        assertTrue(SimRunStream.finished("timed out\nlate").indexOf('\n') < 0);
    }

    @Test
    public void aLineThatIsNoneOfTheseIsRefusedNotMistakenForATick() {
        Heard heard = new Heard();
        for (String line : List.of("{\"foo\": 1}", "not json", "", "[]")) {
            try {
                SimRunStream.accept(line, heard);
                fail("accepted " + line);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(line));
            }
        }
        assertTrue(heard.events.isEmpty());
        assertNull(heard.outcome);
    }

    @Test
    public void aTickAChildWroteIsKeptAsTheLineItPrinted() {
        Heard heard = new Heard();
        String line = SimRunStream.tick(tick(0.5, "1. <aim> & 'launch' = go"));

        SimRunStream.accept(line, heard);

        assertEquals(line, heard.lines.get(0).text());
        assertEquals(0.5, heard.lines.get(0).seconds(), 0);
    }

    @Test
    public void aTickWrittenAnyOtherWayIsKeptAsAChildWouldHaveWrittenIt() {
        Heard heard = new Heard();

        SimRunStream.accept("{\"t\": 0.5, \"x\": 1.50, \"flag\": TRUE, \"step\": \"it\\'s\"}", heard);

        assertEquals(
                "{\"t\":0.5,\"x\":1.50,\"flag\":true,\"step\":\"it\\u0027s\"}",
                heard.lines.get(0).text());
    }

    @Test
    public void aKeptTickIsItsTextAndItsTimeAndNoTreeOfIt() {
        java.util.Map<String, Class<?>> fields = new java.util.TreeMap<>();
        for (java.lang.reflect.Field field : SimRunStream.TickLine.class.getDeclaredFields()) {
            if (!java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                fields.put(field.getName(), field.getType());
            }
        }

        assertEquals(java.util.Map.of("seconds", double.class, "text", String.class), fields);
    }

    @Test
    public void aTickIsStrictJsonOrItIsRefusedSinceItIsServedAsItCame() {
        Heard heard = new Heard();
        for (String line : List.of(
                "{t: 0.5, x: 1}",
                "{'t': 0.5, 'x': 1}",
                "{\"t\": NaN, \"x\": 1}",
                "{\"t\": 0.5, \"x\": Infinity}",
                "{\"t\": 0.5, \"x\": 1,}",
                "{\"t\": 0.5, \"x\": 1} // and a comment",
                "{\"t\": \"soon\", \"x\": 1}")) {
            try {
                SimRunStream.accept(line, heard);
                fail("accepted " + line);
            } catch (IllegalArgumentException expected) {
                assertTrue(expected.getMessage(), expected.getMessage().contains(line));
            }
        }
        assertTrue(heard.events.isEmpty());
    }

    @Test
    public void theChildSaysItsProtocolFirstAndABenchOfTheSameVersionConsumesIt() {
        String hello = SimRunStream.hello();
        assertEquals(1, hello.split("\n").length);
        assertNull("a hello is consumed; the next line is content", SimRunStream.afterHello(hello, TeamRobot.REGINALD));
        assertEquals(SimRunStream.PROTOCOL, SimRunStream.protocolOf(hello));
    }

    @Test
    public void aChildOfThisVersionWaitsToBePlaced() {
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.PLACED_PROTOCOL);
        assertEquals("a seeded child, from protocol 4 on", 4, SimRunStream.SEEDED_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.SEEDED_PROTOCOL);
        assertTrue(SimRunStream.SEEDED_PROTOCOL > SimRunStream.PLACED_PROTOCOL);
        assertEquals("a version-1 child places itself at the origin", 1, SimRunStream.protocolOf("{\"started\":true}"));
        assertTrue(SimRunStream.protocolOf("{\"protocol\":" + SimRunStream.PLACED_PROTOCOL + "}")
                >= SimRunStream.PLACED_PROTOCOL);
        assertTrue(Outcome.cannotPlace(1).startsWith("wrong protocol"));
        assertTrue(Outcome.cannotPlace(1).contains("place"));
    }

    @Test
    public void aVersionOneChildPrintsNoHelloSoItsFirstLineIsContentAndStillReads() {
        assertEquals(1, SimRunStream.OLDEST_PROTOCOL_READ);
        assertEquals("a catalog line comes back as it was", "[]", SimRunStream.afterHello("[]", TeamRobot.REGINALD));
        assertEquals(1, SimRunStream.protocolOf("[]"));
        Heard heard = new Heard();
        String[] versionOne = {
            "{\"started\":true}",
            "{\"t\":0.5,\"x\":1.0,\"y\":2.0,\"heading\":0.0,\"step\":\"1. go\",\"powers\":[0.5,0.5,0.5,0.5],\"packets\":[]}",
            "{\"outcome\":\"done\"}",
        };
        String first = SimRunStream.afterHello(versionOne[0], TeamRobot.REGINALD);
        assertEquals(versionOne[0], first);
        SimRunStream.accept(first, heard);
        SimRunStream.accept(versionOne[1], heard);
        SimRunStream.accept(versionOne[2], heard);
        assertEquals(List.of("started", "tick", "finished"), heard.events);
        assertEquals("1. go", heard.ticks.get(0).get("step").getAsString());
        assertEquals(0.5, heard.lines.get(0).seconds(), 0);
        assertEquals("done", heard.outcome);
    }

    @Test
    public void aChildOfANewerProtocolIsRefusedByName() {
        int newer = SimRunStream.PROTOCOL + 1;
        try {
            SimRunStream.afterHello("{\"protocol\":" + newer + "}", TeamRobot.REGINALD);
            fail("a newer child prints lines this bench cannot read");
        } catch (SimRunStream.WrongProtocol e) {
            assertEquals(newer, e.childProtocol);
            assertTrue(e.getMessage(), e.getMessage().contains("protocol " + newer));
            assertTrue(e.getMessage(), e.getMessage().contains("protocol " + SimRunStream.PROTOCOL));
            assertTrue(
                    "the fix is the server's, not the sources'", e.getMessage().contains("server"));
        }
    }

    @Test
    public void aChildOlderThanTheOldestReadIsRefusedByName() {
        int older = SimRunStream.OLDEST_PROTOCOL_READ - 1;
        try {
            SimRunStream.afterHello("{\"protocol\":" + older + "}", TeamRobot.REGINALD);
            fail("an older child prints lines this bench no longer reads");
        } catch (SimRunStream.WrongProtocol e) {
            assertEquals(older, e.childProtocol);
            assertTrue(e.getMessage(), e.getMessage().contains("protocol " + older));
            assertTrue("the fix is the sources'", e.getMessage().toLowerCase().contains("pull"));
        }
    }

    @Test
    public void theOutcomesAreNamedHereAsTheGlossarySaysThem() {
        assertEquals("done", Outcome.done());
        assertTrue(Outcome.wrongProtocol(3).startsWith("wrong protocol"));
        assertEquals("stopped", Outcome.stopped());
        assertEquals("timed out after 0.3s", Outcome.timedOut(0.3));
        assertEquals("build failed", Outcome.buildFailed());
        assertEquals("child exited with code 3", Outcome.childExited(3));
        assertTrue(Outcome.failed(new IllegalStateException("boom")).startsWith("failed: "));
        assertTrue(Outcome.killed(1.3, "the op mode did not return").startsWith("killed after 1.3s"));
        assertTrue(Outcome.killedAfterStop(2.0).startsWith("killed 2.0s after Stop"));
        assertTrue(Outcome.noOpModeNamed("org.example.Nope").contains("org.example.Nope"));
        assertTrue(Outcome.cannotStart("not a start pose").startsWith("could not take the start line: "));
        assertTrue(Outcome.cannotStart("not a start pose").endsWith("not a start pose"));
    }

    @Test
    public void aChildOfThisVersionIsToldWhereToStartAndWhichRobot() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.hello(), new Pose2d(12, -7, 1.5), 3L, TeamRobot.REGINALD, Constants.defaults());

        assertEquals(SimRunStream.PROTOCOL, handshake.protocol);
        assertFalse(handshake.message, handshake.refused());
        assertNull("its first line was the hello", handshake.firstContentLine);
        assertEquals(
                SimDriverStation.startLine(new Pose2d(12, -7, 1.5), 3L, Constants.defaults()), handshake.startLine);
    }

    @Test
    public void aChildFromBeforeTheSeedRunsWhenNoSeedIsAskedFor() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(3), new Pose2d(12, 0, 0), null, TeamRobot.REGINALD, Constants.defaults());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, 0, 0), null, Constants.defaults()), handshake.startLine);
    }

    @Test
    public void aChildFromBeforeTheSeedIsRefusedOneByNameAndToldTheFix() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(3), StartPoses.ORIGIN, 1L, TeamRobot.REGINALD, Constants.defaults());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotSeed(3), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.SEEDED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("clear the seed"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aChildFromBeforePlacementRunsFromTheOriginAndPlacesItself() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(2), StartPoses.ORIGIN, null, TeamRobot.REGINALD, Constants.defaults());

        assertFalse(handshake.message, handshake.refused());
        assertNull("it is not told where to start, because it cannot be", handshake.startLine);
    }

    @Test
    public void aChildFromBeforePlacementIsRefusedAnywhereElseByNameAndToldTheFix() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(2), new Pose2d(12, 0, 0), null, TeamRobot.REGINALD, Constants.defaults());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotPlace(2), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.PLACED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("back at the origin"));
    }

    @Test
    public void aVersionOneChildsFirstLineIsContent() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                "[{\"name\":\"an op mode\"}]", StartPoses.ORIGIN, null, TeamRobot.REGINALD, Constants.defaults());

        assertEquals(1, handshake.protocol);
        assertFalse(handshake.refused());
        assertEquals("[{\"name\":\"an op mode\"}]", handshake.firstContentLine);
    }

    @Test
    public void aChildThisBenchCannotReadIsRefusedBeforeItIsPlaced() {
        SimRunStream.WrongProtocol wrong = org.junit.Assert.assertThrows(
                SimRunStream.WrongProtocol.class,
                () -> SimRunStream.handshake(
                        SimRunStream.helloOf(SimRunStream.PROTOCOL + 1),
                        StartPoses.ORIGIN,
                        null,
                        TeamRobot.REGINALD,
                        Constants.defaults()));

        assertEquals(SimRunStream.PROTOCOL + 1, wrong.childProtocol);
    }

    @Test
    public void aChildThatSimulatesEachRobotSpeaksProtocolFive() {
        assertEquals(5, SimRunStream.ROBOTS_PROTOCOL);
        assertTrue(SimRunStream.ROBOTS_PROTOCOL > SimRunStream.SEEDED_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.ROBOTS_PROTOCOL);
    }

    @Test
    public void aChildIsToldWhichRobotItIsUnlessItIsReginaldWhomEveryChildSimulatesUntold() {
        assertEquals(List.of(), SimRunStream.robotArguments(TeamRobot.REGINALD));
        assertEquals(List.of("--robot=nugget"), SimRunStream.robotArguments(TeamRobot.NUGGET));
    }

    @Test
    public void aChildFromBeforeThereWereTwoRobotsStillRunsReginald() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(4), new Pose2d(12, 0, 0), 3L, TeamRobot.REGINALD, Constants.defaults());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, 0, 0), 3L, Constants.defaults()), handshake.startLine);
    }

    @Test
    public void aChildFromBeforeThereWereTwoRobotsCannotRunNuggetAndSaysWhoseLineToPull() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(4), StartPoses.ORIGIN, null, TeamRobot.NUGGET, Constants.defaults());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotSimulate(4, TeamRobot.NUGGET), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.contains("Nugget"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.ROBOTS_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("Pull nugget-develop"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aChildOfThisProtocolIsPlacedAsNuggetAsItIsAsReginald() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.hello(), new Pose2d(12, -7, 1.5), 3L, TeamRobot.NUGGET, Constants.defaults());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                SimDriverStation.startLine(new Pose2d(12, -7, 1.5), 3L, Constants.defaults()), handshake.startLine);
    }

    private static Constants regeared(double ticksPerTurn) {
        return Valid.value(Constants.of(java.util.Map.of(Constant.FLYWHEEL_TICKS_PER_TURN, ticksPerTurn)));
    }

    @Test
    public void aChildThatTakesTheConstantsSpeaksProtocolSix() {
        assertEquals(6, SimRunStream.CONSTANTS_PROTOCOL);
        assertTrue(SimRunStream.CONSTANTS_PROTOCOL > SimRunStream.ROBOTS_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.CONSTANTS_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheConstantsTheAdminChanged() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.hello(), new Pose2d(12, -7, 1.5), 3L, TeamRobot.REGINALD, regeared(30.5));

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, -7, 1.5), 3L, regeared(30.5)), handshake.startLine);
        assertEquals(
                30.5,
                handshake
                        .startLine
                        .getAsJsonObject("constants")
                        .get("flywheel_ticks_per_turn")
                        .getAsDouble(),
                0);
    }

    @Test
    public void aChildFromBeforeTheConstantsRunsWhenNoneWereChanged() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(5), new Pose2d(12, 0, 0), 3L, TeamRobot.NUGGET, Constants.defaults());

        assertFalse(handshake.message, handshake.refused());
        assertFalse(handshake.startLine.toString(), handshake.startLine.has("constants"));
    }

    @Test
    public void aChildFromBeforeTheConstantsIsRefusedChangedOnesByNameAndToldTheFix() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(5), StartPoses.ORIGIN, null, TeamRobot.NUGGET, regeared(30.5));

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotTune(5), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.CONSTANTS_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("Flywheel ticks per turn"));
        assertTrue(handshake.message, handshake.message.contains("Pull nugget-develop"));
        assertTrue(handshake.message, handshake.message.contains("put the simulation constants back"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aListingFromAChildFromBeforeThereWereTwoRobotsIsReginaldsAndNotNuggets() {
        assertNull(SimRunStream.afterHello(SimRunStream.helloOf(4), TeamRobot.REGINALD));
        assertNull(SimRunStream.afterHello(SimRunStream.hello(), TeamRobot.NUGGET));

        SimRunStream.WrongProtocol refused = assertThrows(
                SimRunStream.WrongProtocol.class,
                () -> SimRunStream.afterHello(SimRunStream.helloOf(4), TeamRobot.NUGGET));

        assertEquals(4, refused.childProtocol);
        assertTrue(refused.getMessage(), refused.getMessage().contains("Pull nugget-develop"));
    }

    private static Constants set(Constant constant, double value) {
        return Valid.value(Constants.of(java.util.Map.of(constant, value)));
    }

    private static List<String> everyConstantBut(String left) {
        List<String> names = new ArrayList<>();
        for (Constant constant : Constant.values()) {
            if (!constant.asked().equals(left)) {
                names.add(constant.asked());
            }
        }
        return names;
    }

    private static List<String> strings(JsonArray array) {
        List<String> out = new ArrayList<>();
        for (JsonElement element : array) {
            out.add(element.getAsString());
        }
        return out;
    }

    @Test
    public void aChildThatListsTheConstantsItReadsAndTheOnesItRetiredSpeaksProtocolSeven() {
        assertEquals(7, SimRunStream.CONSTANTS_LISTED_PROTOCOL);
        assertTrue(SimRunStream.CONSTANTS_LISTED_PROTOCOL > SimRunStream.CONSTANTS_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.CONSTANTS_LISTED_PROTOCOL);
    }

    @Test
    public void theHelloOfThisVersionListsEveryConstantItReadsInTheirOrderAndEveryOneItRetired() {
        JsonObject hello = new Gson().fromJson(SimRunStream.hello(), JsonObject.class);

        assertEquals(SimRunStream.PROTOCOL, hello.get("protocol").getAsInt());
        assertEquals(everyConstantBut(""), strings(hello.getAsJsonArray("reads")));
        List<String> retired = new ArrayList<>();
        for (Constant.Retired gone : Constant.Retired.values()) {
            retired.add(gone.asked());
        }
        assertEquals(retired, strings(hello.getAsJsonArray("retired")));
        assertTrue("the throw is retired: " + retired, retired.contains("launch_throw"));
        assertEquals(
                "a child of this version says exactly this",
                SimRunStream.hello(),
                SimRunStream.helloOf(SimRunStream.PROTOCOL));
    }

    @Test
    public void aVersionSixChildReadsTheSeventeenConstantsItReadBeforeTheList() {
        assertEquals(
                List.of(
                        "motor_spread",
                        "flattest_volts",
                        "freshest_volts",
                        "sag_volts_per_power",
                        "drain_volts_per_second",
                        "least_traction_g",
                        "most_traction_g",
                        "set_down_inches",
                        "set_down_degrees",
                        "loop_seconds",
                        "loop_spread",
                        "least_loop_seconds",
                        "hiccup_chance",
                        "shortest_hiccup_seconds",
                        "longest_hiccup_seconds",
                        "launch_throw",
                        "turntable_speed"),
                SimRunStream.READ_BEFORE_THE_LIST);

        Constants quicker = set(Constant.TURNTABLE_SPEED, 2000);
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.CONSTANTS_PROTOCOL),
                new Pose2d(12, 0, 0),
                3L,
                TeamRobot.REGINALD,
                quicker);

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, 0, 0), 3L, quicker), handshake.startLine);
    }

    @Test
    public void aVersionSixChildIsRefusedARunOnAConstantItDoesNotReadByNameAndToldToPullTheRobotsDevelop() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.CONSTANTS_PROTOCOL),
                StartPoses.ORIGIN,
                null,
                TeamRobot.NUGGET,
                set(Constant.FLYWHEEL_TICKS_PER_TURN, 30));

        assertTrue(handshake.refused());
        assertEquals(Outcome.doesNotRead(List.of(Constant.FLYWHEEL_TICKS_PER_TURN)), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.contains("Flywheel ticks per turn"));
        assertTrue(handshake.message, handshake.message.contains("Flywheel ticks per turn 30.0"));
        assertTrue(handshake.message, handshake.message.contains("the sources are older than the server"));
        assertTrue(handshake.message, handshake.message.contains("Pull nugget-develop"));
        assertTrue(handshake.message, handshake.message.contains("put the simulation constants back"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aChildOfThisVersionIsRefusedARunOnAConstantItDoesNotListAsReadAndToldToPullDevelop() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.PROTOCOL, everyConstantBut("hiccup_chance"), List.of()),
                StartPoses.ORIGIN,
                null,
                TeamRobot.REGINALD,
                set(Constant.HICCUP_CHANCE, 0));

        assertTrue(handshake.refused());
        assertEquals(Outcome.doesNotRead(List.of(Constant.HICCUP_CHANCE)), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("Hiccup chance"));
        assertTrue(handshake.message, handshake.message.contains("the sources are older than the server"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aChildThatRetiredAConstantTheAdminSetIsRefusedARunOnItByNameAndToldTheServerIsOlder() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(
                        SimRunStream.PROTOCOL, everyConstantBut("hiccup_chance"), List.of("hiccup_chance")),
                StartPoses.ORIGIN,
                null,
                TeamRobot.REGINALD,
                set(Constant.HICCUP_CHANCE, 0));

        assertTrue(handshake.refused());
        assertEquals(Outcome.retired(List.of(Constant.HICCUP_CHANCE)), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.contains("Hiccup chance"));
        assertTrue(handshake.message, handshake.message.contains("Hiccup chance 0.0"));
        assertTrue(handshake.message, handshake.message.contains("the server is older than the sources"));
        assertTrue(handshake.message, handshake.message.contains("Restart the server"));
        assertFalse(handshake.message, handshake.message.contains("Pull"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aRunOnAConstantTheChildRetiredAndOneItDoesNotReadIsRefusedNamingBothInItsOutcomeAndEachFix() {
        List<String> reads = everyConstantBut("hiccup_chance");
        reads.remove("motor_spread");
        Constants both =
                Valid.value(Constants.of(java.util.Map.of(Constant.HICCUP_CHANCE, 0.0, Constant.MOTOR_SPREAD, 0.2)));

        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.PROTOCOL, reads, List.of("hiccup_chance")),
                StartPoses.ORIGIN,
                null,
                TeamRobot.REGINALD,
                both);

        assertTrue(handshake.refused());
        assertEquals(
                Outcome.retiredAndDoesNotRead(List.of(Constant.HICCUP_CHANCE), List.of(Constant.MOTOR_SPREAD)),
                handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.contains("retired Hiccup chance"));
        assertTrue(handshake.outcome, handshake.outcome.contains("does not read Motor spread"));
        assertTrue(handshake.message, handshake.message.contains("Hiccup chance 0.0"));
        assertTrue(handshake.message, handshake.message.contains("Restart the server"));
        assertTrue(handshake.message, handshake.message.contains("Motor spread 0.2"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aChildOfThisVersionRunsWhenItReadsEveryConstantTheAdminChangedWhateverElseItDoesNotRead() {
        Constants spread = set(Constant.MOTOR_SPREAD, 0.2);
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(
                        SimRunStream.PROTOCOL, everyConstantBut("hiccup_chance"), List.of("a_constant_long_gone")),
                new Pose2d(12, 0, 0),
                3L,
                TeamRobot.REGINALD,
                spread);

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, 0, 0), 3L, spread), handshake.startLine);
    }

    private static final String FIRST_LISTED_HELLO = "{\"protocol\":7,\"reads\":[\"motor_spread\",\"flattest_volts\","
            + "\"freshest_volts\",\"sag_volts_per_power\",\"drain_volts_per_second\",\"least_traction_g\","
            + "\"most_traction_g\",\"set_down_inches\",\"set_down_degrees\",\"loop_seconds\",\"loop_spread\","
            + "\"least_loop_seconds\",\"hiccup_chance\",\"shortest_hiccup_seconds\",\"longest_hiccup_seconds\","
            + "\"flywheel_ticks_per_turn\",\"turntable_speed\"],\"retired\":[\"launch_throw\"]}";

    private static Constants changedAlone(Constant constant) {
        for (double value :
                new double[] {constant.most(), constant.least(), 2 * constant.byDefault(), constant.byDefault() / 2}) {
            if (Constants.of(java.util.Map.of(constant, value)) instanceof Checked.Ok<Constants> admitted) {
                return admitted.value();
            }
        }
        throw new AssertionError(constant.asked() + " may be changed alone to none of its ends, nor twice or half"
                + " what it was built as");
    }

    @Test
    public void aChildOfTheFirstVersionToListItsConstantsRunsOnEachOneItReadsAndIsRefusedAnyOtherTheAdminChanged() {
        List<String> reads = strings(
                new Gson().fromJson(FIRST_LISTED_HELLO, JsonObject.class).getAsJsonArray("reads"));

        for (Constant constant : Constant.values()) {
            SimRunStream.Handshake handshake = SimRunStream.handshake(
                    FIRST_LISTED_HELLO, StartPoses.ORIGIN, null, TeamRobot.REGINALD, changedAlone(constant));

            if (reads.contains(constant.asked())) {
                assertFalse(constant.asked() + ": " + handshake.message, handshake.refused());
            } else {
                assertEquals(Outcome.doesNotRead(List.of(constant)), handshake.outcome);
                assertTrue(handshake.message, handshake.message.contains("Pull develop"));
            }
        }
    }

    @Test
    public void aHelloOfThisVersionWithoutAReadableListOfTheConstantsIsRefusedByName() {
        for (String hello : List.of(
                "{\"protocol\":7}",
                "{\"protocol\":7,\"reads\":[\"motor_spread\"]}",
                "{\"protocol\":7,\"retired\":[]}",
                "{\"protocol\":7,\"reads\":\"motor_spread\",\"retired\":[]}",
                "{\"protocol\":7,\"reads\":[1],\"retired\":[]}",
                "{\"protocol\":7,\"reads\":[null],\"retired\":[]}",
                "{\"protocol\":7,\"reads\":[[\"motor_spread\"]],\"retired\":[]}",
                "{\"protocol\":7,\"reads\":[],\"retired\":{}}")) {
            SimRunStream.Handshake handshake =
                    SimRunStream.handshake(hello, StartPoses.ORIGIN, null, TeamRobot.REGINALD, Constants.defaults());

            assertTrue(hello, handshake.refused());
            assertEquals(hello, Outcome.unlistedConstants(7), handshake.outcome);
            assertTrue(handshake.message, handshake.message.contains("protocol 7"));
            assertTrue(handshake.message, handshake.message.contains(hello));
            assertNull("a refused run is sent nothing", handshake.startLine);
            SimRunStream.WrongProtocol listing = assertThrows(
                    SimRunStream.WrongProtocol.class, () -> SimRunStream.afterHello(hello, TeamRobot.REGINALD));
            assertEquals(7, listing.childProtocol);
            assertEquals(handshake.message, listing.getMessage());
        }
    }
}
