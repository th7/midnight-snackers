package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.JsonObject;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.sim.SimRunStream.Outcome;
import org.junit.Test;

/**
 * The lines the child prints are written and read in one place, so the parent and the child
 * agree by construction, and every way a run can end is named there too.
 */
public class SimRunStreamTest {
    /** Remembers what each line said. */
    private static final class Heard implements SimRunStream.Listener {
        final List<String> events = new ArrayList<>();
        final List<JsonObject> ticks = new ArrayList<>();
        String outcome;

        @Override
        public void started() {
            events.add("started");
        }

        @Override
        public void tick(JsonObject tick) {
            events.add("tick");
            ticks.add(tick);
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
        return new SimRecording.Tick(
                seconds, new Pose2d(12.5, -3, Math.PI / 2), step, new double[] {1, 0.75, -0.5, 0.25}, List.of(packet));
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
        assertEquals(0.5, SimRunStream.seconds(first), 0);
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
        assertEquals("rounded to three decimals on the wire", 0.333, SimRunStream.seconds(heard.ticks.get(1)), 0);
        assertEquals("done", heard.outcome);
    }

    /**
     * A tick says where every ball is, {x, y, z} each, or null for one held in the robot; how many
     * the robot holds; and how many each alliance has scored, when any of that is worth saying.
     */
    @Test
    public void aTickCarriesTheBallsWhatTheRobotHoldsAndTheScore() {
        Heard heard = new Heard();
        SimRecording.Tick balls = new SimRecording.Tick(
                1,
                new Pose2d(0, 0, 0),
                "",
                new double[] {0, 0, 0, 0},
                List.of(),
                null,
                null,
                new double[][] {{1, 2, 1.39}, null, {3, 4, 30.5}},
                1,
                java.util.Map.of("Red", 2));

        SimRunStream.accept(SimRunStream.tick(balls), heard);
        SimRunStream.accept(SimRunStream.tick(tick(2, "quiet")), heard);

        JsonObject first = heard.ticks.get(0);
        assertEquals("[[1.0,2.0,1.39],null,[3.0,4.0,30.5]]", first.get("pieces").toString());
        assertEquals(1, first.get("held").getAsInt());
        assertEquals(2, first.getAsJsonObject("scored").get("Red").getAsInt());
        JsonObject quiet = heard.ticks.get(1);
        assertFalse("nothing held, nothing scored: nothing said", quiet.has("held") || quiet.has("scored"));
    }

    /**
     * A tick says how far a hive leans only while it is not leaning the way the field was set up,
     * so a replay stays small. Each tick says it in full, and says nothing about the ticks before
     * it: a hive that has tipped says so in every tick until it tips back, and a hive that has
     * tipped back says nothing again, which is how the page knows to draw it back where it started.
     */
    @Test
    public void aTickSaysAHivesTiltOnlyWhileItIsNotLeaningTheWayTheFieldWasSetUp() {
        Heard heard = new Heard();
        SimField.Hive blue = SimRobot.FIELD.hive("Blue Hive <1>");
        SimField.Hive red = SimRobot.FIELD.hive("Red Hive <1>");
        SimRecording.Tick asSetUp = tilted(1, java.util.Map.of(blue.alliance, blue.tilt, red.alliance, red.tilt));
        SimRecording.Tick tipped = tilted(2, java.util.Map.of(blue.alliance, -blue.tilt, red.alliance, red.tilt));
        SimRecording.Tick tippedBack = tilted(3, java.util.Map.of(blue.alliance, blue.tilt, red.alliance, red.tilt));

        SimRunStream.accept(SimRunStream.tick(asSetUp), heard);
        SimRunStream.accept(SimRunStream.tick(tipped), heard);
        SimRunStream.accept(SimRunStream.tick(tippedBack), heard);

        assertFalse(
                "every hive leans as the field was set up: nothing said",
                heard.ticks.get(0).has("tilt"));
        JsonObject tilt = heard.ticks.get(1).getAsJsonObject("tilt");
        assertEquals(-blue.tilt, tilt.get("Blue").getAsDouble(), 0);
        assertFalse("the hive that has not tipped says nothing", tilt.has("Red"));
        assertFalse(
                "the hive that has tipped back says nothing again",
                heard.ticks.get(2).has("tilt"));
    }

    private static SimRecording.Tick tilted(double seconds, java.util.Map<String, Double> tilt) {
        return new SimRecording.Tick(
                seconds,
                new Pose2d(0, 0, 0),
                "",
                new double[] {0, 0, 0, 0},
                List.of(),
                null,
                null,
                null,
                0,
                java.util.Map.of(),
                tilt);
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
    public void theChildSaysItsProtocolFirstAndABenchOfTheSameVersionConsumesIt() {
        String hello = SimRunStream.hello();
        assertEquals(1, hello.split("\n").length);
        assertNull("a hello is consumed; the next line is content", SimRunStream.afterHello(hello));
        assertEquals(SimRunStream.PROTOCOL, SimRunStream.protocolOf(hello));
    }

    /** A child of this version waits to be placed before its run starts; the bench places one that old. */
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

    /**
     * A child from before the hello existed prints its content first. Those lines are pinned here
     * as such a child printed them, because a bench must still read them.
     */
    @Test
    public void aVersionOneChildPrintsNoHelloSoItsFirstLineIsContentAndStillReads() {
        assertEquals(1, SimRunStream.OLDEST_PROTOCOL_READ);
        assertEquals("a catalog line comes back as it was", "[]", SimRunStream.afterHello("[]"));
        assertEquals(1, SimRunStream.protocolOf("[]"));
        Heard heard = new Heard();
        String[] versionOne = {
            "{\"started\":true}",
            "{\"t\":0.5,\"x\":1.0,\"y\":2.0,\"heading\":0.0,\"step\":\"1. go\",\"powers\":[0.5,0.5,0.5,0.5],\"packets\":[]}",
            "{\"outcome\":\"done\"}",
        };
        String first = SimRunStream.afterHello(versionOne[0]);
        assertEquals(versionOne[0], first);
        SimRunStream.accept(first, heard);
        SimRunStream.accept(versionOne[1], heard);
        SimRunStream.accept(versionOne[2], heard);
        assertEquals(List.of("started", "tick", "finished"), heard.events);
        assertEquals("1. go", heard.ticks.get(0).get("step").getAsString());
        assertEquals(0.5, SimRunStream.seconds(heard.ticks.get(0)), 0);
        assertEquals("done", heard.outcome);
    }

    @Test
    public void aChildOfANewerProtocolIsRefusedByName() {
        int newer = SimRunStream.PROTOCOL + 1;
        try {
            SimRunStream.afterHello("{\"protocol\":" + newer + "}");
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
            SimRunStream.afterHello("{\"protocol\":" + older + "}");
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
    }

    // --- the handshake: what a bench does with the child's first line ---

    /**
     * Whether a run can be made on the child that answered, and what to send it, is one decision
     * read off one line. It used to be stated here in prose, decided in SimBench and realised in
     * SimChild, with the refusals worded twice.
     */
    @Test
    public void aChildOfThisVersionIsToldWhereToStartAndWhichRobot() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(SimRunStream.hello(), new Pose2d(12, -7, 1.5), 3L);

        assertEquals(SimRunStream.PROTOCOL, handshake.protocol);
        assertFalse(handshake.message, handshake.refused());
        assertNull("its first line was the hello", handshake.firstContentLine);
        assertEquals(SimDriverStation.startLine(new Pose2d(12, -7, 1.5), 3L), handshake.startLine);
    }

    /** A child from before the seed runs the exact robot whatever it is told, so it is not told. */
    @Test
    public void aChildFromBeforeTheSeedRunsWhenNoSeedIsAskedFor() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(helloOf(3), new Pose2d(12, 0, 0), null);

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, 0, 0), null), handshake.startLine);
    }

    @Test
    public void aChildFromBeforeTheSeedIsRefusedOneByNameAndToldTheFix() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(helloOf(3), StartPoses.ORIGIN, 1L);

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotSeed(3), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.SEEDED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("clear the seed"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    /** A child from before placement starts at the origin on its own, so it may run from there. */
    @Test
    public void aChildFromBeforePlacementRunsFromTheOriginAndPlacesItself() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(helloOf(2), StartPoses.ORIGIN, null);

        assertFalse(handshake.message, handshake.refused());
        assertNull("it is not told where to start, because it cannot be", handshake.startLine);
    }

    @Test
    public void aChildFromBeforePlacementIsRefusedAnywhereElseByNameAndToldTheFix() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(helloOf(2), new Pose2d(12, 0, 0), null);

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotPlace(2), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.PLACED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("back at the origin"));
    }

    /** A version-one child prints no hello, so its first line is content and comes back as it was. */
    @Test
    public void aVersionOneChildsFirstLineIsContent() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake("[{\"name\":\"an op mode\"}]", StartPoses.ORIGIN, null);

        assertEquals(1, handshake.protocol);
        assertFalse(handshake.refused());
        assertEquals("[{\"name\":\"an op mode\"}]", handshake.firstContentLine);
    }

    /** One that speaks a protocol this bench cannot read is refused before any of that. */
    @Test
    public void aChildThisBenchCannotReadIsRefusedBeforeItIsPlaced() {
        SimRunStream.WrongProtocol wrong = org.junit.Assert.assertThrows(
                SimRunStream.WrongProtocol.class,
                () -> SimRunStream.handshake(helloOf(SimRunStream.PROTOCOL + 1), StartPoses.ORIGIN, null));

        assertEquals(SimRunStream.PROTOCOL + 1, wrong.childProtocol);
    }

    private static String helloOf(int protocol) {
        JsonObject line = new JsonObject();
        line.addProperty("protocol", protocol);
        return line.toString();
    }
}
