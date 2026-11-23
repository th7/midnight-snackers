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
import org.firstinspires.ftc.teamcode.simcore.Configuration;
import org.firstinspires.ftc.teamcode.simcore.Constant;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.EncoderReads;
import org.firstinspires.ftc.teamcode.simcore.Facing;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flywheels;
import org.firstinspires.ftc.teamcode.simcore.Handover;
import org.firstinspires.ftc.teamcode.simcore.HubMounting;
import org.firstinspires.ftc.teamcode.simcore.Intake;
import org.firstinspires.ftc.teamcode.simcore.Launch;
import org.firstinspires.ftc.teamcode.simcore.Mechanism;
import org.firstinspires.ftc.teamcode.simcore.Overrides;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.Spinner;
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
    public void aTickCarriesTheBallsAndTheScore() {
        Heard heard = new Heard();
        SimRecording.Tick balls = SimRecording.Tick.at(1, new Pose2d(0, 0, 0), "", new double[] {0, 0, 0, 0}, List.of())
                .withBalls(new double[][] {{1, 2, 1.39}, null, {3, 4, 30.5}})
                .scoring(java.util.Map.of("Red", 2))
                .tick();

        SimRunStream.accept(SimRunStream.tick(balls), heard);
        SimRunStream.accept(SimRunStream.tick(tick(2, "quiet")), heard);

        JsonObject first = heard.ticks.get(0);
        assertEquals("[[1.0,2.0,1.39],null,[3.0,4.0,30.5]]", first.get("pieces").toString());
        assertEquals(2, first.getAsJsonObject("scored").get("Red").getAsInt());
        JsonObject quiet = heard.ticks.get(1);
        assertFalse("nothing scored: nothing said", quiet.has("scored"));
    }

    @Test
    public void aTickSaysWhatTheRobotHoldsAndHowItsIntakeSpinnerAndRampTurn() {
        Heard heard = new Heard();
        SimRecording.Tick taking = SimRecording.Tick.at(
                        1, new Pose2d(0, 0, 0), "", new double[] {0, 0, 0, 0}, List.of())
                .turning(new SimRecording.Mechanisms(
                        1,
                        Intake.Way.IN,
                        new Spinner.Turning(Spinner.Way.CLOCKWISE, 0.9),
                        36.45,
                        true,
                        1,
                        Handover.Why.HANDING,
                        new Flywheels.Turning(Flywheels.Way.LAUNCHING, 2571.5, 508.5),
                        new Flywheels.Turning(Flywheels.Way.WRONG_WAY, -100, -20),
                        1,
                        new SimRecording.Chamber.Holds(Launch.Why.BACK_WHEEL_WRONG_WAY)))
                .tick();

        SimRunStream.accept(SimRunStream.tick(taking), heard);
        SimRunStream.accept(SimRunStream.tick(tick(2, "quiet")), heard);

        assertEquals(
                "{\"held\":1,\"intake\":\"in\",\"spinner\":{\"way\":\"clockwise\",\"speed\":0.9},"
                        + "\"ramp\":{\"degrees\":36.45,\"up\":true},\"fed\":1,\"handing\":\"handing\","
                        + "\"flywheels\":{\"front\":{\"way\":\"launching\",\"rpm\":2571.5},"
                        + "\"back\":{\"way\":\"wrong_way\",\"rpm\":-100.0},\"mismatch\":528.5},"
                        + "\"launcher\":{\"launched\":1,\"stays\":\"back_wheel_wrong_way\"}}",
                heard.ticks.get(0).get("mechanisms").toString());
        assertEquals(
                "a still robot holding nothing says so",
                "{\"held\":0,\"intake\":\"still\",\"spinner\":{\"way\":\"still\",\"speed\":0.0},"
                        + "\"ramp\":{\"degrees\":0.0,\"up\":false},\"fed\":0,"
                        + "\"handing\":\"spinner_not_clockwise\","
                        + "\"flywheels\":{\"front\":{\"way\":\"still\",\"rpm\":0.0},"
                        + "\"back\":{\"way\":\"still\",\"rpm\":0.0},\"mismatch\":0.0},"
                        + "\"launcher\":{\"launched\":0}}",
                heard.ticks.get(1).get("mechanisms").toString());
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
        assertNull("a hello is consumed; the next line is content", SimRunStream.afterHello(hello));
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
        assertEquals(0.5, heard.lines.get(0).seconds(), 0);
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

    @Test
    public void aChildOfThisVersionIsToldWhereToStartAndWhichRobot() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), new Pose2d(12, -7, 1.5), 3L, RobotDescription.reginalds());

        assertEquals(SimRunStream.PROTOCOL, handshake.protocol);
        assertFalse(handshake.message, handshake.refused());
        assertNull("its first line was the hello", handshake.firstContentLine);
        assertEquals(
                SimDriverStation.startLine(new Pose2d(12, -7, 1.5), 3L, RobotDescription.reginalds()),
                handshake.startLine);
    }

    @Test
    public void aChildFromBeforeTheSeedRunsWhenNoSeedIsAskedFor() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(3), new Pose2d(12, 0, 0), null, RobotDescription.reginalds());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                SimDriverStation.startLine(new Pose2d(12, 0, 0), null, RobotDescription.reginalds()),
                handshake.startLine);
    }

    @Test
    public void aChildFromBeforeTheSeedIsRefusedOneByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(3), StartPoses.ORIGIN, 1L, RobotDescription.reginalds());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotSeed(3), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.SEEDED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("clear the seed"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    @Test
    public void aChildFromBeforePlacementRunsFromTheOriginAndPlacesItself() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(2), StartPoses.ORIGIN, null, RobotDescription.reginalds());

        assertFalse(handshake.message, handshake.refused());
        assertNull("it is not told where to start, because it cannot be", handshake.startLine);
    }

    @Test
    public void aChildFromBeforePlacementIsRefusedAnywhereElseByNameAndToldTheFix() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(2), new Pose2d(12, 0, 0), null, RobotDescription.reginalds());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotPlace(2), handshake.outcome);
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.PLACED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("back at the origin"));
    }

    @Test
    public void aVersionOneChildsFirstLineIsContent() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                "[{\"name\":\"an op mode\"}]", StartPoses.ORIGIN, null, RobotDescription.reginalds());

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
                        RobotDescription.reginalds()));

        assertEquals(SimRunStream.PROTOCOL + 1, wrong.childProtocol);
    }

    @Test
    public void aSeededChildFromBeforeTheConstantsIsPlacedAndSeeded() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(4), new Pose2d(12, 0, 0), 3L, RobotDescription.reginalds());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                SimDriverStation.startLine(new Pose2d(12, 0, 0), 3L, RobotDescription.reginalds()),
                handshake.startLine);
    }

    private static RobotDescription hiccuping(double chancePerLoop) {
        return RobotDescription.reginalds()
                .with(Valid.value(Constants.of(java.util.Map.of(Constant.HICCUP_CHANCE, chancePerLoop))));
    }

    @Test
    public void aChildThatTakesTheConstantsSpeaksProtocolSix() {
        assertEquals(6, SimRunStream.CONSTANTS_PROTOCOL);
        assertTrue(SimRunStream.CONSTANTS_PROTOCOL > SimRunStream.SEEDED_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.CONSTANTS_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheConstantsTheAdminChanged() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), new Pose2d(12, -7, 1.5), 3L, hiccuping(0.25));

        assertFalse(handshake.message, handshake.refused());
        assertEquals(SimDriverStation.startLine(new Pose2d(12, -7, 1.5), 3L, hiccuping(0.25)), handshake.startLine);
        assertEquals(
                0.25,
                handshake
                        .startLine
                        .getAsJsonObject("constants")
                        .get("hiccup_chance")
                        .getAsDouble(),
                0);
    }

    @Test
    public void aChildFromBeforeTheConstantsRunsWhenNoneWereChanged() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(5), new Pose2d(12, 0, 0), 3L, RobotDescription.reginalds());

        assertFalse(handshake.message, handshake.refused());
        assertFalse(handshake.startLine.toString(), handshake.startLine.has("constants"));
    }

    @Test
    public void aChildFromBeforeTheConstantsIsRefusedChangedOnesByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(5), StartPoses.ORIGIN, null, hiccuping(0.25));

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotTune(5), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.CONSTANTS_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("Hiccup chance"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertTrue(handshake.message, handshake.message.contains(SimDescription.FILE));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    private static RobotDescription mountedLogoDown() {
        return RobotDescription.of(
                Configuration.reginalds(),
                Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)),
                Constants.defaults(),
                Directions.reginalds());
    }

    @Test
    public void aChildThatTakesTheWholeDescriptionSpeaksProtocolSeven() {
        assertEquals(7, SimRunStream.DESCRIPTION_PROTOCOL);
        assertTrue(SimRunStream.DESCRIPTION_PROTOCOL > SimRunStream.CONSTANTS_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.DESCRIPTION_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheHubAsTheDescriptionMountsIt() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, mountedLogoDown());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                "DOWN", handshake.startLine.getAsJsonObject("hub").get("logo").getAsString());
    }

    @Test
    public void aChildFromBeforeTheDescriptionStillTakesChangedConstants() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.DESCRIPTION_PROTOCOL - 1), StartPoses.ORIGIN, null, hiccuping(0.25));

        assertFalse(handshake.message, handshake.refused());
    }

    @Test
    public void aChildFromBeforeTheDescriptionIsRefusedAHubThatDiffersByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(6), StartPoses.ORIGIN, null, mountedLogoDown());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotDescribe(6), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.DESCRIPTION_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("hub logo DOWN, USB RIGHT"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertTrue(handshake.message, handshake.message.contains(SimDescription.FILE));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    private static RobotDescription intakeReversed() {
        return RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.INTAKE, true));
    }

    @Test
    public void aChildThatTakesTheDirectionsSpeaksProtocolEight() {
        assertEquals(8, SimRunStream.DIRECTIONS_PROTOCOL);
        assertTrue(SimRunStream.DIRECTIONS_PROTOCOL > SimRunStream.DESCRIPTION_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.DIRECTIONS_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldWhichMechanismsTheDescriptionReverses() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, intakeReversed());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                "pushes_out",
                handshake.startLine.getAsJsonObject("directions").get("intake").getAsString());
    }

    @Test
    public void aChildFromBeforeTheDirectionsStillTakesAHubThatDiffers() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.DIRECTIONS_PROTOCOL - 1), StartPoses.ORIGIN, null, mountedLogoDown());

        assertFalse(handshake.message, handshake.refused());
    }

    @Test
    public void aChildFromBeforeTheDirectionsIsRefusedAReversedMechanismByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(7), StartPoses.ORIGIN, null, intakeReversed());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotDirect(7), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.DIRECTIONS_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("intake pushes out on positive power"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertTrue(handshake.message, handshake.message.contains(SimDescription.FILE));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    private static RobotDescription spinnerReversed() {
        return RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.SPINNER, true));
    }

    private static RobotDescription quickerRamp() {
        return RobotDescription.reginalds()
                .with(Valid.value(Constants.of(java.util.Map.of(Constant.RAMP_SECONDS_PER_60_DEGREES, 0.1))));
    }

    @Test
    public void aChildThatTakesTheSpindexerSpeaksProtocolNine() {
        assertEquals(9, SimRunStream.SPINDEXER_PROTOCOL);
        assertTrue(SimRunStream.SPINDEXER_PROTOCOL > SimRunStream.DIRECTIONS_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.SPINDEXER_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheSpindexerAsTheDescriptionHasIt() {
        SimRunStream.Handshake reversed =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, spinnerReversed());
        SimRunStream.Handshake quicker =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, quickerRamp());

        assertFalse(reversed.message, reversed.refused());
        assertEquals(
                "turns_counterclockwise",
                reversed.startLine.getAsJsonObject("directions").get("spinner").getAsString());
        assertFalse(quicker.message, quicker.refused());
    }

    @Test
    public void aChildFromBeforeTheSpindexerStillTakesAReversedIntake() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.SPINDEXER_PROTOCOL - 1), StartPoses.ORIGIN, null, intakeReversed());

        assertFalse(handshake.message, handshake.refused());
    }

    @Test
    public void aChildFromBeforeTheSpindexerIsRefusedASpindexerThatDiffersByNameAndToldTheFix() {
        for (RobotDescription differs : java.util.List.of(spinnerReversed(), quickerRamp())) {
            SimRunStream.Handshake handshake =
                    SimRunStream.handshake(SimRunStream.helloOf(8), StartPoses.ORIGIN, null, differs);

            assertTrue(handshake.refused());
            assertEquals(Outcome.cannotSpindex(8), handshake.outcome);
            assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
            assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.SPINDEXER_PROTOCOL));
            assertTrue(handshake.message, handshake.message.contains(SimDescription.described(differs)));
            assertTrue(handshake.message, handshake.message.contains("Pull develop"));
            assertNull("a refused run is sent nothing", handshake.startLine);
        }
    }

    private static RobotDescription intakeRenamed() {
        return Valid.value(Overrides.asked(
                        RobotDescription.reginalds(),
                        List.of(new Overrides.Kept.Known(new Overrides.Edit.Renamed("intake", "intakeMotor")))))
                .description();
    }

    private static RobotDescription intakeRewired() {
        return Valid.value(Overrides.asked(
                        RobotDescription.reginalds(),
                        List.of(new Overrides.Kept.Known(new Overrides.Edit.Wired("intake", EncoderReads.NOTHING)))))
                .description();
    }

    @Test
    public void aChildThatTakesRenamedDevicesSpeaksProtocolTen() {
        assertEquals(10, SimRunStream.RENAMED_PROTOCOL);
        assertTrue(SimRunStream.RENAMED_PROTOCOL > SimRunStream.SPINDEXER_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.RENAMED_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheNameARenamedDeviceIsCommittedUnder() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, intakeRenamed());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                "intake",
                handshake
                        .startLine
                        .getAsJsonArray("configuration")
                        .get(4)
                        .getAsJsonObject()
                        .get("committed")
                        .getAsString());
    }

    @Test
    public void aChildFromBeforeRenamedDevicesStillTakesAConfigurationThatDiffersOtherwise() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.RENAMED_PROTOCOL - 1), StartPoses.ORIGIN, null, intakeRewired());

        assertFalse(handshake.message, handshake.refused());
    }

    @Test
    public void aChildFromBeforeRenamedDevicesIsRefusedARenamedDeviceByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(9), StartPoses.ORIGIN, null, intakeRenamed());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotRename(9), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.RENAMED_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("intake as intakeMotor"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    private static RobotDescription frontFlywheelReversed() {
        return RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.FLYWHEEL_FRONT, true));
    }

    private static RobotDescription heavierBackFlywheel() {
        return RobotDescription.reginalds()
                .with(Valid.value(
                        Constants.of(java.util.Map.of(Constant.FLYWHEEL_BACK_KA_VOLT_SECONDS_SQUARED_PER_TICK, 0.02))));
    }

    @Test
    public void aChildThatTakesTheLaunchersFlywheelsSpeaksProtocolEleven() {
        assertEquals(11, SimRunStream.LAUNCHER_PROTOCOL);
        assertTrue(SimRunStream.LAUNCHER_PROTOCOL > SimRunStream.RENAMED_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.LAUNCHER_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheFlywheelsAsTheDescriptionHasThem() {
        SimRunStream.Handshake reversed =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, frontFlywheelReversed());
        SimRunStream.Handshake heavier =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, heavierBackFlywheel());

        assertFalse(reversed.message, reversed.refused());
        assertEquals(
                "turns_clockwise",
                reversed.startLine
                        .getAsJsonObject("directions")
                        .get("flywheel_front")
                        .getAsString());
        assertFalse(heavier.message, heavier.refused());
    }

    @Test
    public void aChildFromBeforeTheFlywheelsStillTakesAReversedSpinner() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.LAUNCHER_PROTOCOL - 1), StartPoses.ORIGIN, null, spinnerReversed());

        assertFalse(handshake.message, handshake.refused());
    }

    @Test
    public void aChildFromBeforeTheFlywheelsIsToldOnlyTheDirectionsThatDifferSoItKnowsEveryMechanismNamed() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.LAUNCHER_PROTOCOL - 1), StartPoses.ORIGIN, null, spinnerReversed());

        assertEquals(
                "{\"spinner\":\"turns_counterclockwise\"}",
                handshake.startLine.getAsJsonObject("directions").toString());
    }

    @Test
    public void aChildFromBeforeTheFlywheelsIsRefusedALauncherThatDiffersByNameAndToldTheFix() {
        for (RobotDescription differs : List.of(frontFlywheelReversed(), heavierBackFlywheel())) {
            SimRunStream.Handshake handshake =
                    SimRunStream.handshake(SimRunStream.helloOf(10), StartPoses.ORIGIN, null, differs);

            assertTrue(handshake.refused());
            assertEquals(Outcome.cannotLaunch(10), handshake.outcome);
            assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
            assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.LAUNCHER_PROTOCOL));
            assertTrue(handshake.message, handshake.message.contains(SimDescription.described(differs)));
            assertTrue(handshake.message, handshake.message.contains("Pull develop"));
            assertNull("a refused run is sent nothing", handshake.startLine);
        }
    }

    private static RobotDescription steeperLaunch() {
        return RobotDescription.reginalds()
                .with(Valid.value(Constants.of(java.util.Map.of(Constant.LAUNCHER_PITCH_DEGREES, 75.0))));
    }

    @Test
    public void aChildThatLaunchesSpeaksProtocolTwelve() {
        assertEquals(12, SimRunStream.LAUNCH_PROTOCOL);
        assertTrue(SimRunStream.LAUNCH_PROTOCOL > SimRunStream.LAUNCHER_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.LAUNCH_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheLaunchersMountingAsTheDescriptionHasIt() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, steeperLaunch());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                75,
                handshake
                        .startLine
                        .getAsJsonObject("constants")
                        .get("launcher_pitch_degrees")
                        .getAsDouble(),
                0);
    }

    @Test
    public void aChildFromBeforeTheLaunchStillTakesAReversedFlywheel() {
        SimRunStream.Handshake handshake = SimRunStream.handshake(
                SimRunStream.helloOf(SimRunStream.LAUNCH_PROTOCOL - 1),
                StartPoses.ORIGIN,
                null,
                frontFlywheelReversed());

        assertFalse(handshake.message, handshake.refused());
    }

    @Test
    public void aChildFromBeforeTheLaunchIsRefusedAMountingThatDiffersByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(11), StartPoses.ORIGIN, null, steeperLaunch());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotMount(11), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.LAUNCH_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains(SimDescription.described(steeperLaunch())));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }

    private static RobotDescription leftFrontReversed() {
        return RobotDescription.reginalds().with(Directions.reginalds().with(Mechanism.LEFT_FRONT, true));
    }

    @Test
    public void aChildThatTakesTheDriveMotorsDirectionsSpeaksProtocolThirteen() {
        assertEquals(13, SimRunStream.DRIVE_MOTORS_PROTOCOL);
        assertTrue(SimRunStream.DRIVE_MOTORS_PROTOCOL > SimRunStream.LAUNCH_PROTOCOL);
        assertTrue(SimRunStream.PROTOCOL >= SimRunStream.DRIVE_MOTORS_PROTOCOL);
    }

    @Test
    public void aChildOfThisVersionIsToldTheDriveMotorsDirectionsAsTheDescriptionHasThem() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.hello(), StartPoses.ORIGIN, null, leftFrontReversed());

        assertFalse(handshake.message, handshake.refused());
        assertEquals(
                "{\"left_front\":\"drives_backward\"}",
                handshake.startLine.getAsJsonObject("directions").toString());
    }

    @Test
    public void aChildFromBeforeTheDriveMotorsDirectionsStillTakesAReversedFlywheelAndAMovedLauncher() {
        for (RobotDescription differs : List.of(frontFlywheelReversed(), steeperLaunch())) {
            SimRunStream.Handshake handshake = SimRunStream.handshake(
                    SimRunStream.helloOf(SimRunStream.DRIVE_MOTORS_PROTOCOL - 1), StartPoses.ORIGIN, null, differs);

            assertFalse(handshake.message, handshake.refused());
        }
    }

    @Test
    public void aChildFromBeforeTheDriveMotorsDirectionsIsRefusedAReversedDriveMotorByNameAndToldTheFix() {
        SimRunStream.Handshake handshake =
                SimRunStream.handshake(SimRunStream.helloOf(12), StartPoses.ORIGIN, null, leftFrontReversed());

        assertTrue(handshake.refused());
        assertEquals(Outcome.cannotDrive(12), handshake.outcome);
        assertTrue(handshake.outcome, handshake.outcome.startsWith("wrong protocol"));
        assertTrue(handshake.message, handshake.message.contains("protocol " + SimRunStream.DRIVE_MOTORS_PROTOCOL));
        assertTrue(handshake.message, handshake.message.contains("left_front drives backward on positive power"));
        assertTrue(handshake.message, handshake.message.contains("Pull develop"));
        assertNull("a refused run is sent nothing", handshake.startLine);
    }
}
