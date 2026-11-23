package org.firstinspires.ftc.teamcode.framework;

import static org.junit.Assert.assertEquals;

import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.framework.robot.Channels;
import org.firstinspires.ftc.framework.robot.Prints;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Test;

public class ChannelsTest {
    private final FakeTelemetry telemetry = new FakeTelemetry();
    private final Channels channels = new Channels(telemetry);
    private final Prints localizer = channels.channel("Localizer");
    private final Prints drive = channels.channel("Drive");
    private final Prints intake = channels.channel("Intake");

    private List<String> printedByEach() {
        telemetry.captions.clear();
        localizer.addData("x", 1);
        drive.addData("power", 1);
        intake.addData("on", true);
        return List.copyOf(telemetry.captions);
    }

    @Test
    public void everyChannelIsOffUntilADriverAsks() {
        assertEquals(Optional.empty(), channels.showing());
        assertEquals(List.of(), printedByEach());
    }

    @Test
    public void eachStepShowsTheNextChannelByNameAndOnlyThatOne() {
        channels.step();
        assertEquals(Optional.of("Drive"), channels.showing());
        assertEquals(List.of("Drive.power"), printedByEach());

        channels.step();
        assertEquals(Optional.of("Intake"), channels.showing());
        assertEquals(List.of("Intake.on"), printedByEach());

        channels.step();
        assertEquals(Optional.of("Localizer"), channels.showing());
        assertEquals(List.of("Localizer.x"), printedByEach());
    }

    @Test
    public void aStepPastTheLastChannelShowsNoneAndTheNextStartsOver() {
        channels.step();
        channels.step();
        channels.step();

        channels.step();
        assertEquals(Optional.empty(), channels.showing());
        assertEquals(List.of(), printedByEach());

        channels.step();
        assertEquals(Optional.of("Drive"), channels.showing());
    }

    @Test
    public void withNoChannelMadeAStepShowsNone() {
        Channels none = new Channels(new FakeTelemetry());

        none.step();

        assertEquals(Optional.empty(), none.showing());
    }
}
