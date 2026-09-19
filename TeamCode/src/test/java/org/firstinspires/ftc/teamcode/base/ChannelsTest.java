package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.junit.Test;

public class ChannelsTest {
    private final FakeTelemetry screen = new FakeTelemetry();
    private final Channels channels = new Channels(screen);
    private final Prints drive = channels.channel("Drive");

    @Test
    public void aChannelIsOffUntilSomeoneAsksForIt() {
        drive.addData("pose", "(1, 2, 3)");

        assertEquals(List.of(), screen.captions);
        assertFalse(channels.isOn("Drive"));
    }

    @Test
    public void whatPrintsToAChannelThatIsOnReachesTheScreenUnderItsName() {
        channels.toggle("Drive");

        drive.addData("pose", "(1, 2, 3)");

        assertEquals(List.of("Drive.pose"), screen.captions);
        assertTrue(channels.isOn("Drive"));
    }

    @Test
    public void theFormattingOverloadIsCarriedThroughToo() {
        channels.toggle("Drive");

        drive.addData("x, y", "%.1f, %.1f", 1.25, 2.5);

        assertEquals(List.of("Drive.x, y"), screen.captions);
    }

    @Test
    public void togglingAgainTurnsItBackOff() {
        channels.toggle("Drive");
        drive.addData("pose", "first");

        channels.toggle("Drive");
        drive.addData("pose", "second");

        assertEquals("only the first got through", List.of("Drive.pose"), screen.captions);
    }

    @Test
    public void oneButtonCanReachSeveralChannelsAtOnce() {
        Prints localizer = channels.channel("Localizer");

        channels.toggle("Drive", "Localizer");

        drive.addData("pose", "a");
        localizer.addData("speed", "b");
        assertEquals(List.of("Drive.pose", "Localizer.speed"), screen.captions);
    }

    @Test
    public void channelsAreIndependentOfOneAnother() {
        Prints camera = channels.channel("Camera");
        channels.toggle("Drive");

        drive.addData("pose", "a");
        camera.addData("tag", "b");

        assertEquals(List.of("Drive.pose"), screen.captions);
        assertEquals(java.util.Set.of("Drive"), channels.areOn());
    }

    @Test
    public void printingStraightToTheDriverStationIsNotAChannelAndIsAlwaysOn() {
        channels.addData("Current Step:", "driveForward");

        assertEquals(List.of("Current Step:"), screen.captions);
    }

    @Test
    public void nowhereSwallowsEverything() {
        Prints.NOWHERE.addData("pose", "(1, 2, 3)");
        Prints.NOWHERE.addData("x", "%.1f", 1.0);

        assertEquals(List.of(), screen.captions);
    }
}
