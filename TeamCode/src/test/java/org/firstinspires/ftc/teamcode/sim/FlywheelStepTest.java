package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.reginald.opmode.FlywheelStep;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.Written;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class FlywheelStepTest {
    private static final String HEADER =
            "seconds,front_ticks_per_second,back_ticks_per_second,front_amps,back_amps,battery_volts";

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private FlywheelStep ranFor(double seconds, File saveTo) {
        SimRobot sim = new SimRobot(RobotDescription.reginalds(), SimNoise.NONE);
        sim.setPose(new Pose2d(0, 0, 0));
        FlywheelStep opMode = new FlywheelStep(sim::nanoTime, () -> saveTo);
        SimRunner.record(
                new SimRecording("FlywheelStep"),
                opMode,
                sim,
                seconds,
                folder.getRoot().toPath(),
                new SimDriverStation(),
                SimRunner.Pace.FASTEST);
        return opMode;
    }

    private static File only(File saved, String extension) {
        File[] found = saved.listFiles((dir, name) -> name.endsWith(extension));
        assertTrue("one " + extension + " file in " + saved, found != null && found.length == 1);
        return found[0];
    }

    private static double[] row(String line) {
        return Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
    }

    @Test
    public void itRecordsBothWheelsFromRestToTheirSpeedForTenSeconds() throws IOException {
        File saved = folder.newFolder("data");
        ranFor(11, saved);

        List<String> lines = Files.readAllLines(only(saved, ".csv").toPath());

        assertEquals(HEADER, lines.get(0));
        double[] first = row(lines.get(1));
        double[] last = row(lines.get(lines.size() - 1));
        assertEquals(0, first[0], 0.05);
        assertEquals(0, first[1], 50);
        assertEquals(0, first[2], 50);
        assertEquals(10, last[0], 0.05);
        assertEquals(1200, Math.abs(last[1]), 60);
        assertEquals(1200, Math.abs(last[2]), 60);
        assertEquals(SimDevices.BATTERY_VOLTS, last[5], 2);
        assertTrue("a row each loop: " + lines.size(), lines.size() > 400);
    }

    @Test
    public void itLeavesBothWheelsUnpoweredOnceTheRecordingIsDone() throws IOException {
        FlywheelStep opMode = ranFor(11, folder.newFolder("data"));

        assertEquals(
                new Written.Powered(0),
                opMode.hardwareMap.get(FakeDcMotorEx.class, "flywheelFront").written());
        assertEquals(
                new Written.Powered(0),
                opMode.hardwareMap.get(FakeDcMotorEx.class, "flywheelBack").written());
    }

    @Test
    public void itSavesNothingWhileTheRecordingIsStillRunning() throws IOException {
        File saved = folder.newFolder("data");
        ranFor(5, saved);

        assertEquals(List.of(), Arrays.asList(saved.list()));
    }

    @Test
    public void aRecordingStoppedEarlyIsSavedWithWhatItHolds() throws IOException {
        File saved = folder.newFolder("data");
        FlywheelStep opMode = ranFor(5, saved);

        opMode.stop();

        List<String> lines = Files.readAllLines(only(saved, ".csv").toPath());
        assertEquals(HEADER, lines.get(0));
        assertEquals(5, row(lines.get(lines.size() - 1))[0], 0.05);
        assertEquals(
                new Written.Powered(0),
                opMode.hardwareMap.get(FakeDcMotorEx.class, "flywheelFront").written());
    }

    @Test
    public void itsSummaryNamesTheTargetAndTheGainsEachMotorDeclares() throws IOException {
        File saved = folder.newFolder("data");
        ranFor(11, saved);

        List<String> summary = Files.readAllLines(only(saved, ".txt").toPath());

        assertTrue(String.join("\n", summary), summary.contains("target ticks per second: 1200.0"));
        assertTrue(
                String.join("\n", summary),
                summary.contains("flywheelFront RUN_USING_ENCODER PIDF: p=0.0 i=0.0 d=0.0 f=0.0 algorithm=PIDF"));
        assertTrue(
                String.join("\n", summary),
                summary.contains("flywheelBack RUN_USING_ENCODER PIDF: p=0.0 i=0.0 d=0.0 f=0.0 algorithm=PIDF"));
        assertTrue(
                String.join("\n", summary),
                summary.stream().anyMatch(line -> line.startsWith("battery volts at start: ")));
    }
}
