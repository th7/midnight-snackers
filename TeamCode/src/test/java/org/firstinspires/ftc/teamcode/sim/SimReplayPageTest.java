package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

public class SimReplayPageTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    @Test
    public void writesASelfContainedPageCarryingEveryRecordedTick() throws IOException {
        SimRecording recording = new SimRecording("SquareAuto");
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#4CAF50").strokePolyline(new double[]{0, 24}, new double[]{0, 24});
        packet.put("xError", 0.25);
        recording.add(new SimRecording.Tick(0.0, new Pose2d(0, 0, 0), "1. waitFor 5.0",
                new double[]{0, 0, 0, 0}, List.of()));
        recording.add(new SimRecording.Tick(0.5, new Pose2d(12.5, -3, Math.PI / 2), "2. driveTo 24, 0, 0",
                new double[]{1, 0.75, -0.5, 0.25}, List.of(packet)));
        recording.add(new SimRecording.Tick(1.0 / 3, new Pose2d(1.0 / 3, 0, 0), "3. rounding",
                new double[]{1, 0.75, -0.5, 0.25}, List.of(packet)));
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
        assertFalse("loads nothing from the network", html.matches("(?s).*(src|href)=\"http.*"));
    }
}
