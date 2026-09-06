package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.junit.After;
import org.junit.Test;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.List;

public class SimLiveServerTest {
    private final SimRecording recording = new SimRecording("LiveAuto");
    private final SimLiveServer server = SimLiveServer.start(recording, 0);

    @After
    public void stopServer() {
        server.stop();
    }

    @Test
    public void servesTheReplayPageInLiveMode() throws IOException {
        String html = get("/");

        assertTrue(html.contains("<canvas"));
        assertTrue(html.contains("<title>LiveAuto"));
        assertTrue(html.contains("\"live\":true"));
    }

    @Test
    public void servesTicksAddedAfterThePageWasLoaded() throws IOException {
        get("/");
        recording.add(tick(0.0, "waiting"));
        recording.add(tick(0.5, "driving"));

        String all = get("/ticks?from=0");
        String rest = get("/ticks?from=1");

        assertTrue(all.contains("\"waiting\"") && all.contains("\"driving\""));
        assertFalse(rest.contains("\"waiting\""));
        assertTrue(rest.contains("\"driving\""));
        assertTrue(all.contains("\"outcome\":null"));
    }

    @Test
    public void reportsTheOutcomeOnceTheRunIsOverAndKnowsWhenAViewerSawIt() throws IOException {
        recording.add(tick(0.0, "waiting"));
        assertFalse(server.viewerSawOutcome());

        recording.finish("done");
        String response = get("/ticks?from=1");

        assertTrue(response.contains("\"outcome\":\"done\""));
        assertTrue(server.viewerSawOutcome());
    }

    @Test
    public void listensOnTheRequestedPort() {
        assertTrue(server.port() > 0);
        assertEquals("http://localhost:" + server.port() + "/", server.url());
    }

    private SimRecording.Tick tick(double seconds, String step) {
        return new SimRecording.Tick(seconds, new Pose2d(0, 0, 0), step, new double[]{0, 0, 0, 0}, List.of());
    }

    private String get(String path) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(server.url() + path.substring(1)).openConnection();
        try (InputStream in = connection.getInputStream()) {
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } finally {
            connection.disconnect();
        }
    }
}
