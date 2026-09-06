package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

/**
 * Serves a {@link SimRecording} to a browser while the run is still going: the replay page in
 * live mode at {@code /}, and {@code /ticks?from=N} for everything recorded since tick N.
 */
public final class SimLiveServer {
    private final TinyHttpServer http;
    private final SimRecording recording;
    private volatile boolean viewerSawOutcome = false;

    private SimLiveServer(SimRecording recording, int port) {
        this.recording = recording;
        this.http = TinyHttpServer.start(port, "sim-live-view", this::handle);
    }

    /**
     * @param port the port to listen on, or 0 for any free port (see {@link #port()})
     */
    public static SimLiveServer start(SimRecording recording, int port) {
        return new SimLiveServer(recording, port);
    }

    public int port() {
        return http.port();
    }

    public String url() {
        return http.url();
    }

    /**
     * Whether some viewer has fetched an update that carried the run's outcome, i.e. has seen the end.
     */
    public boolean viewerSawOutcome() {
        return viewerSawOutcome;
    }

    /**
     * Block until a viewer has seen the end, or {@code maxSeconds} pass.
     */
    public void awaitViewerSawOutcome(double maxSeconds) {
        long deadline = System.nanoTime() + (long) (maxSeconds * 1_000_000_000L);
        while (!viewerSawOutcome && System.nanoTime() < deadline) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    public void stop() {
        http.stop();
    }

    private Response handle(Request request) {
        if (request.path.equals("/")) {
            return Response.html(SimReplayPage.page(recording, true));
        }
        if (request.path.equals("/ticks")) {
            String body = SimReplayPage.update(recording, from(request));
            if (recording.finished()) {
                viewerSawOutcome = true;
            }
            return Response.json(body);
        }
        return Response.error(404, "not found: " + request.path);
    }

    static int from(Request request) {
        String from = request.query("from");
        return from == null ? 0 : Integer.parseInt(from);
    }
}
