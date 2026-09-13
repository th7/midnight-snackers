package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

/**
 * Serves a run to a browser while it is still going: the replay page in live mode at {@code /},
 * and {@code /ticks?from=N} for everything recorded since tick N. The run is a
 * {@link SimReplayPage.Source}, normally a {@link SimRecording} in this JVM.
 */
public final class SimLiveServer {
    private final TinyHttpServer http;
    private final SimReplayPage.Source recording;
    private volatile boolean viewerSawOutcome = false;

    private SimLiveServer(SimReplayPage.Source recording, int port) {
        this.recording = recording;
        Router routes = new Router()
                .route("GET", "/", (request, params) -> Response.html(SimReplayPage.page(recording, true)))
                .route("GET", "/ticks", (request, params) -> {
                    String body = SimReplayPage.update(recording, request.queryInt("from", 0));
                    if (recording.outcome() != null) {
                        viewerSawOutcome = true;
                    }
                    return Response.json(body);
                });
        this.http = TinyHttpServer.start(port, "sim-live-view", routes);
    }

    /**
     * @param port the port to listen on, or 0 for any free port (see {@link #port()})
     */
    public static SimLiveServer start(SimReplayPage.Source recording, int port) {
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

}
