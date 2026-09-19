package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

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

    public static SimLiveServer start(SimReplayPage.Source recording, int port) {
        return new SimLiveServer(recording, port);
    }

    public int port() {
        return http.port();
    }

    public String url() {
        return http.url();
    }

    public boolean viewerSawOutcome() {
        return viewerSawOutcome;
    }

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
