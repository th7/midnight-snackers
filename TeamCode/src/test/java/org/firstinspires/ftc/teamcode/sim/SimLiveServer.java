package org.firstinspires.ftc.teamcode.sim;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Serves a {@link SimRecording} to a browser while the run is still going: the replay page in
 * live mode at {@code /}, and {@code /ticks?from=N} for everything recorded since tick N.
 * <p>
 * A deliberately small HTTP/1.0 responder on a server socket, since the Android compile
 * classpath offers no HTTP server and this only ever talks to a browser on localhost.
 */
public final class SimLiveServer {
    private final ServerSocket socket;
    private final SimRecording recording;
    private final ExecutorService connections = Executors.newCachedThreadPool();
    private volatile boolean viewerSawOutcome = false;

    private SimLiveServer(ServerSocket socket, SimRecording recording) {
        this.socket = socket;
        this.recording = recording;
    }

    /**
     * @param port the port to listen on, or 0 for any free port (see {@link #port()})
     */
    public static SimLiveServer start(SimRecording recording, int port) {
        try {
            ServerSocket socket = new ServerSocket(port, 50, InetAddress.getLoopbackAddress());
            SimLiveServer live = new SimLiveServer(socket, recording);
            Thread acceptor = new Thread(live::acceptLoop, "sim-live-view");
            acceptor.setDaemon(true);
            acceptor.start();
            return live;
        } catch (IOException e) {
            throw new UncheckedIOException("could not start the live view on port " + port, e);
        }
    }

    public int port() {
        return socket.getLocalPort();
    }

    public String url() {
        return "http://localhost:" + port() + "/";
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
        try {
            socket.close();
        } catch (IOException ignored) {
            // closing is best effort; the acceptor loop ends either way
        }
        connections.shutdownNow();
    }

    private void acceptLoop() {
        while (!socket.isClosed()) {
            try {
                Socket connection = socket.accept();
                connections.submit(() -> handle(connection));
            } catch (SocketException closed) {
                return;
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
    }

    private void handle(Socket connection) {
        try (connection;
             BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream(), StandardCharsets.US_ASCII));
             OutputStream out = connection.getOutputStream()) {
            String requestLine = in.readLine();
            if (requestLine == null) {
                return;
            }
            for (String header = in.readLine(); header != null && !header.isEmpty(); header = in.readLine()) {
                // headers are not needed for this server
            }
            String[] parts = requestLine.split(" ");
            String target = parts.length > 1 ? parts[1] : "/";
            String path = target.contains("?") ? target.substring(0, target.indexOf('?')) : target;
            String query = target.contains("?") ? target.substring(target.indexOf('?') + 1) : "";
            if (path.equals("/")) {
                respond(out, 200, "text/html; charset=utf-8", SimReplayPage.page(recording, true));
            } else if (path.equals("/ticks")) {
                respond(out, 200, "application/json; charset=utf-8", ticks(query));
            } else {
                respond(out, 404, "text/plain; charset=utf-8", "not found: " + path);
            }
        } catch (IOException e) {
            // the browser went away mid-response; nothing to do
        }
    }

    private String ticks(String query) {
        int from = 0;
        if (query.startsWith("from=")) {
            from = Integer.parseInt(query.substring("from=".length()));
        }
        String body = SimReplayPage.update(recording, from);
        if (recording.finished()) {
            viewerSawOutcome = true;
        }
        return body;
    }

    private static void respond(OutputStream out, int status, String contentType, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        String head = "HTTP/1.0 " + status + (status == 200 ? " OK" : " Not Found") + "\r\n"
                + "Content-Type: " + contentType + "\r\n"
                + "Content-Length: " + bytes.length + "\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n";
        out.write(head.getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
        out.flush();
    }
}
