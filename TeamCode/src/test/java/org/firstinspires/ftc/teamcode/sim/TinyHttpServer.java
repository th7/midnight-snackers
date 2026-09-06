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
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Function;

/**
 * A deliberately small HTTP/1.0 responder on a server socket, for the simulator's browser pages.
 * The Android compile classpath offers no HTTP server, and this only ever talks to a developer's
 * browser: no keep-alive, no request bodies, no TLS. It listens on every interface because when
 * the tests run in a container, the published port arrives over the bridge, not loopback.
 */
public final class TinyHttpServer {
    public static final class Request {
        public final String method;
        public final String path;
        public final Map<String, String> query;

        Request(String method, String path, Map<String, String> query) {
            this.method = method;
            this.path = path;
            this.query = query;
        }

        public String query(String key) {
            return query.get(key);
        }
    }

    public static final class Response {
        public final int status;
        public final String contentType;
        public final String body;

        public Response(int status, String contentType, String body) {
            this.status = status;
            this.contentType = contentType;
            this.body = body;
        }

        public static Response html(String body) {
            return new Response(200, "text/html; charset=utf-8", body);
        }

        public static Response json(String body) {
            return new Response(200, "application/json; charset=utf-8", body);
        }

        public static Response error(int status, String message) {
            return new Response(status, "text/plain; charset=utf-8", message);
        }
    }

    private static final InetAddress ANY_INTERFACE = null;

    private final ServerSocket socket;
    private final Function<Request, Response> handler;
    private final ExecutorService connections = Executors.newCachedThreadPool();

    private TinyHttpServer(ServerSocket socket, Function<Request, Response> handler) {
        this.socket = socket;
        this.handler = handler;
    }

    /**
     * @param port the port to listen on, or 0 for any free port (see {@link #port()})
     */
    public static TinyHttpServer start(int port, String threadName, Function<Request, Response> handler) {
        try {
            ServerSocket socket = new ServerSocket(port, 50, ANY_INTERFACE);
            TinyHttpServer server = new TinyHttpServer(socket, handler);
            Thread acceptor = new Thread(server::acceptLoop, threadName);
            acceptor.setDaemon(true);
            acceptor.start();
            return server;
        } catch (IOException e) {
            throw new UncheckedIOException("could not listen on port " + port, e);
        }
    }

    public int port() {
        return socket.getLocalPort();
    }

    public String url() {
        return "http://localhost:" + port() + "/";
    }

    public InetAddress bindAddress() {
        return socket.getInetAddress();
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
                // headers are not needed for these pages
            }
            Response response;
            try {
                response = handler.apply(parse(requestLine));
            } catch (RuntimeException e) {
                response = Response.error(500, e.toString());
            }
            write(out, response);
        } catch (IOException e) {
            // the browser went away mid-response; nothing to do
        }
    }

    private static Request parse(String requestLine) {
        String[] parts = requestLine.split(" ");
        String method = parts.length > 0 ? parts[0] : "GET";
        String target = parts.length > 1 ? parts[1] : "/";
        int q = target.indexOf('?');
        String path = q < 0 ? target : target.substring(0, q);
        Map<String, String> query = new HashMap<>();
        if (q >= 0) {
            for (String pair : target.substring(q + 1).split("&")) {
                int eq = pair.indexOf('=');
                String key = eq < 0 ? pair : pair.substring(0, eq);
                String value = eq < 0 ? "" : pair.substring(eq + 1);
                query.put(URLDecoder.decode(key, StandardCharsets.UTF_8), URLDecoder.decode(value, StandardCharsets.UTF_8));
            }
        }
        return new Request(method, path, query);
    }

    private static void write(OutputStream out, Response response) throws IOException {
        byte[] bytes = response.body.getBytes(StandardCharsets.UTF_8);
        String head = "HTTP/1.0 " + response.status + " " + reason(response.status) + "\r\n"
                + "Content-Type: " + response.contentType + "\r\n"
                + "Content-Length: " + bytes.length + "\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n\r\n";
        out.write(head.getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
        out.flush();
    }

    private static String reason(int status) {
        switch (status) {
            case 200: return "OK";
            case 404: return "Not Found";
            case 405: return "Method Not Allowed";
            case 409: return "Conflict";
            case 500: return "Internal Server Error";
            default: return "Status " + status;
        }
    }
}
