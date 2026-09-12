package org.firstinspires.ftc.teamcode.sim;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Function;

/**
 * A deliberately small HTTP/1.0 responder on a server socket, for the simulator's browser pages.
 * The Android compile classpath offers no HTTP server, and this only ever talks to a developer's
 * browser: no keep-alive, no TLS, bodies capped at {@link #MAX_BODY_BYTES}. By default it listens
 * on every interface because when the tests run in a container, the published port arrives over
 * the bridge, not loopback; {@link #start(InetAddress, int, String, Function)} binds one address
 * instead, which is how a page is kept reachable only from the machine it runs on.
 */
public final class TinyHttpServer {
    public static final int MAX_BODY_BYTES = 1 << 20;

    public static final class Request {
        public final String method;
        public final String path;
        public final Map<String, String> query;
        /** Header names lower-cased. */
        public final Map<String, String> headers;
        public final String body;
        public final InetAddress remoteAddress;

        Request(String method, String path, Map<String, String> query, Map<String, String> headers, String body,
                InetAddress remoteAddress) {
            this.method = method;
            this.path = path;
            this.query = query;
            this.headers = headers;
            this.body = body;
            this.remoteAddress = remoteAddress;
        }

        public String query(String key) {
            return query.get(key);
        }

        public String header(String name) {
            return headers.get(name.toLowerCase(Locale.ROOT));
        }

        public String cookie(String name) {
            String cookies = header("cookie");
            if (cookies == null) {
                return null;
            }
            for (String pair : cookies.split(";")) {
                int eq = pair.indexOf('=');
                if (eq >= 0 && pair.substring(0, eq).trim().equals(name)) {
                    return pair.substring(eq + 1).trim();
                }
            }
            return null;
        }
    }

    public static final class Response {
        public final int status;
        public final String contentType;
        public final String body;
        public final Map<String, String> headers;

        public Response(int status, String contentType, String body) {
            this(status, contentType, body, Collections.emptyMap());
        }

        private Response(int status, String contentType, String body, Map<String, String> headers) {
            this.status = status;
            this.contentType = contentType;
            this.body = body;
            this.headers = headers;
        }

        public static Response html(String body) {
            return new Response(200, "text/html; charset=utf-8", body);
        }

        public static Response json(String body) {
            return new Response(200, "application/json; charset=utf-8", body);
        }

        public static Response json(int status, String body) {
            return new Response(status, "application/json; charset=utf-8", body);
        }

        public static Response error(int status, String message) {
            return new Response(status, "text/plain; charset=utf-8", message);
        }

        public Response withHeader(String name, String value) {
            Map<String, String> copy = new LinkedHashMap<>(headers);
            copy.put(name, value);
            return new Response(status, contentType, body, Collections.unmodifiableMap(copy));
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
     * Listens on every interface.
     *
     * @param port the port to listen on, or 0 for any free port (see {@link #port()})
     */
    public static TinyHttpServer start(int port, String threadName, Function<Request, Response> handler) {
        return start(ANY_INTERFACE, port, threadName, handler);
    }

    /**
     * @param bind the one address to listen on; the operating system then refuses connections
     *             arriving on any other interface
     */
    public static TinyHttpServer start(InetAddress bind, int port, String threadName, Function<Request, Response> handler) {
        try {
            ServerSocket socket = new ServerSocket(port, 50, bind);
            TinyHttpServer server = new TinyHttpServer(socket, handler);
            Thread acceptor = new Thread(server::acceptLoop, threadName);
            acceptor.setDaemon(true);
            acceptor.start();
            return server;
        } catch (IOException e) {
            throw new UncheckedIOException("could not listen on " + (bind == null ? "*" : bind.getHostAddress()) + ":" + port, e);
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
             InputStream in = new BufferedInputStream(connection.getInputStream());
             OutputStream out = connection.getOutputStream()) {
            String requestLine = readLine(in);
            if (requestLine == null) {
                return;
            }
            Map<String, String> headers = new HashMap<>();
            for (String line = readLine(in); line != null && !line.isEmpty(); line = readLine(in)) {
                int colon = line.indexOf(':');
                if (colon > 0) {
                    headers.put(line.substring(0, colon).trim().toLowerCase(Locale.ROOT), line.substring(colon + 1).trim());
                }
            }
            long length = contentLength(headers);
            if (length > MAX_BODY_BYTES) {
                write(out, Response.error(413, "request body over " + MAX_BODY_BYTES + " bytes"));
                drain(in, length);
                return;
            }
            byte[] body = in.readNBytes((int) length);
            Response response;
            try {
                response = handler.apply(parse(requestLine, headers, body, connection.getInetAddress()));
            } catch (RuntimeException e) {
                response = Response.error(500, e.toString());
            }
            write(out, response);
        } catch (IOException e) {
            // the browser went away mid-response; nothing to do
        }
    }

    /**
     * Lets the client finish sending an over-cap body so it reads the 413 instead of a reset,
     * within reason: past the drain limit the connection is simply closed.
     */
    private static void drain(InputStream in, long length) throws IOException {
        long remaining = Math.min(length, 16L * MAX_BODY_BYTES);
        byte[] sink = new byte[8192];
        while (remaining > 0) {
            int read = in.read(sink, 0, (int) Math.min(sink.length, remaining));
            if (read < 0) {
                return;
            }
            remaining -= read;
        }
    }

    private static long contentLength(Map<String, String> headers) {
        String value = headers.get("content-length");
        if (value == null) {
            return 0;
        }
        try {
            return Math.max(0, Long.parseLong(value.trim()));
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    /** One header line, without its line ending; null at end of stream. */
    private static String readLine(InputStream in) throws IOException {
        ByteArrayOutputStream line = new ByteArrayOutputStream();
        int c;
        while ((c = in.read()) >= 0) {
            if (c == '\n') {
                break;
            }
            if (c != '\r') {
                line.write(c);
            }
            if (line.size() > 64 * 1024) {
                throw new IOException("header line too long");
            }
        }
        if (c < 0 && line.size() == 0) {
            return null;
        }
        return line.toString(StandardCharsets.ISO_8859_1);
    }

    private static Request parse(String requestLine, Map<String, String> headers, byte[] body, InetAddress remoteAddress) {
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
        return new Request(method, path, query, Collections.unmodifiableMap(headers),
                new String(body, StandardCharsets.UTF_8), remoteAddress);
    }

    private static void write(OutputStream out, Response response) throws IOException {
        byte[] bytes = response.body.getBytes(StandardCharsets.UTF_8);
        StringBuilder head = new StringBuilder("HTTP/1.0 " + response.status + " " + reason(response.status) + "\r\n"
                + "Content-Type: " + response.contentType + "\r\n"
                + "Content-Length: " + bytes.length + "\r\n"
                + "Cache-Control: no-store\r\n"
                + "Connection: close\r\n");
        response.headers.forEach((name, value) -> head.append(name).append(": ").append(value).append("\r\n"));
        head.append("\r\n");
        out.write(head.toString().getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
        out.flush();
    }

    private static String reason(int status) {
        switch (status) {
            case 200: return "OK";
            case 400: return "Bad Request";
            case 403: return "Forbidden";
            case 404: return "Not Found";
            case 405: return "Method Not Allowed";
            case 409: return "Conflict";
            case 413: return "Payload Too Large";
            case 415: return "Unsupported Media Type";
            case 429: return "Too Many Requests";
            case 500: return "Internal Server Error";
            default: return "Status " + status;
        }
    }
}
