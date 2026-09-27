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
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Function;
import java.util.zip.Deflater;
import java.util.zip.GZIPOutputStream;

public final class TinyHttpServer {
    public static final int MAX_BODY_BYTES = 1 << 20;

    public static final class Request {
        public final String method;
        public final String path;
        public final Map<String, String> query;

        public final Map<String, String> headers;

        public final String body;
        public final InetAddress remoteAddress;

        public Request(
                String method,
                String path,
                Map<String, String> query,
                Map<String, String> headers,
                String body,
                InetAddress remoteAddress) {
            this.method = method;
            this.path = path;
            this.query = query;
            this.headers = headers;
            this.body = body;
            this.remoteAddress = remoteAddress;
        }

        public static Request of(String method, String target, String body) {
            return of(method, target, Map.of(), body);
        }

        public static Request of(String method, String target, Map<String, String> headers, String body) {
            return parse(
                    method + " " + target,
                    headers,
                    body.getBytes(StandardCharsets.UTF_8),
                    InetAddress.getLoopbackAddress());
        }

        public String query(String key) {
            return query.get(key);
        }

        public int queryInt(String key, int fallback) {
            String value = query.get(key);
            return value == null ? fallback : Integer.parseInt(value);
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

        private final byte[] binary;

        public final Map<String, String> headers;

        public final boolean revalidated;

        public Response(int status, String contentType, String body) {
            this(status, contentType, body, null, Collections.emptyMap(), false);
        }

        private Response(
                int status,
                String contentType,
                String body,
                byte[] binary,
                Map<String, String> headers,
                boolean revalidated) {
            this.status = status;
            this.contentType = contentType;
            this.body = body;
            this.binary = binary;
            this.headers = headers;
            this.revalidated = revalidated;
        }

        public Response revalidated() {
            return new Response(status, contentType, body, binary, headers, true);
        }

        public static Response bytes(String contentType, byte[] body) {
            return new Response(200, contentType, "", body.clone(), Collections.emptyMap(), false);
        }

        byte[] encoded() {
            return binary != null ? binary : body.getBytes(StandardCharsets.UTF_8);
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
            return new Response(status, contentType, body, binary, Collections.unmodifiableMap(copy), revalidated);
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

    public static TinyHttpServer start(int port, String threadName, Function<Request, Response> handler) {
        return start(ANY_INTERFACE, port, threadName, handler);
    }

    public static TinyHttpServer start(
            InetAddress bind, int port, String threadName, Function<Request, Response> handler) {
        try {
            ServerSocket socket = new ServerSocket(port, 50, bind);
            TinyHttpServer server = new TinyHttpServer(socket, handler);
            Thread acceptor = new Thread(server::acceptLoop, threadName);
            acceptor.setDaemon(true);
            acceptor.start();
            return server;
        } catch (IOException e) {
            throw new UncheckedIOException(
                    "could not listen on " + (bind == null ? "*" : bind.getHostAddress()) + ":" + port, e);
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

    static final class Sent {
        final int status;
        final Map<String, String> headers;
        final byte[] body;

        private Sent(int status, Map<String, String> headers, byte[] body) {
            this.status = status;
            this.headers = Collections.unmodifiableMap(headers);
            this.body = body;
        }
    }

    static Sent sent(Response response, Map<String, String> requestHeaders) {
        Map<String, String> headers = new LinkedHashMap<>();
        byte[] body = response.encoded();
        boolean text = compressible(response.contentType);
        if (text) {
            headers.put("Vary", "Accept-Encoding");
        }
        if (response.revalidated && response.status == 200) {
            String version = "W/\"" + version(body) + "\"";
            headers.put("ETag", version);
            headers.put("Cache-Control", "no-cache");
            if (alreadyHeld(requestHeaders.get("if-none-match"), version)) {
                headers.putAll(response.headers);
                return new Sent(304, headers, new byte[0]);
            }
        } else {
            headers.put("Cache-Control", "no-store");
        }
        if (text && takesGzip(requestHeaders.get("accept-encoding"))) {
            byte[] compressed = gzip(body);
            if (compressed.length < body.length) {
                body = compressed;
                headers.put("Content-Encoding", "gzip");
            }
        }
        Map<String, String> ordered = new LinkedHashMap<>();
        ordered.put("Content-Type", response.contentType);
        ordered.put("Content-Length", String.valueOf(body.length));
        ordered.putAll(headers);
        ordered.putAll(response.headers);
        return new Sent(response.status, ordered, body);
    }

    private static boolean compressible(String contentType) {
        String type = contentType.toLowerCase(Locale.ROOT);
        return type.startsWith("text/")
                || type.startsWith("application/json")
                || type.startsWith("application/javascript");
    }

    private static boolean takesGzip(String acceptEncoding) {
        if (acceptEncoding == null) {
            return false;
        }
        Double gzip = null;
        Double anything = null;
        for (String offered : acceptEncoding.split(",")) {
            String[] parts = offered.split(";");
            String coding = parts[0].trim().toLowerCase(Locale.ROOT);
            double quality = 1;
            for (int i = 1; i < parts.length; i++) {
                String parameter = parts[i].trim().toLowerCase(Locale.ROOT);
                if (parameter.startsWith("q=")) {
                    try {
                        quality = Double.parseDouble(parameter.substring(2).trim());
                    } catch (NumberFormatException e) {
                        quality = 0;
                    }
                }
            }
            if (coding.equals("gzip") || coding.equals("x-gzip")) {
                gzip = quality;
            } else if (coding.equals("*")) {
                anything = quality;
            }
        }
        return gzip != null ? gzip > 0 : anything != null && anything > 0;
    }

    private static boolean alreadyHeld(String ifNoneMatch, String version) {
        if (ifNoneMatch == null) {
            return false;
        }
        for (String held : ifNoneMatch.split(",")) {
            String tag = held.trim();
            if (tag.equals("*") || opaque(tag).equals(opaque(version))) {
                return true;
            }
        }
        return false;
    }

    private static String opaque(String tag) {
        return tag.startsWith("W/") ? tag.substring(2) : tag;
    }

    private static String version(byte[] bytes) {
        try {
            StringBuilder hex = new StringBuilder();
            for (byte b : MessageDigest.getInstance("SHA-256").digest(bytes)) {
                hex.append(Character.forDigit((b >> 4) & 0xf, 16)).append(Character.forDigit(b & 0xf, 16));
            }
            return hex.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }

    private static byte[] gzip(byte[] body) {
        ByteArrayOutputStream out = new ByteArrayOutputStream(body.length / 4 + 64);
        try (GZIPOutputStream gzip = new GZIPOutputStream(out) {
            {
                def.setLevel(Deflater.BEST_SPEED);
            }
        }) {
            gzip.write(body);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return out.toByteArray();
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
                    headers.put(
                            line.substring(0, colon).trim().toLowerCase(Locale.ROOT),
                            line.substring(colon + 1).trim());
                }
            }
            long length = contentLength(headers);
            if (length > MAX_BODY_BYTES) {
                write(out, sent(Response.error(413, "request body over " + MAX_BODY_BYTES + " bytes"), headers));
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
            write(out, sent(response, headers));
        } catch (IOException e) {
        }
    }

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

    private static Request parse(
            String requestLine, Map<String, String> headers, byte[] body, InetAddress remoteAddress) {
        String[] parts = requestLine.split(" ");
        String method = parts.length > 0 ? parts[0] : "GET";
        String target = parts.length > 1 ? parts[1] : "/";
        int q = target.indexOf('?');
        String path = URLDecoder.decode(
                (q < 0 ? target : target.substring(0, q)).replace("+", "%2B"), StandardCharsets.UTF_8);
        Map<String, String> query = new HashMap<>();
        if (q >= 0) {
            for (String pair : target.substring(q + 1).split("&")) {
                int eq = pair.indexOf('=');
                String key = eq < 0 ? pair : pair.substring(0, eq);
                String value = eq < 0 ? "" : pair.substring(eq + 1);
                query.put(
                        URLDecoder.decode(key, StandardCharsets.UTF_8),
                        URLDecoder.decode(value, StandardCharsets.UTF_8));
            }
        }
        return new Request(
                method,
                path,
                query,
                Collections.unmodifiableMap(headers),
                new String(body, StandardCharsets.UTF_8),
                remoteAddress);
    }

    private static void write(OutputStream out, Sent sent) throws IOException {
        StringBuilder head = new StringBuilder("HTTP/1.0 " + sent.status + " " + reason(sent.status) + "\r\n");
        sent.headers.forEach(
                (name, value) -> head.append(name).append(": ").append(value).append("\r\n"));
        head.append("Connection: close\r\n\r\n");
        out.write(head.toString().getBytes(StandardCharsets.US_ASCII));
        out.write(sent.body);
        out.flush();
    }

    private static String reason(int status) {
        switch (status) {
            case 200:
                return "OK";
            case 304:
                return "Not Modified";
            case 308:
                return "Permanent Redirect";
            case 400:
                return "Bad Request";
            case 403:
                return "Forbidden";
            case 404:
                return "Not Found";
            case 405:
                return "Method Not Allowed";
            case 409:
                return "Conflict";
            case 413:
                return "Payload Too Large";
            case 415:
                return "Unsupported Media Type";
            case 429:
                return "Too Many Requests";
            case 500:
                return "Internal Server Error";
            default:
                return "Status " + status;
        }
    }
}
