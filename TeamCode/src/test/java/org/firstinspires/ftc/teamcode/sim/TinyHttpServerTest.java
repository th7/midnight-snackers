package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.After;
import org.junit.Test;

public class TinyHttpServerTest {
    private TinyHttpServer server;
    private final AtomicReference<Request> seen = new AtomicReference<>();

    private TinyHttpServer server(Function<Request, Response> handler) {
        server = TinyHttpServer.start(0, "test-http", request -> {
            seen.set(request);
            return handler.apply(request);
        });
        return server;
    }

    private TinyHttpServer echoServer() {
        return server(
                request -> Response.json("{\"path\":\"" + request.path + "\",\"q\":\"" + request.query("q") + "\"}"));
    }

    @After
    public void stop() {
        if (server != null) {
            server.stop();
        }
    }

    @Test
    public void listensOnEveryInterfaceSoAPublishedContainerPortReachesIt() {
        assertTrue(echoServer().bindAddress().isAnyLocalAddress());
    }

    @Test
    public void routesPathAndDecodedQueryToTheHandler() throws IOException {
        echoServer();

        String body = get("/hel%20lo+there?q=a%20b").body;

        assertEquals("{\"path\":\"/hel lo+there\",\"q\":\"a b\"}", body);
    }

    @Test
    public void aRequestBodyReachesTheHandler() throws IOException {
        server(request -> Response.json("{\"len\":" + request.body.length() + "}"));

        Reply reply = send("POST", "/upload", "héllo, wörld", Map.of());

        assertEquals(200, reply.status);
        assertEquals("héllo, wörld", seen.get().body);
        assertEquals("POST", seen.get().method);
    }

    @Test
    public void aBodyOverTheCapIsRefusedBeforeTheHandlerRuns() throws IOException {
        server(request -> Response.json("{}"));
        String tooBig = "x".repeat(TinyHttpServer.MAX_BODY_BYTES + 1);

        Reply reply = send("POST", "/upload", tooBig, Map.of());

        assertEquals(413, reply.status);
        assertEquals("the handler must not see an oversized body", null, seen.get());
    }

    @Test
    public void cookiesAndHeadersAreParsed() throws IOException {
        server(request -> Response.json("{}"));

        send("GET", "/", null, Map.of("Cookie", "theme=dark; session=abc123", "X-Thing", "yes"));

        assertEquals("abc123", seen.get().cookie("session"));
        assertEquals("dark", seen.get().cookie("theme"));
        assertEquals(null, seen.get().cookie("missing"));
        assertEquals("yes", seen.get().header("x-thing"));
        assertEquals("yes", seen.get().header("X-Thing"));
    }

    @Test
    public void aHandlerCanSetResponseHeaders() throws IOException {
        server(request -> Response.json("{}").withHeader("Set-Cookie", "session=abc; HttpOnly; Path=/"));

        Reply reply = send("GET", "/", null, Map.of());

        assertEquals("session=abc; HttpOnly; Path=/", reply.header("Set-Cookie"));
    }

    @Test
    public void theRemoteAddressIsOnTheRequest() throws IOException {
        server(request -> Response.json("{}"));

        send("GET", "/", null, Map.of());

        assertNotNull(seen.get().remoteAddress);
        assertTrue(seen.get().remoteAddress.isLoopbackAddress());
    }

    @Test
    public void aLoopbackBoundServerIsUnreachableFromTheLan() throws IOException {
        server =
                TinyHttpServer.start(InetAddress.getLoopbackAddress(), 0, "test-admin", request -> Response.json("{}"));
        assertTrue(server.bindAddress().isLoopbackAddress());
        InetAddress lan = firstNonLoopbackAddress();

        try (Socket socket = new Socket()) {
            socket.connect(new InetSocketAddress(lan, server.port()), 2000);
            fail("connected to the loopback-only server via " + lan);
        } catch (ConnectException | SocketTimeoutException refused) {
        }
        assertEquals(200, get("/").status);
    }

    private static InetAddress firstNonLoopbackAddress() throws IOException {
        for (NetworkInterface nic : Collections.list(NetworkInterface.getNetworkInterfaces())) {
            for (InetAddress address : Collections.list(nic.getInetAddresses())) {
                if (!address.isLoopbackAddress() && !address.isLinkLocalAddress() && address.getAddress().length == 4) {
                    return address;
                }
            }
        }
        throw new AssertionError("could not judge: this machine has no non-loopback IPv4 address to connect from");
    }

    @Test
    public void bytesThatAreNotTextSurviveBeingServed() throws IOException {
        byte[] model = new byte[] {(byte) 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A, (byte) 0xFF, 0x00, 0x7F};
        server(request -> Response.bytes("model/gltf-binary", model));

        Bytes reply = getBytes("/field.glb");

        assertEquals(200, reply.status);
        assertArrayEquals(model, reply.body);
    }

    @Test
    public void aBinaryResponseSaysHowLongItIsAndWhatItIs() throws IOException {
        byte[] model = new byte[2048];
        for (int i = 0; i < model.length; i++) {
            model[i] = (byte) i;
        }
        server(request -> Response.bytes("model/gltf-binary", model));

        Bytes reply = getBytes("/field.glb");

        assertArrayEquals(model, reply.body);
        assertEquals("2048", reply.header("Content-Length"));
        assertEquals("model/gltf-binary", reply.header("Content-Type"));
    }

    @Test
    public void textIsStillMeasuredInBytesRatherThanCharacters() throws IOException {
        server(request -> Response.html("héllo"));

        Bytes reply = getBytes("/");

        assertEquals("6", reply.header("Content-Length"));
        assertEquals("héllo", new String(reply.body, StandardCharsets.UTF_8));
    }

    private static final String A_BROWSER_TAKES = "gzip, deflate, br";

    private static String aLongRun() {
        StringBuilder ticks = new StringBuilder("{\"outcome\":null,\"ticks\":[");
        for (int i = 0; i < 200; i++) {
            ticks.append(i == 0 ? "" : ",").append("{\"t\":").append(i * 0.02).append(",\"x\":1.5,\"y\":-2.25}");
        }
        return ticks.append("]}").toString();
    }

    @Test
    public void textIsSentCompressedToABrowserThatTakesIt() throws IOException {
        String run = aLongRun();
        server(request -> Response.json(run));

        Bytes reply = getBytes("/ticks", Map.of("Accept-Encoding", A_BROWSER_TAKES));

        assertEquals("gzip", reply.header("Content-Encoding"));
        assertEquals(String.valueOf(reply.body.length), reply.header("Content-Length"));
        assertTrue(reply.body.length + " bytes for " + run.length(), reply.body.length < run.length() / 4);
        assertEquals(run, new String(gunzip(reply.body), StandardCharsets.UTF_8));
        assertEquals("a cache keeps the two apart", "Accept-Encoding", reply.header("Vary"));
    }

    @Test
    public void textIsSentAsItIsToAClientThatDoesNotTakeGzip() throws IOException {
        String run = aLongRun();
        server(request -> Response.json(run));

        for (Map<String, String> asked : List.of(
                Map.<String, String>of(),
                Map.of("Accept-Encoding", "identity"),
                Map.of("Accept-Encoding", "gzip;q=0, deflate"),
                Map.of("Accept-Encoding", "*;q=0"))) {
            Bytes reply = getBytes("/ticks", asked);

            assertNull(asked.toString(), reply.header("Content-Encoding"));
            assertEquals(asked.toString(), run, new String(reply.body, StandardCharsets.UTF_8));
        }
    }

    @Test
    public void aClientThatTakesAnythingTakesGzip() throws IOException {
        server(request -> Response.html(aLongRun()));

        assertEquals("gzip", getBytes("/", Map.of("Accept-Encoding", "*")).header("Content-Encoding"));
        assertEquals(
                "gzip",
                getBytes("/", Map.of("Accept-Encoding", "br;q=1.0, GZIP;q=0.5")).header("Content-Encoding"));
    }

    @Test
    public void whatCompressionWouldOnlyGrowIsSentAsItIs() throws IOException {
        byte[] image = new byte[4096];
        new java.util.Random(7).nextBytes(image);
        server(request -> request.path.equals("/tag.png") ? Response.bytes("image/png", image) : Response.json("{}"));

        Bytes tiny = getBytes("/", Map.of("Accept-Encoding", A_BROWSER_TAKES));
        Bytes png = getBytes("/tag.png", Map.of("Accept-Encoding", A_BROWSER_TAKES));

        assertNull(tiny.header("Content-Encoding"));
        assertEquals("{}", new String(tiny.body, StandardCharsets.UTF_8));
        assertNull(png.header("Content-Encoding"));
        assertArrayEquals(image, png.body);
    }

    @Test
    public void aRevalidatedResponseIsKeptByTheBrowserAndAskedAboutAgainRatherThanSentAgain() throws IOException {
        byte[] model = new byte[2048];
        new java.util.Random(11).nextBytes(model);
        server(request -> Response.bytes("model/gltf-binary", model).revalidated());

        Bytes first = getBytes("/field.glb", Map.of());
        String version = first.header("ETag");
        Bytes again = getBytes("/field.glb", Map.of("If-None-Match", version));

        assertEquals(200, first.status);
        assertNotNull(version);
        assertEquals("no-cache", first.header("Cache-Control"));
        assertEquals(304, again.status);
        assertEquals("nothing is sent again", 0, again.body.length);
        assertEquals(version, again.header("ETag"));
        assertEquals(
                "one of several versions the browser holds",
                304,
                getBytes("/field.glb", Map.of("If-None-Match", "W/\"older\", " + version)).status);
    }

    @Test
    public void aRevalidatedResponseWhoseBytesChangedIsSentWhole() throws IOException {
        byte[][] served = {"the model as it was".getBytes(StandardCharsets.UTF_8)};
        server(request -> Response.bytes("model/gltf-binary", served[0]).revalidated());
        String was = getBytes("/field.glb", Map.of()).header("ETag");

        served[0] = "the model refreshed".getBytes(StandardCharsets.UTF_8);
        Bytes now = getBytes("/field.glb", Map.of("If-None-Match", was));

        assertEquals(200, now.status);
        assertEquals("the model refreshed", new String(now.body, StandardCharsets.UTF_8));
        assertTrue(was + " then " + now.header("ETag"), !was.equals(now.header("ETag")));
    }

    @Test
    public void aRevalidatedScriptIsTheSameVersionCompressedOrNot() throws IOException {
        String script = "export const one = 1;\n".repeat(200);
        server(request -> new Response(200, "text/javascript; charset=utf-8", script).revalidated());

        Bytes compressed = getBytes("/a.js", Map.of("Accept-Encoding", A_BROWSER_TAKES));
        Bytes plain = getBytes("/a.js", Map.of());

        assertEquals("gzip", compressed.header("Content-Encoding"));
        assertEquals(compressed.header("ETag"), plain.header("ETag"));
        assertEquals(
                304,
                getBytes("/a.js", Map.of("Accept-Encoding", A_BROWSER_TAKES, "If-None-Match", plain.header("ETag")))
                        .status);
    }

    @Test
    public void everythingElseIsNeverKept() throws IOException {
        server(request -> Response.json(aLongRun()));

        Bytes reply = getBytes("/ticks", Map.of("If-None-Match", "*"));

        assertEquals(200, reply.status);
        assertEquals("no-store", reply.header("Cache-Control"));
        assertNull(reply.header("ETag"));
    }

    private static byte[] gunzip(byte[] body) throws IOException {
        try (InputStream in = new java.util.zip.GZIPInputStream(new java.io.ByteArrayInputStream(body))) {
            return in.readAllBytes();
        }
    }

    private Bytes getBytes(String path, Map<String, String> headers) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(server.url() + path.substring(1)).openConnection();
        connection.setRequestMethod("GET");
        headers.forEach(connection::setRequestProperty);
        int status = connection.getResponseCode();
        try (InputStream in = status < 400 ? connection.getInputStream() : connection.getErrorStream()) {
            return new Bytes(status, in == null ? new byte[0] : in.readAllBytes(), connection);
        } finally {
            connection.disconnect();
        }
    }

    private Bytes getBytes(String path) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(server.url() + path.substring(1)).openConnection();
        connection.setRequestMethod("GET");
        int status = connection.getResponseCode();
        try (InputStream in = status < 400 ? connection.getInputStream() : connection.getErrorStream()) {
            return new Bytes(status, in == null ? new byte[0] : in.readAllBytes(), connection);
        } finally {
            connection.disconnect();
        }
    }

    private static final class Bytes {
        final int status;
        final byte[] body;
        private final HttpURLConnection connection;

        Bytes(int status, byte[] body, HttpURLConnection connection) {
            this.status = status;
            this.body = body;
            this.connection = connection;
        }

        String header(String name) {
            return connection.getHeaderField(name);
        }
    }

    private Reply get(String path) throws IOException {
        return send("GET", path, null, Map.of());
    }

    private Reply send(String method, String path, String body, Map<String, String> headers) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(server.url() + path.substring(1)).openConnection();
        connection.setRequestMethod(method);
        headers.forEach(connection::setRequestProperty);
        if (body != null) {
            connection.setDoOutput(true);
            try (OutputStream out = connection.getOutputStream()) {
                out.write(body.getBytes(StandardCharsets.UTF_8));
            }
        }
        int status = connection.getResponseCode();
        try (InputStream in = status < 400 ? connection.getInputStream() : connection.getErrorStream()) {
            return new Reply(
                    status, in == null ? "" : new String(in.readAllBytes(), StandardCharsets.UTF_8), connection);
        } finally {
            connection.disconnect();
        }
    }

    private static final class Reply {
        final int status;
        final String body;
        private final HttpURLConnection connection;

        Reply(int status, String body, HttpURLConnection connection) {
            this.status = status;
            this.body = body;
            this.connection = connection;
        }

        String header(String name) {
            return connection.getHeaderField(name);
        }
    }
}
