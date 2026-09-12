package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.junit.After;
import org.junit.Test;

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
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

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
        return server(request -> Response.json("{\"path\":\"" + request.path + "\",\"q\":\"" + request.query("q") + "\"}"));
    }

    @After
    public void stop() {
        if (server != null) {
            server.stop();
        }
    }

    @Test
    public void listensOnEveryInterfaceSoAPublishedContainerPortReachesIt() {
        // Traffic published from a container arrives over the bridge, not loopback.
        assertTrue(echoServer().bindAddress().isAnyLocalAddress());
    }

    @Test
    public void routesPathAndDecodedQueryToTheHandler() throws IOException {
        echoServer();

        String body = get("/hello?q=a%20b").body;

        assertEquals("{\"path\":\"/hello\",\"q\":\"a b\"}", body);
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
        server = TinyHttpServer.start(InetAddress.getLoopbackAddress(), 0, "test-admin", request -> Response.json("{}"));
        assertTrue(server.bindAddress().isLoopbackAddress());
        InetAddress lan = firstNonLoopbackAddress();

        try (Socket socket = new Socket()) {
            socket.connect(new InetSocketAddress(lan, server.port()), 2000);
            fail("connected to the loopback-only server via " + lan);
        } catch (ConnectException | SocketTimeoutException refused) {
            // the operating system refused it, which is the whole point
        }
        assertEquals(200, get("/").status);
    }

    /**
     * Fails, never skips, when the machine has no LAN address: this test would otherwise be
     * silent on exactly the machines where the property is easiest to get wrong.
     */
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
            return new Reply(status, in == null ? "" : new String(in.readAllBytes(), StandardCharsets.UTF_8), connection);
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
