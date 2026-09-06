package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

public class TinyHttpServerTest {
    private final TinyHttpServer server = TinyHttpServer.start(0, "test-http",
            request -> TinyHttpServer.Response.json("{\"path\":\"" + request.path + "\",\"q\":\"" + request.query("q") + "\"}"));

    @After
    public void stop() {
        server.stop();
    }

    @Test
    public void listensOnEveryInterfaceSoAPublishedContainerPortReachesIt() {
        // Traffic published from a container arrives over the bridge, not loopback.
        assertTrue(server.bindAddress().isAnyLocalAddress());
    }

    @Test
    public void routesPathAndDecodedQueryToTheHandler() throws IOException {
        String body = get("/hello?q=a%20b");

        assertEquals("{\"path\":\"/hello\",\"q\":\"a b\"}", body);
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
