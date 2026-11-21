package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.firstinspires.ftc.teamcode.simcore.WhereTheHostIs;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class KeptFieldTest {
    private static final int CHEAP_SCRYPT = 1 << 4;
    private static final byte[] HIGH = "the high field the host built".getBytes(StandardCharsets.UTF_8);
    private static final byte[] REBUILT_HIGH = "the high field the host built again".getBytes(StandardCharsets.UTF_8);
    private static final byte[] NOT_AS_OFFERED = "the high field the host built AGAIN".getBytes(StandardCharsets.UTF_8);
    private static final byte[] LOW = "the low field the host built".getBytes(StandardCharsets.UTF_8);

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private RobotGateway gateway;
    private FakeHost fake;

    @After
    public void stopTheHosts() throws IOException {
        if (gateway != null) {
            gateway.stop();
        }
        if (fake != null) {
            fake.close();
        }
    }

    private Path stateDir() {
        return folder.getRoot().toPath().resolve("host-state");
    }

    private Path keptDir() {
        return folder.getRoot().toPath().resolve("client").resolve("field-models");
    }

    private KeptField kept() {
        return new KeptField(keptDir(), new OnDiskStore());
    }

    private RobotGateway host() {
        gateway = RobotGateway.start(
                InetAddress.getLoopbackAddress(),
                0,
                0,
                stateDir(),
                CHEAP_SCRYPT,
                new RobotGateway.Assets() {
                    @Override
                    public FieldAssets.Refreshed download(Store store, Path into) {
                        throw new AssertionError("no test reaches for Onshape");
                    }

                    @Override
                    public FieldAssets.Refreshed build(
                            Store store, Path into, List<FieldAssets.Resolution> resolutions) {
                        throw new AssertionError("no test builds a model");
                    }
                },
                0,
                new InetSocketAddress(InetAddress.getLoopbackAddress(), 9));
        return gateway;
    }

    private void hostBuilt(String file, byte[] bytes) throws IOException {
        Path model = stateDir().resolve("assets").resolve(file);
        Files.createDirectories(model.getParent());
        Files.write(model, bytes);
    }

    private void approvedFromHere() throws IOException {
        HttpURLConnection login =
                (HttpURLConnection) new URL(gateway.userUrl() + "login?username=ada").openConnection();
        login.setRequestMethod("POST");
        assertEquals(200, login.getResponseCode());
        login.disconnect();
        HttpURLConnection approve =
                (HttpURLConnection) new URL(gateway.adminUrl() + "admin/logins/1/approve").openConnection();
        approve.setRequestMethod("POST");
        assertEquals(200, approve.getResponseCode());
        approve.disconnect();
    }

    private static WhereTheHostIs at(int port) {
        return Valid.value(WhereTheHostIs.from(Map.of(WhereTheHostIs.ENV, "127.0.0.1:" + port)));
    }

    private WhereTheHostIs theGateway() {
        return at(gateway.userPort());
    }

    private static WhereTheHostIs nowhere() throws IOException {
        int port;
        try (ServerSocket closed = new ServerSocket(0, 1, InetAddress.getLoopbackAddress())) {
            port = closed.getLocalPort();
        }
        return at(port);
    }

    private static byte[] served(KeptField kept, String name) {
        TinyHttpServer.Response response = kept.serve(name);
        assertEquals(name + ": " + response.body, 200, response.status);
        return SimAssets.body(response);
    }

    private static String module(KeptField kept) {
        return new String(served(kept, "field-default.js"), StandardCharsets.UTF_8);
    }

    private List<String> keptFiles() throws IOException {
        if (!Files.isDirectory(keptDir())) {
            return List.of();
        }
        try (Stream<Path> listing = Files.list(keptDir())) {
            return listing.map(path -> path.getFileName().toString()).sorted().toList();
        }
    }

    @Test
    public void anApprovedClientKeepsTheModelsTheHostBuiltAndDrawsTheMostDetailed() throws IOException {
        host();
        hostBuilt("field.glb", LOW);
        hostBuilt("field-high.glb", HIGH);
        approvedFromHere();
        KeptField kept = kept();

        List<String> said = kept.fetchFrom(theGateway());

        assertTrue(said.toString(), said.toString().contains("high"));
        assertArrayEquals(HIGH, served(kept, "field-high.glb"));
        assertArrayEquals(LOW, served(kept, "field.glb"));
        assertTrue(module(kept), module(kept).contains("export const DEFAULT_RESOLUTION = 'high';"));
        assertTrue(module(kept), module(kept).contains("The high field the host built, kept on this computer."));
        assertEquals(List.of("field-high.glb", "field.glb"), keptFiles());
    }

    @Test
    public void awayFromTheHostTheClientStillDrawsWhatItKept() throws IOException {
        host();
        hostBuilt("field-high.glb", HIGH);
        approvedFromHere();
        kept().fetchFrom(theGateway());
        gateway.stop();
        gateway = null;
        KeptField away = kept();

        List<String> said = away.fetchFrom(nowhere());

        assertTrue(said.toString(), said.toString().contains("could not reach"));
        assertArrayEquals(HIGH, served(away, "field-high.glb"));
        assertTrue(module(away), module(away).contains("export const DEFAULT_RESOLUTION = 'high';"));
    }

    @Test
    public void aModelTheHostRebuiltIsKeptAgainOnTheNextStart() throws IOException {
        host();
        hostBuilt("field-high.glb", HIGH);
        approvedFromHere();
        kept().fetchFrom(theGateway());
        hostBuilt("field-high.glb", REBUILT_HIGH);
        KeptField next = kept();

        next.fetchFrom(theGateway());

        assertArrayEquals(REBUILT_HIGH, served(next, "field-high.glb"));
    }

    @Test
    public void aModelAlreadyKeptAsTheHostBuiltItIsNotFetchedAgain() throws IOException {
        host();
        hostBuilt("field-high.glb", HIGH);
        approvedFromHere();
        kept().fetchFrom(theGateway());

        List<String> said = kept().fetchFrom(theGateway());

        assertTrue(said.toString(), said.toString().contains("already kept"));
    }

    @Test
    public void aClientThatNeverReachedTheHostDrawsTheCommittedFieldAndSaysSo() {
        KeptField kept = kept();

        List<String> said = kept.fetchFrom(Valid.value(WhereTheHostIs.from(Map.of())));

        assertTrue(said.toString(), said.toString().contains(WhereTheHostIs.ENV));
        assertArrayEquals(SimAssets.body(SimAssets.serve("field.glb")), served(kept, "field.glb"));
        assertEquals(404, kept.serve("field-high.glb").status);
        assertTrue(module(kept), module(kept).contains("export const DEFAULT_RESOLUTION = 'low';"));
        assertTrue(module(kept), module(kept).contains("committed with the project"));
    }

    @Test
    public void whatTheClientDoesNotKeepIsTheBenchsOwn() {
        assertArrayEquals(SimAssets.body(SimAssets.serve("fieldscene.js")), served(kept(), "fieldscene.js"));
    }

    @Test
    public void anAddressWithNoApprovedLoginIsRefusedAndKeepsNothing() throws IOException {
        host();
        hostBuilt("field-high.glb", HIGH);
        KeptField kept = kept();

        List<String> said = kept.fetchFrom(theGateway());

        assertTrue(said.toString(), said.toString().contains("no approved login"));
        assertEquals(List.of(), keptFiles());
        assertTrue(module(kept), module(kept).contains("committed with the project"));
    }

    @Test
    public void aFetchCutOffPartwayLeavesTheModelKeptBefore() throws IOException {
        host();
        hostBuilt("field-high.glb", HIGH);
        approvedFromHere();
        kept().fetchFrom(theGateway());
        fake = FakeHost.offering(REBUILT_HIGH, REBUILT_HIGH.length / 2);
        KeptField next = kept();

        List<String> said = next.fetchFrom(at(fake.port()));

        assertTrue(said.toString(), said.toString().contains("field-high.glb"));
        assertArrayEquals(HIGH, served(next, "field-high.glb"));
        assertEquals(List.of("field-high.glb"), keptFiles());
    }

    @Test
    public void aModelThatArrivesWholeButNotAsOfferedIsNotKept() throws IOException {
        fake = FakeHost.offering(REBUILT_HIGH, NOT_AS_OFFERED.length, NOT_AS_OFFERED);
        KeptField kept = kept();

        List<String> said = kept.fetchFrom(at(fake.port()));

        assertTrue(said.toString(), said.toString().contains("field-high.glb"));
        assertEquals(List.of(), keptFiles());
    }

    @Test
    public void anOfferInAVersionThisSimulatorDoesNotReadIsRefusedSayingToPull() throws IOException {
        fake = FakeHost.answering("{\"version\":2,\"models\":[]}");
        KeptField kept = kept();

        List<String> said = kept.fetchFrom(at(fake.port()));

        assertTrue(said.toString(), said.toString().contains("version 2"));
        assertTrue(said.toString(), said.toString().contains("pull"));
        assertEquals(List.of(), keptFiles());
    }

    private static final class FakeHost implements AutoCloseable {
        private final ServerSocket socket;
        private final Thread serving;

        private FakeHost(Map<String, byte[]> replies) throws IOException {
            socket = new ServerSocket(0, 50, InetAddress.getLoopbackAddress());
            serving = new Thread(
                    () -> {
                        while (!socket.isClosed()) {
                            try (Socket client = socket.accept()) {
                                String path = requestedPath(client.getInputStream());
                                OutputStream out = client.getOutputStream();
                                out.write(replies.getOrDefault(path, reply(404, "nothing at " + path, -1)));
                                out.flush();
                            } catch (IOException closed) {
                                return;
                            }
                        }
                    },
                    "fake-host");
            serving.setDaemon(true);
            serving.start();
        }

        static FakeHost offering(byte[] model, int sent) throws IOException {
            return offering(model, sent, model);
        }

        static FakeHost offering(byte[] offered, int sent, byte[] arriving) throws IOException {
            String offer = "{\"version\":1,\"models\":[{\"resolution\":\"high\",\"file\":\"field-high.glb\","
                    + "\"bytes\":" + offered.length + ",\"sha256\":\"" + sha256(offered) + "\"}]}";
            byte[] head = ("HTTP/1.1 200 OK\r\nContent-Type: model/gltf-binary\r\nContent-Length: " + arriving.length
                            + "\r\nConnection: close\r\n\r\n")
                    .getBytes(StandardCharsets.UTF_8);
            byte[] model = new byte[head.length + sent];
            System.arraycopy(head, 0, model, 0, head.length);
            System.arraycopy(arriving, 0, model, head.length, sent);
            return new FakeHost(Map.of("/field", reply(200, offer, -1), "/field/field-high.glb", model));
        }

        static FakeHost answering(String offer) throws IOException {
            return new FakeHost(Map.of("/field", reply(200, offer, -1)));
        }

        int port() {
            return socket.getLocalPort();
        }

        @Override
        public void close() throws IOException {
            socket.close();
        }

        private static byte[] reply(int status, String body, int length) {
            byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
            return ("HTTP/1.1 " + status + " X\r\nContent-Type: application/json\r\nContent-Length: "
                            + (length < 0 ? bytes.length : length) + "\r\nConnection: close\r\n\r\n" + body)
                    .getBytes(StandardCharsets.UTF_8);
        }

        private static String requestedPath(InputStream in) throws IOException {
            StringBuilder line = new StringBuilder();
            int c;
            while ((c = in.read()) != -1 && c != '\n') {
                line.append((char) c);
            }
            StringBuilder rest = new StringBuilder();
            while (!rest.toString().endsWith("\r\n\r\n") && (c = in.read()) != -1) {
                rest.append((char) c);
            }
            String[] parts = line.toString().split(" ");
            return parts.length > 1 ? parts[1] : "";
        }

        private static String sha256(byte[] bytes) {
            try {
                return java.util.HexFormat.of()
                        .formatHex(java.security.MessageDigest.getInstance("SHA-256")
                                .digest(bytes));
            } catch (java.security.NoSuchAlgorithmException e) {
                throw new AssertionError(e);
            }
        }
    }
}
