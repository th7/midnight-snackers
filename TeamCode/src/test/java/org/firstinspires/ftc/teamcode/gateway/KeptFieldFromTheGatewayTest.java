package org.firstinspires.ftc.teamcode.gateway;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;
import org.firstinspires.ftc.teamcode.sim.KeptField;
import org.firstinspires.ftc.teamcode.sim.OnDiskStore;
import org.firstinspires.ftc.teamcode.sim.SimAssets;
import org.firstinspires.ftc.teamcode.sim.Store;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer;
import org.firstinspires.ftc.teamcode.sim.Valid;
import org.firstinspires.ftc.teamcode.simcore.WhereTheHostIs;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class KeptFieldFromTheGatewayTest {
    private static final int CHEAP_SCRYPT = 1 << 4;
    private static final byte[] HIGH = "the high field the host built".getBytes(StandardCharsets.UTF_8);
    private static final byte[] REBUILT_HIGH = "the high field the host built again".getBytes(StandardCharsets.UTF_8);
    private static final byte[] NOT_AS_OFFERED = "the high field the host built AGAIN".getBytes(StandardCharsets.UTF_8);
    private static final byte[] LOW = "the low field the host built".getBytes(StandardCharsets.UTF_8);

    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private RobotGateway gateway;

    @After
    public void stopTheHosts() throws IOException {
        if (gateway != null) {
            gateway.stop();
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
    public void anAddressWithNoApprovedLoginIsRefusedAndKeepsNothing() throws IOException {
        host();
        hostBuilt("field-high.glb", HIGH);
        KeptField kept = kept();

        List<String> said = kept.fetchFrom(theGateway());

        assertTrue(said.toString(), said.toString().contains("no approved login"));
        assertEquals(List.of(), keptFiles());
        assertTrue(module(kept), module(kept).contains("committed with the project"));
    }
}
