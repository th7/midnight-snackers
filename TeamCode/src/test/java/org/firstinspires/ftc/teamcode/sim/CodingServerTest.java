package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.gson.JsonObject;
import com.google.gson.Gson;

import org.bouncycastle.crypto.generators.SCrypt;
import org.firstinspires.ftc.teamcode.sim.TestAutos.NeverDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFileAttributeView;
import java.nio.file.attribute.PosixFilePermissions;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.Base64;
import java.util.Map;

public class CodingServerTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    /** Where the server keeps what outlives it, deliberately nowhere near the project root. */
    @Rule
    public TemporaryFolder state = new TemporaryFolder();

    private static final double RUN_TIMEOUT_SECONDS = 0.3;

    private CodingServer server;

    private CodingServer server() {
        if (server == null) {
            serverWith(bench());
        }
        return server;
    }

    private SimBench bench() {
        return new SimBench(SimCatalog.of(ThreeLoopAuto.class, NeverDoneAuto.class), null,
                folder.getRoot().toPath().resolve("sim"), RUN_TIMEOUT_SECONDS, 1);
    }

    private CodingServer serverWith(SimBench bench) {
        server = CodingServer.start(folder.getRoot().toPath(), bench, InetAddress.getLoopbackAddress(), 0, 0, stateDir());
        return server;
    }

    /** A directory the server has to create itself, parents included, the way a first run on a new machine does. */
    private Path stateDir() {
        return state.getRoot().toPath().resolve("nested").resolve("coding-server");
    }

    /** Stops the server and starts a fresh one over the same root and state directory, the way a restart does. */
    private void restart() {
        server.stop();
        server = null;
        server();
    }

    @After
    public void stopServer() {
        if (server != null) {
            server.stop();
        }
    }

    // --- login and approval ---

    @Test
    public void theAdminListenerIsBoundToLoopbackOnly() {
        assertTrue(server().adminBindAddress().isLoopbackAddress());
        assertTrue(server().userBindAddress().isAnyLocalAddress());
    }

    @Test
    public void aVisitorWithoutASessionGetsTheLoginPage() throws IOException {
        Reply page = user("GET", "/", null);

        assertEquals(200, page.status);
        assertTrue(page.body, page.body.contains("name=\"username\""));
        assertTrue(page.body, page.body.contains("/me"));
        assertHiddenWins(page.body);
    }

    @Test
    public void loggingInCreatesAPendingSessionTheAdminCanSee() throws IOException {
        Reply login = user("POST", "/login?username=ada", null);

        assertEquals(200, login.status);
        String cookie = login.sessionCookie();
        assertNotNull(login.header("Set-Cookie"), cookie);
        assertTrue(login.header("Set-Cookie"), login.header("Set-Cookie").contains("HttpOnly"));
        assertTrue(login.header("Set-Cookie"), login.header("Set-Cookie").contains("SameSite=Strict"));
        assertEquals("{\"state\":\"pending\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        String logins = admin("GET", "/admin/logins").body;
        assertTrue(logins, logins.contains("\"username\":\"ada\""));
        assertTrue(logins, logins.contains("\"state\":\"pending\""));
        assertTrue(logins, logins.contains("\"address\":\"127.0.0.1\""));
        assertTrue("the admin list must never carry the session token", !logins.contains(cookie));
    }

    @Test
    public void anApprovedUserSeesTheDashboard() throws IOException {
        String cookie = login("ada");
        assertTrue(user("GET", "/", cookie).body.contains("name=\"username\""));

        Reply approved = admin("POST", "/admin/logins/" + idOf("ada") + "/approve");

        assertEquals(200, approved.status);
        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        Reply page = user("GET", "/", cookie);
        assertTrue(page.body, page.body.contains("id=\"editor\""));
        assertEquals(200, user("GET", "/files", cookie).status);
    }

    @Test
    public void aDeniedUserIsToldAndKeptOut() throws IOException {
        String cookie = login("mallory");

        admin("POST", "/admin/logins/" + idOf("mallory") + "/deny");

        assertEquals("{\"state\":\"denied\",\"username\":\"mallory\"}", user("GET", "/me", cookie).body);
        assertEquals(403, user("GET", "/files", cookie).status);
    }

    @Test
    public void revokingAnApprovedSessionEndsIt() throws IOException {
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        assertEquals(200, user("GET", "/files", cookie).status);

        admin("POST", "/admin/logins/" + idOf("ada") + "/revoke");

        assertEquals(403, user("GET", "/files", cookie).status);
        assertEquals("{\"state\":\"revoked\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
    }

    @Test
    public void aForgedCookieIsNoSession() throws IOException {
        String forged = "session=" + "A".repeat(22);

        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        assertEquals(403, user("GET", "/files", forged).status);
        assertTrue(user("GET", "/", forged).body.contains("name=\"username\""));
    }

    @Test
    public void tooManyPendingLoginsAreRefused() throws IOException {
        for (int i = 0; i < CodingServer.MAX_PENDING_LOGINS; i++) {
            assertEquals(200, user("POST", "/login?username=user" + i, null).status);
        }

        assertEquals(429, user("POST", "/login?username=onemore", null).status);
    }

    @Test
    public void usernamesAreCheckedBeforeASessionExists() throws IOException {
        assertEquals(400, user("POST", "/login?username=", null).status);
        assertEquals(400, user("POST", "/login", null).status);
        assertEquals(400, user("POST", "/login?username=" + "a".repeat(33), null).status);
        assertEquals(400, user("POST", "/login?username=tab%09here", null).status);
        assertEquals(405, user("GET", "/login?username=ada", null).status);
        assertEquals(200, user("POST", "/login?username=" + "a".repeat(32), null).status);
        assertTrue(admin("GET", "/admin/logins").body.contains("\"username\":\"" + "a".repeat(32) + "\""));
    }

    @Test
    public void theAdminRoutesDoNotExistOnTheUserPort() throws IOException {
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");

        assertEquals(404, user("GET", "/admin", cookie).status);
        assertEquals(404, user("GET", "/admin/logins", cookie).status);
        assertEquals(404, user("POST", "/admin/logins/1/approve", cookie).status);
        assertEquals(404, user("GET", "/admin/tree", cookie).status);
    }

    // --- the editable set ---

    @Test
    public void theTreeStaysUnderTheRoot() throws IOException {
        folder.newFolder("TeamCode");
        folder.newFile("TeamCode/Plans.java");
        Path outside = Files.createTempDirectory("editor-outside");
        try {
            Files.write(outside.resolve("secret.txt"), "shh".getBytes(StandardCharsets.UTF_8));
            Files.createSymbolicLink(folder.getRoot().toPath().resolve("escape"), outside);

            assertEquals(400, admin("GET", "/admin/tree?dir=..").status);
            assertEquals(400, admin("GET", "/admin/tree?dir=" + outside.toAbsolutePath()).status);
            assertEquals(400, admin("GET", "/admin/tree?dir=escape").status);
            assertEquals(400, admin("GET", "/admin/tree?dir=TeamCode/../..").status);
            assertEquals(400, admin("POST", "/admin/files/add?path=escape/secret.txt").status);
            assertEquals(400, admin("POST", "/admin/files/add?path=../secret.txt").status);
            assertEquals("{\"files\":[]}", admin("GET", "/admin/files").body);
        } finally {
            Files.deleteIfExists(outside.resolve("secret.txt"));
            Files.deleteIfExists(outside);
        }
    }

    @Test
    public void theTreeListsFilesAndDirectoriesUnderTheRoot() throws IOException {
        folder.newFolder("TeamCode", "src");
        folder.newFile("TeamCode/build.gradle");
        folder.newFile("README.md");

        Reply top = admin("GET", "/admin/tree");
        Reply sub = admin("GET", "/admin/tree?dir=TeamCode");

        assertEquals(200, top.status);
        assertTrue(top.body, top.body.contains("{\"name\":\"README.md\",\"type\":\"file\",\"path\":\"README.md\"}"));
        assertTrue(top.body, top.body.contains("{\"name\":\"TeamCode\",\"type\":\"dir\",\"path\":\"TeamCode\"}"));
        assertTrue(sub.body, sub.body.contains("{\"name\":\"build.gradle\",\"type\":\"file\",\"path\":\"TeamCode/build.gradle\"}"));
        assertTrue(sub.body, sub.body.contains("{\"name\":\"src\",\"type\":\"dir\",\"path\":\"TeamCode/src\"}"));
        assertEquals(400, admin("GET", "/admin/tree?dir=TeamCode/build.gradle").status);
        assertEquals(400, admin("GET", "/admin/tree?dir=nope").status);
    }

    @Test
    public void usersSeeExactlyTheFilesTheAdminPicked() throws IOException {
        folder.newFolder("TeamCode");
        folder.newFile("TeamCode/Plans.java");
        folder.newFile("TeamCode/Drive.java");
        folder.newFile("TeamCode/Secret.java");
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");

        assertEquals(200, admin("POST", "/admin/files/add?path=TeamCode/Plans.java").status);
        assertEquals(200, admin("POST", "/admin/files/add?path=TeamCode/Drive.java").status);
        assertEquals(400, admin("POST", "/admin/files/add?path=TeamCode").status);
        assertEquals(400, admin("POST", "/admin/files/add?path=TeamCode/Missing.java").status);
        assertEquals(200, admin("POST", "/admin/files/remove?path=TeamCode/Drive.java").status);

        Reply files = user("GET", "/files", cookie);
        assertEquals(200, files.status);
        assertTrue(files.body, files.body.contains("\"path\":\"TeamCode/Plans.java\""));
        assertTrue(files.body, !files.body.contains("Drive.java"));
        assertTrue(files.body, !files.body.contains("Secret.java"));
        assertEquals(404, user("GET", "/files/TeamCode/Secret.java", cookie).status);
        assertEquals(404, user("GET", "/files/TeamCode/Drive.java", cookie).status);
        assertEquals(404, user("GET", "/files/TeamCode/../TeamCode/Plans.java", cookie).status);
        assertEquals(404, user("GET", "/admin/files", cookie).status);
    }

    // --- read, write, conflict ---

    private String approvedEditorOf(String... files) throws IOException {
        folder.newFolder("TeamCode");
        for (String file : files) {
            folder.newFile("TeamCode/" + file);
            assertEquals(200, admin("POST", "/admin/files/add?path=TeamCode/" + file).status);
        }
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        return cookie;
    }

    private Path file(String name) {
        return folder.getRoot().toPath().resolve("TeamCode").resolve(name);
    }

    private static String sha256(byte[] bytes) throws IOException {
        try {
            StringBuilder hex = new StringBuilder();
            for (byte b : MessageDigest.getInstance("SHA-256").digest(bytes)) {
                hex.append(String.format("%02x", b));
            }
            return hex.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IOException(e);
        }
    }

    private static JsonObject json(String body) {
        return new Gson().fromJson(body, JsonObject.class);
    }

    private static String edit(String content, String baseVersion) {
        JsonObject body = new JsonObject();
        body.addProperty("content", content);
        body.addProperty("baseVersion", baseVersion);
        return body.toString();
    }

    @Test
    public void readingAFileReturnsItsContentAndAVersionThatIsItsHash() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        byte[] bytes = "class Plans {}\n".getBytes(StandardCharsets.UTF_8);
        Files.write(file("Plans.java"), bytes);

        Reply read = user("GET", "/files/TeamCode/Plans.java", cookie);

        assertEquals(200, read.status);
        JsonObject json = json(read.body);
        assertEquals("class Plans {}\n", json.get("content").getAsString());
        assertEquals(sha256(bytes), json.get("version").getAsString());
        assertEquals("TeamCode/Plans.java", json.get("path").getAsString());
    }

    @Test
    public void anEditFromTheCurrentVersionIsWrittenToDisk() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "old".getBytes(StandardCharsets.UTF_8));
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body).get("version").getAsString();

        Reply written = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("new content\n", version));

        assertEquals(written.body, 200, written.status);
        assertEquals("new content\n", new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
        assertEquals(sha256("new content\n".getBytes(StandardCharsets.UTF_8)),
                json(written.body).get("version").getAsString());
    }

    @Test
    public void anEditFromAStaleVersionIsRefusedWithTheCurrentContent() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "on disk".getBytes(StandardCharsets.UTF_8));

        Reply refused = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("mine", sha256("something else".getBytes(StandardCharsets.UTF_8))));

        assertEquals(409, refused.status);
        JsonObject json = json(refused.body);
        assertEquals("on disk", json.get("content").getAsString());
        assertEquals(sha256("on disk".getBytes(StandardCharsets.UTF_8)), json.get("version").getAsString());
        assertEquals("on disk", new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
    }

    @Test
    public void aChangeMadeOnTheHostBetweenReadAndWriteIsAConflict() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "v1".getBytes(StandardCharsets.UTF_8));
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body).get("version").getAsString();
        Files.write(file("Plans.java"), "v2 from the IDE".getBytes(StandardCharsets.UTF_8));

        Reply refused = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("v2 from the browser", version));

        assertEquals(409, refused.status);
        assertEquals("v2 from the IDE", new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
    }

    @Test
    public void writesLeaveNoTemporaryFilesAndKeepLineEndings() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "a\r\nb\r\n".getBytes(StandardCharsets.UTF_8));
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body).get("version").getAsString();

        Reply written = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("a\r\nb\r\nc", version));

        assertEquals(200, written.status);
        assertArrayEquals("a\r\nb\r\nc".getBytes(StandardCharsets.UTF_8), Files.readAllBytes(file("Plans.java")));
        assertEquals("[Plans.java]", Arrays.toString(file("Plans.java").getParent().toFile().list()));
    }

    @Test
    public void anOversizedEditIsRefused() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body).get("version").getAsString();

        Reply refused = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("x".repeat(TinyHttpServer.MAX_BODY_BYTES + 1), version));

        assertEquals(413, refused.status);
        assertEquals(0, Files.size(file("Plans.java")));
    }

    @Test
    public void aFileThatIsNotUtf8IsListedButNotEditable() throws IOException {
        String cookie = approvedEditorOf("logo.bin");
        Files.write(file("logo.bin"), new byte[]{(byte) 0xff, (byte) 0xfe, 0x00, (byte) 0xc3});

        assertTrue(user("GET", "/files", cookie).body.contains("\"path\":\"TeamCode/logo.bin\""));
        assertEquals(415, user("GET", "/files/TeamCode/logo.bin", cookie).status);
        assertEquals(415, user("PUT", "/files/TeamCode/logo.bin", cookie, edit("text", "whatever")).status);
        assertArrayEquals(new byte[]{(byte) 0xff, (byte) 0xfe, 0x00, (byte) 0xc3}, Files.readAllBytes(file("logo.bin")));
    }

    @Test
    public void onlyApprovedSessionsReadOrWrite() throws IOException {
        approvedEditorOf("Plans.java");
        String pending = login("bob");

        assertEquals(403, user("GET", "/files/TeamCode/Plans.java", pending).status);
        assertEquals(403, user("PUT", "/files/TeamCode/Plans.java", pending, edit("x", "y")).status);
        assertEquals(403, user("PUT", "/files/TeamCode/Plans.java", null, edit("x", "y")).status);
        assertEquals(0, Files.size(file("Plans.java")));
    }

    @Test
    public void theListShowsWhoHasEachFileOpen() throws IOException {
        String ada = approvedEditorOf("Plans.java", "Drive.java");
        String bob = login("bob");
        admin("POST", "/admin/logins/" + idOf("bob") + "/approve");
        user("GET", "/files/TeamCode/Plans.java", ada);
        user("GET", "/files/TeamCode/Plans.java", bob);
        user("GET", "/files/TeamCode/Drive.java", bob);

        String files = user("GET", "/files", ada).body;

        assertTrue(files, files.contains("{\"path\":\"TeamCode/Drive.java\",\"editors\":[\"bob\"]}"));
        assertTrue(files, files.contains("{\"path\":\"TeamCode/Plans.java\",\"editors\":[\"ada\"]}"));
        String logins = admin("GET", "/admin/logins").body;
        assertTrue(logins, logins.contains("\"username\":\"bob\",\"address\":\"127.0.0.1\",\"state\":\"approved\",\"ageSeconds\":0,\"file\":\"TeamCode/Drive.java\""));
    }

    // --- pages ---

    @Test
    public void theDashboardPageHasTheFileListAndAnEditorThatSavesAsYouType() throws IOException {
        String cookie = approvedEditorOf("Plans.java");

        String page = user("GET", "/", cookie).body;

        assertTrue(page, page.contains("id=\"files\""));
        assertTrue(page, page.contains("id=\"editor\""));
        assertTrue(page, page.contains("id=\"status\""));
        assertTrue(page, page.contains("method: 'PUT'"));
        assertTrue(page, page.contains("baseVersion"));
        assertTrue(page, page.contains("setTimeout"));
        assertTrue(page, page.contains("409"));
    }

    @Test
    public void theAdminPageListsLoginsWithDecisionsAndTheFilePicker() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("id=\"logins\""));
        assertTrue(page, page.contains("/admin/logins"));
        assertTrue(page, page.contains("/approve"));
        assertTrue(page, page.contains("/deny"));
        assertTrue(page, page.contains("/revoke"));
        assertTrue(page, page.contains("id=\"tree\""));
        assertTrue(page, page.contains("/admin/tree"));
        assertTrue(page, page.contains("/admin/files/add"));
        assertTrue(page, page.contains("/admin/files/remove"));
        assertTrue(page, page.contains("/admin/info"));
        assertHiddenWins(page);
        assertEquals(page, admin("GET", "/").body);
        String info = admin("GET", "/admin/info").body;
        assertTrue(info, info.contains("\"userPort\":" + server().userPort()));
        assertTrue(info, info.contains("\"addresses\":["));
    }

    // --- simulate ---

    private String approvedUser(String name) throws IOException {
        String cookie = login(name);
        admin("POST", "/admin/logins/" + idOf(name) + "/approve");
        return cookie;
    }

    private String awaitSimStatus(String cookie, String marker) throws Exception {
        long deadline = System.nanoTime() + 10_000_000_000L;
        String status = "";
        while (System.nanoTime() < deadline) {
            status = user("GET", "/sim/status", cookie).body;
            if (status.contains(marker)) {
                return status;
            }
            Thread.sleep(20);
        }
        throw new AssertionError("sim status never contained " + marker + "; last: " + status);
    }

    @Test
    public void theSimCatalogListsTheRunnableAutos() throws IOException {
        String cookie = approvedUser("ada");

        Reply catalog = user("GET", "/sim/catalog", cookie);

        assertEquals(200, catalog.status);
        assertTrue(catalog.body, catalog.body.contains("\"name\":\"Count to three\""));
        assertTrue(catalog.body, catalog.body.contains("\"name\":\"Never done\""));
        assertTrue(catalog.body, catalog.body.contains("\"opMode\":\"" + ThreeLoopAuto.class.getName() + "\""));
    }

    @Test
    public void aRunStartsAndTheStatusFollowsItToItsOutcomeWithWhoStartedIt() throws Exception {
        String cookie = approvedUser("ada");

        Reply started = user("POST", "/sim/run?opmode=" + ThreeLoopAuto.class.getName(), cookie);

        assertEquals(started.body, 200, started.status);
        String id = json(started.body).get("id").getAsString();
        String status = awaitSimStatus(cookie, "\"outcome\":\"done\"");
        assertTrue(status, status.contains("\"id\":" + id));
        assertTrue(status, status.contains("\"name\":\"Count to three\""));
        assertTrue(status, status.contains("\"startedBy\":\"ada\""));
        assertTrue(status, status.contains("\"running\":false"));
        Reply live = user("GET", "/sim/runs/" + id + "/", cookie);
        assertEquals(200, live.status);
        assertTrue(live.body, live.body.contains("<canvas"));
        assertTrue(live.body, live.body.contains("\"live\":true"));
        Reply ticks = user("GET", "/sim/runs/" + id + "/ticks?from=0", cookie);
        assertEquals(200, ticks.status);
        assertEquals(3, ticks.body.split("\"step\"").length - 1);
        assertTrue(folder.getRoot().toPath().resolve("sim").resolve("ThreeLoopAuto.html").toFile().exists());
    }

    @Test
    public void oneRunAtATimeForEveryone() throws Exception {
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        assertEquals(200, user("POST", "/sim/run?opmode=" + NeverDoneAuto.class.getName(), ada).status);
        assertTrue(user("GET", "/sim/status", bob).body.contains("\"running\":true"));

        Reply second = user("POST", "/sim/run?opmode=" + ThreeLoopAuto.class.getName(), bob);

        assertEquals(409, second.status);
        assertTrue(second.body, second.body.contains("ada"));
        String status = awaitSimStatus(bob, "\"outcome\":\"timed out");
        assertTrue(status, status.contains("\"running\":false"));
    }

    @Test
    public void theSimIsForApprovedSessionsOnly() throws IOException {
        String pending = login("bob");

        assertEquals(403, user("GET", "/sim/catalog", pending).status);
        assertEquals(403, user("GET", "/sim/status", pending).status);
        assertEquals(403, user("POST", "/sim/run?opmode=" + ThreeLoopAuto.class.getName(), pending).status);
        assertEquals(403, user("GET", "/sim/runs/1/", pending).status);
        assertEquals(403, user("GET", "/sim/runs/1/ticks?from=0", null).status);
    }

    @Test
    public void unknownOpModesAndWrongMethodsAreRejected() throws IOException {
        String cookie = approvedUser("ada");

        assertEquals(404, user("POST", "/sim/run?opmode=org.example.Nope", cookie).status);
        assertEquals(405, user("GET", "/sim/run?opmode=" + ThreeLoopAuto.class.getName(), cookie).status);
        assertEquals(404, user("GET", "/sim/runs/999/ticks?from=0", cookie).status);
        assertEquals(404, user("GET", "/sim/nope", cookie).status);
    }

    @Test
    public void anEditSavedInTheEditorDrivesTheNextRun() throws Exception {
        Path sourceRoot = SimBenchTest.sourceRootWith(folder.getRoot().toPath(), SimBenchTest.tempAuto(2));
        serverWith(new SimBench(null, sourceRoot, folder.getRoot().toPath().resolve("sim"), 2, 1));
        String key = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/TempAuto.java";
        assertEquals(200, admin("POST", "/admin/files/add?path=" + key).status);
        String cookie = approvedUser("ada");
        assertTrue(user("GET", "/sim/catalog", cookie).body.contains("\"name\":\"Temp\""));

        String version = json(user("GET", "/files/" + key, cookie).body).get("version").getAsString();
        assertEquals(200, user("PUT", "/files/" + key, cookie, edit(SimBenchTest.tempAuto(4), version)).status);
        Reply started = user("POST", "/sim/run?opmode=" + SimBenchTest.TEMP_AUTO_CLASS, cookie);

        assertEquals(started.body, 200, started.status);
        String status = awaitSimStatus(cookie, "\"outcome\":\"done\"");
        assertTrue(status, status.contains("\"loops\":4"));
        assertTrue(status, status.contains("\"phase\":\"finished\""));
        assertTrue(status, status.contains("\"message\":null"));
    }

    @Test
    public void aBrokenEditIsReportedByTheRunAndTheCatalog() throws Exception {
        Path sourceRoot = SimBenchTest.sourceRootWith(folder.getRoot().toPath(), SimBenchTest.tempAuto(2).replace("loops = 0", "loops = "));
        serverWith(new SimBench(null, sourceRoot, folder.getRoot().toPath().resolve("sim"), 2, 1));
        String cookie = approvedUser("ada");

        Reply catalog = user("GET", "/sim/catalog", cookie);
        assertEquals(500, catalog.status);
        assertTrue(catalog.body, catalog.body.contains("TempAuto.java:8"));
        assertEquals(200, user("POST", "/sim/run?opmode=" + SimBenchTest.TEMP_AUTO_CLASS, cookie).status);
        String status = awaitSimStatus(cookie, "\"outcome\":\"build failed\"");
        assertTrue(status, status.contains("TempAuto.java:8"));
    }

    @Test
    public void theRunLogIsWhatTheChildWroteToStderr() throws Exception {
        serverWith(new SimBench(SimCatalog.of(TestAutos.ChattyAuto.class), null, folder.getRoot().toPath().resolve("sim"), 2, 1));
        String cookie = approvedUser("ada");
        String id = json(user("POST", "/sim/run?opmode=" + TestAutos.ChattyAuto.class.getName(), cookie).body).get("id").getAsString();
        awaitSimStatus(cookie, "\"outcome\":\"done\"");

        Reply log = user("GET", "/sim/runs/" + id + "/log", cookie);

        assertEquals(200, log.status);
        assertTrue(log.body, log.body.contains("hello from the op mode"));
        assertEquals(403, user("GET", "/sim/runs/" + id + "/log", null).status);
    }

    // --- compile on save ---

    @Test
    public void aSaveCanBeCheckedAndProblemsNameTheEditorsFileAndLine() throws Exception {
        Path sourceRoot = SimBenchTest.sourceRootWith(folder.getRoot().toPath(), SimBenchTest.tempAuto(2));
        serverWith(new SimBench(null, sourceRoot, folder.getRoot().toPath().resolve("sim"), 2, 1));
        String key = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/TempAuto.java";
        admin("POST", "/admin/files/add?path=" + key);
        String cookie = approvedUser("ada");

        Reply good = user("GET", "/build", cookie);
        assertEquals(200, good.status);
        assertEquals("{\"available\":true,\"ok\":true,\"problems\":[]}", good.body);

        String version = json(user("GET", "/files/" + key, cookie).body).get("version").getAsString();
        user("PUT", "/files/" + key, cookie, edit(SimBenchTest.tempAuto(2).replace("loops = 0", "loops = "), version));
        Reply broken = user("GET", "/build", cookie);
        assertEquals(200, broken.status);
        JsonObject body = json(broken.body);
        assertEquals(false, body.get("ok").getAsBoolean());
        JsonObject problem = body.getAsJsonArray("problems").get(0).getAsJsonObject();
        assertEquals(key, problem.get("file").getAsString());
        assertEquals(8, problem.get("line").getAsInt());
        assertTrue(problem.toString(), problem.get("message").getAsString().contains("illegal start of expression"));
        assertEquals(403, user("GET", "/build", login("bob")).status);
    }

    @Test
    public void aServerWithoutSourcesSaysSo() throws IOException {
        String cookie = approvedUser("ada");

        assertEquals("{\"available\":false}", user("GET", "/build", cookie).body);
    }

    @Test
    public void theEditTabChecksEachSaveAndListsTheProblems() throws IOException {
        String cookie = approvedEditorOf("Plans.java");

        String page = user("GET", "/", cookie).body;

        assertTrue(page, page.contains("fetch('/build')"));
        assertTrue(page, page.contains("id=\"problems\""));
        assertTrue(page, page.contains("id=\"build-status\""));
        assertTrue(page, page.contains("problem.line"));
        assertTrue(page, page.contains("compiles"));
    }

    @Test
    public void theDashboardHasEditAndSimulateTabs() throws IOException {
        String cookie = approvedEditorOf("Plans.java");

        String page = user("GET", "/", cookie).body;

        assertTrue(page, page.contains("data-tab=\"edit\""));
        assertTrue(page, page.contains("data-tab=\"simulate\""));
        assertTrue(page, page.contains("id=\"opmodes\""));
        assertTrue(page, page.contains("id=\"history\""));
        assertTrue(page, page.contains("id=\"stage\""));
        assertTrue(page, page.contains("'/sim/catalog'"));
        assertTrue(page, page.contains("'/sim/run?opmode='"));
        assertTrue(page, page.contains("'/sim/status'"));
        assertTrue(page, page.contains("'/sim/runs/'"));
        assertTrue(page, page.contains("location.hash"));
        assertHiddenWins(page);
    }

    @Test
    public void theSimulateTabShowsBuildingAndTheRunsMessageAndRefreshesTheCatalog() throws IOException {
        String cookie = approvedEditorOf("Plans.java");

        String page = user("GET", "/", cookie).body;

        assertTrue(page, page.contains("run.phase === 'building'"));
        assertTrue(page, page.contains("run.message"));
        assertTrue(page, page.contains("id=\"run-message\""));
        assertTrue(page, page.contains("as last saved"));
        assertFalse(page, page.contains("started with"));
        // the catalog is fetched every time the tab opens and again when a run ends, never cached for the page's life
        assertTrue(page, page.split("fetch\\('/sim/catalog'\\)").length - 1 >= 1);
        assertTrue(page, page.contains("loadCatalog()"));
        assertTrue(page, page.contains("wasRunning && !running"));
    }

    // --- the editor: CodeMirror, served from the host because the robot's wifi has no internet ---

    @Test
    public void theEditorBundleIsServedFromTheHostToAnyoneOnTheUserPort() throws IOException {
        Reply bundle = user("GET", "/static/codemirror.js", null);

        assertEquals(200, bundle.status);
        assertTrue(bundle.header("Content-Type"), bundle.header("Content-Type").startsWith("application/javascript"));
        assertTrue(bundle.body.contains("window.CM"));
        assertTrue("a real bundle, not a stub: " + bundle.body.length() + " bytes", bundle.body.length() > 100_000);
    }

    @Test
    public void theStaticRouteServesOnlyTheBundle() throws IOException {
        assertEquals(404, user("GET", "/static/nope.js", null).status);
        assertEquals(404, user("GET", "/static/dashboard.html", null).status);
        assertEquals(404, user("GET", "/static/admin.html", null).status);
        assertEquals(404, user("GET", "/static/../CodingServer.class", null).status);
        assertEquals(404, user("GET", "/static/", null).status);
        assertEquals(404, admin("GET", "/static/codemirror.js").status);
    }

    @Test
    public void theDashboardEditsInCodeMirrorAndShowsTheProblemsAsDiagnostics() throws IOException {
        String cookie = approvedEditorOf("Plans.java");

        String page = user("GET", "/", cookie).body;

        assertTrue(page, page.contains("<script src=\"/static/codemirror.js\"></script>"));
        assertTrue(page, page.contains("id=\"editor\""));
        assertFalse("the textarea is gone", page.contains("<textarea"));
        assertTrue(page, page.contains("new CM.EditorView("));
        assertTrue(page, page.contains("CM.java()"));
        assertTrue(page, page.contains("CM.setDiagnostics("));
        assertTrue(page, page.contains("CM.lintGutter()"));
        assertTrue("Ctrl-S / Cmd-S saves now", page.contains("key: 'Mod-s'"));
        assertTrue("Tab indents inside the editor", page.contains("CM.indentWithTab"));
        assertTrue(page, page.contains("CM.oneDark"));
    }

    /** The page only ever reaches the bundle through {@code CM.<name>}; each such name must be one the bundle exports. */
    @Test
    public void everyEditorNameThePageUsesIsInTheBundle() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String page = user("GET", "/", cookie).body;
        String bundle = user("GET", "/static/codemirror.js", null).body;
        String exports = bundle.substring(bundle.indexOf("window.CM="));
        exports = exports.substring(0, exports.indexOf("}") + 1);

        java.util.regex.Matcher names = java.util.regex.Pattern.compile("\\bCM\\.(\\w+)").matcher(page);
        java.util.Set<String> used = new java.util.TreeSet<>();
        while (names.find()) {
            used.add(names.group(1));
        }

        assertTrue("the page uses the editor: " + used, used.size() >= 5);
        for (String name : used) {
            assertTrue(name + " is not exported by the bundle: " + exports, exports.matches("(?s).*\\b" + name + ":.*"));
        }
    }

    /**
     * Every page toggles elements with the {@code hidden} attribute, and any author
     * {@code display:} rule on the same element silently beats it unless the page says otherwise.
     */
    private static void assertHiddenWins(String page) {
        assertTrue(page, page.replaceAll("\\s+", " ").contains("[hidden] { display: none !important; }"));
    }

    // --- persistence: sessions and the editable set outlive the process ---

    @Test
    public void anApprovedSessionSurvivesARestart() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String id = idOf("ada");

        restart();

        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        assertEquals(200, user("GET", "/files/TeamCode/Plans.java", cookie).status);
        assertEquals(id, idOf("ada"));
        assertTrue(user("GET", "/", cookie).body.contains("id=\"editor\""));
    }

    @Test
    public void aPendingLoginSurvivesARestartAndCanStillBeDecided() throws IOException {
        String cookie = login("bob");

        restart();

        assertEquals("{\"state\":\"pending\",\"username\":\"bob\"}", user("GET", "/me", cookie).body);
        String logins = admin("GET", "/admin/logins").body;
        assertTrue(logins, logins.contains("\"username\":\"bob\",\"address\":\"127.0.0.1\",\"state\":\"pending\""));
        admin("POST", "/admin/logins/" + idOf("bob") + "/approve");
        assertEquals("{\"state\":\"approved\",\"username\":\"bob\"}", user("GET", "/me", cookie).body);
    }

    @Test
    public void aRevokedSessionStaysRevokedAcrossARestart() throws IOException {
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        admin("POST", "/admin/logins/" + idOf("ada") + "/revoke");

        restart();

        assertEquals("{\"state\":\"revoked\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        assertEquals(403, user("GET", "/files", cookie).status);
    }

    @Test
    public void loginsAfterARestartGetIdsNobodyHasHad() throws IOException {
        login("ada");
        String adaId = idOf("ada");

        restart();
        login("bob");

        assertEquals(adaId, idOf("ada"));
        assertFalse(adaId.equals(idOf("bob")));
        assertEquals(2, json(admin("GET", "/admin/logins").body).getAsJsonArray("logins").size());
    }

    /**
     * The token in a teammate's cookie is the only thing that proves who they are, so at rest it
     * is kept the way a password would be: as a salted scrypt hash, never as the token itself and
     * never as a fast digest of it.
     */
    @Test
    public void theSessionStoreHoldsASaltedScryptHashOfEachSecretAndNeverTheSecret() throws IOException {
        String ada = login("ada");
        String bob = login("bob");

        String stored = new String(Files.readAllBytes(sessionsFile()), StandardCharsets.UTF_8);

        assertFalse(stored, stored.contains(secretOf(ada)));
        assertFalse(stored, stored.contains(secretOf(bob)));
        JsonObject kdf = storedSession("ada").getAsJsonObject("secret");
        assertEquals("scrypt", kdf.get("kdf").getAsString());
        assertTrue(kdf.toString(), kdf.get("n").getAsInt() >= 1 << 14);
        assertTrue(kdf.toString(), kdf.get("r").getAsInt() >= 8);
        assertTrue(kdf.toString(), kdf.get("p").getAsInt() >= 1);
        byte[] salt = Base64.getDecoder().decode(kdf.get("salt").getAsString());
        byte[] hash = Base64.getDecoder().decode(kdf.get("hash").getAsString());
        assertTrue("salt of " + salt.length + " bytes", salt.length >= 16);
        assertTrue("hash of " + hash.length + " bytes", hash.length >= 32);
        assertArrayEquals(SCrypt.generate(secretOf(ada).getBytes(StandardCharsets.UTF_8), salt,
                kdf.get("n").getAsInt(), kdf.get("r").getAsInt(), kdf.get("p").getAsInt(), hash.length), hash);
        assertFalse("every session gets its own salt",
                kdf.get("salt").getAsString().equals(storedSession("bob").getAsJsonObject("secret").get("salt").getAsString()));
        if (Files.getFileStore(sessionsFile()).supportsFileAttributeView(PosixFileAttributeView.class)) {
            assertEquals("rw-------", PosixFilePermissions.toString(Files.getPosixFilePermissions(sessionsFile())));
            assertEquals("rwx------", PosixFilePermissions.toString(Files.getPosixFilePermissions(stateDir())));
        }
    }

    @Test
    public void aCookieWithARealIdAndTheWrongSecretIsNoSession() throws IOException {
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        String forged = cookie.substring(0, cookie.lastIndexOf('.') + 1) + "B".repeat(secretOf(cookie).length());

        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        restart();
        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        assertEquals(403, user("GET", "/files", forged).status);
        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
    }

    @Test
    public void aSessionStoreThatCannotBeReadStopsTheServerFromStarting() throws IOException {
        login("ada");
        server.stop();
        server = null;
        Files.write(sessionsFile(), "{not json".getBytes(StandardCharsets.UTF_8));

        try {
            server();
            fail("the server started over a session store it could not read");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(sessionsFile().toString()));
        }
    }

    @Test
    public void theEditableSetSurvivesARestart() throws IOException {
        String cookie = approvedEditorOf("Plans.java", "Drive.java");
        admin("POST", "/admin/files/remove?path=TeamCode/Drive.java");

        restart();

        assertEquals("{\"files\":[{\"path\":\"TeamCode/Plans.java\"}]}", admin("GET", "/admin/files").body);
        assertEquals(200, user("GET", "/files/TeamCode/Plans.java", cookie).status);
        assertEquals(404, user("GET", "/files/TeamCode/Drive.java", cookie).status);
    }

    @Test
    public void theEditableSetIsRememberedPerProjectRoot() throws IOException {
        approvedEditorOf("Plans.java");
        Path otherRoot = state.newFolder("other-root").toPath();
        Files.createFile(otherRoot.resolve("Other.java"));
        CodingServer other = CodingServer.start(otherRoot, bench(), InetAddress.getLoopbackAddress(), 0, 0, stateDir());
        try {
            assertEquals("{\"files\":[]}", request(other.adminUrl(), "GET", "/admin/files", null, null).body);
            request(other.adminUrl(), "POST", "/admin/files/add?path=Other.java", null, null);
        } finally {
            other.stop();
        }

        restart();

        assertEquals("{\"files\":[{\"path\":\"TeamCode/Plans.java\"}]}", admin("GET", "/admin/files").body);
        other = CodingServer.start(otherRoot, bench(), InetAddress.getLoopbackAddress(), 0, 0, stateDir());
        try {
            assertEquals("{\"files\":[{\"path\":\"Other.java\"}]}", request(other.adminUrl(), "GET", "/admin/files", null, null).body);
        } finally {
            other.stop();
        }
    }

    @Test
    public void nothingIsWrittenUnderTheProjectRoot() throws IOException {
        approvedEditorOf("Plans.java");

        assertEquals("[TeamCode]", Arrays.toString(folder.getRoot().list()));
    }

    /** The XDG Base Directory spec: {@code $XDG_STATE_HOME}, else {@code ~/.local/state}, and a relative value is ignored. */
    @Test
    public void theStateDirectoryFollowsTheXdgBaseDirectoryConvention() {
        Path xdg = state.getRoot().toPath().resolve("xdg");
        Path home = state.getRoot().toPath().resolve("home");
        Path underHome = home.resolve(".local/state/midnight-snackers/coding-server");

        assertEquals(xdg.resolve("midnight-snackers/coding-server"),
                CodingServer.stateDir(Map.of("XDG_STATE_HOME", xdg.toString(), "HOME", home.toString())));
        assertEquals(underHome, CodingServer.stateDir(Map.of("HOME", home.toString())));
        assertEquals(underHome, CodingServer.stateDir(Map.of("XDG_STATE_HOME", "relative/state", "HOME", home.toString())));
        assertEquals(underHome, CodingServer.stateDir(Map.of("XDG_STATE_HOME", "", "HOME", home.toString())));
    }

    private Path sessionsFile() {
        return stateDir().resolve("sessions.json");
    }

    /** The random part of a {@code session=<id>.<secret>} cookie. */
    private static String secretOf(String cookie) {
        return cookie.substring(cookie.lastIndexOf('.') + 1);
    }

    private JsonObject storedSession(String username) throws IOException {
        for (var element : json(new String(Files.readAllBytes(sessionsFile()), StandardCharsets.UTF_8)).getAsJsonArray("sessions")) {
            if (element.getAsJsonObject().get("username").getAsString().equals(username)) {
                return element.getAsJsonObject();
            }
        }
        throw new AssertionError("no stored session for " + username);
    }

    // --- helpers ---

    private String login(String username) throws IOException {
        Reply login = user("POST", "/login?username=" + username, null);
        assertEquals(200, login.status);
        return login.sessionCookie();
    }

    /** The admin's id for the session of that username, read off the admin listing. */
    private String idOf(String username) throws IOException {
        String logins = admin("GET", "/admin/logins").body;
        int at = logins.indexOf("\"username\":\"" + username + "\"");
        assertTrue(logins, at >= 0);
        int idAt = logins.lastIndexOf("\"id\":", at) + "\"id\":".length();
        int end = idAt;
        while (Character.isDigit(logins.charAt(end))) {
            end++;
        }
        return logins.substring(idAt, end);
    }

    private Reply user(String method, String path, String cookie) throws IOException {
        return request(server().userUrl(), method, path, cookie, null);
    }

    private Reply user(String method, String path, String cookie, String body) throws IOException {
        return request(server().userUrl(), method, path, cookie, body);
    }

    private Reply admin(String method, String path) throws IOException {
        return request(server().adminUrl(), method, path, null, null);
    }

    private Reply request(String base, String method, String path, String cookie, String body) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(base + path.substring(1)).openConnection();
        connection.setRequestMethod(method);
        if (cookie != null) {
            connection.setRequestProperty("Cookie", cookie);
        }
        if (body != null) {
            connection.setDoOutput(true);
            connection.setRequestProperty("Content-Type", "application/json; charset=utf-8");
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

        /** The {@code session=...} pair from Set-Cookie, ready to send back as a Cookie header. */
        String sessionCookie() {
            String set = header("Set-Cookie");
            return set == null ? null : set.split(";")[0].trim();
        }
    }
}
