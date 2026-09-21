package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Base64;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.bouncycastle.crypto.generators.SCrypt;
import org.firstinspires.ftc.teamcode.sim.TestAutos.NeverDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class CodingServerTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    @Rule
    public TemporaryFolder state = new TemporaryFolder();

    private static final double RUN_TIMEOUT_SECONDS = 0.3;

    /**
     * What the server's benches wait in a test: a run's budget in a moment. The silence stays what
     * the bench waits in earnest, since no run here goes quiet and a shorter one would only be a
     * busy machine's chance to kill a healthy child for pausing.
     */
    private static final SimBench.Waits WAITS = SimBench.Waits.ofTheBench()
            .runTimeout(RUN_TIMEOUT_SECONDS)
            .teleOpPeriod(30)
            .killGrace(1);

    private CodingServer server;

    private Path root;

    @Before
    public void aProjectRoot() throws IOException {
        root = folder.getRoot().toPath();
        Files.createDirectories(root);
    }

    private Git git;

    /**
     * Which git the server works the repository with. A test of the server -- a login, a route, a
     * page, a file the admin picked -- gets the fake, which passes the same contract RealGitTest
     * holds the real one to, so it forks nothing. A test that then reads git's own answers about
     * the repository asks for the real one first.
     */
    private Git git() {
        if (git == null) {
            git = FakeGit.ofWhatIsOnDisk(root, Worktrees.DEVELOP);
        }
        return git;
    }

    /** Whatever is in the project root, committed on develop, by whichever git this test has. */
    private void committed(String message) {
        git().stageEverything(root);
        git().commitStaged(root, Git.Author.of("fixture", "fixture@example.invalid"), message);
    }

    /** A repository on disk, for a test whose question is one only git can answer. */
    private void aRealRepository() throws IOException {
        if (git != null) {
            throw new IllegalStateException("the git was already made; ask for the real one before the server");
        }
        GitFixture.init(root);
        git = new RealGit("git", root.toAbsolutePath().normalize());
    }

    private final java.util.List<SimBench> benches = new java.util.ArrayList<>();

    private CodingServer server() {
        if (server == null) {
            serverWith(worktree -> bench());
        }
        return server;
    }

    private static SimBench.Factory sourcesBench() {
        return worktree -> new SimBench(null, worktree, worktree.resolve("TeamCode/build/sim"), WAITS.runTimeout(2));
    }

    private static String encode(String name) throws java.io.UnsupportedEncodingException {
        return java.net.URLEncoder.encode(name, "UTF-8");
    }

    private SimBench bench() {
        return new SimBench(
                SimCatalog.of(ThreeLoopAuto.class, NeverDoneAuto.class),
                null,
                folder.getRoot().toPath().resolve("sim"),
                WAITS);
    }

    private CodingServer serverWith(SimBench bench) {
        return serverWith(worktree -> bench);
    }

    private static final int CHEAP_SCRYPT = 1 << 4;

    private CodingServer serverWith(SimBench.Factory benches) {
        return serverWith(benches, CHEAP_SCRYPT);
    }

    private CodingServer serverWith(SimBench.Factory benches, int scryptN) {
        return serverWith(benches, scryptN, refusingToReachOnshape());
    }

    private CodingServer serverWith(SimBench.Factory factory, int scryptN, CodingServer.Assets assets) {
        // Every bench the server makes, however the test asked for it, so the teardown can say whether
        // any of them was left running.
        SimBench.Factory recorded = worktree -> {
            SimBench made = factory.create(worktree);
            benches.add(made);
            return made;
        };
        server = CodingServer.start(
                root, recorded, InetAddress.getLoopbackAddress(), 0, 0, stateDir(), scryptN, assets, git());
        return server;
    }

    private static CodingServer.Assets refusingToReachOnshape() {
        return new CodingServer.Assets() {
            @Override
            public FieldAssets.Refreshed download(Store store, Path into) {
                throw new AssertionError("a test reached for Onshape; no test may");
            }

            @Override
            public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Detail> details) {
                throw new AssertionError("a test built a model it never downloaded an export for");
            }
        };
    }

    private static final class Writing implements CodingServer.Assets {
        final List<String> names;
        int downloads;
        int builds;

        Writing(String... names) {
            this.names = List.of(names);
        }

        @Override
        public FieldAssets.Refreshed download(Store store, Path into) {
            downloads++;
            java.util.Map<String, Integer> written = new java.util.LinkedHashMap<>();
            written.putAll(wrote(store, into, FieldAssets.EXPORT_FILE));
            for (String name : names) {
                written.putAll(wrote(store, into, name));
            }
            return new FieldAssets.Refreshed(written);
        }

        @Override
        public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Detail> details) {
            builds++;
            if (!store.isFile(into.resolve(FieldAssets.EXPORT_FILE))) {
                throw new FieldAssets.NothingDownloaded("nothing has been downloaded yet, download first");
            }
            java.util.Map<String, Integer> written = new java.util.LinkedHashMap<>();
            for (FieldAssets.Detail detail : details) {
                written.putAll(wrote(store, into, detail.file));
            }
            return new FieldAssets.Refreshed(written);
        }

        private static java.util.Map<String, Integer> wrote(Store store, Path into, String name) {
            byte[] body = ("fetched " + name).getBytes(StandardCharsets.UTF_8);
            store.writeWhole(into.resolve(name), body);
            return java.util.Map.of(name, body.length);
        }
    }

    private static CodingServer.Assets writing(String... names) {
        return new Writing(names);
    }

    private CodingServer serverThatHashesAsItWouldInEarnest() {
        return serverWith(
                worktree -> {
                    SimBench bench = bench();
                    benches.add(bench);
                    return bench;
                },
                CodingServer.SCRYPT_N);
    }

    private Path stateDir() {
        return state.getRoot().toPath().resolve("nested").resolve("coding-server");
    }

    private void restart() {
        server.stop();
        server = null;
        server();
    }

    @After
    public void stopServer() {
        List<String> left = runsStillInFlight();
        if (server != null) {
            server.stop();
        }
        // The suite writes what it cost from a shutdown hook, and a run performs on a daemon thread.
        // A test that ends with one still going leaves the suite spending after it: whether that run's
        // child JVM is started at all, and whether it lands before the ledger is taken, is a race with
        // the JVM winding down. So the count comes out one short now and then, on the machine that is
        // slowest that day. A test sees its runs to an outcome, or it is not done.
        assertEquals("this test ended with a simulation still running", List.of(), left);
    }

    private List<String> runsStillInFlight() {
        List<String> left = new ArrayList<>();
        for (SimBench bench : benches) {
            SimBench.Run run = bench.current();
            if (run != null) {
                left.add(run.entry.name + " (" + run.phase() + ")");
            }
        }
        return left;
    }

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
        String logins = admin("GET", "/admin/users").body;
        assertTrue(logins, logins.contains("\"username\":\"ada\""));
        assertTrue(logins, logins.contains("\"state\":\"pending\""));
        assertTrue(logins, logins.contains("\"address\":\"127.0.0.1\""));
        assertTrue("the admin list must never carry the session token", !logins.contains(cookie));
    }

    @Test
    public void theAdminListingGathersEachUsersLoginsUnderThem() throws IOException {
        login("ada");
        login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        login("bob");
        admin("POST", "/admin/logins/" + idOf("bob") + "/deny");

        JsonArray users = json(admin("GET", "/admin/users").body).getAsJsonArray("users");

        assertEquals(2, users.size());
        JsonObject ada = users.get(0).getAsJsonObject();
        JsonObject bob = users.get(1).getAsJsonObject();
        assertEquals("ada", ada.get("username").getAsString());
        assertEquals("bob", bob.get("username").getAsString());
        JsonArray logins = ada.getAsJsonArray("sessions");
        assertEquals(2, logins.size());
        JsonObject older = logins.get(0).getAsJsonObject();
        JsonObject newer = logins.get(1).getAsJsonObject();
        assertTrue(
                logins.toString(), older.get("id").getAsInt() < newer.get("id").getAsInt());
        assertEquals("pending", older.get("state").getAsString());
        assertEquals("approved", newer.get("state").getAsString());
        assertEquals("coding/ada", ada.get("branch").getAsString());
        assertEquals(0, ada.getAsJsonObject("status").get("behind").getAsInt());
        assertTrue(ada.get("worktree").getAsString().contains("ada"));
        for (String usersOwn : new String[] {"worktree", "branch", "status", "statusError", "lastMerge"}) {
            assertFalse("a login must not carry the user's " + usersOwn, newer.has(usersOwn));
        }
        assertEquals(1, bob.getAsJsonArray("sessions").size());
        assertTrue(bob.toString(), bob.get("worktree").isJsonNull());
        assertTrue(bob.toString(), bob.get("status").isJsonNull());
    }

    @Test
    public void anApprovedUserSeesTheDashboard() throws IOException {
        String cookie = login("ada");
        assertTrue(user("GET", "/", cookie).body.contains("name=\"username\""));

        Reply approved = admin("POST", "/admin/logins/" + idOf("ada") + "/approve");

        assertEquals(200, approved.status);
        assertEquals(
                "{\"state\":\"approved\",\"username\":\"ada\",\"branch\":\"coding/ada\"}",
                user("GET", "/me", cookie).body);
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
        assertTrue(admin("GET", "/admin/users").body.contains("\"username\":\"" + "a".repeat(32) + "\""));
    }

    @Test
    public void theAdminRoutesDoNotExistOnTheUserPort() throws IOException {
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");

        assertEquals(404, user("GET", "/admin", cookie).status);
        assertEquals(404, user("GET", "/admin/users", cookie).status);
        assertEquals(404, user("POST", "/admin/logins/1/approve", cookie).status);
        assertEquals(404, user("GET", "/admin/tree", cookie).status);
    }

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
        assertTrue(
                sub.body,
                sub.body.contains("{\"name\":\"build.gradle\",\"type\":\"file\",\"path\":\"TeamCode/build.gradle\"}"));
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
        committed("the files");
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

    private String approvedEditorOf(String... files) throws IOException {
        folder.newFolder("TeamCode");
        for (String file : files) {
            folder.newFile("TeamCode/" + file);
            assertEquals(200, admin("POST", "/admin/files/add?path=TeamCode/" + file).status);
        }
        committed("the files");
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        return cookie;
    }

    private Path file(String name) throws IOException {
        return worktreeOf("ada").resolve("TeamCode").resolve(name);
    }

    private Path worktreeOf(String username) throws IOException {
        JsonObject user = userOf(username);
        if (user.get("worktree").isJsonNull()) {
            throw new AssertionError("no worktree for " + username);
        }
        return Path.of(user.get("worktree").getAsString());
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
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();

        Reply written = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("new content\n", version));

        assertEquals(written.body, 200, written.status);
        assertEquals("new content\n", new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
        assertEquals(
                sha256("new content\n".getBytes(StandardCharsets.UTF_8)),
                json(written.body).get("version").getAsString());
    }

    @Test
    public void anEditFromAStaleVersionIsRefusedWithTheCurrentContent() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "on disk".getBytes(StandardCharsets.UTF_8));

        Reply refused = user(
                "PUT",
                "/files/TeamCode/Plans.java",
                cookie,
                edit("mine", sha256("something else".getBytes(StandardCharsets.UTF_8))));

        assertEquals(409, refused.status);
        JsonObject json = json(refused.body);
        assertEquals("on disk", json.get("content").getAsString());
        assertEquals(
                sha256("on disk".getBytes(StandardCharsets.UTF_8)),
                json.get("version").getAsString());
        assertEquals("on disk", new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
    }

    @Test
    public void aChangeMadeOnTheHostBetweenReadAndWriteIsAConflict() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "v1".getBytes(StandardCharsets.UTF_8));
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();
        Files.write(file("Plans.java"), "v2 from the IDE".getBytes(StandardCharsets.UTF_8));

        Reply refused = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("v2 from the browser", version));

        assertEquals(409, refused.status);
        assertEquals("v2 from the IDE", new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
    }

    @Test
    public void writesLeaveNoTemporaryFilesAndKeepLineEndings() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        Files.write(file("Plans.java"), "a\r\nb\r\n".getBytes(StandardCharsets.UTF_8));
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();

        Reply written = user("PUT", "/files/TeamCode/Plans.java", cookie, edit("a\r\nb\r\nc", version));

        assertEquals(200, written.status);
        assertArrayEquals("a\r\nb\r\nc".getBytes(StandardCharsets.UTF_8), Files.readAllBytes(file("Plans.java")));
        assertEquals(
                "[Plans.java]",
                Arrays.toString(file("Plans.java").getParent().toFile().list()));
    }

    @Test
    public void anOversizedEditIsRefused() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();

        Reply refused = user(
                "PUT",
                "/files/TeamCode/Plans.java",
                cookie,
                edit("x".repeat(TinyHttpServer.MAX_BODY_BYTES + 1), version));

        assertEquals(413, refused.status);
        assertEquals(0, Files.size(file("Plans.java")));
    }

    @Test
    public void aFileThatIsNotUtf8IsListedButNotEditable() throws IOException {
        // the file this is about is not text, so only a real git can hold it
        aRealRepository();
        String cookie = approvedEditorOf("logo.bin");
        Files.write(file("logo.bin"), new byte[] {(byte) 0xff, (byte) 0xfe, 0x00, (byte) 0xc3});

        assertTrue(user("GET", "/files", cookie).body.contains("\"path\":\"TeamCode/logo.bin\""));
        assertEquals(415, user("GET", "/files/TeamCode/logo.bin", cookie).status);
        assertEquals(415, user("PUT", "/files/TeamCode/logo.bin", cookie, edit("text", "whatever")).status);
        assertArrayEquals(
                new byte[] {(byte) 0xff, (byte) 0xfe, 0x00, (byte) 0xc3}, Files.readAllBytes(file("logo.bin")));
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
        JsonObject bobsLogin = newestSessionOf("bob");
        assertEquals("TeamCode/Drive.java", bobsLogin.get("file").getAsString());
        assertEquals("approved", bobsLogin.get("state").getAsString());
        assertEquals("127.0.0.1", bobsLogin.get("address").getAsString());
    }

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
    public void theAdminPageListsUsersWithDecisionsAndTheFilePicker() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("id=\"users\""));
        assertTrue(page, page.contains("/admin/users"));
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

    @Test
    public void theAdminPageKeepsEachUsersLoginsUnderThemBehindAToggle() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("user.sessions"));
        assertTrue(page, page.contains("aria-expanded"));
        assertTrue("the fold survives the listing's refresh", page.contains("expandedByUsername"));
        assertTrue("a pending login opens its user", page.contains("pending"));
        assertTrue(page, page.contains("session.state"));
        assertTrue("each login keeps its own decisions", page.contains("'/admin/logins/' + session.id"));
    }

    private String approvedUser(String name) throws IOException {
        String cookie = login(name);
        admin("POST", "/admin/logins/" + idOf(name) + "/approve");
        return cookie;
    }

    /** A run whose op mode's time has begun, which is to say one whose child is up. */
    private static final String RUNNING = "\"phase\":\"running\"";

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
        assertTrue(catalog.body, catalog.body.contains("\"where\":\"" + ThreeLoopAuto.class.getName() + "\""));
    }

    @Test
    public void aRunStartsAndTheStatusFollowsItToItsOutcomeWithWhoStartedIt() throws Exception {
        String cookie = approvedUser("ada");

        Reply started = user("POST", "/sim/run?opmode=" + encode("Count to three"), cookie);

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
        assertTrue(folder.getRoot()
                .toPath()
                .resolve("sim")
                .resolve("Count to three.html")
                .toFile()
                .exists());
    }

    @Test
    public void oneRunAtATimePerUserAndTwoUsersRunAtOnce() throws Exception {
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Never done"), ada).status);
        assertTrue(user("GET", "/sim/status", ada).body.contains("\"running\":true"));

        Reply adaAgain = user("POST", "/sim/run?opmode=" + encode("Count to three"), ada);
        Reply bobToo = user("POST", "/sim/run?opmode=" + encode("Never done"), bob);

        assertEquals(409, adaAgain.status);
        assertTrue(adaAgain.body, adaAgain.body.contains("ada"));
        assertEquals(bobToo.body, 200, bobToo.status);
        assertTrue(user("GET", "/sim/status", bob).body.contains("\"running\":true"));
        String status = awaitSimStatus(ada, "\"outcome\":\"timed out");
        assertTrue(status, status.contains("\"running\":false"));
        awaitSimStatus(bob, "\"outcome\":\"timed out");
    }

    @Test
    public void aUsersSimStatusShowsTheirRunsOnly() throws Exception {
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Count to three"), ada).status);
        awaitSimStatus(ada, "\"outcome\":\"done\"");

        String bobs = user("GET", "/sim/status", bob).body;

        assertEquals("{\"running\":false,\"runs\":[]}", bobs);
        assertEquals(404, user("GET", "/sim/runs/1/", bob).status);
        assertEquals(200, user("GET", "/sim/runs/1/", ada).status);
    }

    @Test
    public void stoppingTheServerStopsEveryUsersBenchAndChild() throws Exception {
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Never done"), ada).status);
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Never done"), bob).status);
        assertEquals(2, benches.size());
        // A child is what this stops, so wait until there is one: a run stopped while its bench
        // was still starting the child proves nothing, and starts one child fewer.
        awaitSimStatus(ada, RUNNING);
        awaitSimStatus(bob, RUNNING);

        server.stop();

        for (SimBench bench : benches) {
            assertNull(bench.current());
            assertTrue(bench.status(), bench.status().contains("\"outcome\":\"stopped\""));
        }
    }

    @Test
    public void oneUsersBrokenEditDoesNotBreakAnothers() throws Exception {
        // the real project, whose resources are not all text: a real git carries the bytes
        aRealRepository();
        SimBenchTest.projectWith(root, SimBenchTest.tempPlans(2));
        committed("the auto");
        serverWith(sourcesBench());
        String key = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Plans.java";
        assertEquals(200, admin("POST", "/admin/files/add?path=" + key).status);
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        String version =
                json(user("GET", "/files/" + key, ada).body).get("version").getAsString();
        assertEquals(
                200,
                user(
                                "PUT",
                                "/files/" + key,
                                ada,
                                edit(SimBenchTest.tempPlans(2).replace("loops = 0", "loops = "), version))
                        .status);

        Reply adas = user("GET", "/sim/catalog", ada);
        Reply bobs = user("GET", "/sim/catalog", bob);

        assertEquals(500, adas.status);
        assertEquals(bobs.body, 200, bobs.status);
        assertTrue(bobs.body, bobs.body.contains("\"name\":\"Temp\""));
        assertEquals(200, user("POST", "/sim/run?opmode=" + SimBenchTest.TEMP_NAME, bob).status);
        assertEquals(200, user("POST", "/sim/run?opmode=" + SimBenchTest.TEMP_NAME, ada).status);
        String bobsRun = awaitSimStatus(bob, "\"outcome\":\"done\"");
        assertTrue(bobsRun, bobsRun.contains("\"loops\":2"));
        String adasRun = awaitSimStatus(ada, "\"outcome\":\"build failed\"");
        assertTrue(adasRun, adasRun.contains("Plans.java:" + SimBenchTest.TEMP_LOOPS_LINE));
    }

    @Test
    public void theSimIsForApprovedSessionsOnly() throws IOException {
        String pending = login("bob");

        assertEquals(403, user("GET", "/sim/catalog", pending).status);
        assertEquals(403, user("GET", "/sim/status", pending).status);
        assertEquals(403, user("POST", "/sim/run?opmode=" + encode("Count to three"), pending).status);
        assertEquals(403, user("GET", "/sim/runs/1/", pending).status);
        assertEquals(403, user("GET", "/sim/runs/1/ticks?from=0", null).status);
        assertEquals(403, user("GET", "/sim/start?opmode=" + encode("Count to three"), pending).status);
        assertEquals(
                403,
                user("PUT", "/sim/start?opmode=" + encode("Count to three"), pending, "{\"x\":1,\"y\":2,\"heading\":0}")
                        .status);
        assertEquals(403, user("GET", "/sim/place?opmode=" + encode("Count to three"), pending).status);
    }

    @Test
    public void eachUserPlacesTheRobotOnTheirOwnBench() throws Exception {
        serverWith(worktree ->
                new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, worktree.resolve("TeamCode/build/sim"), WAITS));
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        String start = "/sim/start?opmode=" + encode("Count to three");

        Reply placed = user("PUT", start, ada, "{\"x\": 24, \"y\": -12, \"heading\": 0.5}");

        assertEquals(placed.body, 200, placed.status);
        assertEquals("{\"x\":24.0,\"y\":-12.0,\"heading\":0.5}", user("GET", start, ada).body);
        assertEquals("{\"x\":0.0,\"y\":0.0,\"heading\":0.0}", user("GET", start, bob).body);
        Reply page = user("GET", "/sim/place?opmode=" + encode("Count to three"), ada);
        assertEquals(200, page.status);
        assertTrue(page.body, page.body.contains("\"placing\":{\"x\":24.0,\"y\":-12.0,\"heading\":0.5}"));

        assertEquals(200, user("PUT", "/sim/seed?opmode=" + encode("Count to three"), ada, "{\"seed\": null}").status);
        Reply started = user("POST", "/sim/run?opmode=" + encode("Count to three"), ada);
        assertEquals(started.body, 200, started.status);
        String id = json(started.body).get("id").getAsString();
        awaitSimStatus(ada, "\"outcome\":\"done\"");
        String ticks = user("GET", "/sim/runs/" + id + "/ticks?from=0", ada).body;
        assertTrue(ticks, ticks.contains("\"x\":24.0,\"y\":-12.0,\"heading\":0.5"));
    }

    @Test
    public void eachUserSeedsTheRobotOnTheirOwnBench() throws Exception {
        serverWith(worktree ->
                new SimBench(SimCatalog.of(ThreeLoopAuto.class), null, worktree.resolve("TeamCode/build/sim"), WAITS));
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        String seed = "/sim/seed?opmode=" + encode("Count to three");

        Reply set = user("PUT", seed, ada, "{\"seed\": 4}");

        assertEquals(set.body, 200, set.status);
        assertEquals("{\"seed\":4}", user("GET", seed, ada).body);
        assertEquals("{\"seed\":1}", user("GET", seed, bob).body);
        assertTrue(user("GET", "/sim/catalog", ada).body.contains("\"seed\":4"));
        assertTrue(user("GET", "/sim/catalog", bob).body.contains("\"seed\":1"));
        Reply started = user("POST", "/sim/run?opmode=" + encode("Count to three"), ada);
        assertEquals(started.body, 200, started.status);
        awaitSimStatus(ada, "\"outcome\":\"done\"");
        assertTrue(user("GET", "/sim/status", ada).body.contains("\"seed\":4"));
    }

    @Test
    public void unknownOpModesAndWrongMethodsAreRejected() throws IOException {
        String cookie = approvedUser("ada");

        assertEquals(404, user("POST", "/sim/run?opmode=org.example.Nope", cookie).status);
        assertEquals(405, user("GET", "/sim/run?opmode=" + encode("Count to three"), cookie).status);
        assertEquals(404, user("GET", "/sim/runs/999/ticks?from=0", cookie).status);
        assertEquals(404, user("GET", "/sim/nope", cookie).status);
    }

    @Test
    public void anEditSavedInTheEditorDrivesTheNextRun() throws Exception {
        // the real project, whose resources are not all text: a real git carries the bytes
        aRealRepository();
        SimBenchTest.projectWith(root, SimBenchTest.tempPlans(2));
        committed("the auto");
        serverWith(sourcesBench());
        String key = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Plans.java";
        assertEquals(200, admin("POST", "/admin/files/add?path=" + key).status);
        String cookie = approvedUser("ada");
        assertTrue(user("GET", "/sim/catalog", cookie).body.contains("\"name\":\"Temp\""));

        String version =
                json(user("GET", "/files/" + key, cookie).body).get("version").getAsString();
        assertEquals(200, user("PUT", "/files/" + key, cookie, edit(SimBenchTest.tempPlans(4), version)).status);
        Reply started = user("POST", "/sim/run?opmode=" + SimBenchTest.TEMP_NAME, cookie);

        assertEquals(started.body, 200, started.status);
        String status = awaitSimStatus(cookie, "\"outcome\":\"done\"");
        assertTrue(status, status.contains("\"loops\":4"));
        assertTrue(status, status.contains("\"phase\":\"finished\""));
        assertTrue(status, status.contains("\"message\":null"));
    }

    @Test
    public void aBrokenEditIsReportedByTheRunAndTheCatalog() throws Exception {
        // the real project, whose resources are not all text: a real git carries the bytes
        aRealRepository();
        SimBenchTest.projectWith(root, SimBenchTest.tempPlans(2).replace("loops = 0", "loops = "));
        committed("the broken auto");
        serverWith(sourcesBench());
        String cookie = approvedUser("ada");

        Reply catalog = user("GET", "/sim/catalog", cookie);
        assertEquals(500, catalog.status);
        assertTrue(catalog.body, catalog.body.contains("Plans.java:" + SimBenchTest.TEMP_LOOPS_LINE));
        assertEquals(200, user("POST", "/sim/run?opmode=" + SimBenchTest.TEMP_NAME, cookie).status);
        String status = awaitSimStatus(cookie, "\"outcome\":\"build failed\"");
        assertTrue(status, status.contains("Plans.java:" + SimBenchTest.TEMP_LOOPS_LINE));
    }

    @Test
    public void theRunLogIsWhatTheChildWroteToStderr() throws Exception {
        serverWith(new SimBench(
                SimCatalog.of(TestAutos.ChattyAuto.class),
                null,
                folder.getRoot().toPath().resolve("sim"),
                WAITS.runTimeout(2)));
        String cookie = approvedUser("ada");
        String id = json(user("POST", "/sim/run?opmode=" + encode("Chatty"), cookie).body)
                .get("id")
                .getAsString();
        awaitSimStatus(cookie, "\"outcome\":\"done\"");

        Reply log = user("GET", "/sim/runs/" + id + "/log", cookie);

        assertEquals(200, log.status);
        assertTrue(log.body, log.body.contains("hello from the op mode"));
        assertEquals(403, user("GET", "/sim/runs/" + id + "/log", null).status);
    }

    @Test
    public void aSaveCanBeCheckedAndProblemsNameTheEditorsFileAndLine() throws Exception {
        // the real project, whose resources are not all text: a real git carries the bytes
        aRealRepository();
        SimBenchTest.projectWith(root, SimBenchTest.tempPlans(2));
        committed("the auto");
        serverWith(sourcesBench());
        String key = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Plans.java";
        admin("POST", "/admin/files/add?path=" + key);
        String cookie = approvedUser("ada");

        Reply good = user("GET", "/build", cookie);
        assertEquals(200, good.status);
        assertEquals("{\"available\":true,\"ok\":true,\"problems\":[]}", good.body);

        String version =
                json(user("GET", "/files/" + key, cookie).body).get("version").getAsString();
        user("PUT", "/files/" + key, cookie, edit(SimBenchTest.tempPlans(2).replace("loops = 0", "loops = "), version));
        Reply broken = user("GET", "/build", cookie);
        assertEquals(200, broken.status);
        JsonObject body = json(broken.body);
        assertEquals(false, body.get("ok").getAsBoolean());
        JsonObject problem = body.getAsJsonArray("problems").get(0).getAsJsonObject();
        assertEquals(key, problem.get("file").getAsString());
        assertEquals(SimBenchTest.TEMP_LOOPS_LINE, problem.get("line").getAsInt());
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
        assertTrue(page, page.contains("'/sim/place?opmode='"));
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

        assertTrue(page, page.split("fetch\\('/sim/catalog'\\)").length - 1 >= 1);
        assertTrue(page, page.contains("loadCatalog()"));
        assertTrue(page, page.contains("wasRunning && !running"));
    }

    @Test
    public void theEditorBundleIsServedFromTheHostToAnApprovedSession() throws IOException {
        Reply bundle = user("GET", "/static/codemirror.js", approvedUser("mia"));

        assertEquals(200, bundle.status);
        assertTrue(bundle.header("Content-Type"), bundle.header("Content-Type").startsWith("application/javascript"));
        assertTrue(bundle.body.contains("window.CM"));
        assertTrue("a real bundle, not a stub: " + bundle.body.length() + " bytes", bundle.body.length() > 100_000);
    }

    @Test
    public void theStaticRouteServesOnlyTheBundle() throws IOException {
        String cookie = approvedUser("mia");
        assertEquals(404, user("GET", "/static/nope.js", cookie).status);
        assertEquals(404, user("GET", "/static/dashboard.html", cookie).status);
        assertEquals(404, user("GET", "/static/admin.html", cookie).status);
        assertEquals(404, user("GET", "/static/../CodingServer.class", cookie).status);
        assertEquals(404, user("GET", "/static/", cookie).status);
        assertEquals(404, admin("GET", "/static/codemirror.js").status);
    }

    @Test
    public void onlyTheLoginAnswersASessionThatIsNotApproved() throws IOException {
        for (String path : new String[] {
            "/static/codemirror.js",
            "/files",
            "/files/Plans.java",
            "/git/status",
            "/build",
            "/sim/catalog",
            "/nav/definition",
            "/source/Plans.java",
            "/sim/assets/field.glb",
            "/sim/assets/textures/GoalAprilTag_bluescoring.png",
            "/sim/assets/vendor/three.module.min.js"
        }) {
            assertEquals(path + " needs an approved session", 403, user("GET", path, null).status);
        }
    }

    @Test
    public void aServerThatHasFetchedNothingDrawsTheModelCommittedForTests() throws IOException {
        String cookie = approvedUser("mia");

        Reply model = user("GET", "/sim/assets/field.glb", cookie);
        Reply said = admin("GET", "/admin/assets");

        assertEquals("the committed model still draws", 200, model.status);
        assertTrue(said.body, said.body.contains("\"complete\":false"));
        assertTrue(said.body, said.body.contains("the model committed for tests"));
    }

    @Test
    public void aRefreshFetchesTheAssetsAndTheyAreWhatTheBenchThenServes() throws IOException {
        serverWith(
                worktree -> bench(),
                CHEAP_SCRYPT,
                writing(FieldAssets.everyAsset().toArray(new String[0])));
        String cookie = approvedUser("mia");

        Reply refreshed = admin("POST", "/admin/assets/refresh");
        Reply model = user("GET", "/sim/assets/field.glb", cookie);

        assertEquals(refreshed.body, 200, refreshed.status);
        assertTrue(refreshed.body, refreshed.body.contains("\"complete\":true"));
        assertTrue(refreshed.body, refreshed.body.contains("the model this server fetched"));
        assertEquals(200, model.status);
        assertEquals("the fetched model, not the committed one", "fetched field.glb", model.body);
    }

    @Test
    public void anAssetTheRefreshDidNotWriteStillComesFromWhatIsCommitted() throws IOException {
        serverWith(worktree -> bench(), CHEAP_SCRYPT, writing());
        String cookie = approvedUser("mia");

        admin("POST", "/admin/assets/refresh");

        assertEquals("fetched field.glb", user("GET", "/sim/assets/field.glb", cookie).body);
        assertEquals(
                "the tag artwork falls back rather than going missing",
                200,
                user("GET", "/sim/assets/textures/GoalAprilTag_bluescoring.png", cookie).status);
    }

    @Test
    public void aRefreshOnshapeWillNotAnswerSaysSoAndLeavesThePagesDrawing() throws IOException {
        serverWith(worktree -> bench(), CHEAP_SCRYPT, new CodingServer.Assets() {
            @Override
            public FieldAssets.Refreshed download(Store store, Path into) {
                throw new Onshape.NoCredentials("no key pair and no proxy");
            }

            @Override
            public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Detail> details) {
                throw new AssertionError("a download that never happened must not reach a build");
            }
        });
        String cookie = approvedUser("mia");

        Reply refused = admin("POST", "/admin/assets/refresh");

        assertEquals(502, refused.status);
        assertTrue(refused.body, refused.body.contains("no key pair"));
        assertEquals("and the page still draws", 200, user("GET", "/sim/assets/field.glb", cookie).status);
    }

    @Test
    public void theFullModelIsFetchedOnlyWhenItIsAskedFor() throws IOException {
        serverWith(worktree -> bench(), CHEAP_SCRYPT, writing());
        String cookie = approvedUser("mia");

        Reply normal = admin("POST", "/admin/assets/refresh?detail=normal");

        assertEquals(normal.body, 200, normal.status);
        assertTrue(normal.body, normal.body.contains("\"normal\":true"));
        assertTrue(normal.body, normal.body.contains("\"full\":false"));
        assertEquals(
                "a page asking for the full model is told it is not there",
                404,
                user("GET", "/sim/assets/field-full.glb", cookie).status);

        Reply both = admin("POST", "/admin/assets/refresh?detail=both");

        assertTrue(both.body, both.body.contains("\"full\":true"));
        assertEquals(200, user("GET", "/sim/assets/field-full.glb", cookie).status);
        assertEquals("fetched field-full.glb", user("GET", "/sim/assets/field-full.glb", cookie).body);
    }

    @Test
    public void aDownloadAndABuildAreAskedForSeparately() throws IOException {
        Writing assets = new Writing(FieldAssets.everyAsset().toArray(new String[0]));
        serverWith(worktree -> bench(), CHEAP_SCRYPT, assets);
        String cookie = approvedUser("mia");

        Reply downloaded = admin("POST", "/admin/assets/download");

        assertEquals(downloaded.body, 200, downloaded.status);
        assertEquals(1, assets.downloads);
        assertEquals("a download is the dear half, and it builds nothing", 0, assets.builds);
        assertTrue(downloaded.body, downloaded.body.contains("\"downloaded\":true"));

        Reply built = admin("POST", "/admin/assets/build?detail=both");

        assertEquals(built.body, 200, built.status);
        assertEquals("and a build reuses what was downloaded rather than fetching again", 1, assets.downloads);
        assertEquals(1, assets.builds);
        assertTrue(built.body, built.body.contains("\"full\":true"));
        assertEquals(200, user("GET", "/sim/assets/field-full.glb", cookie).status);
    }

    @Test
    public void aBuildWithNothingDownloadedIsRefusedAndSaysWhatToDo() throws IOException {
        serverWith(worktree -> bench(), CHEAP_SCRYPT, writing());

        Reply refused = admin("POST", "/admin/assets/build?detail=normal");

        assertEquals(refused.body, 409, refused.status);
        assertTrue(refused.body, refused.body.contains("nothing has been downloaded"));
    }

    @Test
    public void aDetailNobodyBuildsIsRefusedBeforeAnythingIsBuilt() throws IOException {
        Writing assets = new Writing();
        serverWith(worktree -> bench(), CHEAP_SCRYPT, assets);

        Reply refused = admin("POST", "/admin/assets/build?detail=finest");

        assertEquals(400, refused.status);
        assertTrue(refused.body, refused.body.contains("normal, full or both"));
        assertEquals(0, assets.builds);
    }

    @Test
    public void aDetailNobodyBuildsIsRefusedRatherThanFetched() throws IOException {
        serverWith(worktree -> bench(), CHEAP_SCRYPT, writing());

        Reply refused = admin("POST", "/admin/assets/refresh?detail=finest");

        assertEquals(400, refused.status);
        assertTrue(refused.body, refused.body.contains("normal, full or both"));
    }

    @Test
    public void aTabRemembersWhatWasAskedOfItRatherThanBouncingToTheOtherOne() throws Exception {
        String[] asked = {
            "#simulate",
            "#simulate?detail=full",
            "#simulate?view=camera&cost",
            "#edit",
            "#",
            "",
            "#nonsense",
            "#edit?detail=full"
        };

        String[] got = tabsOnTheDashboard(asked);

        assertEquals("simulate ", got[0]);
        assertEquals("a tab keeps the options asked of it", "simulate detail=full", got[1]);
        assertEquals("simulate view=camera&cost", got[2]);
        assertEquals("edit ", got[3]);
        assertEquals("edit ", got[4]);
        assertEquals("edit ", got[5]);
        assertEquals("a tab nobody has is the editor", "edit ", got[6]);
        assertEquals("and options only mean something to the tab that reads them", "edit ", got[7]);
    }

    private String[] tabsOnTheDashboard(String[] asked) throws Exception {
        String page = SimAssets.page("dashboard.html");
        Matcher rule = Pattern.compile("\n  function tabOf\\(hash\\) \\{.*?\n  \\}", Pattern.DOTALL)
                .matcher(page);
        assertTrue("the dashboard works a tab out in tabOf(hash)", rule.find());
        Path script = folder.newFile("tabOf.js").toPath();
        StringBuilder source = new StringBuilder(rule.group()).append("\n");
        source.append("const asked = ").append(new Gson().toJson(asked)).append(";\n");
        source.append("console.log(JSON.stringify(asked.map(h => { const t = tabOf(h); ")
                .append("return t.name + ' ' + t.options; })));\n");
        Files.write(script, source.toString().getBytes(StandardCharsets.UTF_8));
        return new Gson().fromJson(inNode(script), String[].class);
    }

    /** The one place a rule off the dashboard page is run in node, so the one place it is counted. */
    private static String inNode(Path script) throws Exception {
        String out;
        try (Cost.Spent spent = Cost.start(Cost.Kind.NODE)) {
            Process node = new ProcessBuilder("node", script.toString())
                    .redirectErrorStream(true)
                    .start();
            out = new String(node.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            assertEquals("node ran the dashboard's rule: " + out, 0, node.waitFor());
        }
        return out.trim();
    }

    @Test
    public void theViewIsOpenedAgainWhenWhatWasAskedOfItChanges() throws Exception {
        String[][] asked = {
            {"null", "null", "1", ""},
            {"1", "", "1", ""},
            {"1", "", "1", "detail=full"},
            {"1", "detail=full", "1", "detail=full"},
            {"1", "detail=full", "2", "detail=full"},
            {"1", "detail=full", "1", ""}
        };

        boolean[] opens = stageOpenings(asked);

        assertTrue("nothing shown yet", opens[0]);
        assertFalse("the same run, asked the same way, is already open", opens[1]);
        assertTrue("the same run asked a different way is opened again", opens[2]);
        assertFalse(opens[3]);
        assertTrue("another run", opens[4]);
        assertTrue("and dropping an option is a change too", opens[5]);
    }

    private boolean[] stageOpenings(String[][] asked) throws Exception {
        String page = SimAssets.page("dashboard.html");
        Matcher rule = Pattern.compile(
                        "\n  function stageNeedsOpening\\(shownId, shownOptions, id, options\\) \\{.*?\n  \\}",
                        Pattern.DOTALL)
                .matcher(page);
        assertTrue("the dashboard decides in stageNeedsOpening(...)", rule.find());
        Path script = folder.newFile("stageNeedsOpening.js").toPath();
        StringBuilder source = new StringBuilder(rule.group()).append("\n");
        source.append("const asked = ").append(new Gson().toJson(asked)).append(";\n");
        source.append("console.log(JSON.stringify(asked.map(a => stageNeedsOpening(")
                .append("a[0] === 'null' ? null : Number(a[0]), a[1] === 'null' ? null : a[1], ")
                .append("Number(a[2]), a[3]))));\n");
        Files.write(script, source.toString().getBytes(StandardCharsets.UTF_8));
        return new Gson().fromJson(inNode(script), boolean[].class);
    }

    @Test
    public void theSimulateTabPassesWhatWasAskedOfItToTheLiveView() {
        String page = SimAssets.page("dashboard.html");

        assertTrue("the live view is opened with them", page.contains("withTabOptions('/sim/runs/'"));
        assertTrue(page.contains("function withTabOptions(url)"));
    }

    @Test
    public void theAdminPageOffersTheRefresh() throws IOException {
        server();

        Reply page = admin("GET", "/admin");

        assertTrue(page.body, page.body.contains("Refresh assets"));
        assertTrue(page.body, page.body.contains("/admin/assets/refresh"));
        assertTrue(
                "a download that is kept is worth a button of its own", page.body.contains("/admin/assets/download"));
        assertTrue("and rebuilding from it is the cheap half", page.body.contains("/admin/assets/build"));
    }

    @Test
    public void theLiveViewsAssetsAreServedUnderTheBenchsPrefix() throws IOException {
        String cookie = approvedUser("mia");

        Reply scene = user("GET", "/sim/assets/fieldscene.js", cookie);
        assertEquals("the scene the live view draws the field with", 200, scene.status);
        assertTrue("and it is what fetches the model: " + scene.body, scene.body.contains("'field.glb'"));
        assertEquals("which is served there too", 200, user("GET", "/sim/assets/field.glb", cookie).status);
        assertEquals("with the probe", 200, user("GET", "/sim/assets/framecost.js", cookie).status);
        assertEquals("and the webcam's view", 200, user("GET", "/sim/assets/webcam.js", cookie).status);

        assertEquals(
                "the field page is retired; the live view draws it", 404, user("GET", "/sim/field", cookie).status);
        assertEquals(
                "and not at the root, which is the coding server's own", 404, user("GET", "/field", cookie).status);
    }

    @Test
    public void whatIsOpenIsTheLoginAndNothingElse() throws IOException {
        assertEquals("the page that offers the login", 200, user("GET", "/", null).status);
        assertTrue(user("GET", "/", null).body.contains("login"));

        Reply me = user("GET", "/me", null);
        assertEquals("the login page polls this to see when it is approved", 200, me.status);
        assertTrue("and it says nothing about anyone else: " + me.body, me.body.contains("\"none\""));
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

    @Test
    public void everyEditorNameThePageUsesIsInTheBundle() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String page = user("GET", "/", cookie).body;
        String bundle = user("GET", "/static/codemirror.js", cookie).body;
        String exports = bundle.substring(bundle.indexOf("window.CM="));
        exports = exports.substring(0, exports.indexOf("}") + 1);

        java.util.regex.Matcher names =
                java.util.regex.Pattern.compile("\\bCM\\.(\\w+)").matcher(page);
        java.util.Set<String> used = new java.util.TreeSet<>();
        while (names.find()) {
            used.add(names.group(1));
        }

        assertTrue("the page uses the editor: " + used, used.size() >= 5);
        for (String name : used) {
            assertTrue(
                    name + " is not exported by the bundle: " + exports, exports.matches("(?s).*\\b" + name + ":.*"));
        }
    }

    private static void assertHiddenWins(String page) {
        assertTrue(page, page.replaceAll("\\s+", " ").contains("[hidden] { display: none !important; }"));
    }

    @Test
    public void anApprovedSessionSurvivesARestart() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String id = idOf("ada");

        restart();

        assertEquals(
                "{\"state\":\"approved\",\"username\":\"ada\",\"branch\":\"coding/ada\"}",
                user("GET", "/me", cookie).body);
        assertEquals(200, user("GET", "/files/TeamCode/Plans.java", cookie).status);
        assertEquals(id, idOf("ada"));
        assertTrue(user("GET", "/", cookie).body.contains("id=\"editor\""));
    }

    @Test
    public void aPendingLoginSurvivesARestartAndCanStillBeDecided() throws IOException {
        String cookie = login("bob");

        restart();

        assertEquals("{\"state\":\"pending\",\"username\":\"bob\"}", user("GET", "/me", cookie).body);
        JsonObject session = newestSessionOf("bob");
        assertEquals("pending", session.get("state").getAsString());
        assertEquals("127.0.0.1", session.get("address").getAsString());
        admin("POST", "/admin/logins/" + idOf("bob") + "/approve");
        assertEquals(
                "{\"state\":\"approved\",\"username\":\"bob\",\"branch\":\"coding/bob\"}",
                user("GET", "/me", cookie).body);
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
        assertEquals(
                2,
                json(admin("GET", "/admin/users").body).getAsJsonArray("users").size());
    }

    @Test
    public void theSessionStoreHoldsASaltedScryptHashOfEachSecretAndNeverTheSecret() throws IOException {
        serverThatHashesAsItWouldInEarnest();
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
        byte[] again;
        try (Cost.Spent spent = Cost.start(Cost.Kind.PASSWORD_HASH)) {
            again = SCrypt.generate(
                    secretOf(ada).getBytes(StandardCharsets.UTF_8),
                    salt,
                    kdf.get("n").getAsInt(),
                    kdf.get("r").getAsInt(),
                    kdf.get("p").getAsInt(),
                    hash.length);
        }
        assertArrayEquals(again, hash);
        assertFalse(
                "every session gets its own salt",
                kdf.get("salt")
                        .getAsString()
                        .equals(storedSession("bob")
                                .getAsJsonObject("secret")
                                .get("salt")
                                .getAsString()));
        if (Files.getFileStore(sessionsFile()).supportsFileAttributeView(PosixFileAttributeView.class)) {
            assertEquals("rw-------", PosixFilePermissions.toString(Files.getPosixFilePermissions(sessionsFile())));
            assertEquals("rwx------", PosixFilePermissions.toString(Files.getPosixFilePermissions(stateDir())));
        }
    }

    @Test
    public void aCookieWithARealIdAndTheWrongSecretIsNoSession() throws IOException {
        String cookie = login("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/approve");
        String forged = cookie.substring(0, cookie.lastIndexOf('.') + 1)
                + "B".repeat(secretOf(cookie).length());

        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        restart();
        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        assertEquals(403, user("GET", "/files", forged).status);
        assertEquals(
                "{\"state\":\"approved\",\"username\":\"ada\",\"branch\":\"coding/ada\"}",
                user("GET", "/me", cookie).body);
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
        GitFixture.init(otherRoot);
        Files.createFile(otherRoot.resolve("Other.java"));
        SimBench otherBench = bench();
        CodingServer other = CodingServer.start(
                otherRoot, worktree -> otherBench, InetAddress.getLoopbackAddress(), 0, 0, stateDir());
        try {
            assertEquals("{\"files\":[]}", request(other.adminUrl(), "GET", "/admin/files", null, null).body);
            request(other.adminUrl(), "POST", "/admin/files/add?path=Other.java", null, null);
        } finally {
            other.stop();
        }

        restart();

        assertEquals("{\"files\":[{\"path\":\"TeamCode/Plans.java\"}]}", admin("GET", "/admin/files").body);
        other = CodingServer.start(
                otherRoot, worktree -> otherBench, InetAddress.getLoopbackAddress(), 0, 0, stateDir());
        try {
            assertEquals(
                    "{\"files\":[{\"path\":\"Other.java\"}]}",
                    request(other.adminUrl(), "GET", "/admin/files", null, null).body);
        } finally {
            other.stop();
        }
    }

    @Test
    public void nothingIsWrittenUnderTheProjectRootOutsideDotGit() throws IOException {
        aRealRepository();
        String cookie = approvedEditorOf("Plans.java");
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();
        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", cookie, edit("class Plans {}", version)).status);

        String[] names = folder.getRoot().list();
        Arrays.sort(names);
        assertEquals("[.git, README, TeamCode]", Arrays.toString(names));
        assertEquals(
                "[Plans.java]",
                Arrays.toString(
                        folder.getRoot().toPath().resolve("TeamCode").toFile().list()));
        assertTrue(worktreeOf("ada").startsWith(stateDir()));
    }

    @Test
    public void approvingALoginMakesAWorktreeAndASaveChangesItNotTheHostCheckout() throws IOException {
        aRealRepository();
        folder.newFolder("TeamCode");
        Files.write(
                root.resolve("TeamCode").resolve("Plans.java"), "class Plans {}\n".getBytes(StandardCharsets.UTF_8));
        committed("the file");
        assertEquals(200, admin("POST", "/admin/files/add?path=TeamCode/Plans.java").status);
        String cookie = approvedUser("ada");

        String logins = admin("GET", "/admin/users").body;
        assertTrue(logins, logins.contains("\"branch\":\"coding/ada\""));
        Path worktree = worktreeOf("ada");
        assertTrue(worktree.toString(), worktree.startsWith(stateDir()));
        assertEquals(
                "coding/ada",
                GitFixture.git(worktree, "rev-parse", "--abbrev-ref", "HEAD").trim());
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();
        assertEquals(
                200,
                user("PUT", "/files/TeamCode/Plans.java", cookie, edit("class Plans { int edited; }\n", version))
                        .status);
        assertEquals(
                "class Plans { int edited; }\n",
                new String(Files.readAllBytes(worktree.resolve("TeamCode/Plans.java")), StandardCharsets.UTF_8));
        assertEquals(
                "class Plans {}\n",
                new String(Files.readAllBytes(root.resolve("TeamCode/Plans.java")), StandardCharsets.UTF_8));
        assertEquals("the host checkout is clean", "", GitFixture.git(root, "status", "--porcelain"));
    }

    @Test
    public void twoUsersEditingTheSameFileDoNotConflict() throws IOException {
        String ada = approvedEditorOf("Plans.java");
        String bob = approvedUser("bob");
        String adaVersion = json(user("GET", "/files/TeamCode/Plans.java", ada).body)
                .get("version")
                .getAsString();
        String bobVersion = json(user("GET", "/files/TeamCode/Plans.java", bob).body)
                .get("version")
                .getAsString();

        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", ada, edit("ada's", adaVersion)).status);
        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", bob, edit("bob's", bobVersion)).status);

        assertEquals(
                "ada's",
                json(user("GET", "/files/TeamCode/Plans.java", ada).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                "bob's",
                json(user("GET", "/files/TeamCode/Plans.java", bob).body)
                        .get("content")
                        .getAsString());
        assertNotEquals(worktreeOf("ada"), worktreeOf("bob"));
    }

    @Test
    public void theSameUsernameApprovedAgainSeesTheEarlierSave() throws IOException {
        String first = approvedEditorOf("Plans.java");
        String version = json(user("GET", "/files/TeamCode/Plans.java", first).body)
                .get("version")
                .getAsString();
        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", first, edit("ada's", version)).status);

        String second = approvedUser("ada");

        assertNotEquals(first, second);
        assertEquals(
                "ada's",
                json(user("GET", "/files/TeamCode/Plans.java", second).body)
                        .get("content")
                        .getAsString());
        assertEquals(1, worktreeOf("ada").getParent().toFile().list().length);
    }

    @Test
    public void aPickedFileTheUsersBranchLacksIsA404NamingIt() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        folder.newFile("TeamCode/Later.java");
        committed("a file added after ada's branch began");
        assertEquals(200, admin("POST", "/admin/files/add?path=TeamCode/Later.java").status);

        Reply missing = user("GET", "/files/TeamCode/Later.java", cookie);

        assertEquals(404, missing.status);
        assertTrue(missing.body, missing.body.contains("TeamCode/Later.java"));
        assertTrue(user("GET", "/files", cookie).body.contains("TeamCode/Later.java"));
    }

    @Test
    public void theDashboardShowsTheBranch() throws IOException {
        String cookie = approvedUser("ada");

        String page = user("GET", "/", cookie).body;

        assertTrue(page, page.contains("me.branch"));
        assertTrue(page, page.contains("id=\"branch\""));
    }

    @Test
    public void theAdminPageShowsEachUsersWorktreeAndBranch() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("user.branch"));
        assertTrue(page, page.contains("user.worktree"));
    }

    @Test
    public void aRootThatIsNotARepositoryStopsTheServerFromStarting() throws IOException {
        Path plain = state.newFolder("plain").toPath();
        SimBench bench = bench();

        try {
            CodingServer.start(plain, worktree -> bench, InetAddress.getLoopbackAddress(), 0, 0, stateDir())
                    .stop();
            fail("no repository, no worktrees, no server");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(plain.toString()));
        }
    }

    @Test
    public void approvedWorktreesAndTheirUncommittedEditsSurviveARestart() throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();
        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", cookie, edit("ada's", version)).status);
        Path before = worktreeOf("ada");

        restart();

        assertEquals(before, worktreeOf("ada"));
        assertEquals(
                "ada's",
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
    }

    @Test
    public void whenGitRefusesApprovalIsA500AndTheSessionStaysPending() throws IOException {
        aRealRepository();
        String cookie = login("ada");
        GitFixture.git(root, "checkout", "-q", "-b", "main");
        GitFixture.git(root, "branch", "-D", "develop");

        Reply approved = admin("POST", "/admin/logins/" + idOf("ada") + "/approve");

        assertEquals(500, approved.status);
        assertTrue(approved.body, approved.body.contains("develop"));
        assertEquals("{\"state\":\"pending\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        assertEquals(403, user("GET", "/files", cookie).status);
    }

    @Test
    public void theStateDirectoryFollowsTheXdgBaseDirectoryConvention() {
        Path xdg = state.getRoot().toPath().resolve("xdg");
        Path home = state.getRoot().toPath().resolve("home");
        Path underHome = home.resolve(".local/state/midnight-snackers/coding-server");

        assertEquals(
                xdg.resolve("midnight-snackers/coding-server"),
                CodingServer.stateDir(Map.of("XDG_STATE_HOME", xdg.toString(), "HOME", home.toString())));
        assertEquals(underHome, CodingServer.stateDir(Map.of("HOME", home.toString())));
        assertEquals(
                underHome, CodingServer.stateDir(Map.of("XDG_STATE_HOME", "relative/state", "HOME", home.toString())));
        assertEquals(underHome, CodingServer.stateDir(Map.of("XDG_STATE_HOME", "", "HOME", home.toString())));
    }

    private Path sessionsFile() {
        return stateDir().resolve("sessions.json");
    }

    private static String secretOf(String cookie) {
        return cookie.substring(cookie.lastIndexOf('.') + 1);
    }

    private JsonObject storedSession(String username) throws IOException {
        for (var element : json(new String(Files.readAllBytes(sessionsFile()), StandardCharsets.UTF_8))
                .getAsJsonArray("sessions")) {
            if (element.getAsJsonObject().get("username").getAsString().equals(username)) {
                return element.getAsJsonObject();
            }
        }
        throw new AssertionError("no stored session for " + username);
    }

    private static String message(String text) {
        JsonObject body = new JsonObject();
        body.addProperty("message", text);
        return body.toString();
    }

    private String savedEditor(String content) throws IOException {
        String cookie = approvedEditorOf("Plans.java");
        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();
        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", cookie, edit(content, version)).status);
        return cookie;
    }

    @Test
    public void commitMakesOneCommitOnTheUserBranchAuthoredByTheUsername() throws IOException {
        aRealRepository();
        String cookie = savedEditor(FORMATTED);
        String develop = GitFixture.commitOf(root, "develop");

        Reply committed = user("POST", "/git/commit", cookie, message("my change"));

        assertEquals(committed.body, 200, committed.status);
        JsonObject body = json(committed.body);
        assertTrue(body.get("committed").getAsBoolean());
        assertEquals("[\"TeamCode/Plans.java\"]", body.getAsJsonArray("files").toString());
        Path worktree = worktreeOf("ada");
        assertEquals("ada|my change\n", GitFixture.git(worktree, "log", "-1", "--format=%an|%s"));
        assertEquals(FORMATTED, GitFixture.git(worktree, "show", "HEAD:TeamCode/Plans.java"));
        assertEquals("", GitFixture.git(worktree, "status", "--porcelain"));
        assertEquals(
                "1",
                GitFixture.git(root, "rev-list", "--count", "develop..coding/ada")
                        .trim());
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
    }

    @Test
    public void commitWithNothingChangedIsASuccessThatSaysSo() throws IOException {
        aRealRepository();
        String cookie = approvedEditorOf("Plans.java");

        Reply committed = user("POST", "/git/commit", cookie, message("nothing"));

        assertEquals(committed.body, 200, committed.status);
        assertFalse(json(committed.body).get("committed").getAsBoolean());
        assertEquals(
                "0",
                GitFixture.git(root, "rev-list", "--count", "develop..coding/ada")
                        .trim());
    }

    @Test
    public void commitWithoutAMessageIsRefused() throws IOException {
        aRealRepository();
        String cookie = savedEditor("edited");

        assertEquals(400, user("POST", "/git/commit", cookie, "{}").status);
        assertEquals(400, user("POST", "/git/commit", cookie, message("   ")).status);
        assertEquals(400, user("POST", "/git/commit", cookie, "not json").status);
        assertEquals(405, user("GET", "/git/commit", cookie).status);
        assertEquals(
                "0",
                GitFixture.git(root, "rev-list", "--count", "develop..coding/ada")
                        .trim());
    }

    @Test
    public void gitStatusListsTheChangedFilesAndTheCommitsAheadAndBehind() throws IOException {
        String cookie = savedEditor("edited");

        JsonObject before = json(user("GET", "/git/status", cookie).body);
        assertEquals(200, user("POST", "/git/commit", cookie, message("my change")).status);
        JsonObject after = json(user("GET", "/git/status", cookie).body);
        Files.write(root.resolve("README"), "on develop\n".getBytes(StandardCharsets.UTF_8));
        committed("a commit on develop");
        JsonObject later = json(user("GET", "/git/status", cookie).body);

        assertEquals(
                "[\"TeamCode/Plans.java\"]", before.getAsJsonArray("changed").toString());
        assertEquals(0, before.get("ahead").getAsInt());
        assertEquals("coding/ada", before.get("branch").getAsString());
        assertEquals("[]", after.getAsJsonArray("changed").toString());
        assertEquals(1, after.get("ahead").getAsInt());
        assertEquals(0, after.get("behind").getAsInt());
        assertEquals(1, later.get("behind").getAsInt());
    }

    @Test
    public void unapprovedSessionsGet403OnEveryGitRoute() throws IOException {
        String pending = login("bob");

        assertEquals(403, user("GET", "/git/status", pending).status);
        assertEquals(403, user("POST", "/git/commit", pending, message("x")).status);
        assertEquals(403, user("POST", "/git/pull", pending).status);
        assertEquals(403, user("POST", "/git/push", pending).status);
        assertEquals(403, user("GET", "/git/status", null).status);
    }

    @Test
    public void theEditTabHasTheGitLineWithACommitButtonThatAsksForAMessage() throws IOException {
        String page = user("GET", "/", approvedUser("ada")).body;

        assertTrue(page, page.contains("id=\"git\""));
        assertTrue(page, page.contains("id=\"commit\""));
        assertTrue(page, page.contains("id=\"changed\""));
        assertTrue(page, page.contains("'/git/status'"));
        assertTrue(page, page.contains("'/git/commit'"));
        assertTrue(page, page.contains("prompt("));
    }

    private static final String BADLY_INDENTED = "class Plans {\n  int edited;\n      void go( ) {int x=1;}\n}\n";

    private static final String FORMATTED =
            "class Plans {\n    int edited;\n\n    void go() {\n        int x = 1;\n    }\n}\n";

    private static String plans(String field) {
        return "class Plans {\n    int " + field + ";\n}\n";
    }

    private void save(String cookie, String name, String content) throws IOException {
        String version = json(user("GET", "/files/TeamCode/" + name, cookie).body)
                .get("version")
                .getAsString();
        Reply written = user("PUT", "/files/TeamCode/" + name, cookie, edit(content, version));
        assertEquals(written.body, 200, written.status);
    }

    private String contentOf(String cookie, String name) throws IOException {
        return json(user("GET", "/files/TeamCode/" + name, cookie).body)
                .get("content")
                .getAsString();
    }

    @Test
    public void commitFormatsTheJavaItCommitsAndNamesWhatItChanged() throws IOException {
        aRealRepository();
        String cookie = savedEditor(BADLY_INDENTED);

        Reply committed = user("POST", "/git/commit", cookie, message("my change"));

        assertEquals(committed.body, 200, committed.status);
        JsonObject body = json(committed.body);
        assertTrue(body.get("committed").getAsBoolean());
        assertEquals(
                "[\"TeamCode/Plans.java\"]", body.getAsJsonArray("formatted").toString());
        assertTrue(
                body.get("message").getAsString(),
                body.get("message").getAsString().contains("formatted"));
        Path worktree = worktreeOf("ada");
        assertEquals(FORMATTED, GitFixture.git(worktree, "show", "HEAD:TeamCode/Plans.java"));
        assertEquals("the formatted text is what is on disk too", FORMATTED, contentOf(cookie, "Plans.java"));
        assertEquals("nothing left uncommitted", "", GitFixture.git(worktree, "status", "--porcelain"));
    }

    @Test
    public void aSecondCommitOfFormattedWorkHasNothingToDo() throws IOException {
        String cookie = savedEditor(BADLY_INDENTED);
        assertEquals(200, user("POST", "/git/commit", cookie, message("my change")).status);

        Reply again = user("POST", "/git/commit", cookie, message("again"));

        assertEquals(again.body, 200, again.status);
        JsonObject body = json(again.body);
        assertFalse(body.get("committed").getAsBoolean());
        assertEquals("[]", body.getAsJsonArray("formatted").toString());
        assertEquals("nothing to commit", body.get("message").getAsString());
    }

    @Test
    public void aFileThatIsNotJavaAndJavaThatIsAlreadyFormattedAreCommittedByteForByte() throws IOException {
        aRealRepository();
        String notJava = "class  Notes {  this file is not Java  }\n";
        String cookie = approvedEditorOf("Plans.java", "notes.txt");
        save(cookie, "Plans.java", FORMATTED);
        save(cookie, "notes.txt", notJava);

        Reply committed = user("POST", "/git/commit", cookie, message("both"));

        assertEquals(committed.body, 200, committed.status);
        JsonObject body = json(committed.body);
        assertEquals(
                "[\"TeamCode/Plans.java\",\"TeamCode/notes.txt\"]",
                body.getAsJsonArray("files").toString());
        assertEquals("[]", body.getAsJsonArray("formatted").toString());
        assertEquals("committed 2 files", body.get("message").getAsString());
        Path worktree = worktreeOf("ada");
        assertEquals(FORMATTED, GitFixture.git(worktree, "show", "HEAD:TeamCode/Plans.java"));
        assertEquals(notJava, GitFixture.git(worktree, "show", "HEAD:TeamCode/notes.txt"));
    }

    @Test
    public void javaTheFormatterCannotParseIsCommittedAsWrittenAndNamedInAWarning() throws IOException {
        aRealRepository();
        String broken = "class Plans { this is not java\n";
        String cookie = savedEditor(broken);

        Reply committed = user("POST", "/git/commit", cookie, message("halfway through"));

        assertEquals(committed.body, 200, committed.status);
        JsonObject body = json(committed.body);
        assertTrue("the commit is still a save point", body.get("committed").getAsBoolean());
        assertEquals("[]", body.getAsJsonArray("formatted").toString());
        String warning = body.get("warning").getAsString();
        assertTrue(warning, warning.contains("TeamCode/Plans.java"));
        assertTrue(warning, warning.contains("1:16: error: illegal start of type"));
        assertEquals(broken, GitFixture.git(worktreeOf("ada"), "show", "HEAD:TeamCode/Plans.java"));
    }

    @Test
    public void aSaveIsNotFormatted() throws IOException {
        String cookie = savedEditor(BADLY_INDENTED);

        assertEquals(BADLY_INDENTED, contentOf(cookie, "Plans.java"));
        assertEquals(BADLY_INDENTED, new String(Files.readAllBytes(file("Plans.java")), StandardCharsets.UTF_8));
        assertEquals(
                "[\"TeamCode/Plans.java\"]",
                json(user("GET", "/git/status", cookie).body)
                        .getAsJsonArray("changed")
                        .toString());
    }

    @Test
    public void theEditTabReloadsWhatIsOpenAfterACommitSoTheFormattedTextIsShown() throws IOException {
        String page = user("GET", "/", approvedUser("ada")).body;

        assertTrue(page, page.contains("function reloadOpen("));
        String commit = functionBody(page, "commit");
        assertTrue(commit, commit.contains("'/git/commit'"));
        assertTrue(commit, commit.contains("result.formatted"));
        assertTrue(commit, commit.contains("reloadOpen("));
    }

    private static String functionBody(String page, String name) {
        int at = page.indexOf("function " + name + "(");
        assertTrue("no function " + name + " in the page", at >= 0);
        int open = page.indexOf('{', at);
        int depth = 0;
        for (int i = open; i < page.length(); i++) {
            if (page.charAt(i) == '{') {
                depth++;
            } else if (page.charAt(i) == '}' && --depth == 0) {
                return page.substring(open, i + 1);
            }
        }
        fail("function " + name + " in the page is never closed");
        return null;
    }

    private void commitOnDevelop(String file, String content) throws IOException {
        Files.write(root.resolve(file), content.getBytes(StandardCharsets.UTF_8));
        committed("a commit on develop: " + file);
    }

    @Test
    public void pullBringsDevelopIntoTheWorktreeAndTheOpenFile() throws IOException {
        aRealRepository();
        String cookie = approvedEditorOf("Plans.java");
        commitOnDevelop("TeamCode/Plans.java", plans("fromDevelop"));
        assertEquals(
                1, json(user("GET", "/git/status", cookie).body).get("behind").getAsInt());

        Reply pulled = user("POST", "/git/pull", cookie);

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals("pulled", json(pulled.body).get("outcome").getAsString());
        assertEquals(
                plans("fromDevelop"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                0, json(user("GET", "/git/status", cookie).body).get("behind").getAsInt());
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(root, "coding/ada"));
    }

    @Test
    public void pullMergesWhenTheUserHasCommitsOfTheirOwn() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("mine")).status);
        commitOnDevelop("README", "on develop\n");
        String develop = GitFixture.commitOf(root, "develop");

        Reply pulled = user("POST", "/git/pull", cookie);

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals(
                "on develop\n",
                new String(Files.readAllBytes(worktreeOf("ada").resolve("README")), StandardCharsets.UTF_8));
        assertEquals(
                plans("mine"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                2, json(user("GET", "/git/status", cookie).body).get("ahead").getAsInt());
        assertEquals("develop did not move", develop, GitFixture.commitOf(root, "develop"));
    }

    @Test
    public void pullKeepsAnUncommittedEditThatDevelopDidNotTouch() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        commitOnDevelop("README", "on develop\n");

        Reply pulled = user("POST", "/git/pull", cookie);

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals("pulled", json(pulled.body).get("outcome").getAsString());
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(root, "coding/ada"));
        assertEquals(
                "on develop\n",
                new String(Files.readAllBytes(worktreeOf("ada").resolve("README")), StandardCharsets.UTF_8));
        assertEquals(
                plans("mine"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                "[\"TeamCode/Plans.java\"]",
                json(user("GET", "/git/status", cookie).body)
                        .getAsJsonArray("changed")
                        .toString());

        Reply committed = user("POST", "/git/commit", cookie, message("mine"));

        assertEquals(committed.body, 200, committed.status);
        assertEquals(
                "[\"TeamCode/Plans.java\"]",
                json(committed.body).getAsJsonArray("files").toString());
        assertEquals(
                1, json(user("GET", "/git/status", cookie).body).get("ahead").getAsInt());
    }

    @Test
    public void pullWithAnUncommittedEditInAFileDevelopChangedIsRefusedNamingIt() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        commitOnDevelop("TeamCode/Plans.java", plans("fromDevelop"));
        commitOnDevelop("README", "on develop\n");
        String head = GitFixture.commitOf(root, "coding/ada");

        Reply refused = user("POST", "/git/pull", cookie);

        assertEquals(409, refused.status);
        assertEquals("uncommitted", json(refused.body).get("outcome").getAsString());
        assertEquals(
                "[\"TeamCode/Plans.java\"]",
                json(refused.body).getAsJsonArray("files").toString());
        assertTrue(refused.body, json(refused.body).get("message").getAsString().contains("commit first"));
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(
                plans("mine"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                "nothing of develop's arrived",
                "hello\n",
                new String(Files.readAllBytes(worktreeOf("ada").resolve("README")), StandardCharsets.UTF_8));
    }

    @Test
    public void pullWithNothingNewIsASuccessThatSaysSo() throws IOException {
        aRealRepository();
        String cookie = approvedEditorOf("Plans.java");
        String head = GitFixture.commitOf(root, "coding/ada");

        Reply pulled = user("POST", "/git/pull", cookie);

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals("nothing", json(pulled.body).get("outcome").getAsString());
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(405, user("GET", "/git/pull", cookie).status);
    }

    @Test
    public void aPullThatConflictsChangesNothingAndTellsTheUserToAskTheirCoach() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("ada"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("ada's")).status);
        commitOnDevelop("TeamCode/Plans.java", plans("develop"));
        String head = GitFixture.commitOf(root, "coding/ada");
        String develop = GitFixture.commitOf(root, "develop");

        Reply conflicted = user("POST", "/git/pull", cookie);

        assertEquals(409, conflicted.status);
        JsonObject body = json(conflicted.body);
        assertEquals("conflicts", body.get("outcome").getAsString());
        assertEquals("[\"TeamCode/Plans.java\"]", body.getAsJsonArray("files").toString());
        assertTrue(body.toString(), body.get("message").getAsString().contains("coach"));
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals(
                plans("ada"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals("", GitFixture.git(worktreeOf("ada"), "status", "--porcelain"));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
        String logins = admin("GET", "/admin/users").body;
        assertTrue(
                logins,
                logins.contains(
                        "\"lastMerge\":{\"op\":\"pull\",\"outcome\":\"conflicts\",\"files\":[\"TeamCode/Plans.java\"]"));
        assertEquals("a conflict needs a coach", "bad", body.get("severity").getAsString());
        assertTrue("the recipe names the real worktree", logins.contains("git merge develop"));
        assertTrue(logins, logins.contains(worktreeOf("ada").toString().replace("\\", "\\\\")));
        String page = admin("GET", "/admin").body;
        assertTrue(page, page.contains("user.lastMerge"));
        assertTrue(page, page.contains("last.recipe"));
    }

    @Test
    public void theEditTabHasAPullButtonThatAnimatesWhileThereIsSomethingToPull() throws IOException {
        String page = user("GET", "/", approvedUser("ada")).body;

        assertTrue(page, page.contains("id=\"pull\""));
        assertTrue(page, page.contains("'/git/pull'"));
        assertTrue(page, page.contains("status.behind"));
        assertTrue(page, page.contains("@keyframes"));
        assertTrue(page, page.contains("prefers-reduced-motion"));
        assertTrue("the reply's message, coach and all, is what the page shows", page.contains("say(result.message"));
    }

    private JsonObject userOf(String username) throws IOException {
        JsonObject found = null;
        for (var element : json(admin("GET", "/admin/users").body).getAsJsonArray("users")) {
            JsonObject user = element.getAsJsonObject();
            if (user.get("username").getAsString().equals(username)) {
                found = user;
            }
        }
        assertNotNull("no user for " + username, found);
        return found;
    }

    private JsonObject newestSessionOf(String username) throws IOException {
        JsonArray sessions = userOf(username).getAsJsonArray("sessions");
        assertTrue(username + " has no sessions", sessions.size() > 0);
        return sessions.get(sessions.size() - 1).getAsJsonObject();
    }

    @Test
    public void theAdminListingShowsEachUsersChangedFilesAndCommitsAheadAndBehind() throws IOException {
        String cookie = savedEditor("edited");

        JsonObject before = userOf("ada").getAsJsonObject("status");
        assertEquals(200, user("POST", "/git/commit", cookie, message("my change")).status);
        JsonObject after = userOf("ada").getAsJsonObject("status");
        commitOnDevelop("README", "on develop\n");
        JsonObject later = userOf("ada").getAsJsonObject("status");

        assertEquals(
                "[\"TeamCode/Plans.java\"]", before.getAsJsonArray("changed").toString());
        assertEquals(0, before.get("ahead").getAsInt());
        assertEquals(0, before.get("behind").getAsInt());
        assertEquals("[]", after.getAsJsonArray("changed").toString());
        assertEquals(1, after.get("ahead").getAsInt());
        assertEquals(1, later.get("behind").getAsInt());
        assertEquals(1, later.get("ahead").getAsInt());
    }

    @Test
    public void aUserWithoutAWorktreeHasNoStatusInTheAdminListing() throws IOException {
        login("bob");

        assertTrue(userOf("bob").get("status").isJsonNull());
        assertTrue(userOf("bob").get("worktree").isJsonNull());
    }

    @Test
    public void theAdminPageShowsEachUsersStatusAndHasAPullButton() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("user.status"));
        assertTrue(page, page.contains("status.changed"));
        assertTrue(page, page.contains("status.ahead"));
        assertTrue(page, page.contains("status.behind"));
        assertTrue(page, page.contains("'/pull'"));
        assertTrue("the reply's message is what the page shows", page.contains("result.message"));
    }

    @Test
    public void theAdminCanPullDevelopIntoAUsersWorktree() throws IOException {
        aRealRepository();
        String cookie = approvedEditorOf("Plans.java");
        commitOnDevelop("TeamCode/Plans.java", plans("fromDevelop"));
        String develop = GitFixture.commitOf(root, "develop");
        assertEquals(1, userOf("ada").getAsJsonObject("status").get("behind").getAsInt());

        Reply pulled = admin("POST", "/admin/logins/" + idOf("ada") + "/pull");

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals("pulled", json(pulled.body).get("outcome").getAsString());
        assertTrue(pulled.body, json(pulled.body).get("message").getAsString().contains("ada"));
        assertEquals(
                plans("fromDevelop"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(0, userOf("ada").getAsJsonObject("status").get("behind").getAsInt());
        assertEquals(develop, GitFixture.commitOf(root, "coding/ada"));
        assertEquals("develop did not move", develop, GitFixture.commitOf(root, "develop"));
        assertEquals(
                "pull", userOf("ada").getAsJsonObject("lastMerge").get("op").getAsString());
        assertEquals(
                "pulled",
                userOf("ada").getAsJsonObject("lastMerge").get("outcome").getAsString());
    }

    @Test
    public void anAdminPullMovesTheTipTheUsersStatusReportsSoTheirEditorReloadsWhatIsOpen() throws IOException {
        aRealRepository();
        String cookie = approvedEditorOf("Plans.java");
        String before =
                json(user("GET", "/git/status", cookie).body).get("head").getAsString();
        assertEquals(GitFixture.commitOf(root, "coding/ada"), before);
        commitOnDevelop("TeamCode/Plans.java", plans("fromDevelop"));
        assertEquals(
                "a commit on develop alone moves nothing of the user's",
                before,
                json(user("GET", "/git/status", cookie).body).get("head").getAsString());

        assertEquals(200, admin("POST", "/admin/logins/" + idOf("ada") + "/pull").status);

        assertEquals(
                GitFixture.commitOf(root, "develop"),
                json(user("GET", "/git/status", cookie).body).get("head").getAsString());
        String dashboard = user("GET", "/", cookie).body;
        assertTrue(dashboard, dashboard.contains("status.head"));
        assertTrue(
                "a tip that moved under a clean editor reloads the open file", dashboard.contains("load(open.path)"));
    }

    @Test
    public void theAdminListingStillAnswersWhenOneWorktreesStatusCannotBeRead() throws IOException {
        aRealRepository();
        approvedEditorOf("Plans.java");
        approvedUser("bob");

        GitFixture.git(root, "update-ref", "-d", "refs/heads/coding/ada");

        Reply listing = admin("GET", "/admin/users");

        assertEquals(listing.body, 200, listing.status);
        assertTrue(userOf("ada").get("status").isJsonNull());
        assertTrue(
                userOf("ada").toString(),
                userOf("ada").get("statusError").getAsString().contains("coding/ada"));
        assertEquals(0, userOf("bob").getAsJsonObject("status").get("behind").getAsInt());
        assertTrue(userOf("bob").get("statusError").isJsonNull());
        String page = admin("GET", "/admin").body;
        assertTrue(page, page.contains("user.statusError"));
    }

    @Test
    public void anAdminPullWithNothingNewIsASuccessThatSaysSo() throws IOException {
        aRealRepository();
        approvedEditorOf("Plans.java");
        String head = GitFixture.commitOf(root, "coding/ada");

        Reply pulled = admin("POST", "/admin/logins/" + idOf("ada") + "/pull");

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals("nothing", json(pulled.body).get("outcome").getAsString());
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(405, admin("GET", "/admin/logins/" + idOf("ada") + "/pull").status);
    }

    @Test
    public void anAdminPullWithAnUncommittedEditInAFileDevelopChangedIsRefusedNamingItAndTheUser() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        commitOnDevelop("TeamCode/Plans.java", plans("fromDevelop"));
        commitOnDevelop("README", "on develop\n");
        String head = GitFixture.commitOf(root, "coding/ada");

        Reply refused = admin("POST", "/admin/logins/" + idOf("ada") + "/pull");

        assertEquals(409, refused.status);
        assertEquals("uncommitted", json(refused.body).get("outcome").getAsString());
        assertEquals(
                "[\"TeamCode/Plans.java\"]",
                json(refused.body).getAsJsonArray("files").toString());
        assertTrue(refused.body, json(refused.body).get("message").getAsString().contains("ada"));
        assertTrue(refused.body, json(refused.body).get("message").getAsString().contains("commit first"));
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(
                plans("mine"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                "nothing of develop's arrived",
                "hello\n",
                new String(Files.readAllBytes(worktreeOf("ada").resolve("README")), StandardCharsets.UTF_8));
    }

    @Test
    public void anAdminPullKeepsAnUncommittedEditThatDevelopDidNotTouch() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        commitOnDevelop("README", "on develop\n");

        Reply pulled = admin("POST", "/admin/logins/" + idOf("ada") + "/pull");

        assertEquals(pulled.body, 200, pulled.status);
        assertEquals("pulled", json(pulled.body).get("outcome").getAsString());
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(root, "coding/ada"));
        assertEquals(
                "on develop\n",
                new String(Files.readAllBytes(worktreeOf("ada").resolve("README")), StandardCharsets.UTF_8));
        assertEquals(
                plans("mine"),
                json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                        .get("content")
                        .getAsString());
        assertEquals(
                "[\"TeamCode/Plans.java\"]",
                userOf("ada")
                        .getAsJsonObject("status")
                        .getAsJsonArray("changed")
                        .toString());
    }

    @Test
    public void anAdminPullThatConflictsChangesNothingAndShowsTheCoachTheRecipe() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("ada"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("ada's")).status);
        commitOnDevelop("TeamCode/Plans.java", plans("develop"));
        String head = GitFixture.commitOf(root, "coding/ada");
        String develop = GitFixture.commitOf(root, "develop");

        Reply conflicted = admin("POST", "/admin/logins/" + idOf("ada") + "/pull");

        assertEquals(409, conflicted.status);
        JsonObject body = json(conflicted.body);
        assertEquals("conflicts", body.get("outcome").getAsString());
        assertEquals("[\"TeamCode/Plans.java\"]", body.getAsJsonArray("files").toString());
        assertFalse(
                "the coach is not told to ask the coach: " + body,
                body.get("message").getAsString().contains("coach"));
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals("", GitFixture.git(worktreeOf("ada"), "status", "--porcelain"));
        String logins = admin("GET", "/admin/users").body;
        assertTrue(
                logins,
                logins.contains(
                        "\"lastMerge\":{\"op\":\"pull\",\"outcome\":\"conflicts\",\"files\":[\"TeamCode/Plans.java\"]"));
    }

    @Test
    public void anAdminPullForALoginWithoutAWorktreeIs404() throws IOException {
        login("bob");

        Reply refused = admin("POST", "/admin/logins/" + idOf("bob") + "/pull");

        assertEquals(404, refused.status);
        assertTrue(refused.body, refused.body.contains("bob"));
        assertEquals(404, admin("POST", "/admin/logins/999/pull").status);
    }

    @Test
    public void theAdminPullRouteDoesNotExistOnTheUserPort() throws IOException {
        String cookie = approvedEditorOf("Plans.java");

        assertEquals(404, user("POST", "/admin/logins/" + idOf("ada") + "/pull", cookie).status);
    }

    @Test
    public void pushLandsTheUsersCommitsOnDevelopAndTheHostCheckoutShowsThem() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("mine")).status);
        String oldDevelop = GitFixture.commitOf(root, "develop");

        Reply pushed = user("POST", "/git/push", cookie);

        assertEquals(pushed.body, 200, pushed.status);
        assertEquals("pushed", json(pushed.body).get("outcome").getAsString());
        assertEquals(
                plans("mine"),
                new String(Files.readAllBytes(root.resolve("TeamCode/Plans.java")), StandardCharsets.UTF_8));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
        assertNotEquals(oldDevelop, GitFixture.commitOf(root, "develop"));
        assertEquals("ada", GitFixture.git(root, "log", "-1", "--format=%an").trim());
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(root, "coding/ada"));
        JsonObject status = json(user("GET", "/git/status", cookie).body);
        assertEquals(0, status.get("ahead").getAsInt());
        assertEquals(0, status.get("behind").getAsInt());
        assertEquals(405, user("GET", "/git/push", cookie).status);
    }

    @Test
    public void pushWithUncommittedChangesIsRefusedAndWithNothingNewSaysSo() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("mine"));
        String develop = GitFixture.commitOf(root, "develop");

        Reply refused = user("POST", "/git/push", cookie);
        assertEquals(409, refused.status);
        assertEquals("uncommitted", json(refused.body).get("outcome").getAsString());
        assertEquals(develop, GitFixture.commitOf(root, "develop"));

        assertEquals(200, user("POST", "/git/commit", cookie, message("mine")).status);
        assertEquals(200, user("POST", "/git/push", cookie).status);
        Reply again = user("POST", "/git/push", cookie);
        assertEquals(again.body, 200, again.status);
        assertEquals("nothing", json(again.body).get("outcome").getAsString());
    }

    @Test
    public void aPushThatConflictsChangesNothingAndTellsTheUserToAskTheirCoach() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("ada"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("ada's")).status);
        commitOnDevelop("TeamCode/Plans.java", plans("develop"));
        String develop = GitFixture.commitOf(root, "develop");
        String head = GitFixture.commitOf(root, "coding/ada");

        Reply conflicted = user("POST", "/git/push", cookie);

        assertEquals(409, conflicted.status);
        JsonObject body = json(conflicted.body);
        assertEquals("conflicts", body.get("outcome").getAsString());
        assertEquals("[\"TeamCode/Plans.java\"]", body.getAsJsonArray("files").toString());
        assertTrue(body.toString(), body.get("message").getAsString().contains("coach"));
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals(head, GitFixture.commitOf(root, "coding/ada"));
        assertEquals(
                plans("develop"),
                new String(Files.readAllBytes(root.resolve("TeamCode/Plans.java")), StandardCharsets.UTF_8));
        assertEquals("", GitFixture.git(root, "status", "--porcelain"));
        assertEquals("", GitFixture.git(worktreeOf("ada"), "status", "--porcelain"));
        String logins = admin("GET", "/admin/users").body;
        assertTrue(
                logins,
                logins.contains(
                        "\"lastMerge\":{\"op\":\"push\",\"outcome\":\"conflicts\",\"files\":[\"TeamCode/Plans.java\"]"));
    }

    @Test
    public void aPushIsRefusedWhenTheHostsUncommittedEditWouldBeOverwrittenAndTheEditIsIntact() throws IOException {
        aRealRepository();
        String cookie = savedEditor(plans("ada"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("ada's")).status);
        Files.write(root.resolve("TeamCode/Plans.java"), "the coach's unsaved work\n".getBytes(StandardCharsets.UTF_8));
        String develop = GitFixture.commitOf(root, "develop");

        Reply refused = user("POST", "/git/push", cookie);

        assertEquals(409, refused.status);
        JsonObject body = json(refused.body);
        assertEquals("refused", body.get("outcome").getAsString());
        assertTrue(body.toString(), body.get("message").getAsString().contains("coach"));
        assertEquals(develop, GitFixture.commitOf(root, "develop"));
        assertEquals(
                "the coach's unsaved work\n",
                new String(Files.readAllBytes(root.resolve("TeamCode/Plans.java")), StandardCharsets.UTF_8));
        assertFalse(Files.exists(root.resolve(".git/MERGE_HEAD")));
        String logins = admin("GET", "/admin/users").body;
        assertTrue(logins, logins.contains("\"op\":\"push\",\"outcome\":\"refused\""));
        assertTrue(logins, logins.contains("Plans.java"));
    }

    private boolean pushable(String cookie) throws IOException {
        return json(user("GET", "/git/status", cookie).body).get("pushable").getAsBoolean();
    }

    @Test
    public void gitStatusSaysWhetherAPushWouldLandAndThePushRouteAgrees() throws IOException {
        String cookie = savedEditor(plans("mine"));

        assertFalse("an uncommitted edit is not something a push can take", pushable(cookie));
        assertEquals(409, user("POST", "/git/push", cookie).status);

        assertEquals(200, user("POST", "/git/commit", cookie, message("mine")).status);
        assertTrue("a commit develop lacks, and nothing in the way", pushable(cookie));

        String version = json(user("GET", "/files/TeamCode/Plans.java", cookie).body)
                .get("version")
                .getAsString();
        assertEquals(200, user("PUT", "/files/TeamCode/Plans.java", cookie, edit(plans("typing"), version)).status);

        assertFalse("typing after a commit puts the edit back in the way", pushable(cookie));
        assertEquals(
                1, json(user("GET", "/git/status", cookie).body).get("ahead").getAsInt());
        assertEquals("and a push would still only be refused", 409, user("POST", "/git/push", cookie).status);

        assertEquals(200, user("POST", "/git/commit", cookie, message("typing")).status);
        assertEquals(200, user("POST", "/git/push", cookie).status);

        assertFalse("everything is on develop now", pushable(cookie));
    }

    @Test
    public void theEditTabHasAPushButton() throws IOException {
        String page = user("GET", "/", approvedUser("ada")).body;

        assertTrue(page, page.contains("id=\"push\""));
        assertTrue(page, page.contains("'/git/push'"));
        assertTrue(
                "whether a push would land is the server's judgement, and the page wears it",
                page.contains("status.pushable"));
    }

    @Test
    public void pushLandsOnOriginTooAndTheReplySaysSo() throws IOException {
        aRealRepository();
        Path origin = state.getRoot().toPath().resolve("origin.git");
        GitFixture.withOrigin(root, origin);
        String cookie = savedEditor(plans("mine"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("mine")).status);

        Reply pushed = user("POST", "/git/push", cookie);

        assertEquals(pushed.body, 200, pushed.status);
        JsonObject body = json(pushed.body);
        assertEquals("pushed", body.get("outcome").getAsString());
        assertEquals("origin", body.getAsJsonObject("remote").get("name").getAsString());
        assertEquals("pushed", body.getAsJsonObject("remote").get("outcome").getAsString());
        assertTrue(body.toString(), body.get("message").getAsString().contains("origin"));
        assertEquals(GitFixture.commitOf(root, "develop"), GitFixture.commitOf(origin, "develop"));
    }

    @Test
    public void whenOriginIsUnreachableThePushStillLandsAndTheAdminSeesTheProblem() throws IOException {
        aRealRepository();
        GitFixture.git(
                root,
                "remote",
                "add",
                "origin",
                state.getRoot().toPath().resolve("no-such-origin.git").toString());
        String cookie = savedEditor(plans("mine"));
        assertEquals(200, user("POST", "/git/commit", cookie, message("mine")).status);
        String oldDevelop = GitFixture.commitOf(root, "develop");

        Reply pushed = user("POST", "/git/push", cookie);

        assertEquals(pushed.body, 200, pushed.status);
        JsonObject body = json(pushed.body);
        assertEquals("pushed", body.get("outcome").getAsString());
        assertEquals("failed", body.getAsJsonObject("remote").get("outcome").getAsString());
        assertTrue(body.toString(), body.get("message").getAsString().contains("could not push to origin"));
        assertNotEquals(oldDevelop, GitFixture.commitOf(root, "develop"));
        String logins = admin("GET", "/admin/users").body;
        assertTrue(logins, logins.contains("\"remote\":{\"name\":\"origin\",\"outcome\":\"failed\""));
        assertEquals(
                "a push that landed but did not reach origin is a warning",
                "warn",
                body.get("severity").getAsString());
        assertTrue(
                "the commands are git's, so the server writes them rather than the page",
                logins.contains("git push origin develop"));
        String page = admin("GET", "/admin").body;
        assertTrue(page, page.contains("last.recipe"));
        String dashboard = user("GET", "/", cookie).body;
        assertTrue("the page wears the severity it is given", dashboard.contains("result.severity"));
        assertTrue("and so does the admin's", page.contains("result.severity"));
        assertTrue("including the recipe it prints", page.contains("last.severity"));
        assertFalse(
                "neither page works severity out for itself, which is why MergeReport says it",
                page.contains("result.outcome === 'nothing'"));
    }

    private static final String SRC = "TeamCode/src/main/java/org/example/";

    private String navigatingUser() throws IOException {
        Path src = root.resolve(SRC);
        Files.createDirectories(src);
        Files.write(src.resolve("Plans.java"), SourceNavigatorTest.PLANS_SOURCE.getBytes(StandardCharsets.UTF_8));
        Files.write(src.resolve("Auto.java"), SourceNavigatorTest.AUTO_SOURCE.getBytes(StandardCharsets.UTF_8));
        SimBenchTest.simulatorInto(root);
        committed("two classes");
        serverWith(sourcesBench());
        assertEquals(200, admin("POST", "/admin/files/add?path=" + SRC + "Auto.java").status);
        return approvedUser("ada");
    }

    private static String nav(String route, String file, String source, String lineText, String token) {
        int[] where = SourceNavigatorTest.at(source, lineText, token);
        return "/nav/" + route + "?file=" + file + "&line=" + where[0] + "&column=" + where[1];
    }

    @Test
    public void aJumpToADefinitionLandsInAFileTheUserMayReadButNotEdit() throws IOException {
        // the simulator's resources are not all text: a real git carries the bytes
        aRealRepository();
        String cookie = navigatingUser();

        Reply definition = user(
                "GET",
                nav(
                        "definition",
                        SRC + "Auto.java",
                        SourceNavigatorTest.AUTO_SOURCE,
                        "int total = Plans.count();",
                        "count"),
                cookie);

        assertEquals(definition.body, 200, definition.status);
        JsonObject body = json(definition.body);
        assertEquals("org.example.Plans.count()", body.get("symbol").getAsString());
        assertEquals("method", body.get("kind").getAsString());
        assertEquals(SRC + "Plans.java", body.get("file").getAsString());
        int[] where = SourceNavigatorTest.at(SourceNavigatorTest.PLANS_SOURCE, "public static int count() {", "count");
        assertEquals(where[0], body.get("line").getAsInt());
        assertEquals(where[1], body.get("column").getAsInt());
        assertEquals("public static int count() {", body.get("text").getAsString());

        Reply source = user("GET", "/source/" + SRC + "Plans.java", cookie);
        assertEquals(source.body, 200, source.status);
        JsonObject plans = json(source.body);
        assertEquals(SourceNavigatorTest.PLANS_SOURCE, plans.get("content").getAsString());
        assertFalse(plans.get("editable").getAsBoolean());
        assertEquals(
                sha256(SourceNavigatorTest.PLANS_SOURCE.getBytes(StandardCharsets.UTF_8)),
                plans.get("version").getAsString());
        assertTrue(json(user("GET", "/source/" + SRC + "Auto.java", cookie).body)
                .get("editable")
                .getAsBoolean());
        assertEquals(
                "editing is still only the editable set",
                404,
                user("GET", "/files/" + SRC + "Plans.java", cookie).status);
        assertEquals(404, user("GET", "/source/README", cookie).status);
        assertEquals(
                404, user("GET", "/source/TeamCode/src/main/java/org/example/../example/Plans.java", cookie).status);
        assertEquals(
                405,
                user(
                                "PUT",
                                "/source/" + SRC + "Plans.java",
                                cookie,
                                edit("x", plans.get("version").getAsString()))
                        .status);
        String pending = login("bob");
        assertEquals(403, user("GET", "/source/" + SRC + "Plans.java", pending).status);
        assertEquals(
                403,
                user(
                                "GET",
                                nav(
                                        "definition",
                                        SRC + "Auto.java",
                                        SourceNavigatorTest.AUTO_SOURCE,
                                        "int total = Plans.count();",
                                        "count"),
                                pending)
                        .status);
        assertEquals(403, user("GET", "/nav/usages?file=x&line=1&column=1", pending).status);
    }

    @Test
    public void aSymbolFromOutsideTheSourcesHasNoFileAndNothingUnderTheCursorIsA404() throws IOException {
        // the simulator's resources are not all text: a real git carries the bytes
        aRealRepository();
        String cookie = navigatingUser();

        Reply list = user(
                "GET",
                nav("definition", SRC + "Auto.java", SourceNavigatorTest.AUTO_SOURCE, "List<String> names", "List"),
                cookie);
        assertEquals(list.body, 200, list.status);
        assertEquals("java.util.List", json(list.body).get("symbol").getAsString());
        assertTrue(list.body, json(list.body).get("file").isJsonNull());

        assertEquals(404, user("GET", "/nav/definition?file=" + SRC + "Auto.java&line=2&column=1", cookie).status);
        assertEquals(404, user("GET", "/nav/definition?file=README&line=1&column=1", cookie).status);
        assertEquals(400, user("GET", "/nav/definition?file=" + SRC + "Auto.java&line=x&column=1", cookie).status);
    }

    @Test
    public void usagesComeFromTheUsersOwnWorktree() throws IOException {
        // the simulator's resources are not all text: a real git carries the bytes
        aRealRepository();
        String ada = navigatingUser();
        String bob = approvedUser("bob");
        String version = json(user("GET", "/files/" + SRC + "Auto.java", ada).body)
                .get("version")
                .getAsString();
        String edited = SourceNavigatorTest.AUTO_SOURCE.replace(
                "return total + names.size();", "return total + names.size() + Plans.count();");
        assertEquals(200, user("PUT", "/files/" + SRC + "Auto.java", ada, edit(edited, version)).status);

        Reply adas = user(
                "GET",
                nav(
                        "usages",
                        SRC + "Plans.java",
                        SourceNavigatorTest.PLANS_SOURCE,
                        "public static int count() {",
                        "count"),
                ada);
        Reply bobs = user(
                "GET",
                nav(
                        "usages",
                        SRC + "Plans.java",
                        SourceNavigatorTest.PLANS_SOURCE,
                        "public static int count() {",
                        "count"),
                bob);

        assertEquals(adas.body, 200, adas.status);
        JsonObject body = json(adas.body);
        assertEquals("org.example.Plans.count()", body.get("symbol").getAsString());
        assertEquals(
                SRC + "Plans.java",
                body.getAsJsonObject("definition").get("file").getAsString());
        assertEquals(2, body.getAsJsonArray("usages").size());
        JsonObject second = body.getAsJsonArray("usages").get(1).getAsJsonObject();
        assertEquals(SRC + "Auto.java", second.get("file").getAsString());
        int[] where = SourceNavigatorTest.at(edited, "return total + names.size() + Plans.count();", "count");
        assertEquals(where[0], second.get("line").getAsInt());
        assertEquals(where[1], second.get("column").getAsInt());
        assertEquals(
                "return total + names.size() + Plans.count();",
                second.get("text").getAsString());
        assertEquals(1, json(bobs.body).getAsJsonArray("usages").size());
    }

    @Test
    public void navigationOnAServerWithoutSourcesSaysSo() throws IOException {
        String cookie = approvedUser("ada");

        assertEquals("{\"available\":false}", user("GET", "/nav/definition?file=x&line=1&column=1", cookie).body);
        assertEquals("{\"available\":false}", user("GET", "/nav/usages?file=x&line=1&column=1", cookie).body);
        assertEquals(404, user("GET", "/source/x", cookie).status);
    }

    @Test
    public void theEditTabJumpsToDefinitionsAndListsUsages() throws IOException {
        String page = user("GET", "/", approvedUser("ada")).body;

        assertTrue(page, page.contains("'/nav/definition"));
        assertTrue(page, page.contains("'/nav/usages"));
        assertTrue(page, page.contains("'F12'"));
        assertTrue(page, page.contains("'Shift-F12'"));
        assertTrue(page, page.contains("posAtCoords"));
        assertTrue(page, page.contains("id=\"usages\""));
        assertTrue(page, page.contains("'/source/'"));
        assertTrue(page, page.contains("view only"));
    }

    private boolean listed(String username) throws IOException {
        return admin("GET", "/admin/users").body.contains("\"username\":\"" + username + "\"");
    }

    @Test
    public void deletingAUserEndsEveryOneOfTheirLoginsAndTakesTheirWorktree() throws IOException {
        aRealRepository();
        String cookie = approvedUser("ada");
        String second = login("ada");
        approvedUser("bob");
        Path worktree = worktreeOf("ada");
        Path bobs = worktreeOf("bob");
        assertEquals(200, user("GET", "/files", cookie).status);

        Reply deleted = admin("POST", "/admin/users/delete?username=ada");

        assertEquals(deleted.body, 200, deleted.status);
        assertEquals("{\"deleted\":true,\"username\":\"ada\",\"branch\":\"coding/ada\",\"logins\":2}", deleted.body);
        assertFalse("ada has left the listing", listed("ada"));
        assertTrue("and nobody else has", listed("bob"));
        assertEquals(403, user("GET", "/files", cookie).status);
        assertEquals("{\"state\":\"none\"}", user("GET", "/me", cookie).body);
        assertEquals("both her logins went", "{\"state\":\"none\"}", user("GET", "/me", second).body);
        assertFalse(Files.exists(worktree));
        assertEquals("her branch stayed", GitFixture.head(root), GitFixture.commitOf(root, "coding/ada"));
        assertTrue("bob's worktree is where it was", Files.isDirectory(bobs));
    }

    @Test
    public void deletingAUserWithWorkDevelopDoesNotHaveIsRefusedAndChangesNothing() throws IOException {
        String cookie = savedEditor("edited");
        Path worktree = worktreeOf("ada");

        Reply refused = admin("POST", "/admin/users/delete?username=ada");

        assertEquals(refused.body, 409, refused.status);
        JsonObject body = json(refused.body);
        assertEquals(
                "ada has 1 changed file that develop does not have; push it first, or delete anyway",
                body.get("message").getAsString());
        assertEquals("[\"TeamCode/Plans.java\"]", body.getAsJsonArray("changed").toString());
        assertEquals(0, body.get("ahead").getAsInt());
        assertTrue("her worktree is untouched", Files.isDirectory(worktree));
        assertEquals("and her login still works", 200, user("GET", "/files", cookie).status);
        assertTrue(listed("ada"));
    }

    @Test
    public void aRefusalCountsTheCommitsDevelopLacksAsWellAsTheChangedFiles() throws IOException {
        String cookie = savedEditor("edited");
        assertEquals(200, user("POST", "/git/commit", cookie, message("my change")).status);

        Reply refused = admin("POST", "/admin/users/delete?username=ada");

        assertEquals(refused.body, 409, refused.status);
        JsonObject body = json(refused.body);
        assertEquals(
                "ada has 1 commit that develop does not have; push it first, or delete anyway",
                body.get("message").getAsString());
        assertEquals("[]", body.getAsJsonArray("changed").toString());
        assertEquals(1, body.get("ahead").getAsInt());
    }

    @Test
    public void aForcedDeleteTakesTheWorktreeAndTheirLoginsAnyway() throws IOException {
        String cookie = savedEditor("edited");
        Path worktree = worktreeOf("ada");

        Reply deleted = admin("POST", "/admin/users/delete?username=ada&force=true");

        assertEquals(deleted.body, 200, deleted.status);
        assertFalse(Files.exists(worktree));
        assertFalse(listed("ada"));
        assertEquals(403, user("GET", "/files", cookie).status);
    }

    @Test
    public void aDeletedUserWhoLogsInAgainGetsTheirBranchBackWithTheirCommits() throws IOException {
        String cookie = savedEditor("ada's work");
        assertEquals(200, user("POST", "/git/commit", cookie, message("my change")).status);
        Path worktree = worktreeOf("ada");
        assertEquals(200, admin("POST", "/admin/users/delete?username=ada&force=true").status);
        assertFalse(Files.exists(worktree));

        String again = approvedUser("ada");

        assertEquals(
                "{\"state\":\"approved\",\"username\":\"ada\",\"branch\":\"coding/ada\"}",
                user("GET", "/me", again).body);
        assertEquals("the same worktree, on the same branch", worktree, worktreeOf("ada"));
        assertEquals(
                "ada's work",
                json(user("GET", "/files/TeamCode/Plans.java", again).body)
                        .get("content")
                        .getAsString());
    }

    @Test
    public void deletingAUserStopsTheirBenchAndLeavesEveryoneElsesRunning() throws Exception {
        String ada = approvedUser("ada");
        String bob = approvedUser("bob");
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Never done"), ada).status);
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Never done"), bob).status);
        assertEquals(2, benches.size());

        assertEquals(200, admin("POST", "/admin/users/delete?username=ada").status);

        assertNull(benches.get(0).current());
        assertTrue(benches.get(0).status(), benches.get(0).status().contains("\"outcome\":\"stopped\""));
        assertNotNull("bob's run is not ada's to stop", benches.get(1).current());
        assertTrue(user("GET", "/sim/status", bob).body.contains("\"running\":true"));
        String again = approvedUser("ada");
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Count to three"), again).status);
        assertEquals("she comes back to a bench of her own", 3, benches.size());
        awaitSimStatus(bob, "\"outcome\":\"timed out");
        awaitSimStatus(again, "\"outcome\":\"done");
    }

    @Test
    public void aRefusedDeleteLeavesTheirRunningSimulationAlone() throws Exception {
        String ada = approvedUser("ada");
        Files.write(worktreeOf("ada").resolve("README"), "typing\n".getBytes(StandardCharsets.UTF_8));
        assertEquals(200, user("POST", "/sim/run?opmode=" + encode("Never done"), ada).status);
        awaitSimStatus(ada, RUNNING);

        Reply refused = admin("POST", "/admin/users/delete?username=ada");

        assertEquals(refused.body, 409, refused.status);
        assertNotNull(benches.get(0).current());
        assertTrue(user("GET", "/sim/status", ada).body.contains("\"running\":true"));
        awaitSimStatus(ada, "\"outcome\":\"timed out");
    }

    @Test
    public void deletingAUserNobodyHasIs404AndDeletingNobodyIs400() throws IOException {
        approvedUser("ada");

        Reply missing = admin("POST", "/admin/users/delete?username=bob");

        assertEquals(404, missing.status);
        assertTrue(missing.body, missing.body.contains("bob"));
        assertEquals(400, admin("POST", "/admin/users/delete").status);
        assertTrue(listed("ada"));
    }

    @Test
    public void theDeleteRouteDoesNotExistOnTheUserPort() throws IOException {
        String cookie = approvedUser("ada");

        assertEquals(404, user("POST", "/admin/users/delete?username=ada", cookie).status);

        assertEquals(200, user("GET", "/files", cookie).status);
        assertTrue(listed("ada"));
    }

    @Test
    public void aDeletedUserIsStillGoneAfterARestart() throws IOException {
        String cookie = approvedUser("ada");
        approvedUser("bob");
        assertEquals(200, admin("POST", "/admin/users/delete?username=ada").status);

        restart();

        assertFalse(listed("ada"));
        assertTrue(listed("bob"));
        assertEquals(403, user("GET", "/files", cookie).status);
        assertFalse(
                "the sessions were saved without her, not just dropped from the map",
                new String(Files.readAllBytes(sessionsFile()), StandardCharsets.UTF_8).contains("\"ada\""));
    }

    @Test
    public void theAdminPageDeletesAUserAndOffersToDeleteAnywayWhenTheServerRefuses() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("'/admin/users/delete?username='"));
        assertTrue(page, page.contains("'&force=true'"));
        assertTrue(page, page.contains("Delete anyway"));
        assertTrue("the server's refusal is what the admin reads", page.contains("deleteArmedByUsername"));
    }

    @Test
    public void theUserListingSaysWhetherADeleteWouldBeRefused() throws IOException {
        approvedUser("bob");
        assertTrue("nothing to lose", deletable("bob"));

        String cookie = savedEditor("edited");
        assertFalse("an uncommitted file would be thrown away", deletable("ada"));

        assertEquals(200, user("POST", "/git/commit", cookie, message("my change")).status);
        assertFalse("a commit develop lacks would be thrown away", deletable("ada"));

        assertEquals(200, user("POST", "/git/push", cookie).status);
        assertTrue("develop has it now", deletable("ada"));
    }

    @Test
    public void theListingAgreesWithWhatTheDeleteDoes() throws IOException {
        savedEditor("edited");

        assertFalse(deletable("ada"));
        assertEquals(409, admin("POST", "/admin/users/delete?username=ada").status);

        assertEquals(200, admin("POST", "/admin/users/delete?username=ada&force=true").status);
        assertTrue("a user who is gone stands in nobody's way", deletable("bob"));
    }

    private boolean deletable(String username) throws IOException {
        for (var element : json(admin("GET", "/admin/users").body).getAsJsonArray("users")) {
            JsonObject user = element.getAsJsonObject();
            if (user.get("username").getAsString().equals(username)) {
                return user.get("deletable").getAsBoolean();
            }
        }
        return true;
    }

    private String login(String username) throws IOException {
        Reply login = user("POST", "/login?username=" + username, null);
        assertEquals(200, login.status);
        return login.sessionCookie();
    }

    private String idOf(String username) throws IOException {
        return newestSessionOf(username).get("id").getAsString();
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

        String sessionCookie() {
            String set = header("Set-Cookie");
            return set == null ? null : set.split(";")[0].trim();
        }
    }
}
