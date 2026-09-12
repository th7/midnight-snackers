package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.google.gson.JsonObject;
import com.google.gson.Gson;

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
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;

public class SharedEditorServerTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private SharedEditorServer server;

    private SharedEditorServer server() {
        if (server == null) {
            server = SharedEditorServer.start(folder.getRoot().toPath(), InetAddress.getLoopbackAddress(), 0, 0);
        }
        return server;
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
        for (int i = 0; i < SharedEditorServer.MAX_PENDING_LOGINS; i++) {
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
