package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

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
