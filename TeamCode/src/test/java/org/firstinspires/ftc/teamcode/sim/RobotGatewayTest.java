package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.nio.file.attribute.PosixFileAttributeView;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.bouncycastle.crypto.generators.SCrypt;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class RobotGatewayTest {
    @Rule
    public TemporaryFolder state = new TemporaryFolder();

    private static final int CHEAP_SCRYPT = 1 << 4;

    private RobotGateway gateway;

    private RobotGateway gateway() {
        if (gateway == null) {
            gatewayWith(refusingToReachOnshape());
        }
        return gateway;
    }

    private RobotGateway gatewayWith(RobotGateway.Assets assets) {
        return gatewayWith(assets, CHEAP_SCRYPT);
    }

    private RobotGateway gatewayWith(RobotGateway.Assets assets, int scryptN) {
        gateway =
                RobotGateway.start(InetAddress.getLoopbackAddress(), 0, 0, stateDir(), scryptN, assets, 0, relayedTo());
        return gateway;
    }

    private static RobotGateway.Assets refusingToReachOnshape() {
        return new RobotGateway.Assets() {
            @Override
            public FieldAssets.Refreshed download(Store store, Path into) {
                throw new AssertionError("a test reached for Onshape; no test may");
            }

            @Override
            public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Resolution> resolutions) {
                throw new AssertionError("a test built a model it never downloaded an export for");
            }
        };
    }

    private static final class Writing implements RobotGateway.Assets {
        final List<String> names;
        int downloads;
        int builds;

        Writing(String... names) {
            this.names = List.of(names);
        }

        @Override
        public FieldAssets.Refreshed download(Store store, Path into) {
            downloads++;
            Map<String, Integer> written = new java.util.LinkedHashMap<>();
            written.putAll(wrote(store, into, FieldAssets.EXPORT_FILE));
            for (String name : names) {
                written.putAll(wrote(store, into, name));
            }
            return new FieldAssets.Refreshed(written);
        }

        @Override
        public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Resolution> resolutions) {
            builds++;
            if (!store.isFile(into.resolve(FieldAssets.EXPORT_FILE))) {
                throw new FieldAssets.NothingDownloaded("nothing has been downloaded yet, download first");
            }
            Map<String, Integer> written = new java.util.LinkedHashMap<>();
            for (FieldAssets.Resolution resolution : resolutions) {
                written.putAll(wrote(store, into, resolution.file));
            }
            return new FieldAssets.Refreshed(written);
        }

        private static Map<String, Integer> wrote(Store store, Path into, String name) {
            byte[] body = ("fetched " + name).getBytes(StandardCharsets.UTF_8);
            store.writeWhole(into.resolve(name), body);
            return Map.of(name, body.length);
        }
    }

    private Path stateDir() {
        return state.getRoot().toPath().resolve("nested").resolve("coding-server");
    }

    private Path sessionsFile() {
        return stateDir().resolve("sessions.json");
    }

    private void restart() {
        gateway.stop();
        gateway = null;
        gateway();
    }

    @After
    public void stopTheGateway() {
        if (gateway != null) {
            gateway.stop();
        }
    }

    @Test
    public void theAdminListenerIsBoundToLoopbackOnly() {
        assertTrue(gateway().adminBindAddress().isLoopbackAddress());
        assertTrue(gateway().userBindAddress().isAnyLocalAddress());
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
        assertFalse("a login is for no robot", older.has("robot") || newer.has("robot"));
        assertEquals(1, bob.getAsJsonArray("sessions").size());
        assertEquals("denied", newestSessionOf("bob").get("state").getAsString());
    }

    @Test
    public void anApprovedUsersPageIsTheAdbConnectLineAndNothingElseToDo() throws IOException {
        String cookie = login("ada");
        String waiting = user("GET", "/", cookie).body;

        assertEquals(200, admin("POST", "/admin/logins/" + idOf("ada") + "/approve").status);

        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        String page = user("GET", "/", cookie).body;
        assertEquals("one page, which shows what the session's state calls for", waiting, page);
        assertTrue(page, page.contains("'/relay'"));
        assertTrue(page, page.contains("id=\"adb\""));
        for (String gone : new String[] {"id=\"editor\"", "codemirror", "/files", "/git/", "/sim/", "/deploy"}) {
            assertFalse("the page offers nothing it no longer does: " + gone, page.contains(gone));
        }
    }

    @Test
    public void noRouteOnEitherListenerReadsOrWritesTheProject() throws IOException {
        String cookie = approvedUser("ada");
        String[][] userRoutes = {
            {"GET", "/files"},
            {"GET", "/files/TeamCode/build.gradle"},
            {"PUT", "/files/TeamCode/build.gradle"},
            {"GET", "/nav"},
            {"GET", "/source"},
            {"GET", "/git/status"},
            {"POST", "/git/commit"},
            {"POST", "/git/pull"},
            {"POST", "/git/push"},
            {"POST", "/deploy"},
            {"POST", "/robot"},
            {"GET", "/build"},
            {"GET", "/sim/catalog"},
            {"GET", "/sim/status"},
            {"GET", "/sim/assets/field.glb"},
            {"GET", "/static/codemirror.js"},
        };
        String[][] adminRoutes = {
            {"GET", "/admin/tree"},
            {"GET", "/admin/files"},
            {"POST", "/admin/files/add"},
            {"POST", "/admin/files/remove"},
            {"GET", "/admin/constants"},
            {"PUT", "/admin/constants"},
            {"POST", "/admin/users/robots"},
            {"POST", "/admin/users/pull"},
            {"POST", "/admin/assets/default"},
        };

        for (String[] route : userRoutes) {
            assertEquals(route[0] + " " + route[1], 404, user(route[0], route[1], cookie).status);
        }
        for (String[] route : adminRoutes) {
            assertEquals(route[0] + " " + route[1], 404, admin(route[0], route[1]).status);
        }
    }

    @Test
    public void aLoginIsAUsernameAloneAndARobotItNamesIsNotHeard() throws IOException {
        Reply login = user("POST", "/login?username=ada&robot=nugget", null);

        assertEquals(200, login.status);
        assertEquals("{\"state\":\"pending\",\"username\":\"ada\"}", login.body);
        assertEquals("{\"state\":\"pending\",\"username\":\"ada\"}", user("GET", "/me", login.sessionCookie()).body);
        assertFalse(storedSession("ada").has("robot"));
    }

    @Test
    public void theLoginPageAsksForAUsernameAndNothingElse() throws IOException {
        String page = user("GET", "/", null).body;

        Matcher inputs = Pattern.compile("<input[^>]*>").matcher(page);
        List<String> asked = new ArrayList<>();
        while (inputs.find()) {
            asked.add(inputs.group());
        }
        assertEquals(asked.toString(), 1, asked.size());
        assertTrue(asked.get(0), asked.get(0).contains("id=\"username\""));
        assertFalse(page, page.contains("robot="));
    }

    @Test
    public void aDeniedUserIsToldAndKeptOut() throws IOException {
        String cookie = login("mallory");

        admin("POST", "/admin/logins/" + idOf("mallory") + "/deny");

        assertEquals("{\"state\":\"denied\",\"username\":\"mallory\"}", user("GET", "/me", cookie).body);
        assertEquals(403, user("GET", "/relay", cookie).status);
    }

    @Test
    public void revokingAnApprovedSessionEndsIt() throws IOException {
        String cookie = approvedUser("ada");
        assertEquals(200, user("GET", "/relay", cookie).status);

        admin("POST", "/admin/logins/" + idOf("ada") + "/revoke");

        assertEquals(403, user("GET", "/relay", cookie).status);
        assertEquals("{\"state\":\"revoked\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
    }

    @Test
    public void anUnknownDecisionOrLoginIsRefused() throws IOException {
        login("ada");

        assertEquals(404, admin("POST", "/admin/logins/" + idOf("ada") + "/promote").status);
        assertEquals(404, admin("POST", "/admin/logins/999/approve").status);
        assertEquals("pending", newestSessionOf("ada").get("state").getAsString());
    }

    @Test
    public void aForgedCookieIsNoSession() throws IOException {
        String forged = "session=" + "A".repeat(22);

        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        assertEquals(403, user("GET", "/relay", forged).status);
        assertTrue(user("GET", "/", forged).body.contains("name=\"username\""));
    }

    @Test
    public void tooManyPendingLoginsAreRefused() throws IOException {
        for (int i = 0; i < RobotGateway.MAX_PENDING_LOGINS; i++) {
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
        String cookie = approvedUser("ada");

        assertEquals(404, user("GET", "/admin", cookie).status);
        assertEquals(404, user("GET", "/admin/users", cookie).status);
        assertEquals(404, user("POST", "/admin/logins/1/approve", cookie).status);
        assertEquals(404, user("POST", "/admin/users/delete?username=ada", cookie).status);
        assertEquals(404, user("GET", "/admin/assets", cookie).status);
        assertTrue(listed("ada"));
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
    public void theAdminPageListsUsersWithDecisionsTheRelayAndTheFieldModels() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("id=\"users\""));
        assertTrue(page, page.contains("/admin/users"));
        assertTrue(page, page.contains("/approve"));
        assertTrue(page, page.contains("/deny"));
        assertTrue(page, page.contains("/revoke"));
        assertTrue(page, page.contains("/admin/info"));
        assertTrue(page, page.contains("Refresh assets"));
        assertTrue(page, page.contains("/admin/assets/refresh"));
        assertTrue(page, page.contains("/admin/assets/download"));
        assertTrue(page, page.contains("/admin/assets/build"));
        for (String gone : new String[] {
            "/admin/tree",
            "/admin/files",
            "/admin/constants",
            "/admin/assets/default",
            "/admin/users/robots",
            "/admin/users/pull",
            "worktree"
        }) {
            assertFalse("the admin page offers nothing the gateway no longer does: " + gone, page.contains(gone));
        }
        assertHiddenWins(page);
        assertEquals(page, admin("GET", "/").body);
        String info = admin("GET", "/admin/info").body;
        assertTrue(info, info.contains("\"userPort\":" + gateway().userPort()));
        assertTrue(info, info.contains("\"addresses\":["));
    }

    @Test
    public void theAdminPageKeepsEachUsersLoginsUnderThemBehindAToggle() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("user.sessions"));
        assertTrue(page, page.contains("aria-expanded"));
        assertTrue("the fold survives the listing's refresh", page.contains("expandedByUsername"));
        assertTrue(page, page.contains("session.state"));
        assertTrue("each login keeps its own decisions", page.contains("'/admin/logins/' + session.id"));
    }

    @Test
    public void theAdminPageDeletesAUser() throws IOException {
        String page = admin("GET", "/admin").body;

        assertTrue(page, page.contains("'/admin/users/delete?username='"));
        assertFalse("a delete is never refused, so there is nothing to force", page.contains("force=true"));
    }

    @Test
    public void aGatewayThatHasFetchedNothingSaysSo() throws IOException {
        Reply said = admin("GET", "/admin/assets");

        assertEquals(200, said.status);
        assertTrue(said.body, said.body.contains("\"complete\":false"));
        assertTrue(said.body, said.body.contains("\"downloaded\":false"));
        assertFalse(gateway().hasFetchedAssets());
    }

    @Test
    public void aRefreshFetchesTheAssetsIntoTheStateDirectory() throws IOException {
        gatewayWith(new Writing(
                FieldAssets.everyAsset(FieldAssets.Resolution.DEFAULT).toArray(new String[0])));

        Reply refreshed = admin("POST", "/admin/assets/refresh");

        assertEquals(refreshed.body, 200, refreshed.status);
        assertTrue(refreshed.body, refreshed.body.contains("\"complete\":true"));
        assertEquals(
                "fetched field-high.glb",
                readString(stateDir().resolve("assets").resolve("field-high.glb")));
        assertTrue(gateway().hasFetchedAssets());
    }

    @Test
    public void aRefreshOnshapeWillNotAnswerSaysSo() throws IOException {
        gatewayWith(new RobotGateway.Assets() {
            @Override
            public FieldAssets.Refreshed download(Store store, Path into) {
                throw new Onshape.NoCredentials("no key pair and no proxy");
            }

            @Override
            public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Resolution> resolutions) {
                throw new AssertionError("a download that never happened must not reach a build");
            }
        });

        Reply refused = admin("POST", "/admin/assets/refresh");

        assertEquals(502, refused.status);
        assertTrue(refused.body, refused.body.contains("no key pair"));
    }

    @Test
    public void onlyTheResolutionAskedForIsFetched() throws IOException {
        gatewayWith(new Writing());

        Reply low = admin("POST", "/admin/assets/refresh?resolution=low");

        assertEquals(low.body, 200, low.status);
        assertTrue(low.body, low.body.contains("\"low\":true"));
        assertTrue(low.body, low.body.contains("\"high\":false"));

        Reply all = admin("POST", "/admin/assets/refresh?resolution=all");

        assertTrue(all.body, all.body.contains("\"high\":true"));
    }

    @Test
    public void aDownloadAndABuildAreAskedForSeparately() throws IOException {
        Writing assets = new Writing(
                FieldAssets.everyAsset(FieldAssets.Resolution.DEFAULT).toArray(new String[0]));
        gatewayWith(assets);

        Reply downloaded = admin("POST", "/admin/assets/download");

        assertEquals(downloaded.body, 200, downloaded.status);
        assertEquals(1, assets.downloads);
        assertEquals("a download is the dear half, and it builds nothing", 0, assets.builds);
        assertTrue(downloaded.body, downloaded.body.contains("\"downloaded\":true"));

        Reply built = admin("POST", "/admin/assets/build?resolution=all");

        assertEquals(built.body, 200, built.status);
        assertEquals("and a build reuses what was downloaded rather than fetching again", 1, assets.downloads);
        assertEquals(1, assets.builds);
        assertTrue(built.body, built.body.contains("\"high\":true"));
    }

    @Test
    public void aBuildWithNothingDownloadedIsRefusedAndSaysWhatToDo() throws IOException {
        gatewayWith(new Writing());

        Reply refused = admin("POST", "/admin/assets/build?resolution=low");

        assertEquals(refused.body, 409, refused.status);
        assertTrue(refused.body, refused.body.contains("nothing has been downloaded"));
    }

    @Test
    public void aResolutionNobodyBuildsIsRefusedBeforeAnythingIsBuiltOrFetched() throws IOException {
        Writing assets = new Writing();
        gatewayWith(assets);

        Reply build = admin("POST", "/admin/assets/build?resolution=finest");
        Reply refresh = admin("POST", "/admin/assets/refresh?resolution=finest");

        assertEquals(400, build.status);
        assertTrue(build.body, build.body.contains("low, medium, high or all"));
        assertEquals(400, refresh.status);
        assertEquals(0, assets.builds);
        assertEquals(0, assets.downloads);
    }

    @Test
    public void anApprovedSessionSurvivesARestart() throws IOException {
        String cookie = approvedUser("ada");
        String id = idOf("ada");

        restart();

        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        assertEquals(id, idOf("ada"));
        assertEquals(200, user("GET", "/relay", cookie).status);
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
        assertEquals("{\"state\":\"approved\",\"username\":\"bob\"}", user("GET", "/me", cookie).body);
    }

    @Test
    public void aRevokedSessionStaysRevokedAcrossARestart() throws IOException {
        String cookie = approvedUser("ada");
        admin("POST", "/admin/logins/" + idOf("ada") + "/revoke");

        restart();

        assertEquals("{\"state\":\"revoked\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        assertEquals(403, user("GET", "/relay", cookie).status);
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

    private static final String SECRET_THE_CODING_SERVER_HANDED_OUT = "c2VjcmV0LWhhbmRlZC1vdXQ";

    private static String scryptOf(String secret, byte[] salt, int n) {
        byte[] hash;
        try (Cost.Spent spent = Cost.start(Cost.Kind.PASSWORD_HASH)) {
            hash = SCrypt.generate(secret.getBytes(StandardCharsets.UTF_8), salt, n, 8, 1, 32);
        }
        return Base64.getEncoder().encodeToString(hash);
    }

    private void theCodingServerLeft(String sessions, String users, String settings) throws IOException {
        Files.createDirectories(stateDir());
        writeString(sessionsFile(), sessions);
        writeString(stateDir().resolve("users.json"), users);
        writeString(stateDir().resolve("settings.json"), settings);
    }

    @Test
    public void approvalsTheCodingServerStoredStayApprovedAndWhatElseItStoredIsLeftAlone() throws IOException {
        byte[] salt = "sixteen byte sal".getBytes(StandardCharsets.UTF_8);
        String secret = "{\"kdf\":\"scrypt\",\"n\":16,\"r\":8,\"p\":1,\"salt\":\""
                + Base64.getEncoder().encodeToString(salt) + "\",\"hash\":\""
                + scryptOf(SECRET_THE_CODING_SERVER_HANDED_OUT, salt, 16) + "\"}";
        String users = "{\"users\":{\"ada\":[\"reginald\",\"nugget\"]}}";
        String settings = "{\"defaultResolution\":\"high\",\"constants\":{\"driveMaxSpeed\":1.5}}";
        theCodingServerLeft(
                "{\"sessions\":[{\"id\":7,\"username\":\"ada\",\"robot\":\"nugget\",\"address\":\"127.0.0.1\","
                        + "\"state\":\"approved\",\"createdAtMillis\":1700000000000,\"secret\":" + secret + "},"
                        + "{\"id\":8,\"username\":\"bob\",\"address\":\"127.0.0.1\",\"state\":\"pending\","
                        + "\"createdAtMillis\":1700000000001,\"secret\":" + secret + "}]}",
                users,
                settings);
        String cookie = "session=7." + SECRET_THE_CODING_SERVER_HANDED_OUT;

        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
        assertEquals(200, user("GET", "/relay", cookie).status);
        assertEquals("pending", newestSessionOf("bob").get("state").getAsString());
        login("cy");
        assertEquals("9", idOf("cy"));
        assertFalse(
                "the robot a login was once for is read, dropped on the next write and never written again",
                readString(sessionsFile()).contains("\"robot\""));
        assertEquals(users, readString(stateDir().resolve("users.json")));
        assertEquals(settings, readString(stateDir().resolve("settings.json")));
    }

    @Test
    public void theSessionStoreHoldsASaltedScryptHashOfEachSecretAndNeverTheSecret() throws IOException {
        gatewayWith(refusingToReachOnshape(), RobotGateway.SCRYPT_N);
        String ada = login("ada");
        String bob = login("bob");

        String stored = readString(sessionsFile());

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
        String cookie = approvedUser("ada");
        String forged = cookie.substring(0, cookie.lastIndexOf('.') + 1)
                + "B".repeat(secretOf(cookie).length());

        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        restart();
        assertEquals("{\"state\":\"none\"}", user("GET", "/me", forged).body);
        assertEquals(403, user("GET", "/relay", forged).status);
        assertEquals("{\"state\":\"approved\",\"username\":\"ada\"}", user("GET", "/me", cookie).body);
    }

    @Test
    public void aSessionStoreThatCannotBeReadStopsTheGatewayFromStarting() throws IOException {
        login("ada");
        gateway.stop();
        gateway = null;
        Files.write(sessionsFile(), "{not json".getBytes(StandardCharsets.UTF_8));

        try {
            gateway();
            fail("the gateway started over a session store it could not read");
        } catch (IllegalStateException e) {
            assertTrue(e.getMessage(), e.getMessage().contains(sessionsFile().toString()));
        }
    }

    @Test
    public void theStateDirectoryIsWhereTheCodingServerKeptItUnderTheXdgBaseDirectoryConvention() {
        Path xdg = state.getRoot().toPath().resolve("xdg");
        Path home = state.getRoot().toPath().resolve("home");
        Path underHome = home.resolve(".local/state/midnight-snackers/coding-server");

        assertEquals(
                xdg.resolve("midnight-snackers/coding-server"),
                RobotGateway.stateDir(Map.of("XDG_STATE_HOME", xdg.toString(), "HOME", home.toString())));
        assertEquals(underHome, RobotGateway.stateDir(Map.of("HOME", home.toString())));
        assertEquals(
                underHome, RobotGateway.stateDir(Map.of("XDG_STATE_HOME", "relative/state", "HOME", home.toString())));
        assertEquals(underHome, RobotGateway.stateDir(Map.of("XDG_STATE_HOME", "", "HOME", home.toString())));
    }

    private Map<String, String> everythingUnder(Path root) throws IOException {
        Map<String, String> found = new TreeMap<>();
        try (Stream<Path> walk = Files.walk(root)) {
            for (Path path : (Iterable<Path>) walk::iterator) {
                FileTime modified = Files.getLastModifiedTime(path);
                String what = Files.isDirectory(path) ? "dir " + modified : readString(path) + " @ " + modified;
                found.put(root.relativize(path).toString(), what);
            }
        }
        return found;
    }

    @Test
    public void worktreesTheCodingServerLeftAreNamedAtStartupAndNeverChangedOrDeleted() throws Exception {
        Path worktrees = stateDir().resolve("worktrees");
        Path reginalds = worktrees.resolve("midnight-snackers-1a2b3c4d");
        Path nuggets = worktrees.resolve("midnight-snackers-1a2b3c4d-nugget");
        Files.createDirectories(reginalds.resolve("ada/TeamCode"));
        Files.createDirectories(nuggets.resolve("ada"));
        writeString(reginalds.resolve("ada/TeamCode/Plans.java"), "unpushed work");
        writeString(nuggets.resolve("ada/.git"), "gitdir: /somewhere/else");
        writeString(worktrees.resolve("stray.txt"), "not a worktree");
        Map<String, String> before = everythingUnder(worktrees);

        List<String> said = gateway().said();
        approvedUser("ada");
        assertEquals(200, admin("POST", "/admin/users/delete?username=ada").status);
        approvedUser("ada");
        restart();

        int named = said.indexOf("  trees  " + reginalds);
        assertTrue(String.join("\n", said), named > 0);
        assertEquals("  trees  " + nuggets, said.get(named + 1));
        assertTrue(said.get(named + 2), said.get(named + 2).contains("no longer manages them"));
        assertFalse("only directories are worktrees", String.join("\n", said).contains("stray.txt"));
        assertEquals(before, everythingUnder(worktrees));
    }

    @Test
    public void aGatewayWithNoWorktreesLeftSaysNothingOfThem() {
        List<String> said = gateway().said();

        assertTrue(String.join("\n", said), said.contains("  state  " + stateDir()));
        assertFalse(String.join("\n", said), String.join("\n", said).contains("trees"));
    }

    @Test
    public void deletingAUserEndsEveryOneOfTheirLogins() throws IOException {
        String cookie = approvedUser("ada");
        String second = login("ada");
        approvedUser("bob");

        Reply deleted = admin("POST", "/admin/users/delete?username=ada");

        assertEquals(deleted.body, 200, deleted.status);
        assertEquals(2, json(deleted.body).get("logins").getAsInt());
        assertEquals("{\"state\":\"none\"}", user("GET", "/me", cookie).body);
        assertEquals("{\"state\":\"none\"}", user("GET", "/me", second).body);
        assertFalse(listed("ada"));
        assertTrue(listed("bob"));
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
    public void aDeletedUserIsStillGoneAfterARestart() throws IOException {
        String cookie = approvedUser("ada");
        approvedUser("bob");
        assertEquals(200, admin("POST", "/admin/users/delete?username=ada").status);

        restart();

        assertFalse(listed("ada"));
        assertTrue(listed("bob"));
        assertEquals(403, user("GET", "/relay", cookie).status);
        assertFalse(
                "the sessions were saved without her, not just dropped from the map",
                readString(sessionsFile()).contains("\"ada\""));
    }

    private FakeHub hub;

    private InetSocketAddress hubAddress = null;

    private FakeHub hub() {
        if (hub == null) {
            hub = FakeHub.start();
        }
        return hub;
    }

    private InetSocketAddress relayedTo() {
        return hubAddress == null ? hub().address() : hubAddress;
    }

    @After
    public void stopTheHub() {
        if (hub != null) {
            hub.close();
        }
    }

    private Socket relayed() throws IOException {
        return relayedFrom(InetAddress.getLoopbackAddress());
    }

    private Socket relayedFrom(InetAddress from) throws IOException {
        Socket client = new Socket();
        client.setSoTimeout(10_000);
        client.bind(new InetSocketAddress(from, 0));
        client.connect(
                new InetSocketAddress(
                        InetAddress.getLoopbackAddress(), gateway().relayPort()),
                2_000);
        return client;
    }

    private static String echoed(Socket client, String text) throws IOException {
        client.getOutputStream().write(text.getBytes(StandardCharsets.UTF_8));
        client.getOutputStream().flush();
        return new String(client.getInputStream().readNBytes(text.length()), StandardCharsets.UTF_8);
    }

    private static void assertClosedByTheRelay(Socket client) throws IOException {
        try {
            assertEquals(
                    "the relay closed the connection",
                    -1,
                    client.getInputStream().read());
        } catch (java.net.SocketException reset) {
            assertTrue(
                    reset.getMessage(),
                    reset.getMessage().toLowerCase(java.util.Locale.ROOT).contains("reset"));
        } catch (java.net.SocketTimeoutException e) {
            throw new AssertionError("the relay left the connection hanging", e);
        }
    }

    private JsonObject userListed(String username) throws IOException {
        for (var element : json(admin("GET", "/admin/users").body).getAsJsonArray("users")) {
            JsonObject user = element.getAsJsonObject();
            if (user.get("username").getAsString().equals(username)) {
                return user;
            }
        }
        throw new AssertionError("no user " + username + " is listed");
    }

    private JsonObject relayListed() throws IOException {
        return json(admin("GET", "/admin/users").body).getAsJsonObject("relay");
    }

    private void awaitListedRelaying(String username, boolean relaying) throws Exception {
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (userListed(username).get("relayed").getAsBoolean() != relaying) {
            if (System.nanoTime() > deadline) {
                throw new AssertionError(username + " is never listed with relayed " + relaying);
            }
            Thread.sleep(10);
        }
    }

    @Test
    public void aClientFromWhereAnApprovedLoginCameFromIsRelayedToTheHub() throws IOException {
        approvedUser("ada");

        try (Socket client = relayed()) {
            assertEquals("CNXN host::", echoed(client, "CNXN host::"));
        }
        assertEquals("CNXN host::", hub().received());
    }

    @Test
    public void aClientFromWhereOnlyAPendingLoginCameFromIsClosedWithoutReachingTheHub() throws IOException {
        login("ada");

        try (Socket client = relayed()) {
            client.getOutputStream().write("CNXN host::".getBytes(StandardCharsets.UTF_8));
            assertClosedByTheRelay(client);
        }
        assertEquals("", hub().received());
    }

    @Test
    public void aClientFromAnotherAddressThanTheApprovedLoginsIsClosedWithoutReachingTheHub() throws IOException {
        approvedUser("ada");
        InetAddress another = SecondLoopback.address();

        try (Socket client = relayedFrom(another)) {
            client.getOutputStream().write("CNXN host::".getBytes(StandardCharsets.UTF_8));
            assertClosedByTheRelay(client);
        }
        assertEquals("", hub().received());
    }

    @Test
    public void theRelayListensOnEveryInterface() {
        assertTrue(gateway().relayBindAddress().isAnyLocalAddress());
    }

    @Test
    public void revokingALoginClosesTheConnectionsRelayedForWhereItCameFrom() throws IOException {
        approvedUser("ada");

        try (Socket client = relayed()) {
            echoed(client, "x");
            assertEquals(200, admin("POST", "/admin/logins/" + idOf("ada") + "/revoke").status);

            assertClosedByTheRelay(client);
        }
    }

    @Test
    public void revokingALoginLeavesTheConnectionOpenWhileAnotherApprovedLoginSharesItsAddress() throws IOException {
        approvedUser("ada");
        approvedUser("bob");

        try (Socket client = relayed()) {
            echoed(client, "x");
            assertEquals(200, admin("POST", "/admin/logins/" + idOf("ada") + "/revoke").status);

            assertEquals("y", echoed(client, "y"));
        }
    }

    @Test
    public void deletingAUserClosesTheConnectionsRelayedForWhereTheirLoginsCameFrom() throws IOException {
        approvedUser("ada");

        try (Socket client = relayed()) {
            echoed(client, "x");
            assertEquals(200, admin("POST", "/admin/users/delete?username=ada").status);

            assertClosedByTheRelay(client);
        }
    }

    @Test
    public void theUserListingSaysWhoseConnectionIsRelayed() throws Exception {
        approvedUser("ada");
        login("bob");
        assertFalse(userListed("ada").get("relayed").getAsBoolean());

        try (Socket client = relayed()) {
            echoed(client, "x");

            assertTrue(userListed("ada").get("relayed").getAsBoolean());
            assertFalse(userListed("bob").get("relayed").getAsBoolean());
        }
        awaitListedRelaying("ada", false);
    }

    @Test
    public void theUserListingSaysWhereTheRelayListensAndWhichHubItReaches() throws IOException {
        JsonObject relay = relayListed();

        assertEquals(gateway().relayPort(), relay.get("port").getAsInt());
        assertEquals("127.0.0.1:" + hub().address().getPort(), relay.get("hub").getAsString());
        for (JsonElement line : relay.getAsJsonArray("connect")) {
            assertTrue(line.getAsString(), line.getAsString().startsWith("adb connect "));
            assertTrue(
                    line.getAsString(),
                    line.getAsString().endsWith(":" + gateway().relayPort()));
        }
        assertTrue(relay.toString(), relay.getAsJsonArray("connect").size() > 0);
    }

    @Test
    public void theUserListingSaysAHubThatCannotBeReached() throws Exception {
        hubAddress = FakeHub.nobodyListening();
        relayListed();

        long deadline = System.nanoTime() + 10_000_000_000L;
        JsonObject relay = relayListed();
        while (!relay.get("hubState").getAsString().equals("unreachable")) {
            if (System.nanoTime() > deadline) {
                throw new AssertionError("the hub is never listed as unreachable: " + relay);
            }
            Thread.sleep(10);
            relay = relayListed();
        }
        assertFalse(relay.get("hubWhy").getAsString().isEmpty());
        assertTrue(relay.get("hubCheckedSecondsAgo").getAsLong() >= 0);
    }

    @Test
    public void theUserListingSaysAHubThatWasReached() throws Exception {
        approvedUser("ada");
        try (Socket client = relayed()) {
            echoed(client, "x");
        }

        assertEquals("reachable", relayListed().get("hubState").getAsString());
    }

    @Test
    public void anApprovedLoginIsToldTheAdbConnectLineForTheAddressItReachedTheHostBy() throws IOException {
        String cookie = approvedUser("ada");

        Reply relay = user("GET", "/relay", cookie);

        assertEquals(relay.body, 200, relay.status);
        assertEquals(
                "adb connect 127.0.0.1:" + gateway().relayPort(),
                json(relay.body).get("line").getAsString());
    }

    @Test
    public void aLoginNotApprovedIsToldNoAdbConnectLine() throws IOException {
        String cookie = login("ada");

        assertEquals(403, user("GET", "/relay", cookie).status);
    }

    private static void assertHiddenWins(String page) {
        assertTrue(page, page.replaceAll("\\s+", " ").contains("[hidden] { display: none !important; }"));
    }

    private static String readString(Path path) throws IOException {
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
    }

    private static void writeString(Path path, String text) throws IOException {
        Files.write(path, text.getBytes(StandardCharsets.UTF_8));
    }

    private static JsonObject json(String body) {
        return new Gson().fromJson(body, JsonObject.class);
    }

    private static String secretOf(String cookie) {
        return cookie.substring(cookie.lastIndexOf('.') + 1);
    }

    private JsonObject storedSession(String username) throws IOException {
        for (var element : json(readString(sessionsFile())).getAsJsonArray("sessions")) {
            if (element.getAsJsonObject().get("username").getAsString().equals(username)) {
                return element.getAsJsonObject();
            }
        }
        throw new AssertionError("no stored session for " + username);
    }

    private boolean listed(String username) throws IOException {
        return admin("GET", "/admin/users").body.contains("\"username\":\"" + username + "\"");
    }

    private JsonObject newestSessionOf(String username) throws IOException {
        JsonArray sessions = userListed(username).getAsJsonArray("sessions");
        assertTrue(username + " has no sessions", sessions.size() > 0);
        return sessions.get(sessions.size() - 1).getAsJsonObject();
    }

    private String approvedUser(String name) throws IOException {
        String cookie = login(name);
        assertEquals(200, admin("POST", "/admin/logins/" + idOf(name) + "/approve").status);
        return cookie;
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
        return request(gateway().userUrl(), method, path, cookie);
    }

    private Reply admin(String method, String path) throws IOException {
        return request(gateway().adminUrl(), method, path, null);
    }

    private static Reply request(String base, String method, String path, String cookie) throws IOException {
        HttpURLConnection connection = (HttpURLConnection) new URL(base + path.substring(1)).openConnection();
        connection.setRequestMethod(method);
        if (cookie != null) {
            connection.setRequestProperty("Cookie", cookie);
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
