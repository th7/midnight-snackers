package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Base64;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.bouncycastle.crypto.generators.SCrypt;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;
import org.firstinspires.ftc.teamcode.simcore.GatewayPorts;
import org.firstinspires.ftc.teamcode.simcore.HubState;
import org.firstinspires.ftc.teamcode.simcore.LeftWorktrees;
import org.firstinspires.ftc.teamcode.simcore.Peer;
import org.firstinspires.ftc.teamcode.simcore.RelayAdmission;

public final class RobotGateway {
    public static final int MAX_PENDING_LOGINS = 20;
    public static final int MAX_USERNAME_LENGTH = 32;
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final String COOKIE = "session";

    private static final String SESSIONS_FILE = "sessions.json";
    private static final String ASSETS_DIR = "assets";
    private static final String WORKTREES_DIR = "worktrees";

    static final int SCRYPT_N = 1 << 14;

    private static final int SCRYPT_R = 8;
    private static final int SCRYPT_P = 1;
    private static final int SALT_BYTES = 16;
    private static final int HASH_BYTES = 32;
    private static final int SECRET_BYTES = 16;

    enum State {
        PENDING,
        APPROVED,
        DENIED,
        REVOKED
    }

    private static final class Session {
        final int id;
        final String username;
        final InetAddress address;
        final long createdAtMillis;
        final Secret secret;
        State state;

        String token;

        Session(int id, String username, InetAddress address, long createdAtMillis, Secret secret, State state) {
            this.id = id;
            this.username = username;
            this.address = address;
            this.createdAtMillis = createdAtMillis;
            this.secret = secret;
            this.state = state;
        }

        String addressSaid() {
            return address == null ? "" : address.getHostAddress();
        }

        JsonObject toJson() {
            JsonObject item = new JsonObject();
            item.addProperty("id", id);
            item.addProperty("username", username);
            item.addProperty("address", addressSaid());
            item.addProperty("state", state.name().toLowerCase(Locale.ROOT));
            item.addProperty("createdAtMillis", createdAtMillis);
            item.add("secret", secret.toJson());
            return item;
        }

        static Session fromJson(JsonObject item) {
            String address = item.get("address").getAsString();
            InetAddress parsed = null;
            if (!address.isEmpty()) {
                try {
                    parsed = InetAddress.getByName(address);
                } catch (UnknownHostException e) {
                    throw new IllegalStateException("not an address: " + address);
                }
            }
            return new Session(
                    item.get("id").getAsInt(),
                    item.get("username").getAsString(),
                    parsed,
                    item.get("createdAtMillis").getAsLong(),
                    Secret.fromJson(item.getAsJsonObject("secret")),
                    State.valueOf(item.get("state").getAsString().toUpperCase(Locale.ROOT)));
        }
    }

    private static final class Secret {
        final int n;
        final int r;
        final int p;
        final byte[] salt;
        final byte[] hash;

        Secret(int n, int r, int p, byte[] salt, byte[] hash) {
            this.n = n;
            this.r = r;
            this.p = p;
            this.salt = salt;
            this.hash = hash;
        }

        static Secret of(String secret, SecureRandom random, int n) {
            byte[] salt = new byte[SALT_BYTES];
            random.nextBytes(salt);
            return new Secret(n, SCRYPT_R, SCRYPT_P, salt, derive(secret, salt, n, SCRYPT_R, SCRYPT_P, HASH_BYTES));
        }

        boolean matches(String secret) {
            return MessageDigest.isEqual(hash, derive(secret, salt, n, r, p, hash.length));
        }

        private static byte[] derive(String secret, byte[] salt, int n, int r, int p, int length) {
            try (Cost.Spent spent = Cost.start(Cost.Kind.PASSWORD_HASH)) {
                return SCrypt.generate(secret.getBytes(StandardCharsets.UTF_8), salt, n, r, p, length);
            }
        }

        JsonObject toJson() {
            JsonObject item = new JsonObject();
            item.addProperty("kdf", "scrypt");
            item.addProperty("n", n);
            item.addProperty("r", r);
            item.addProperty("p", p);
            item.addProperty("salt", Base64.getEncoder().encodeToString(salt));
            item.addProperty("hash", Base64.getEncoder().encodeToString(hash));
            return item;
        }

        static Secret fromJson(JsonObject item) {
            if (!"scrypt".equals(item.get("kdf").getAsString())) {
                throw new IllegalStateException("unknown kdf " + item.get("kdf"));
            }
            return new Secret(
                    item.get("n").getAsInt(),
                    item.get("r").getAsInt(),
                    item.get("p").getAsInt(),
                    Base64.getDecoder().decode(item.get("salt").getAsString()),
                    Base64.getDecoder().decode(item.get("hash").getAsString()));
        }
    }

    private final Path stateDir;
    private final int scryptN;
    private final Router userRoutes = userRoutes();
    private final TinyHttpServer admin;
    private final TinyHttpServer users;
    private final RobotRelay relay;
    private final SecureRandom random = new SecureRandom();

    private final Map<Integer, Session> sessions = new LinkedHashMap<>();

    private int nextSessionId = 1;

    private final Path assetsDir;

    private final Store assetStore = new OnDiskStore();
    private final Assets assets;

    /**
     * The two halves of getting a field model, kept apart because they cost such different things: the
     * download is tens of seconds of Onshape, the build is under a second of arithmetic on what the
     * download left behind. Only the first half is given an Onshape to reach.
     */
    public interface Assets {
        FieldAssets.Refreshed download(Store store, Path into);

        FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Resolution> resolutions);
    }

    public static Assets fromOnshape() {
        return new Assets() {
            @Override
            public FieldAssets.Refreshed download(Store store, Path into) {
                return FieldAssets.download(Onshape.configured(), store, into);
            }

            @Override
            public FieldAssets.Refreshed build(Store store, Path into, List<FieldAssets.Resolution> resolutions) {
                return FieldAssets.build(store, into, resolutions);
            }
        };
    }

    private RobotGateway(
            InetAddress adminBind,
            int adminPort,
            int userPort,
            Path stateDir,
            int scryptN,
            Assets assets,
            int relayPort,
            InetSocketAddress hub) {
        this.scryptN = scryptN;
        this.assets = assets;
        this.stateDir = stateDir.toAbsolutePath().normalize();
        loadSessions();
        this.assetsDir = this.stateDir.resolve(ASSETS_DIR);
        this.relay = RobotRelay.start(null, relayPort, hub, this::relayAdmission);
        this.admin = TinyHttpServer.start(adminBind, adminPort, "gateway-admin", adminRoutes());
        this.users = TinyHttpServer.start(userPort, "gateway-users", this::handleUser);
    }

    static RobotGateway start(
            InetAddress adminBind,
            int adminPort,
            int userPort,
            Path stateDir,
            int scryptN,
            Assets assets,
            int relayPort,
            InetSocketAddress hub) {
        return new RobotGateway(adminBind, adminPort, userPort, stateDir, scryptN, assets, relayPort, hub);
    }

    static Path stateDir(Map<String, String> env) {
        String xdg = env.get("XDG_STATE_HOME");
        Path base;
        if (xdg != null && !xdg.isBlank() && Path.of(xdg).isAbsolute()) {
            base = Path.of(xdg);
        } else {
            String home = env.get("HOME");
            if (home == null || home.isBlank()) {
                home = System.getProperty("user.home");
            }
            base = Path.of(home, ".local", "state");
        }
        return base.resolve("midnight-snackers").resolve("coding-server");
    }

    public static void main(String[] args) throws InterruptedException {
        Map<String, String> env = System.getenv();
        GatewayPorts ports = Valid.value(GatewayPorts.from(env));
        RobotGateway gateway = start(
                InetAddress.getLoopbackAddress(),
                ports.admin(),
                ports.users(),
                stateDir(env),
                SCRYPT_N,
                fromOnshape(),
                RobotRelay.port(env),
                RobotRelay.hub(env));
        gateway.said().forEach(System.out::println);
        if (!gateway.hasFetchedAssets()) {
            gateway.refreshAssetsInTheBackground();
        }
        Thread.currentThread().join();
    }

    List<String> said() {
        List<String> lines = new ArrayList<>();
        lines.add("Robot gateway");
        lines.add("  admin  " + adminUrl() + "admin   (this machine only)");
        lines.add("  users  http://<this machine's LAN address>:" + userPort() + "/   (Ctrl-C to stop)");
        lines.add("  robot  " + RelayAdmission.connectLine("<this machine's LAN address>", relayPort())
                + "   relays to the hub at " + relay.hubAddress());
        lines.add("  state  " + stateDir);
        lines.addAll(LeftWorktrees.notice(leftWorktrees()));
        if (hasFetchedAssets()) {
            lines.add("  assets " + assetsDir);
        }
        return Collections.unmodifiableList(lines);
    }

    private List<String> leftWorktrees() {
        Path under = stateDir.resolve(WORKTREES_DIR);
        if (!Files.isDirectory(under)) {
            return List.of();
        }
        try (Stream<Path> listing = Files.list(under)) {
            return listing.filter(Files::isDirectory)
                    .map(Path::toString)
                    .sorted()
                    .toList();
        } catch (IOException e) {
            return List.of(under + " (could not be listed: " + e.getMessage() + ")");
        }
    }

    public String adminUrl() {
        return admin.url();
    }

    public String userUrl() {
        return users.url();
    }

    public int userPort() {
        return users.port();
    }

    public InetAddress adminBindAddress() {
        return admin.bindAddress();
    }

    public InetAddress userBindAddress() {
        return users.bindAddress();
    }

    public int relayPort() {
        return relay.port();
    }

    public InetAddress relayBindAddress() {
        return relay.bindAddress();
    }

    public void stop() {
        admin.stop();
        users.stop();
        relay.stop();
    }

    private Router userRoutes() {
        Router approved = new Router()
                .guard(request ->
                        isApproved(sessionOf(request)) ? null : Response.error(403, "not an approved session"))
                .route("GET", "/relay", (request, params) -> relayLine(request));
        return new Router()
                .route("GET", "/", (request, params) -> Response.html(page("login.html")))
                .route("POST", "/login", (request, params) -> login(request))
                .route("GET", "/me", (request, params) -> Response.json(GSON.toJson(me(sessionOf(request)))))
                .mount("", approved);
    }

    private static boolean isApproved(Session session) {
        return session != null && session.state == State.APPROVED;
    }

    private synchronized RelayAdmission relayAdmission() {
        List<RelayAdmission.Login> logins = new ArrayList<>();
        for (Session session : sessions.values()) {
            Peer.of(session.addressSaid())
                    .fold(
                            from -> logins.add(
                                    new RelayAdmission.Login(session.username, from, session.state == State.APPROVED)),
                            rule -> false);
        }
        return RelayAdmission.of(logins);
    }

    private Response swept(Response response) {
        relay.sweep();
        return response;
    }

    private Response relayLine(Request request) {
        JsonObject body = new JsonObject();
        body.addProperty("line", RelayAdmission.connectLine(request.localAddress.getHostAddress(), relay.port()));
        body.addProperty("port", relay.port());
        return Response.json(GSON.toJson(body));
    }

    private JsonObject relayListed(long now) {
        relay.probeIfDue();
        HubState hub = relay.hubState();
        JsonObject body = new JsonObject();
        body.addProperty("port", relay.port());
        body.addProperty("hub", relay.hubAddress());
        body.addProperty("hubState", hub.said());
        body.addProperty("hubWhy", hub.why());
        JsonArray connect = new JsonArray();
        List<String> lan = lanAddresses();
        for (String address : lan.isEmpty() ? List.of("<this machine's LAN address>") : lan) {
            connect.add(RelayAdmission.connectLine(address, relay.port()));
        }
        body.add("connect", connect);
        body.add(
                "hubCheckedSecondsAgo",
                hub.checkedAtMillis().isPresent()
                        ? GSON.toJsonTree(
                                Math.max(0, (now - hub.checkedAtMillis().getAsLong()) / 1000))
                        : JsonNull.INSTANCE);
        return body;
    }

    private Response handleUser(Request request) {
        return userRoutes.handle(request);
    }

    private Response login(Request request) {
        String username = request.query("username");
        username = username == null ? "" : username.trim();
        if (username.isEmpty() || username.length() > MAX_USERNAME_LENGTH || !printable(username)) {
            return Response.error(400, "username must be 1 to " + MAX_USERNAME_LENGTH + " printable characters");
        }
        Session session;
        synchronized (this) {
            if (pendingCount() >= MAX_PENDING_LOGINS) {
                return Response.error(429, "too many logins waiting for approval; ask the admin to clear the list");
            }
            byte[] bytes = new byte[SECRET_BYTES];
            random.nextBytes(bytes);
            String secret = Base64.getUrlEncoder().withoutPadding().encodeToString(bytes);
            int id = nextSessionId++;
            session = new Session(
                    id,
                    username,
                    request.remoteAddress,
                    System.currentTimeMillis(),
                    Secret.of(secret, random, scryptN),
                    State.PENDING);
            session.token = id + "." + secret;
            sessions.put(id, session);
            try {
                saveSessions();
            } catch (RuntimeException e) {
                sessions.remove(id);
                throw e;
            }
        }
        return Response.json(GSON.toJson(me(session)))
                .withHeader("Set-Cookie", COOKIE + "=" + session.token + "; HttpOnly; SameSite=Strict; Path=/");
    }

    private static boolean printable(String username) {
        return username.chars().noneMatch(c -> c < 0x20 || Character.isISOControl(c));
    }

    private static JsonObject me(Session session) {
        JsonObject body = new JsonObject();
        if (session == null) {
            body.addProperty("state", "none");
            return body;
        }
        body.addProperty("state", session.state.name().toLowerCase(Locale.ROOT));
        body.addProperty("username", session.username);
        return body;
    }

    private synchronized Session sessionOf(Request request) {
        String token = request.cookie(COOKIE);
        if (token == null) {
            return null;
        }
        int dot = token.indexOf('.');
        if (dot <= 0) {
            return null;
        }
        Session session;
        try {
            session = sessions.get(Integer.parseInt(token.substring(0, dot)));
        } catch (NumberFormatException e) {
            return null;
        }
        if (session == null) {
            return null;
        }
        if (session.token != null) {
            return token.equals(session.token) ? session : null;
        }
        if (!session.secret.matches(token.substring(dot + 1))) {
            return null;
        }
        session.token = token;
        return session;
    }

    private int pendingCount() {
        int count = 0;
        for (Session session : sessions.values()) {
            if (session.state == State.PENDING) {
                count++;
            }
        }
        return count;
    }

    private Router adminRoutes() {
        return new Router()
                .route("GET", "/", (request, params) -> Response.html(page("admin.html")))
                .route("GET", "/admin", (request, params) -> Response.html(page("admin.html")))
                .route("GET", "/admin/users", (request, params) -> Response.json(users()))
                .route("POST", "/admin/users/delete", (request, params) -> swept(deleteUser(request.query("username"))))
                .route("GET", "/admin/info", (request, params) -> Response.json(info()))
                .route("GET", "/admin/assets", (request, params) -> Response.json(GSON.toJson(assetsFetched())))
                .route("POST", "/admin/assets/download", (request, params) -> downloadAssets())
                .route("POST", "/admin/assets/build", (request, params) -> buildAssets(request.query("resolution")))
                .route("POST", "/admin/assets/refresh", (request, params) -> refreshAssets(request.query("resolution")))
                .route(
                        "POST",
                        "/admin/logins/{id}/{decision}",
                        (request, params) -> swept(decide(params.get("id"), params.get("decision"))));
    }

    private JsonObject assetsFetched() {
        JsonObject out = new JsonObject();
        out.addProperty("under", assetsDir.toString());
        JsonObject held = new JsonObject();
        for (String name : FieldAssets.everyAsset(FieldAssets.Resolution.DEFAULT)) {
            assetStore.readIfThere(assetsDir.resolve(name)).ifPresent(bytes -> held.addProperty(name, bytes.length));
        }
        out.add("fetched", held);
        out.addProperty(
                "complete",
                held.size()
                        == FieldAssets.everyAsset(FieldAssets.Resolution.DEFAULT)
                                .size());
        JsonObject resolutions = new JsonObject();
        for (FieldAssets.Resolution one : FieldAssets.Resolution.values()) {
            resolutions.addProperty(one.asked, assetStore.isFile(assetsDir.resolve(one.file)));
        }
        out.add("resolution", resolutions);
        out.addProperty("downloaded", assetStore.isFile(assetsDir.resolve(FieldAssets.EXPORT_FILE)));
        return out;
    }

    private Response downloadAssets() {
        return whatItWrote(() -> assets.download(assetStore, assetsDir));
    }

    private Response buildAssets(String resolution) {
        List<FieldAssets.Resolution> resolutions;
        try {
            resolutions = FieldAssets.resolutionsNamed(resolution, FieldAssets.Resolution.DEFAULT);
        } catch (FieldAssets.NotAnAsset wrong) {
            return Response.error(400, wrong.getMessage());
        }
        return whatItWrote(() -> assets.build(assetStore, assetsDir, resolutions));
    }

    private Response refreshAssets(String resolution) {
        List<FieldAssets.Resolution> resolutions;
        try {
            resolutions = FieldAssets.resolutionsNamed(resolution, FieldAssets.Resolution.DEFAULT);
        } catch (FieldAssets.NotAnAsset wrong) {
            return Response.error(400, wrong.getMessage());
        }
        return whatItWrote(() -> {
            FieldAssets.Refreshed downloaded = assets.download(assetStore, assetsDir);
            FieldAssets.Refreshed built = assets.build(assetStore, assetsDir, resolutions);
            Map<String, Integer> written = new LinkedHashMap<>(downloaded.written);
            written.putAll(built.written);
            return new FieldAssets.Refreshed(written);
        });
    }

    private Response whatItWrote(Supplier<FieldAssets.Refreshed> work) {
        try {
            FieldAssets.Refreshed refreshed = work.get();
            JsonObject out = assetsFetched();
            out.addProperty("refreshed", refreshed.written.size());
            out.addProperty("bytes", refreshed.bytes());
            return Response.json(GSON.toJson(out));
        } catch (FieldAssets.NothingDownloaded first) {
            return Response.error(409, first.getMessage());
        } catch (Onshape.NoCredentials refused) {
            return Response.error(502, "Onshape would not answer: " + refused.getMessage());
        } catch (RuntimeException wrong) {
            return Response.error(502, "the assets could not be refreshed: " + wrong.getMessage());
        }
    }

    boolean hasFetchedAssets() {
        return assetStore.isFile(assetsDir.resolve(FieldAssets.Resolution.DEFAULT.file));
    }

    void refreshAssetsInTheBackground() {
        Thread fetching = new Thread(
                () -> {
                    try {
                        System.out.println("  assets fetching from Onshape into " + assetsDir);
                        assets.download(assetStore, assetsDir);
                        System.out.println("  assets "
                                + assets.build(assetStore, assetsDir, List.of(FieldAssets.Resolution.DEFAULT)));
                    } catch (RuntimeException wrong) {
                        System.out.println("  assets not fetched (" + wrong.getMessage()
                                + "); download and build them from the admin page");
                    }
                },
                "assets-refresh");
        fetching.setDaemon(true);
        fetching.start();
    }

    private static List<String> lanAddresses() {
        List<String> addresses = new ArrayList<>();
        try {
            for (NetworkInterface nic : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                for (InetAddress address : Collections.list(nic.getInetAddresses())) {
                    if (!address.isLoopbackAddress()
                            && !address.isLinkLocalAddress()
                            && address.getAddress().length == 4) {
                        addresses.add(address.getHostAddress());
                    }
                }
            }
        } catch (SocketException e) {
        }
        return addresses;
    }

    private String info() {
        JsonArray addresses = new JsonArray();
        lanAddresses().forEach(addresses::add);
        JsonObject body = new JsonObject();
        body.addProperty("userPort", users.port());
        body.add("addresses", addresses);
        return GSON.toJson(body);
    }

    private synchronized String users() {
        JsonArray list = new JsonArray();
        long now = System.currentTimeMillis();
        List<String> relaying = relayAdmission().relaying(relay.openPeers());
        Map<String, JsonObject> byUsername = new LinkedHashMap<>();
        for (Session session : sessions.values()) {
            JsonObject user = byUsername.get(session.username);
            if (user == null) {
                user = new JsonObject();
                user.addProperty("username", session.username);
                user.addProperty("relayed", relaying.contains(session.username));
                user.add("sessions", new JsonArray());
                byUsername.put(session.username, user);
                list.add(user);
            }
            JsonObject login = new JsonObject();
            login.addProperty("id", session.id);
            login.addProperty("address", session.addressSaid());
            login.addProperty("state", session.state.name().toLowerCase(Locale.ROOT));
            login.addProperty("ageSeconds", (now - session.createdAtMillis) / 1000);
            user.getAsJsonArray("sessions").add(login);
        }
        JsonObject root = new JsonObject();
        root.add("users", list);
        root.add("relay", relayListed(now));
        return GSON.toJson(root);
    }

    private synchronized Session sessionById(String id) {
        try {
            return sessions.get(Integer.parseInt(id));
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private synchronized Response decide(String id, String verb) {
        Session found = sessionById(id);
        if (found == null) {
            return Response.error(404, "no login with id " + id);
        }
        switch (verb) {
            case "approve":
                found.state = State.APPROVED;
                break;
            case "deny":
                found.state = State.DENIED;
                break;
            case "revoke":
                found.state = State.REVOKED;
                break;
            default:
                return Response.error(404, "POST /admin/logins/<id>/approve|deny|revoke");
        }
        saveSessions();
        return Response.json(GSON.toJson(me(found)));
    }

    private synchronized Response deleteUser(String username) {
        if (username == null || username.isEmpty()) {
            return Response.error(400, "POST /admin/users/delete?username=<name>");
        }
        int logins = 0;
        for (Session session : sessions.values()) {
            if (session.username.equals(username)) {
                logins++;
            }
        }
        if (logins == 0) {
            return Response.error(404, "no user named " + username);
        }
        sessions.values().removeIf(session -> session.username.equals(username));
        saveSessions();
        JsonObject body = new JsonObject();
        body.addProperty("deleted", true);
        body.addProperty("username", username);
        body.addProperty("logins", logins);
        return Response.json(GSON.toJson(body));
    }

    private void loadSessions() {
        JsonObject stored = StateStore.load(stateDir.resolve(SESSIONS_FILE));
        if (stored == null) {
            return;
        }
        try {
            for (JsonElement element : stored.getAsJsonArray("sessions")) {
                Session session = Session.fromJson(element.getAsJsonObject());
                sessions.put(session.id, session);
                nextSessionId = Math.max(nextSessionId, session.id + 1);
            }
        } catch (RuntimeException e) {
            throw new IllegalStateException(
                    "could not read the sessions in " + stateDir.resolve(SESSIONS_FILE) + ": " + e, e);
        }
    }

    private void saveSessions() {
        JsonArray list = new JsonArray();
        for (Session session : sessions.values()) {
            list.add(session.toJson());
        }
        JsonObject body = new JsonObject();
        body.add("sessions", list);
        StateStore.save(stateDir.resolve(SESSIONS_FILE), body);
    }

    private static String page(String name) {
        try (InputStream in = RobotGateway.class.getResourceAsStream(name)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + name + " next to " + RobotGateway.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
