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
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.charset.CharacterCodingException;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Base64;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Stream;
import org.bouncycastle.crypto.generators.SCrypt;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

public final class CodingServer {
    public static final String ADMIN_PORT_ENV = "CODING_ADMIN_PORT";
    public static final String USER_PORT_ENV = "CODING_USER_PORT";
    public static final int DEFAULT_ADMIN_PORT = 21987;
    public static final int DEFAULT_USER_PORT = 21986;
    public static final int MAX_PENDING_LOGINS = 20;
    public static final int MAX_USERNAME_LENGTH = 32;
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final String COOKIE = "session";

    private static final Set<String> STATIC = Set.of("codemirror.js");

    private static final String SESSIONS_FILE = "sessions.json";
    private static final String EDITABLE_FILE = "editable.json";
    private static final String ASSETS_DIR = "assets";

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

        String openFile;

        Session(int id, String username, InetAddress address, long createdAtMillis, Secret secret, State state) {
            this.id = id;
            this.username = username;
            this.address = address;
            this.createdAtMillis = createdAtMillis;
            this.secret = secret;
            this.state = state;
        }

        JsonObject toJson() {
            JsonObject item = new JsonObject();
            item.addProperty("id", id);
            item.addProperty("username", username);
            item.addProperty("address", address == null ? "" : address.getHostAddress());
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
            return SCrypt.generate(secret.getBytes(StandardCharsets.UTF_8), salt, n, r, p, length);
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

    private final Path root;
    private final Path stateDir;
    private final int scryptN;
    private final Worktrees worktrees;
    private final SimBench.Factory benches;

    private final Map<String, SimBench> benchByUsername = new LinkedHashMap<>();

    private final Map<String, JsonObject> lastMergeByUsername = new LinkedHashMap<>();

    private final Map<String, SourceNavigator> navigatorByUsername = new LinkedHashMap<>();

    private final Map<String, Router> benchRoutesByUsername = new LinkedHashMap<>();

    private final Router userRoutes = userRoutes();
    private final TinyHttpServer admin;
    private final TinyHttpServer users;
    private final SecureRandom random = new SecureRandom();

    private final Map<Integer, Session> sessions = new LinkedHashMap<>();

    private int nextSessionId = 1;

    private final EditableSet editable;

    private final Path assetsDir;
    private final Store assetStore = new OnDiskStore();
    private final Assets assets;

    public interface Assets {
        FieldAssets.Refreshed refresh(Store store, Path into, List<FieldAssets.Detail> details);
    }

    public static Assets fromOnshape() {
        return (store, into, details) -> FieldAssets.refresh(Onshape.configured(), store, into, details);
    }

    private CodingServer(
            Path root,
            SimBench.Factory benches,
            InetAddress adminBind,
            int adminPort,
            int userPort,
            Path stateDir,
            int scryptN,
            Assets assets) {
        this.scryptN = scryptN;
        this.assets = assets;
        this.root = root.toAbsolutePath().normalize();
        this.stateDir = stateDir.toAbsolutePath().normalize();
        this.worktrees = new Worktrees(this.root, this.stateDir, "git");
        this.benches = benches;
        loadSessions();
        this.editable = new EditableSet(this.root, stateDir.resolve(EDITABLE_FILE));
        this.assetsDir = this.stateDir.resolve(ASSETS_DIR);
        this.admin = TinyHttpServer.start(adminBind, adminPort, "coding-admin", adminRoutes());
        this.users = TinyHttpServer.start(userPort, "coding-users", this::handleUser);
    }

    public static CodingServer start(
            Path root, SimBench.Factory benches, InetAddress adminBind, int adminPort, int userPort, Path stateDir) {
        return start(root, benches, adminBind, adminPort, userPort, stateDir, SCRYPT_N);
    }

    static CodingServer start(
            Path root,
            SimBench.Factory benches,
            InetAddress adminBind,
            int adminPort,
            int userPort,
            Path stateDir,
            int scryptN) {
        return start(root, benches, adminBind, adminPort, userPort, stateDir, scryptN, fromOnshape());
    }

    static CodingServer start(
            Path root,
            SimBench.Factory benches,
            InetAddress adminBind,
            int adminPort,
            int userPort,
            Path stateDir,
            int scryptN,
            Assets assets) {
        JavaFormatter.check();
        return new CodingServer(root, benches, adminBind, adminPort, userPort, stateDir, scryptN, assets);
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
        Path root = Path.of("").toAbsolutePath();
        SimBench.Factory benches = worktree -> new SimBench(
                null,
                worktree,
                worktree.resolve("TeamCode").resolve(SimRunner.DEFAULT_OUTPUT_DIR),
                SimBench.DEFAULT_RUN_TIMEOUT_SECONDS,
                SimBench.DEFAULT_TELEOP_SECONDS,
                SimBench.DEFAULT_KILL_GRACE_SECONDS);
        Map<String, String> env = System.getenv();
        CodingServer server = start(
                root,
                benches,
                InetAddress.getLoopbackAddress(),
                port(env, ADMIN_PORT_ENV, DEFAULT_ADMIN_PORT),
                port(env, USER_PORT_ENV, DEFAULT_USER_PORT),
                stateDir(env));
        System.out.println("Coding server");
        System.out.println("  admin  " + server.adminUrl() + "admin   (this machine only)");
        System.out.println(
                "  users  http://<this machine's LAN address>:" + server.userPort() + "/   (Ctrl-C to stop)");
        System.out.println("  state  " + server.stateDir);
        System.out.println("  trees  " + server.worktrees.directory());
        if (server.hasFetchedAssets()) {
            System.out.println("  assets " + server.assetsDir);
        } else {
            server.refreshAssetsInTheBackground();
        }
        Thread.currentThread().join();
    }

    static int port(Map<String, String> env, String name, int fallback) {
        String value = env.get(name);
        if (value == null || value.isBlank()) {
            return fallback;
        }
        try {
            int port = Integer.parseInt(value.trim());
            if (port < 0 || port > 65535) {
                throw new NumberFormatException("out of range");
            }
            return port;
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(name + " must be a port number, not '" + value + "'", e);
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

    public void stop() {
        admin.stop();
        users.stop();
        synchronized (benchByUsername) {
            for (SimBench bench : benchByUsername.values()) {
                bench.stop();
            }
        }
    }

    private Worktrees.Worktree worktreeOf(Session session) {
        return worktrees.ensure(session.username);
    }

    private SimBench benchOf(Session session) {
        synchronized (benchByUsername) {
            SimBench bench = benchByUsername.get(session.username);
            if (bench == null) {
                bench = benches.create(worktreeOf(session).path);
                benchByUsername.put(session.username, bench);
                benchRoutesByUsername.put(session.username, bench.routes(session.username));
            }
            return bench;
        }
    }

    private Router benchRoutesOf(Session session) {
        synchronized (benchByUsername) {
            benchOf(session);
            return benchRoutesByUsername.get(session.username);
        }
    }

    private Router userRoutes() {
        Router approved = new Router()
                .guard(request ->
                        isApproved(sessionOf(request)) ? null : Response.error(403, "not an approved session"))
                .route("GET", "/files", (request, params) -> Response.json(GSON.toJson(fileList(true))))
                .route("GET", "/files/{key*}", (request, params) -> file(sessionOf(request), params.get("key"), null))
                .route(
                        "PUT",
                        "/files/{key*}",
                        (request, params) -> file(sessionOf(request), params.get("key"), request.body))
                .route("GET", "/sim/assets/{name*}", (request, params) -> asset(params.get("name")))
                .mount("/sim", request -> benchRoutesOf(sessionOf(request)))
                .route("GET", "/nav/{op}", (request, params) -> navigate(sessionOf(request), params.get("op"), request))
                .route("GET", "/source/{key*}", (request, params) -> source(sessionOf(request), params.get("key")))
                .route("GET", "/git/status", (request, params) -> gitStatus(sessionOf(request)))
                .route("POST", "/git/commit", (request, params) -> gitCommit(sessionOf(request), request.body))
                .route("POST", "/git/pull", (request, params) -> gitPull(sessionOf(request)))
                .route("POST", "/git/push", (request, params) -> gitPush(sessionOf(request)))
                .route("GET", "/build", (request, params) -> Response.json(GSON.toJson(buildCheck(sessionOf(request)))))
                .route(
                        "GET",
                        "/static/{name}",
                        (request, params) -> STATIC.contains(params.get("name"))
                                ? new Response(200, "application/javascript; charset=utf-8", page(params.get("name")))
                                : Response.error(404, "not found: " + request.path));
        return new Router()
                .route(
                        "GET",
                        "/",
                        (request, params) -> Response.html(
                                isApproved(sessionOf(request)) ? page("dashboard.html") : page("login.html")))
                .route("POST", "/login", (request, params) -> login(request))
                .route("GET", "/me", (request, params) -> Response.json(GSON.toJson(me(sessionOf(request)))))
                .mount("", approved);
    }

    private static boolean isApproved(Session session) {
        return session != null && session.state == State.APPROVED;
    }

    private Response handleUser(Request request) {
        try {
            return userRoutes.handle(request);
        } catch (Git.Failed e) {
            Session session = sessionOf(request);
            return Response.error(
                    500, "git failed for " + (session == null ? "?" : session.username) + ": " + e.getMessage());
        }
    }

    private Response file(Session session, String path, String edit) {
        synchronized (this) {
            Optional<Key> key = editable.lookUp(path);
            if (key.isEmpty()) {
                return Response.error(404, "not an editable file: " + path);
            }
            Path worktree = worktreeOf(session).path;
            return edit == null ? read(session, worktree, key.get()) : write(worktree, key.get(), edit);
        }
    }

    private synchronized JsonObject fileList(boolean withEditors) {
        JsonArray list = new JsonArray();
        for (Key key : editable.list()) {
            String path = key.path();
            JsonObject item = new JsonObject();
            item.addProperty("path", path);
            if (withEditors) {
                JsonArray editors = new JsonArray();
                for (Session session : sessions.values()) {
                    if (session.state == State.APPROVED && path.equals(session.openFile)) {
                        editors.add(session.username);
                    }
                }
                item.add("editors", editors);
            }
            list.add(item);
        }
        JsonObject body = new JsonObject();
        body.add("files", list);
        return body;
    }

    private static final class Current {
        final String content;
        final String version;
        final Response problem;

        Current(String content, String version) {
            this.content = content;
            this.version = version;
            this.problem = null;
        }

        Current(Response problem) {
            this.content = null;
            this.version = null;
            this.problem = problem;
        }
    }

    private Current current(Path worktree, Key key) {
        byte[] bytes;
        try {
            bytes = Files.readAllBytes(key.under(worktree));
        } catch (IOException e) {
            return new Current(Response.error(404, "could not read " + key + ": " + e.getMessage()));
        }
        try {
            return new Current(utf8(bytes), version(bytes));
        } catch (CharacterCodingException e) {
            return new Current(Response.error(415, key + " is not UTF-8 text, so it cannot be edited here"));
        }
    }

    private static String utf8(byte[] bytes) throws CharacterCodingException {
        return StandardCharsets.UTF_8
                .newDecoder()
                .onMalformedInput(CodingErrorAction.REPORT)
                .onUnmappableCharacter(CodingErrorAction.REPORT)
                .decode(ByteBuffer.wrap(bytes))
                .toString();
    }

    static String version(byte[] bytes) {
        try {
            StringBuilder hex = new StringBuilder();
            for (byte b : MessageDigest.getInstance("SHA-256").digest(bytes)) {
                hex.append(Character.forDigit((b >> 4) & 0xf, 16)).append(Character.forDigit(b & 0xf, 16));
            }
            return hex.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException(e);
        }
    }

    private Response read(Session session, Path worktree, Key key) {
        Current current = current(worktree, key);
        if (current.problem != null) {
            return current.problem;
        }
        session.openFile = key.path();
        JsonObject body = new JsonObject();
        body.addProperty("path", key.path());
        body.addProperty("content", current.content);
        body.addProperty("version", current.version);
        return Response.json(GSON.toJson(body));
    }

    private Response write(Path worktree, Key key, String requestBody) {
        JsonObject edit;
        try {
            edit = GSON.fromJson(requestBody, JsonObject.class);
        } catch (RuntimeException e) {
            edit = null;
        }
        if (edit == null || !edit.has("content") || !edit.has("baseVersion")) {
            return Response.error(400, "PUT a JSON body with content and baseVersion");
        }
        Current current = current(worktree, key);
        if (current.problem != null) {
            return current.problem;
        }
        if (!current.version.equals(edit.get("baseVersion").getAsString())) {
            JsonObject body = new JsonObject();
            body.addProperty("path", key.path());
            body.addProperty("content", current.content);
            body.addProperty("version", current.version);
            return Response.json(409, GSON.toJson(body));
        }
        byte[] bytes = edit.get("content").getAsString().getBytes(StandardCharsets.UTF_8);
        try {
            replace(key.under(worktree), bytes);
        } catch (IOException e) {
            return Response.error(500, "could not write " + key + ": " + e.getMessage());
        }
        JsonObject body = new JsonObject();
        body.addProperty("path", key.path());
        body.addProperty("version", version(bytes));
        return Response.json(GSON.toJson(body));
    }

    private static void replace(Path target, byte[] bytes) throws IOException {
        Path temp = Files.createTempFile(target.getParent(), "." + target.getFileName(), ".editing");
        try {
            Files.write(temp, bytes);
            Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
        } finally {
            Files.deleteIfExists(temp);
        }
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

    private JsonObject me(Session session) {
        JsonObject body = new JsonObject();
        if (session == null) {
            body.addProperty("state", "none");
            return body;
        }
        body.addProperty("state", session.state.name().toLowerCase(Locale.ROOT));
        body.addProperty("username", session.username);
        Worktrees.Worktree worktree = worktrees.find(session.username);
        if (session.state == State.APPROVED && worktree != null) {
            body.addProperty("branch", worktree.branch);
        }
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

    private static final class Sources {
        final Path worktree;

        final String prefix;

        final SourceNavigator navigator;

        Sources(Path worktree, String prefix, SourceNavigator navigator) {
            this.worktree = worktree;
            this.prefix = prefix;
            this.navigator = navigator;
        }

        String sourceOf(String path) {
            if (path == null || !path.startsWith(prefix)) {
                return null;
            }
            String source = path.substring(prefix.length());
            return navigator.files().contains(source) ? source : null;
        }

        Optional<Key> lookUp(String path) {
            return sourceOf(path) == null ? Optional.empty() : Key.under(worktree, path);
        }

        String keyOf(String source) {
            return source == null ? null : prefix + source;
        }
    }

    private Sources sourcesOf(Session session) {
        Path sourceRoot = benchOf(session).sourceRoot();
        if (sourceRoot == null) {
            return null;
        }
        Path worktree = worktreeOf(session).path;
        SourceNavigator navigator;
        synchronized (navigatorByUsername) {
            navigator = navigatorByUsername.get(session.username);
            if (navigator == null) {
                navigator = new SourceNavigator(sourceRoot);
                navigatorByUsername.put(session.username, navigator);
            }
        }
        return new Sources(worktree, worktree.relativize(sourceRoot).toString().replace('\\', '/') + "/", navigator);
    }

    private Response navigate(Session session, String op, Request request) {
        if (!op.equals("definition") && !op.equals("usages")) {
            return Response.error(404, "not found: " + request.path);
        }
        Sources sources = sourcesOf(session);
        if (sources == null) {
            JsonObject body = new JsonObject();
            body.addProperty("available", false);
            return Response.json(GSON.toJson(body));
        }
        int line;
        int column;
        try {
            line = Integer.parseInt(request.query("line"));
            column = Integer.parseInt(request.query("column"));
        } catch (RuntimeException e) {
            return Response.error(400, "line and column must be numbers");
        }
        String key = request.query("file");
        String source = sources.sourceOf(key);
        if (source == null) {
            return Response.error(404, "not a source file: " + key);
        }
        if (op.equals("definition")) {
            SourceNavigator.Symbol symbol = sources.navigator.definition(source, line, column);
            if (symbol == null) {
                return Response.error(404, "nothing at " + key + ":" + line + ":" + column);
            }
            JsonObject body = symbolJson(sources, symbol);
            JsonObject definition = locationJson(sources, symbol.definition);
            for (Map.Entry<String, JsonElement> entry : definition.entrySet()) {
                body.add(entry.getKey(), entry.getValue());
            }
            return Response.json(GSON.toJson(body));
        }
        SourceNavigator.Usages usages = sources.navigator.usages(source, line, column);
        if (usages == null) {
            return Response.error(404, "nothing at " + key + ":" + line + ":" + column);
        }
        JsonObject body = symbolJson(sources, usages.symbol);
        body.add(
                "definition",
                usages.symbol.definition == null ? null : locationJson(sources, usages.symbol.definition));
        JsonArray list = new JsonArray();
        for (SourceNavigator.Location usage : usages.usages) {
            list.add(locationJson(sources, usage));
        }
        body.add("usages", list);
        return Response.json(GSON.toJson(body));
    }

    private static JsonObject symbolJson(Sources sources, SourceNavigator.Symbol symbol) {
        JsonObject body = new JsonObject();
        body.addProperty("symbol", symbol.name);
        body.addProperty("kind", symbol.kind);
        return body;
    }

    private static JsonObject locationJson(Sources sources, SourceNavigator.Location location) {
        JsonObject body = new JsonObject();
        body.addProperty("file", location == null ? null : sources.keyOf(location.file));
        body.addProperty("line", location == null ? null : location.line);
        body.addProperty("column", location == null ? null : location.column);
        body.addProperty("text", location == null ? null : location.text);
        return body;
    }

    private Response source(Session session, String path) {
        Sources sources = sourcesOf(session);
        Optional<Key> key = sources == null ? Optional.empty() : sources.lookUp(path);
        if (key.isEmpty()) {
            return Response.error(404, "not a source file: " + path);
        }
        Current current = current(sources.worktree, key.get());
        if (current.problem != null) {
            return current.problem;
        }
        JsonObject body = new JsonObject();
        body.addProperty("path", key.get().path());
        body.addProperty("content", current.content);
        body.addProperty("version", current.version);
        synchronized (this) {
            body.addProperty("editable", editable.contains(key.get()));
        }
        return Response.json(GSON.toJson(body));
    }

    private Response gitStatus(Session session) {
        return Response.json(GSON.toJson(statusJson(worktrees.status(session.username))));
    }

    private Response gitCommit(Session session, String requestBody) {
        {
            JsonObject body;
            try {
                body = GSON.fromJson(requestBody, JsonObject.class);
            } catch (RuntimeException e) {
                body = null;
            }
            String message =
                    body == null || !body.has("message") || body.get("message").isJsonNull()
                            ? ""
                            : body.get("message").getAsString().trim();
            if (message.isEmpty()) {
                return Response.error(400, "a commit needs a message");
            }
            Worktrees.Commit commit;
            Formatting formatting;
            synchronized (this) {
                formatting = format(worktreeOf(session).path, worktrees.uncommitted(session.username));
                commit = worktrees.commit(session.username, message);
            }
            JsonObject reply = new JsonObject();
            reply.addProperty("committed", commit.made);
            reply.addProperty("commit", commit.commit);
            reply.add("files", GSON.toJsonTree(commit.files));
            reply.add("formatted", GSON.toJsonTree(formatting.formatted));
            reply.addProperty("warning", formatting.warning());
            reply.addProperty(
                    "message",
                    commit.made
                            ? "committed " + count(commit.files.size(), "file") + formatting.said()
                            : "nothing to commit");
            return Response.json(GSON.toJson(reply));
        }
    }

    private static String count(int n, String noun) {
        return n + " " + noun + (n == 1 ? "" : "s");
    }

    private static final class Formatting {
        final List<String> formatted = new ArrayList<>();

        final List<String> refused = new ArrayList<>();

        String said() {
            return formatted.isEmpty() ? "" : ", formatted " + count(formatted.size(), "file");
        }

        String warning() {
            return refused.isEmpty()
                    ? null
                    : "could not format " + String.join(", ", refused) + "; committed as you wrote "
                            + (refused.size() == 1 ? "it" : "them");
        }
    }

    private static Formatting format(Path worktree, List<String> uncommitted) {
        Formatting formatting = new Formatting();
        for (String named : uncommitted) {
            Optional<Key> found = Key.under(worktree, named);
            if (found.isEmpty() || !found.get().isJava()) {
                continue;
            }
            Key key = found.get();
            Path file = key.under(worktree);
            if (!Files.isRegularFile(file)) {
                continue;
            }
            try {
                byte[] before = Files.readAllBytes(file);
                byte[] after = JavaFormatter.format(utf8(before)).getBytes(StandardCharsets.UTF_8);
                if (!Arrays.equals(before, after)) {
                    replace(file, after);
                    formatting.formatted.add(key.path());
                }
            } catch (JavaFormatter.Unparseable e) {
                formatting.refused.add(key + " (" + e.getMessage() + ")");
            } catch (CharacterCodingException e) {
                formatting.refused.add(key + " (it is not UTF-8 text)");
            } catch (IOException e) {
                formatting.refused.add(key + " (" + e.getMessage() + ")");
            }
        }
        return formatting;
    }

    private Response gitPull(Session session) {
        Worktrees.Merge merge;
        synchronized (this) {
            merge = worktrees.pull(session.username);
        }
        return reply(merge, MergeReport.Op.PULL, session.username, MergeReport.Voice.USER);
    }

    private Response adminPull(String id) {
        Session found = sessionById(id);
        if (found == null) {
            return Response.error(404, "no login with id " + id);
        }
        if (worktrees.find(found.username) == null) {
            return Response.error(404, found.username + " has no worktree yet: approve the login first");
        }
        Worktrees.Merge merge;
        try {
            synchronized (this) {
                merge = worktrees.pull(found.username);
            }
        } catch (Git.Failed e) {
            return Response.error(500, "git failed for " + found.username + ": " + e.getMessage());
        }
        return reply(merge, MergeReport.Op.PULL, found.username, MergeReport.Voice.ADMIN);
    }

    private Response gitPush(Session session) {
        Worktrees.Merge merge;
        synchronized (this) {
            merge = worktrees.push(session.username);
        }
        return reply(merge, MergeReport.Op.PUSH, session.username, MergeReport.Voice.USER);
    }

    private Response reply(Worktrees.Merge merge, MergeReport.Op op, String username, MergeReport.Voice voice) {
        Worktrees.Worktree worktree = worktrees.find(username);
        MergeReport report =
                MergeReport.of(merge, op, username, voice, worktree == null ? null : worktree.path.toString());
        synchronized (this) {
            lastMergeByUsername.put(username, report.record(System.currentTimeMillis()));
        }
        return Response.json(report.status(), GSON.toJson(report.json()));
    }

    private static JsonObject statusJson(Worktrees.Status status) {
        JsonObject body = new JsonObject();
        body.addProperty("branch", status.branch);
        body.add("changed", GSON.toJsonTree(status.changed));
        body.addProperty("ahead", status.ahead);
        body.addProperty("behind", status.behind);
        body.addProperty("head", status.head);

        body.addProperty("pushable", status.pushable());
        return body;
    }

    private JsonObject buildCheck(Session session) {
        JsonObject body = new JsonObject();
        SimBench bench = benchOf(session);
        Path worktree = worktreeOf(session).path;
        SimBuild.Result result;
        try {
            result = bench.check();
        } catch (RuntimeException e) {
            body.addProperty("available", true);
            body.addProperty("ok", false);
            JsonArray problems = new JsonArray();
            JsonObject problem = new JsonObject();
            problem.addProperty("file", "");
            problem.addProperty("line", 0);
            problem.addProperty("message", e.getMessage());
            problems.add(problem);
            body.add("problems", problems);
            return body;
        }
        if (result == null) {
            body.addProperty("available", false);
            return body;
        }
        body.addProperty("available", true);
        body.addProperty("ok", result.classes != null);
        JsonArray problems = new JsonArray();
        Path sourceRoot = bench.sourceRoot();
        for (SimBuild.Problem p : result.problems) {
            JsonObject problem = new JsonObject();
            String file = p.file.isEmpty()
                    ? ""
                    : worktree.relativize(sourceRoot.resolve(p.file)).toString().replace('\\', '/');
            problem.addProperty("file", file);
            problem.addProperty("line", p.line);
            problem.addProperty("message", p.message);
            problems.add(problem);
        }
        body.add("problems", problems);
        return body;
    }

    private Router adminRoutes() {
        return new Router()
                .route("GET", "/", (request, params) -> Response.html(page("admin.html")))
                .route("GET", "/admin", (request, params) -> Response.html(page("admin.html")))
                .route("GET", "/admin/users", (request, params) -> Response.json(users()))
                .route(
                        "POST",
                        "/admin/users/delete",
                        (request, params) ->
                                deleteUser(request.query("username"), "true".equals(request.query("force"))))
                .route("GET", "/admin/info", (request, params) -> Response.json(info()))
                .route("GET", "/admin/assets", (request, params) -> Response.json(GSON.toJson(assetsFetched())))
                .route("POST", "/admin/assets/refresh", (request, params) -> refreshAssets(request.query("detail")))
                .route("POST", "/admin/logins/{id}/pull", (request, params) -> adminPull(params.get("id")))
                .route(
                        "POST",
                        "/admin/logins/{id}/{decision}",
                        (request, params) -> decide(params.get("id"), params.get("decision")))
                .route("GET", "/admin/tree", (request, params) -> tree(request.query("dir")))
                .route("GET", "/admin/files", (request, params) -> Response.json(GSON.toJson(fileList(false))))
                .route("POST", "/admin/files/add", (request, params) -> addEditable(request.query("path")))
                .route("POST", "/admin/files/remove", (request, params) -> removeEditable(request.query("path")));
    }

    private Response asset(String name) {
        Response fetched = SimAssets.serveUnder(assetsDir, assetStore, name);
        return fetched.status == 200 ? fetched : SimAssets.serve(name);
    }

    private JsonObject assetsFetched() {
        JsonObject out = new JsonObject();
        out.addProperty("under", assetsDir.toString());
        JsonObject held = new JsonObject();
        for (String name : FieldAssets.everyAsset()) {
            assetStore.readIfThere(assetsDir.resolve(name)).ifPresent(bytes -> held.addProperty(name, bytes.length));
        }
        out.add("fetched", held);
        out.addProperty("complete", held.size() == FieldAssets.everyAsset().size());
        out.addProperty(
                "drawing",
                held.has(FieldAssets.FIELD_GLB) ? "the model this server fetched" : "the model committed for tests");
        JsonObject details = new JsonObject();
        for (FieldAssets.Detail one : FieldAssets.Detail.values()) {
            details.addProperty(one.asked, assetStore.isFile(assetsDir.resolve(one.file)));
        }
        out.add("detail", details);
        return out;
    }

    private Response refreshAssets(String detail) {
        List<FieldAssets.Detail> details;
        try {
            details = FieldAssets.detailsNamed(detail);
        } catch (FieldAssets.NotAnAsset wrong) {
            return Response.error(400, wrong.getMessage());
        }
        try {
            FieldAssets.Refreshed refreshed = assets.refresh(assetStore, assetsDir, details);
            JsonObject out = assetsFetched();
            out.addProperty("refreshed", refreshed.written.size());
            out.addProperty("bytes", refreshed.bytes());
            return Response.json(GSON.toJson(out));
        } catch (Onshape.NoCredentials refused) {
            return Response.error(502, "Onshape would not answer: " + refused.getMessage());
        } catch (RuntimeException wrong) {
            return Response.error(502, "the assets could not be refreshed: " + wrong.getMessage());
        }
    }

    boolean hasFetchedAssets() {
        return assetStore.isFile(assetsDir.resolve(FieldAssets.FIELD_GLB));
    }

    void refreshAssetsInTheBackground() {
        Thread fetching = new Thread(
                () -> {
                    try {
                        System.out.println("  assets fetching from Onshape into " + assetsDir);
                        System.out.println("  assets "
                                + assets.refresh(assetStore, assetsDir, List.of(FieldAssets.Detail.NORMAL)));
                    } catch (RuntimeException wrong) {
                        System.out.println("  assets not fetched (" + wrong.getMessage()
                                + "); the pages draw the model committed for tests");
                    }
                },
                "assets-refresh");
        fetching.setDaemon(true);
        fetching.start();
    }

    private Path underRoot(String relative) {
        if (relative == null) {
            relative = "";
        }
        if (relative.startsWith("/") || relative.startsWith("\\") || relative.contains(":")) {
            return null;
        }
        Path path = root.resolve(relative).normalize();
        if (!path.startsWith(root) || !Files.exists(path)) {
            return null;
        }
        try {
            if (!path.toRealPath().startsWith(root.toRealPath())) {
                return null;
            }
        } catch (IOException e) {
            return null;
        }
        return path;
    }

    private String keyOf(Path path) {
        return Key.of(root, path).path();
    }

    private Response tree(String dir) {
        Path directory = underRoot(dir);
        if (directory == null || !Files.isDirectory(directory)) {
            return Response.error(400, "not a directory under the project root: " + dir);
        }
        List<Path> children = new ArrayList<>();
        try (Stream<Path> listing = Files.list(directory)) {
            listing.forEach(children::add);
        } catch (IOException e) {
            return Response.error(400, "could not list " + dir + ": " + e.getMessage());
        }
        children.sort((a, b) -> {
            boolean da = Files.isDirectory(a);
            boolean db = Files.isDirectory(b);
            return da != db
                    ? (da ? -1 : 1)
                    : a.getFileName().toString().compareTo(b.getFileName().toString());
        });
        JsonArray entries = new JsonArray();
        for (Path child : children) {
            JsonObject item = new JsonObject();
            item.addProperty("name", child.getFileName().toString());
            item.addProperty("type", Files.isDirectory(child) ? "dir" : "file");
            item.addProperty("path", keyOf(child));
            entries.add(item);
        }
        JsonObject body = new JsonObject();
        body.addProperty("dir", directory.equals(root) ? "" : keyOf(directory));
        body.add("entries", entries);
        return Response.json(GSON.toJson(body));
    }

    private Response addEditable(String relative) {
        Path file = underRoot(relative);
        if (file == null || !Files.isRegularFile(file)) {
            return Response.error(400, "not a file under the project root: " + relative);
        }
        editable.add(Key.of(root, file));
        return Response.json(GSON.toJson(fileList(false)));
    }

    private Response removeEditable(String path) {
        if (path == null || !editable.remove(path)) {
            return Response.error(404, "not an editable file: " + path);
        }
        return Response.json(GSON.toJson(fileList(false)));
    }

    private String info() {
        JsonArray addresses = new JsonArray();
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
        JsonObject body = new JsonObject();
        body.addProperty("userPort", users.port());
        body.addProperty("root", root.toString());
        body.addProperty("worktreesDir", worktrees.directory().toString());
        body.add("addresses", addresses);
        return GSON.toJson(body);
    }

    private synchronized String users() {
        JsonArray list = new JsonArray();
        long now = System.currentTimeMillis();
        Map<String, JsonObject> byUsername = new LinkedHashMap<>();
        for (Session session : sessions.values()) {
            JsonObject user = byUsername.get(session.username);
            if (user == null) {
                user = new JsonObject();
                user.addProperty("username", session.username);
                Worktrees.Worktree worktree = worktrees.find(session.username);
                user.addProperty("worktree", worktree == null ? null : worktree.path.toString());
                user.addProperty("branch", worktree == null ? null : worktree.branch);
                JsonElement status = JsonNull.INSTANCE;
                String statusError = null;

                boolean deletable = worktree == null;
                if (worktree != null) {
                    try {
                        status = statusJson(worktrees.status(session.username));
                        deletable = worktrees.unsaved(session.username).none();
                    } catch (Git.Failed e) {
                        statusError = e.getMessage();
                    }
                }
                user.add("status", status);
                user.addProperty("statusError", statusError);
                user.addProperty("deletable", deletable);
                user.add("lastMerge", lastMergeByUsername.get(session.username));
                user.add("sessions", new JsonArray());
                byUsername.put(session.username, user);
                list.add(user);
            }
            JsonObject login = new JsonObject();
            login.addProperty("id", session.id);
            login.addProperty("address", session.address == null ? "" : session.address.getHostAddress());
            login.addProperty("state", session.state.name().toLowerCase(Locale.ROOT));
            login.addProperty("ageSeconds", (now - session.createdAtMillis) / 1000);
            login.addProperty("file", session.openFile);
            user.getAsJsonArray("sessions").add(login);
        }
        JsonObject root = new JsonObject();
        root.add("users", list);
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
                try {
                    worktrees.ensure(found.username);
                } catch (Git.Failed e) {
                    return Response.error(
                            500, "could not make a worktree for " + found.username + ": " + e.getMessage());
                }
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

    private Response deleteUser(String username, boolean force) {
        if (username == null || username.isEmpty()) {
            return Response.error(400, "POST /admin/users/delete?username=<name>");
        }
        int logins = sessionCountOf(username);
        if (logins == 0) {
            return Response.error(404, "no user named " + username);
        }
        Worktrees.Worktree worktree = worktrees.find(username);
        try {
            if (!force) {
                Worktrees.Unsaved unsaved = worktrees.unsaved(username);
                if (!unsaved.none()) {
                    return refusal(username, unsaved);
                }
            }
            stopBench(username);
            synchronized (navigatorByUsername) {
                navigatorByUsername.remove(username);
            }

            Worktrees.Removal removal = worktrees.remove(username, force);
            if (removal.refused != null) {
                return refusal(username, removal.refused);
            }
        } catch (Git.Failed e) {
            return Response.error(500, "git failed for " + username + ": " + e.getMessage());
        }
        synchronized (this) {
            lastMergeByUsername.remove(username);
            sessions.values().removeIf(session -> session.username.equals(username));
            saveSessions();
        }
        JsonObject body = new JsonObject();
        body.addProperty("deleted", true);
        body.addProperty("username", username);
        body.addProperty("branch", worktree == null ? null : worktree.branch);
        body.addProperty("logins", logins);
        return Response.json(GSON.toJson(body));
    }

    private synchronized int sessionCountOf(String username) {
        int count = 0;
        for (Session session : sessions.values()) {
            if (session.username.equals(username)) {
                count++;
            }
        }
        return count;
    }

    private void stopBench(String username) {
        synchronized (benchByUsername) {
            SimBench bench = benchByUsername.remove(username);
            benchRoutesByUsername.remove(username);
            if (bench != null) {
                bench.stop();
            }
        }
    }

    private static Response refusal(String username, Worktrees.Unsaved unsaved) {
        List<String> parts = new ArrayList<>();
        if (!unsaved.changed.isEmpty()) {
            parts.add(plural(unsaved.changed.size(), "changed file"));
        }
        if (unsaved.ahead > 0) {
            parts.add(plural(unsaved.ahead, "commit"));
        }
        JsonObject body = new JsonObject();
        body.addProperty(
                "message",
                username + " has " + String.join(" and ", parts) + " that " + Worktrees.DEVELOP
                        + " does not have; push " + (unsaved.changed.size() + unsaved.ahead == 1 ? "it" : "them")
                        + " first, or delete anyway");
        body.add("changed", GSON.toJsonTree(unsaved.changed));
        body.addProperty("ahead", unsaved.ahead);
        return Response.json(409, GSON.toJson(body));
    }

    private static String plural(int count, String one) {
        return count + " " + (count == 1 ? one : one + "s");
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
        try (InputStream in = CodingServer.class.getResourceAsStream(name)) {
            if (in == null) {
                throw new IllegalStateException(
                        "missing resource " + name + " next to " + CodingServer.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
