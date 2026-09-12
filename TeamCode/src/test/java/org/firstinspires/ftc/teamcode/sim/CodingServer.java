package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import org.bouncycastle.crypto.generators.SCrypt;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

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
import java.util.Base64;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;

/**
 * The coding server: teammates on the LAN log in with a username, the person at this machine
 * approves them from a page only this machine can reach, and approved teammates edit the files
 * the admin has picked, each in a git worktree of their own, with every edit written straight
 * to disk there.
 * <pre>
 * ./gradlew :TeamCode:codingServer
 *     admin  http://localhost:21987/admin     (loopback only)
 *     users  http://&lt;this machine's LAN address&gt;:21986/
 * </pre>
 * Approving a login makes the user's {@link Worktrees worktree}, one per username on its own
 * branch off {@code develop}, under the state directory. The host checkout is never written by a
 * user's save. The Simulate tab runs the autonomous op modes on the simulated robot through a
 * {@link SimBench} per worktree, one run at a time per user. Every run recompiles that worktree's
 * main sources and runs in a child JVM, so a saved edit is what the next run executes.
 * <p>
 * Sessions, the editable set, and the worktrees outlive the process. They live in the XDG state
 * directory ({@code $XDG_STATE_HOME/midnight-snackers/coding-server}, else
 * {@code ~/.local/state/midnight-snackers/coding-server}), readable by this user only. A
 * session's cookie is {@code <id>.<secret>}; the store holds a salted scrypt hash of the secret,
 * never the secret, so the file on disk cannot be replayed as a login. The editable set and the
 * worktrees are kept per project root, so two checkouts on one machine do not share them.
 */
public final class CodingServer {
    public static final String ADMIN_PORT_ENV = "CODING_ADMIN_PORT";
    public static final String USER_PORT_ENV = "CODING_USER_PORT";
    public static final int DEFAULT_ADMIN_PORT = 21987;
    public static final int DEFAULT_USER_PORT = 21986;
    public static final int MAX_PENDING_LOGINS = 20;
    public static final int MAX_USERNAME_LENGTH = 32;
    private static final Gson GSON = new GsonBuilder().serializeNulls().create();
    private static final String COOKIE = "session";
    /** What the pages load besides themselves: the editor, bundled so the host serves it without internet. */
    private static final Set<String> STATIC = Set.of("codemirror.js");
    private static final String SESSIONS_FILE = "sessions.json";
    private static final String EDITABLE_FILE = "editable.json";
    /** scrypt at 16 MiB and roughly 50 ms per guess: sized for a stolen store, not for the LAN. */
    private static final int SCRYPT_N = 1 << 14;
    private static final int SCRYPT_R = 8;
    private static final int SCRYPT_P = 1;
    private static final int SALT_BYTES = 16;
    private static final int HASH_BYTES = 32;
    private static final int SECRET_BYTES = 16;

    enum State { PENDING, APPROVED, DENIED, REVOKED }

    private static final class Session {
        final int id;
        final String username;
        final InetAddress address;
        final long createdAtMillis;
        final Secret secret;
        State state;
        /**
         * The cookie value this session has been shown to own, so the scrypt check runs once per
         * process. Null after a reload until the browser next presents it.
         */
        String token;
        /** The editable key this session last fetched, so others can see who has a file open. */
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
            return new Session(item.get("id").getAsInt(), item.get("username").getAsString(), parsed,
                    item.get("createdAtMillis").getAsLong(), Secret.fromJson(item.getAsJsonObject("secret")),
                    State.valueOf(item.get("state").getAsString().toUpperCase(Locale.ROOT)));
        }
    }

    /** A session secret at rest: its salted scrypt hash and the parameters that made it. */
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

        static Secret of(String secret, SecureRandom random) {
            byte[] salt = new byte[SALT_BYTES];
            random.nextBytes(salt);
            return new Secret(SCRYPT_N, SCRYPT_R, SCRYPT_P, salt, derive(secret, salt, SCRYPT_N, SCRYPT_R, SCRYPT_P, HASH_BYTES));
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
            return new Secret(item.get("n").getAsInt(), item.get("r").getAsInt(), item.get("p").getAsInt(),
                    Base64.getDecoder().decode(item.get("salt").getAsString()),
                    Base64.getDecoder().decode(item.get("hash").getAsString()));
        }
    }

    private final Path root;
    private final Path stateDir;
    private final Worktrees worktrees;
    private final SimBench.Factory benches;
    /** Each user's bench, made over their worktree on first use; by username. */
    private final Map<String, SimBench> benchByUsername = new LinkedHashMap<>();
    private final TinyHttpServer admin;
    private final TinyHttpServer users;
    private final SecureRandom random = new SecureRandom();
    /** By id, in id order. */
    private final Map<Integer, Session> sessions = new LinkedHashMap<>();
    private int nextSessionId = 1;
    /**
     * Root-relative paths with '/' separators, exactly as users must name them. A user-supplied
     * path is only ever looked up here by exact match, never resolved against the filesystem.
     */
    private final TreeSet<String> editable = new TreeSet<>();

    private CodingServer(Path root, SimBench.Factory benches, InetAddress adminBind, int adminPort, int userPort, Path stateDir) {
        this.root = root.toAbsolutePath().normalize();
        this.stateDir = stateDir.toAbsolutePath().normalize();
        this.worktrees = new Worktrees(this.root, this.stateDir, "git");
        this.benches = benches;
        loadSessions();
        loadEditable();
        this.admin = TinyHttpServer.start(adminBind, adminPort, "coding-admin", this::handleAdmin);
        this.users = TinyHttpServer.start(userPort, "coding-users", this::handleUser);
    }

    /**
     * @param root      the project checkout: a git repository with a {@code develop} branch
     * @param benches   makes the bench for a user's worktree, the first time that user builds or runs
     * @param adminBind the one address the admin listener answers on; {@link #main} always passes
     *                  loopback, and this is a parameter only so a test can prove the property
     * @param stateDir  where sessions, the editable set, and the worktrees are kept between runs;
     *                  created on demand
     * @throws IllegalStateException when a store under {@code stateDir} exists but cannot be read,
     *                               rather than starting over and silently logging everyone out;
     *                               or when git, the repository, or {@code develop} is missing
     */
    public static CodingServer start(Path root, SimBench.Factory benches, InetAddress adminBind, int adminPort, int userPort, Path stateDir) {
        return new CodingServer(root, benches, adminBind, adminPort, userPort, stateDir);
    }

    /**
     * The state directory by the XDG Base Directory convention: {@code $XDG_STATE_HOME} when it is
     * set to an absolute path, else {@code $HOME/.local/state}, then this project's own subdirectory.
     */
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

    /** Run from the repository root (the Gradle task does); replays land where the bench puts them. */
    public static void main(String[] args) throws InterruptedException {
        Path root = Path.of("").toAbsolutePath();
        SimBench.Factory benches = worktree -> new SimBench(null, worktree.resolve("TeamCode/src/main/java"),
                worktree.resolve("TeamCode").resolve(SimRunner.DEFAULT_OUTPUT_DIR),
                SimDevServer.DEFAULT_RUN_TIMEOUT_SECONDS, SimDevServer.DEFAULT_KILL_GRACE_SECONDS);
        CodingServer server = start(root, benches, InetAddress.getLoopbackAddress(),
                port(ADMIN_PORT_ENV, DEFAULT_ADMIN_PORT), port(USER_PORT_ENV, DEFAULT_USER_PORT), stateDir(System.getenv()));
        System.out.println("Coding server");
        System.out.println("  admin  " + server.adminUrl() + "admin   (this machine only)");
        System.out.println("  users  http://<this machine's LAN address>:" + server.userPort() + "/   (Ctrl-C to stop)");
        System.out.println("  state  " + server.stateDir);
        System.out.println("  trees  " + server.worktrees.directory());
        Thread.currentThread().join();
    }

    private static int port(String env, int fallback) {
        String value = System.getenv(env);
        return value == null || value.isBlank() ? fallback : Integer.parseInt(value.trim());
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

    /** The user's worktree, made if need be. */
    private Worktrees.Worktree worktreeOf(Session session) {
        return worktrees.ensure(session.username);
    }

    /** The user's bench, made over their worktree the first time they build or run. */
    private SimBench benchOf(Session session) {
        synchronized (benchByUsername) {
            SimBench bench = benchByUsername.get(session.username);
            if (bench == null) {
                bench = benches.create(worktreeOf(session).path);
                benchByUsername.put(session.username, bench);
            }
            return bench;
        }
    }

    // --- the user listener ---

    private Response handleUser(Request request) {
        Session session = sessionOf(request);
        if (request.path.equals("/")) {
            return Response.html(session != null && session.state == State.APPROVED ? page("dashboard.html") : page("login.html"));
        }
        if (request.path.equals("/login")) {
            return login(request);
        }
        if (request.path.startsWith("/static/")) {
            String name = request.path.substring("/static/".length());
            if (!STATIC.contains(name)) {
                return Response.error(404, "not found: " + request.path);
            }
            return new Response(200, "application/javascript; charset=utf-8", page(name));
        }
        if (request.path.equals("/me")) {
            return Response.json(GSON.toJson(me(session)));
        }
        if (request.path.equals("/files") || request.path.startsWith("/files/")) {
            if (session == null || session.state != State.APPROVED) {
                return Response.error(403, "not an approved session");
            }
            if (request.path.equals("/files")) {
                return Response.json(GSON.toJson(fileList(true)));
            }
            String key = request.path.substring("/files/".length());
            synchronized (this) {
                if (!editable.contains(key)) {
                    return Response.error(404, "not an editable file: " + key);
                }
                Path worktree;
                try {
                    worktree = worktreeOf(session).path;
                } catch (Worktrees.GitFailed e) {
                    return Response.error(500, "no worktree for " + session.username + ": " + e.getMessage());
                }
                if (request.method.equals("GET")) {
                    return read(session, worktree, key);
                }
                if (request.method.equals("PUT")) {
                    return write(worktree, key, request.body);
                }
            }
            return Response.error(405, "GET or PUT /files/<path>");
        }
        if (request.path.startsWith("/sim/")) {
            if (session == null || session.state != State.APPROVED) {
                return Response.error(403, "not an approved session");
            }
            try {
                return benchOf(session).handle(request.path.substring("/sim".length()), request, session.username);
            } catch (Worktrees.GitFailed e) {
                return Response.error(500, "no worktree for " + session.username + ": " + e.getMessage());
            }
        }
        if (request.path.startsWith("/git/")) {
            if (session == null || session.state != State.APPROVED) {
                return Response.error(403, "not an approved session");
            }
            try {
                return git(session, request);
            } catch (Worktrees.GitFailed e) {
                return Response.error(500, "git failed for " + session.username + ": " + e.getMessage());
            }
        }
        if (request.path.equals("/build")) {
            if (session == null || session.state != State.APPROVED) {
                return Response.error(403, "not an approved session");
            }
            try {
                return Response.json(GSON.toJson(buildCheck(session)));
            } catch (Worktrees.GitFailed e) {
                return Response.error(500, "no worktree for " + session.username + ": " + e.getMessage());
            }
        }
        return Response.error(404, "not found: " + request.path);
    }

    private synchronized JsonObject fileList(boolean withEditors) {
        JsonArray list = new JsonArray();
        for (String path : editable) {
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

    // --- read, write, conflict ---

    /** The file's text and version, or the response explaining why it has none. */
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

    private Current current(Path worktree, String key) {
        byte[] bytes;
        try {
            bytes = Files.readAllBytes(worktree.resolve(key));
        } catch (IOException e) {
            return new Current(Response.error(404, "could not read " + key + ": " + e.getMessage()));
        }
        try {
            String content = StandardCharsets.UTF_8.newDecoder()
                    .onMalformedInput(CodingErrorAction.REPORT)
                    .onUnmappableCharacter(CodingErrorAction.REPORT)
                    .decode(ByteBuffer.wrap(bytes)).toString();
            return new Current(content, version(bytes));
        } catch (CharacterCodingException e) {
            return new Current(Response.error(415, key + " is not UTF-8 text, so it cannot be edited here"));
        }
    }

    /** The version of a file is the SHA-256 of its bytes: stateless, and it notices edits made outside the server. */
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

    private Response read(Session session, Path worktree, String key) {
        Current current = current(worktree, key);
        if (current.problem != null) {
            return current.problem;
        }
        session.openFile = key;
        JsonObject body = new JsonObject();
        body.addProperty("path", key);
        body.addProperty("content", current.content);
        body.addProperty("version", current.version);
        return Response.json(GSON.toJson(body));
    }

    private Response write(Path worktree, String key, String requestBody) {
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
            body.addProperty("path", key);
            body.addProperty("content", current.content);
            body.addProperty("version", current.version);
            return Response.json(409, GSON.toJson(body));
        }
        byte[] bytes = edit.get("content").getAsString().getBytes(StandardCharsets.UTF_8);
        Path target = worktree.resolve(key);
        try {
            Path temp = Files.createTempFile(target.getParent(), "." + target.getFileName(), ".editing");
            try {
                Files.write(temp, bytes);
                Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
            } finally {
                Files.deleteIfExists(temp);
            }
        } catch (IOException e) {
            return Response.error(500, "could not write " + key + ": " + e.getMessage());
        }
        JsonObject body = new JsonObject();
        body.addProperty("path", key);
        body.addProperty("version", version(bytes));
        return Response.json(GSON.toJson(body));
    }

    private Response login(Request request) {
        if (!request.method.equals("POST")) {
            return Response.error(405, "POST /login?username=<name> to ask for access");
        }
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
            session = new Session(id, username, request.remoteAddress, System.currentTimeMillis(),
                    Secret.of(secret, random), State.PENDING);
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

    /**
     * The session a cookie proves, or null. The id names the session and the secret is checked
     * against its hash once per process; after that the cookie value itself is remembered.
     */
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

    // --- the user's branch: status, commit, pull, push ---

    private Response git(Session session, Request request) {
        String op = request.path.substring("/git/".length());
        if (op.equals("status")) {
            return Response.json(GSON.toJson(statusJson(worktrees.status(session.username))));
        }
        if (op.equals("commit")) {
            if (!request.method.equals("POST")) {
                return Response.error(405, "POST /git/commit with a JSON body naming the message");
            }
            JsonObject body;
            try {
                body = GSON.fromJson(request.body, JsonObject.class);
            } catch (RuntimeException e) {
                body = null;
            }
            String message = body == null || !body.has("message") || body.get("message").isJsonNull() ? "" : body.get("message").getAsString().trim();
            if (message.isEmpty()) {
                return Response.error(400, "a commit needs a message");
            }
            Worktrees.Commit commit;
            synchronized (this) {
                // under the server's lock, so a save in flight lands before or after, never inside
                commit = worktrees.commit(session.username, message);
            }
            JsonObject reply = new JsonObject();
            reply.addProperty("committed", commit.made);
            reply.addProperty("commit", commit.commit);
            reply.add("files", GSON.toJsonTree(commit.files));
            reply.addProperty("message", commit.made ? "committed " + commit.files.size() + (commit.files.size() == 1 ? " file" : " files") : "nothing to commit");
            return Response.json(GSON.toJson(reply));
        }
        return Response.error(404, "not found: " + request.path);
    }

    private static JsonObject statusJson(Worktrees.Status status) {
        JsonObject body = new JsonObject();
        body.addProperty("branch", status.branch);
        body.add("changed", GSON.toJsonTree(status.changed));
        body.addProperty("ahead", status.ahead);
        body.addProperty("behind", status.behind);
        return body;
    }

    /** The compile result of the user's sources as saved, with problems named by root-relative file. */
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
            String file = p.file.isEmpty() ? "" : worktree.relativize(sourceRoot.resolve(p.file)).toString().replace('\\', '/');
            problem.addProperty("file", file);
            problem.addProperty("line", p.line);
            problem.addProperty("message", p.message);
            problems.add(problem);
        }
        body.add("problems", problems);
        return body;
    }

    // --- the admin listener ---

    private Response handleAdmin(Request request) {
        if (request.path.equals("/") || request.path.equals("/admin")) {
            return Response.html(page("admin.html"));
        }
        if (request.path.equals("/admin/logins")) {
            return Response.json(logins());
        }
        if (request.path.equals("/admin/info")) {
            return Response.json(info());
        }
        if (request.path.startsWith("/admin/logins/")) {
            String[] parts = request.path.split("/");
            if (parts.length == 5 && request.method.equals("POST")) {
                return decide(parts[3], parts[4]);
            }
            return Response.error(404, "POST /admin/logins/<id>/approve|deny|revoke");
        }
        if (request.path.equals("/admin/tree")) {
            return tree(request.query("dir"));
        }
        if (request.path.equals("/admin/files")) {
            return Response.json(GSON.toJson(fileList(false)));
        }
        if (request.path.equals("/admin/files/add") || request.path.equals("/admin/files/remove")) {
            if (!request.method.equals("POST")) {
                return Response.error(405, "POST " + request.path + "?path=<root-relative path>");
            }
            return request.path.endsWith("add") ? addEditable(request.query("path")) : removeEditable(request.query("path"));
        }
        return Response.error(404, "not found: " + request.path);
    }

    // --- the editable set ---

    /**
     * The directory or file at a root-relative path, or null when the path is absolute, climbs
     * out of the root, follows a symlink out of it, or does not exist.
     */
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
        return root.relativize(path).toString().replace('\\', '/');
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
            return da != db ? (da ? -1 : 1) : a.getFileName().toString().compareTo(b.getFileName().toString());
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
        synchronized (this) {
            editable.add(keyOf(file));
            saveEditable();
        }
        return Response.json(GSON.toJson(fileList(false)));
    }

    private Response removeEditable(String key) {
        synchronized (this) {
            if (key == null || !editable.remove(key)) {
                return Response.error(404, "not an editable file: " + key);
            }
            saveEditable();
        }
        return Response.json(GSON.toJson(fileList(false)));
    }

    /** The user port and this machine's LAN addresses, so the admin can tell teammates where to go. */
    private String info() {
        JsonArray addresses = new JsonArray();
        try {
            for (NetworkInterface nic : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                for (InetAddress address : Collections.list(nic.getInetAddresses())) {
                    if (!address.isLoopbackAddress() && !address.isLinkLocalAddress() && address.getAddress().length == 4) {
                        addresses.add(address.getHostAddress());
                    }
                }
            }
        } catch (SocketException e) {
            // no addresses to offer; the admin can still find one by hand
        }
        JsonObject body = new JsonObject();
        body.addProperty("userPort", users.port());
        body.addProperty("root", root.toString());
        body.addProperty("worktreesDir", worktrees.directory().toString());
        body.add("addresses", addresses);
        return GSON.toJson(body);
    }

    private synchronized String logins() {
        JsonArray list = new JsonArray();
        long now = System.currentTimeMillis();
        for (Session session : sessions.values()) {
            JsonObject item = new JsonObject();
            item.addProperty("id", session.id);
            item.addProperty("username", session.username);
            item.addProperty("address", session.address == null ? "" : session.address.getHostAddress());
            item.addProperty("state", session.state.name().toLowerCase(Locale.ROOT));
            item.addProperty("ageSeconds", (now - session.createdAtMillis) / 1000);
            item.addProperty("file", session.openFile);
            Worktrees.Worktree worktree = worktrees.find(session.username);
            item.addProperty("worktree", worktree == null ? null : worktree.path.toString());
            item.addProperty("branch", worktree == null ? null : worktree.branch);
            list.add(item);
        }
        JsonObject root = new JsonObject();
        root.add("logins", list);
        return GSON.toJson(root);
    }

    private synchronized Response decide(String id, String verb) {
        Session found;
        try {
            found = sessions.get(Integer.parseInt(id));
        } catch (NumberFormatException e) {
            found = null;
        }
        if (found == null) {
            return Response.error(404, "no login with id " + id);
        }
        switch (verb) {
            case "approve":
                // the worktree is made here, in front of the admin, not at the user's first save
                try {
                    worktrees.ensure(found.username);
                } catch (Worktrees.GitFailed e) {
                    return Response.error(500, "could not make a worktree for " + found.username + ": " + e.getMessage());
                }
                found.state = State.APPROVED;
                break;
            case "deny": found.state = State.DENIED; break;
            case "revoke": found.state = State.REVOKED; break;
            default: return Response.error(404, "POST /admin/logins/<id>/approve|deny|revoke");
        }
        saveSessions();
        return Response.json(GSON.toJson(me(found)));
    }

    // --- what outlives the process ---

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
            throw new IllegalStateException("could not read the sessions in " + stateDir.resolve(SESSIONS_FILE) + ": " + e, e);
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

    /** The editable set is stored under this root's absolute path, next to any other checkout's. */
    private void loadEditable() {
        JsonObject stored = StateStore.load(stateDir.resolve(EDITABLE_FILE));
        if (stored == null) {
            return;
        }
        try {
            JsonElement ours = stored.getAsJsonObject("roots").get(root.toString());
            if (ours != null) {
                for (JsonElement element : ours.getAsJsonArray()) {
                    editable.add(element.getAsString());
                }
            }
        } catch (RuntimeException e) {
            throw new IllegalStateException("could not read the editable files in " + stateDir.resolve(EDITABLE_FILE) + ": " + e, e);
        }
    }

    private void saveEditable() {
        Path file = stateDir.resolve(EDITABLE_FILE);
        JsonObject stored = StateStore.load(file);
        JsonObject roots = stored == null || !stored.has("roots") ? new JsonObject() : stored.getAsJsonObject("roots");
        JsonArray ours = new JsonArray();
        for (String key : editable) {
            ours.add(key);
        }
        roots.add(root.toString(), ours);
        JsonObject body = new JsonObject();
        body.add("roots", roots);
        StateStore.save(file, body);
    }

    // --- pages ---

    private static String page(String name) {
        try (InputStream in = CodingServer.class.getResourceAsStream(name)) {
            if (in == null) {
                throw new IllegalStateException("missing resource " + name + " next to " + CodingServer.class.getName());
            }
            return new String(in.readAllBytes(), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
