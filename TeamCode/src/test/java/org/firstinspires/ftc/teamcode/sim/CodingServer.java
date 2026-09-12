package org.firstinspires.ftc.teamcode.sim;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Request;
import org.firstinspires.ftc.teamcode.sim.TinyHttpServer.Response;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
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
import java.util.TreeSet;
import java.util.stream.Stream;

/**
 * The coding server: teammates on the LAN log in with a username, the person at this machine
 * approves them from a page only this machine can reach, and approved teammates edit the files
 * the admin has picked, with every edit written straight to disk.
 * <pre>
 * ./gradlew :TeamCode:codingServer
 *     admin  http://localhost:21987/admin     (loopback only)
 *     users  http://&lt;this machine's LAN address&gt;:21986/
 * </pre>
 * The Simulate tab runs the autonomous op modes on the simulated robot through the same
 * {@link SimBench} as the bench, one run at a time for everyone. Every run recompiles the main
 * sources and runs in a child JVM, so a saved edit is what the next run executes.
 * Sessions live in memory: restarting the server logs everyone out, and no token is ever written
 * to disk.
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

    enum State { PENDING, APPROVED, DENIED, REVOKED }

    private static final class Session {
        final int id;
        final String token;
        final String username;
        final InetAddress address;
        final long createdAtMillis = System.currentTimeMillis();
        State state = State.PENDING;
        /** The editable key this session last fetched, so others can see who has a file open. */
        String openFile;

        Session(int id, String token, String username, InetAddress address) {
            this.id = id;
            this.token = token;
            this.username = username;
            this.address = address;
        }
    }

    private final Path root;
    private final SimBench bench;
    private final TinyHttpServer admin;
    private final TinyHttpServer users;
    private final SecureRandom random = new SecureRandom();
    private final Map<String, Session> sessions = new LinkedHashMap<>();
    private int nextSessionId = 1;
    /**
     * Root-relative paths with '/' separators, exactly as users must name them. A user-supplied
     * path is only ever looked up here by exact match, never resolved against the filesystem.
     */
    private final TreeSet<String> editable = new TreeSet<>();

    private CodingServer(Path root, SimBench bench, InetAddress adminBind, int adminPort, int userPort) {
        this.root = root.toAbsolutePath().normalize();
        this.bench = bench;
        this.admin = TinyHttpServer.start(adminBind, adminPort, "coding-admin", this::handleAdmin);
        this.users = TinyHttpServer.start(userPort, "coding-users", this::handleUser);
    }

    /**
     * @param adminBind the one address the admin listener answers on; {@link #main} always passes
     *                  loopback, and this is a parameter only so a test can prove the property
     */
    public static CodingServer start(Path root, SimBench bench, InetAddress adminBind, int adminPort, int userPort) {
        return new CodingServer(root, bench, adminBind, adminPort, userPort);
    }

    /** Run from the repository root (the Gradle task does); replays land where the bench puts them. */
    public static void main(String[] args) throws InterruptedException {
        Path root = Path.of("").toAbsolutePath();
        SimBench bench = new SimBench(null, root.resolve("TeamCode/src/main/java"),
                root.resolve("TeamCode").resolve(SimRunner.DEFAULT_OUTPUT_DIR),
                SimDevServer.DEFAULT_RUN_TIMEOUT_SECONDS, SimDevServer.DEFAULT_KILL_GRACE_SECONDS);
        CodingServer server = start(root, bench, InetAddress.getLoopbackAddress(),
                port(ADMIN_PORT_ENV, DEFAULT_ADMIN_PORT), port(USER_PORT_ENV, DEFAULT_USER_PORT));
        System.out.println("Coding server");
        System.out.println("  admin  " + server.adminUrl() + "admin   (this machine only)");
        System.out.println("  users  http://<this machine's LAN address>:" + server.userPort() + "/   (Ctrl-C to stop)");
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
        bench.stop();
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
                if (request.method.equals("GET")) {
                    return read(session, key);
                }
                if (request.method.equals("PUT")) {
                    return write(key, request.body);
                }
            }
            return Response.error(405, "GET or PUT /files/<path>");
        }
        if (request.path.startsWith("/sim/")) {
            if (session == null || session.state != State.APPROVED) {
                return Response.error(403, "not an approved session");
            }
            return bench.handle(request.path.substring("/sim".length()), request, session.username);
        }
        if (request.path.equals("/build")) {
            if (session == null || session.state != State.APPROVED) {
                return Response.error(403, "not an approved session");
            }
            return Response.json(GSON.toJson(buildCheck()));
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

    private Current current(String key) {
        byte[] bytes;
        try {
            bytes = Files.readAllBytes(root.resolve(key));
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

    private Response read(Session session, String key) {
        Current current = current(key);
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

    private Response write(String key, String requestBody) {
        JsonObject edit;
        try {
            edit = GSON.fromJson(requestBody, JsonObject.class);
        } catch (RuntimeException e) {
            edit = null;
        }
        if (edit == null || !edit.has("content") || !edit.has("baseVersion")) {
            return Response.error(400, "PUT a JSON body with content and baseVersion");
        }
        Current current = current(key);
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
        Path target = root.resolve(key);
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
            byte[] bytes = new byte[16];
            random.nextBytes(bytes);
            String token = Base64.getUrlEncoder().withoutPadding().encodeToString(bytes);
            session = new Session(nextSessionId++, token, username, request.remoteAddress);
            sessions.put(token, session);
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
        return token == null ? null : sessions.get(token);
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

    /** The compile result of the sources as saved, with problems named by root-relative file. */
    private JsonObject buildCheck() {
        JsonObject body = new JsonObject();
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
            String file = p.file.isEmpty() ? "" : root.relativize(sourceRoot.resolve(p.file)).toString().replace('\\', '/');
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
        }
        return Response.json(GSON.toJson(fileList(false)));
    }

    private Response removeEditable(String key) {
        synchronized (this) {
            if (key == null || !editable.remove(key)) {
                return Response.error(404, "not an editable file: " + key);
            }
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
            list.add(item);
        }
        JsonObject root = new JsonObject();
        root.add("logins", list);
        return GSON.toJson(root);
    }

    private synchronized Response decide(String id, String verb) {
        Session found = null;
        for (Session session : sessions.values()) {
            if (String.valueOf(session.id).equals(id)) {
                found = session;
            }
        }
        if (found == null) {
            return Response.error(404, "no login with id " + id);
        }
        switch (verb) {
            case "approve": found.state = State.APPROVED; break;
            case "deny": found.state = State.DENIED; break;
            case "revoke": found.state = State.REVOKED; break;
            default: return Response.error(404, "POST /admin/logins/<id>/approve|deny|revoke");
        }
        return Response.json(GSON.toJson(me(found)));
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
