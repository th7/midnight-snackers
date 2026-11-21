package org.firstinspires.ftc.teamcode.sim;

import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.simcore.HubState;
import org.firstinspires.ftc.teamcode.simcore.Peer;
import org.firstinspires.ftc.teamcode.simcore.RelayAdmission;

final class RobotRelay {
    static final String PORT_ENV = "ROBOT_RELAY_PORT";
    static final String HUB_ENV = "ROBOT_HUB_ADDRESS";
    static final String DEFAULT_HUB = "192.168.43.1:" + RelayAdmission.ADB_PORT;
    static final int HUB_CONNECT_TIMEOUT_MILLIS = 3_000;

    private static final int BUFFER_BYTES = 16 * 1024;

    private final ServerSocket socket;
    private final InetSocketAddress hub;
    private final Supplier<RelayAdmission> admission;
    private final Object admitting = new Object();
    private final Set<Relayed> open = ConcurrentHashMap.newKeySet();
    private final AtomicBoolean probing = new AtomicBoolean();
    private volatile HubState hubState = HubState.untried();

    private RobotRelay(ServerSocket socket, InetSocketAddress hub, Supplier<RelayAdmission> admission) {
        this.socket = socket;
        this.hub = hub;
        this.admission = admission;
    }

    static RobotRelay start(InetAddress bind, int port, InetSocketAddress hub, Supplier<RelayAdmission> admission) {
        try {
            RobotRelay relay = new RobotRelay(new ServerSocket(port, 50, bind), hub, admission);
            daemon("robot-relay", relay::acceptLoop);
            return relay;
        } catch (IOException e) {
            throw new UncheckedIOException(
                    "could not listen for the robot relay on port " + port + "; set " + PORT_ENV + " to a free port",
                    e);
        }
    }

    static int port(Map<String, String> env) {
        return CodingServer.port(env, PORT_ENV, RelayAdmission.ADB_PORT);
    }

    static InetSocketAddress hub(Map<String, String> env) {
        String value = env.getOrDefault(HUB_ENV, "");
        String written = value.isBlank() ? DEFAULT_HUB : value.trim();
        int colon = written.lastIndexOf(':');
        String host = colon <= 0 ? "" : written.substring(0, colon);
        if (host.startsWith("[") && host.endsWith("]")) {
            host = host.substring(1, host.length() - 1);
        } else if (host.contains(":")) {
            host = "";
        }
        int port;
        try {
            port = colon <= 0 ? 0 : Integer.parseInt(written.substring(colon + 1));
        } catch (NumberFormatException e) {
            port = 0;
        }
        if (host.isEmpty() || port < 1 || port > 65535) {
            throw new IllegalArgumentException(
                    HUB_ENV + " must be host:port, such as " + DEFAULT_HUB + ", not '" + value + "'");
        }
        return new InetSocketAddress(host, port);
    }

    int port() {
        return socket.getLocalPort();
    }

    InetAddress bindAddress() {
        return socket.getInetAddress();
    }

    String hubAddress() {
        String host = hub.getAddress() == null
                ? hub.getHostString()
                : hub.getAddress().getHostAddress();
        return (host.contains(":") ? "[" + host + "]" : host) + ":" + hub.getPort();
    }

    HubState hubState() {
        return hubState;
    }

    List<Peer> openPeers() {
        List<Peer> peers = new ArrayList<>();
        for (Relayed relayed : open) {
            peers.add(relayed.peer);
        }
        return peers;
    }

    void sweep() {
        synchronized (admitting) {
            List<Peer> closing = admission.get().toClose(openPeers());
            for (Relayed relayed : open) {
                if (closing.contains(relayed.peer)) {
                    relayed.close();
                }
            }
        }
    }

    void probeIfDue() {
        if (hubState.probeDue(System.currentTimeMillis()) && probing.compareAndSet(false, true)) {
            daemon("robot-relay-probe", () -> {
                try (Socket probe = toHub()) {
                    probe.getInetAddress();
                } catch (IOException e) {
                } finally {
                    probing.set(false);
                }
            });
        }
    }

    void stop() {
        closeQuietly(socket);
        for (Relayed relayed : open) {
            relayed.close();
        }
    }

    private void acceptLoop() {
        while (!socket.isClosed()) {
            try {
                Socket client = socket.accept();
                daemon("robot-relay-client", () -> relay(client));
            } catch (SocketException closed) {
                return;
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
        }
    }

    private void relay(Socket client) {
        Relayed relayed;
        synchronized (admitting) {
            Optional<Relayed> admitted = Peer.of(client.getInetAddress().getHostAddress())
                    .then(from -> admission.get().admit(from).map(users -> new Relayed(from, client)))
                    .fold(Optional::of, rule -> refuse(client, rule));
            if (admitted.isEmpty()) {
                return;
            }
            relayed = admitted.get();
            open.add(relayed);
        }
        Socket robot;
        try {
            robot = toHub();
        } catch (IOException e) {
            relayed.close();
            return;
        }
        if (relayed.attach(robot)) {
            daemon("robot-relay-up", () -> pump(relayed, client, robot));
            daemon("robot-relay-down", () -> pump(relayed, robot, client));
        }
    }

    private static Optional<Relayed> refuse(Socket client, String rule) {
        System.out.println("robot relay: refused " + client.getInetAddress().getHostAddress() + ": " + rule);
        closeQuietly(client);
        return Optional.empty();
    }

    private Socket toHub() throws IOException {
        Socket robot = new Socket();
        try {
            robot.connect(hub, HUB_CONNECT_TIMEOUT_MILLIS);
        } catch (IOException e) {
            closeQuietly(robot);
            hubState = HubState.unreached(System.currentTimeMillis(), why(e));
            throw e;
        }
        hubState = HubState.reached(System.currentTimeMillis());
        return robot;
    }

    private static String why(IOException e) {
        return e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage();
    }

    private static void pump(Relayed relayed, Socket from, Socket to) {
        try (InputStream in = from.getInputStream();
                OutputStream out = to.getOutputStream()) {
            byte[] buffer = new byte[BUFFER_BYTES];
            for (int read = in.read(buffer); read >= 0; read = in.read(buffer)) {
                out.write(buffer, 0, read);
                out.flush();
            }
        } catch (IOException e) {
        } finally {
            relayed.close();
        }
    }

    private final class Relayed {
        final Peer peer;
        private final Socket client;
        private Socket robot;
        private boolean closed;

        Relayed(Peer peer, Socket client) {
            this.peer = peer;
            this.client = client;
        }

        synchronized boolean attach(Socket robot) {
            if (closed) {
                closeQuietly(robot);
                return false;
            }
            this.robot = robot;
            return true;
        }

        void close() {
            synchronized (this) {
                closed = true;
                closeQuietly(client);
                if (robot != null) {
                    closeQuietly(robot);
                }
            }
            open.remove(this);
        }
    }

    private static void daemon(String name, Runnable work) {
        Thread thread = new Thread(work, name);
        thread.setDaemon(true);
        thread.start();
    }

    private static void closeQuietly(Closeable closeable) {
        try {
            closeable.close();
        } catch (IOException ignored) {
        }
    }
}
