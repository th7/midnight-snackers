package org.firstinspires.ftc.teamcode.sim;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

final class FakeHub implements AutoCloseable {
    private final ServerSocket socket;
    private final List<Socket> connections = new ArrayList<>();
    private final ByteArrayOutputStream received = new ByteArrayOutputStream();
    private int accepted;

    private FakeHub(ServerSocket socket) {
        this.socket = socket;
    }

    static FakeHub start() {
        try {
            FakeHub hub = new FakeHub(new ServerSocket(0, 50, InetAddress.getLoopbackAddress()));
            Thread acceptor = new Thread(hub::acceptLoop, "fake-hub");
            acceptor.setDaemon(true);
            acceptor.start();
            return hub;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    static InetSocketAddress nobodyListening() {
        try (ServerSocket taken = new ServerSocket(0, 1, InetAddress.getLoopbackAddress())) {
            return new InetSocketAddress(InetAddress.getLoopbackAddress(), taken.getLocalPort());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    InetSocketAddress address() {
        return new InetSocketAddress(InetAddress.getLoopbackAddress(), socket.getLocalPort());
    }

    synchronized int accepted() {
        return accepted;
    }

    synchronized String received() {
        return received.toString(StandardCharsets.UTF_8);
    }

    synchronized int open() {
        connections.removeIf(Socket::isClosed);
        return connections.size();
    }

    void dropEveryConnection() {
        List<Socket> dropping;
        synchronized (this) {
            dropping = new ArrayList<>(connections);
            connections.clear();
        }
        for (Socket connection : dropping) {
            closeQuietly(connection);
        }
    }

    @Override
    public void close() {
        closeQuietly(socket);
        dropEveryConnection();
    }

    private void acceptLoop() {
        while (!socket.isClosed()) {
            Socket connection;
            try {
                connection = socket.accept();
            } catch (SocketException closed) {
                return;
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
            synchronized (this) {
                accepted++;
                connections.add(connection);
            }
            Thread echo = new Thread(() -> echo(connection), "fake-hub-connection");
            echo.setDaemon(true);
            echo.start();
        }
    }

    private void echo(Socket connection) {
        try (connection;
                InputStream in = connection.getInputStream();
                OutputStream out = connection.getOutputStream()) {
            byte[] buffer = new byte[4096];
            for (int read = in.read(buffer); read >= 0; read = in.read(buffer)) {
                synchronized (this) {
                    received.write(buffer, 0, read);
                }
                out.write(buffer, 0, read);
                out.flush();
            }
        } catch (IOException e) {
        }
    }

    private static void closeQuietly(java.io.Closeable closeable) {
        try {
            closeable.close();
        } catch (IOException ignored) {
        }
    }
}
