package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.channels.SeekableByteChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.Properties;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import org.firstinspires.ftc.teamcode.simcore.Deployment;
import org.firstinspires.ftc.teamcode.simcore.Seconds;

public final class GradleDeploy implements Deploy {
    private static final int OUTPUT_BYTES = 65536;
    private final Path root;
    private final Seconds timeout;
    private final ReentrantLock installing = new ReentrantLock();

    public GradleDeploy(Path root) {
        this(root, Seconds.of(600).orElse(Seconds.zero()));
    }

    GradleDeploy(Path root, Seconds timeout) {
        this.root = root;
        this.timeout = timeout;
    }

    @Override
    public Deployment from(Path worktree) {
        if (!installing.tryLock()) {
            return Deployment.busy();
        }
        Path log = null;
        Process process = null;
        try {
            Path directory = worktree.toAbsolutePath().normalize();
            boolean windows =
                    System.getProperty("os.name").toLowerCase(Locale.ROOT).contains("windows");
            String wrapper =
                    directory.resolve(windows ? "gradlew.bat" : "gradlew").toString();
            List<String> command = windows
                    ? List.of(
                            "cmd.exe", "/d", "/c", wrapper, ":TeamCode:installDebug", "--console=plain", "--no-daemon")
                    : List.of(wrapper, ":TeamCode:installDebug", "--console=plain", "--no-daemon");
            log = Files.createTempFile("robot-deploy-", ".log");
            ProcessBuilder builder = new ProcessBuilder(command)
                    .directory(directory.toFile())
                    .redirectErrorStream(true)
                    .redirectOutput(log.toFile());
            builder.environment().put("JAVA_HOME", System.getProperty("java.home"));
            Path properties = root.resolve("local.properties");
            if (Files.isRegularFile(properties)) {
                Properties local = new Properties();
                try (InputStream input = Files.newInputStream(properties)) {
                    local.load(input);
                }
                String sdk = local.getProperty("sdk.dir");
                if (sdk != null && !sdk.isBlank()) {
                    builder.environment().put("ANDROID_HOME", sdk);
                    builder.environment().put("ANDROID_SDK_ROOT", sdk);
                }
            }
            try (Cost.Spent ignored = Cost.start(Cost.Kind.GRADLE_DEPLOY)) {
                process = builder.start();
                process.getOutputStream().close();
                if (!process.waitFor((long) (timeout.value() * 1000), TimeUnit.MILLISECONDS)) {
                    stop(process);
                    return Deployment.timedOut(output(log));
                }
                return Deployment.exited(process.exitValue(), output(log));
            }
        } catch (InterruptedException e) {
            if (process != null) {
                stop(process);
            }
            Thread.currentThread().interrupt();
            return Deployment.interrupted(log == null ? "" : output(log));
        } catch (IOException e) {
            return Deployment.couldNotStart(e.toString());
        } finally {
            if (process != null && process.isAlive()) {
                stop(process);
            }
            if (log != null) {
                try {
                    Files.deleteIfExists(log);
                } catch (IOException ignored) {
                }
            }
            installing.unlock();
        }
    }

    private static String output(Path log) {
        try (SeekableByteChannel channel = Files.newByteChannel(log)) {
            channel.position(Math.max(0, channel.size() - OUTPUT_BYTES));
            ByteBuffer bytes = ByteBuffer.allocate(OUTPUT_BYTES);
            while (bytes.hasRemaining() && channel.read(bytes) != -1) {}
            bytes.flip();
            return StandardCharsets.UTF_8.decode(bytes).toString();
        } catch (IOException e) {
            return "could not read deployment output: " + e;
        }
    }

    private static void stop(Process process) {
        ProcessTree.killDescendantsOf(process);
        process.destroyForcibly();
        try {
            process.waitFor(5, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
