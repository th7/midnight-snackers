package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

public final class FakeChild implements Child {
    private static final String END = "\u0000end";

    private final List<String> lines = new ArrayList<>();
    private String whyItWillNotStart;
    private boolean staysAliveWhenItRunsOut;
    private boolean ignoresStop;
    private int exit;

    private final List<String> heard = new ArrayList<>();
    private final List<List<String>> startedWith = new ArrayList<>();
    private final List<String> prints = new ArrayList<>();
    private Running running;

    public static FakeChild thatSays(String... lines) {
        FakeChild child = new FakeChild();
        child.lines.addAll(List.of(lines));
        return child;
    }

    public FakeChild thatWillNotStart(String why) {
        whyItWillNotStart = why;
        return this;
    }

    public FakeChild thatStaysAliveSayingNothingMore() {
        staysAliveWhenItRunsOut = true;
        return this;
    }

    public FakeChild thatIgnoresStop() {
        ignoresStop = true;
        return this;
    }

    /** What it writes to its stderr as it starts, which is what the bench keeps as the run's log. */
    public FakeChild thatPrints(String... stderr) {
        prints.addAll(List.of(stderr));
        return this;
    }

    public FakeChild thatExitsWith(int code) {
        exit = code;
        return this;
    }

    public List<String> whatItWasTold() {
        return List.copyOf(heard);
    }

    /** The arguments of each start, so a test can hold the bench to what it asks the child for. */
    public List<List<String>> whatItWasStartedWith() {
        return List.copyOf(startedWith);
    }

    public List<String> theLastStart() {
        if (startedWith.isEmpty()) {
            throw new IllegalStateException("nothing was started");
        }
        return startedWith.get(startedWith.size() - 1);
    }

    public Running running() {
        return running;
    }

    private final class Scripted implements Running {
        private final BlockingQueue<String> toSay = new ArrayBlockingQueue<>(Math.max(1, lines.size() + 1));
        private volatile boolean alive = true;

        Scripted() {
            toSay.addAll(lines);
            if (!staysAliveWhenItRunsOut) {
                toSay.add(END);
            }
        }

        @Override
        public boolean say(String line) {
            heard.add(line);
            if (line.contains("\"stop\"") && !ignoresStop) {
                toSay.add(END);
            }
            return alive;
        }

        @Override
        public String hear() {
            try {
                String line = toSay.poll(30, TimeUnit.SECONDS);
                if (line == null || END.equals(line)) {
                    alive = false;
                    return null;
                }
                return line;
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            }
        }

        @Override
        public boolean alive() {
            return alive;
        }

        @Override
        public boolean endedWithin(double seconds) {
            long deadline = System.nanoTime() + (long) (seconds * 1e9);
            while (alive && System.nanoTime() < deadline) {
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return !alive;
                }
            }
            return !alive;
        }

        @Override
        public void kill() {
            alive = false;
            toSay.offer(END);
        }

        @Override
        public OptionalInt exitCode() {
            return alive ? OptionalInt.empty() : OptionalInt.of(exit);
        }

        @Override
        public void close() {
            kill();
        }
    }

    private Running started(Consumer<String> log, String... args) {
        startedWith.add(List.of(args));
        if (whyItWillNotStart != null) {
            throw new CouldNotStart(whyItWillNotStart, new IllegalStateException(whyItWillNotStart));
        }
        prints.forEach(log);
        running = new Scripted();
        return running;
    }

    @Override
    public Running onTheClassesAt(Path classes, Consumer<String> log, String... args) {
        return started(log, args);
    }

    @Override
    public Running onThisClasspath(Consumer<String> log, String... args) {
        return started(log, args);
    }
}
