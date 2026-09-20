package org.firstinspires.ftc.teamcode.sim;

import java.nio.file.Path;
import java.util.OptionalInt;
import java.util.function.Consumer;

public interface Child {

    final class CouldNotStart extends RuntimeException {
        public CouldNotStart(String message, Throwable cause) {
            super(message, cause);
        }
    }

    interface Running extends AutoCloseable {
        boolean say(String line);

        String hear();

        boolean alive();

        boolean endedWithin(double seconds);

        void kill();

        OptionalInt exitCode();

        @Override
        void close();
    }

    Running onTheClassesAt(Path classes, Consumer<String> log, String... args);

    Running onThisClasspath(Consumer<String> log, String... args);
}
