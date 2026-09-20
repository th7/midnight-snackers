package org.firstinspires.ftc.teamcode.sim;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.FileTime;
import java.util.stream.Stream;

/**
 * A project on disk for a bench to build and run: the robot's sources, the simulator beside them,
 * and whatever a test has changed in either. Copying one is the dearest thing a test of the bench
 * or of the coding server does before a compile, so it is made here, in one place, and counted.
 */
public final class SimProject {

    public static final String PLANS = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Plans.java";
    public static final String HARDWARE =
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/Hardware.java";
    public static final String SIM_ROBOT = "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimRobot.java";
    public static final String SIM_DEVICES =
            "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimDevices.java";
    public static final String SIM_CHILD = "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimChild.java";
    public static final String SIM_RUN_STREAM =
            "TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim/SimRunStream.java";

    /** The name the plans below register their op mode under, and the line its loop count is on. */
    public static final String TEMP_NAME = "Temp";

    public static final int TEMP_LOOPS_LINE = 8;

    private final Path root;

    private SimProject(Path root) {
        this.root = root;
    }

    public Path root() {
        return root;
    }

    /** A project already on disk, to read or to change. */
    public static SimProject at(Path root) {
        return new SimProject(root);
    }

    /** The robot's sources and the simulator, as they are in this checkout, copied under a root. */
    public static SimProject copiedTo(Path root) {
        try (Cost.Spent spent = Cost.start(Cost.Kind.PROJECT_COPY)) {
            copyTree(here("src/main/java"), root.resolve("TeamCode/src/main/java"));
            simulatorInto(root);
            return new SimProject(root);
        }
    }

    /** The simulator alone, for a project whose robot sources the test writes itself. */
    public static SimProject simulatorOnlyIn(Path root) {
        try (Cost.Spent spent = Cost.start(Cost.Kind.PROJECT_COPY)) {
            simulatorInto(root);
            return new SimProject(root);
        }
    }

    private static void simulatorInto(Path root) {
        copyTree(here("src/test/java"), root.resolve("TeamCode/src/test/java"));
        copyTree(here("src/test/resources"), root.resolve("TeamCode/src/test/resources"));
    }

    /** This project's own {@code Plans.java}, replacing whatever was there. */
    public SimProject withPlans(String source) {
        Path file = root.resolve(PLANS);
        try {
            Files.createDirectories(file.getParent());
            Files.write(file, source.getBytes(StandardCharsets.UTF_8));
            // A build is fingerprinted by what it last saw; a copy and an edit in the same
            // millisecond would read as the same sources.
            Files.setLastModifiedTime(file, FileTime.fromMillis(System.currentTimeMillis() + 2000));
        } catch (IOException e) {
            throw new UncheckedIOException("could not write " + file, e);
        }
        return this;
    }

    /** One replacement in one of this project's files, which must be there to be replaced. */
    public SimProject edited(String key, String from, String to) {
        Path file = root.resolve(key);
        try {
            String source = new String(Files.readAllBytes(file), StandardCharsets.UTF_8);
            if (!source.contains(from)) {
                throw new IllegalStateException(key + " has no " + from);
            }
            Files.write(file, source.replace(from, to).getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            throw new UncheckedIOException("could not edit " + file, e);
        }
        return this;
    }

    public Path sourceRoot() {
        return root.resolve("TeamCode/src/main/java");
    }

    /** A {@code Plans.java} whose one auto is done after so many loops. */
    public static String tempPlans(int loops) {
        return tempPlans(loops, "Test");
    }

    public static String tempPlans(int loops, String group) {
        return "package org.firstinspires.ftc.teamcode;\n"
                + "import org.firstinspires.ftc.teamcode.opmode.Auto;\n"
                + "import org.firstinspires.ftc.teamcode.base.Loopable;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.PlanPart;\n"
                + "import org.firstinspires.ftc.teamcode.planrunner.Step;\n"
                + "\n"
                + "public class Plans implements Loopable {\n"
                + "    private int loops = 0;\n"
                + "    public Plans(org.firstinspires.ftc.teamcode.Drive drive,"
                + " org.firstinspires.ftc.teamcode.Nav nav,"
                + " org.firstinspires.ftc.teamcode.Launcher launcher,"
                + " java.util.function.LongSupplier nanoClock) { }\n"
                + "    @Auto(name = \"" + TEMP_NAME + "\", group = \"" + group
                + "\", alliance = org.firstinspires.ftc.teamcode.base.Alliance.RELATIVE)\n"
                + "    public PlanPart temp() { return new Step(\"count\", () -> { }, () -> ++loops >= " + loops
                + "); }\n"
                + "    public void loop() { }\n"
                + "}\n";
    }

    /** A directory of this checkout, which the tests run from the TeamCode module. */
    public static Path here(String relative) {
        Path path = Paths.get(relative).toAbsolutePath();
        if (!Files.isDirectory(path)) {
            throw new IllegalStateException("tests run from the TeamCode module directory; there is no " + path);
        }
        return path;
    }

    static void copyTree(Path from, Path to) {
        try (Stream<Path> files = Files.walk(from)) {
            for (Path file : (Iterable<Path>) files::iterator) {
                Path target = to.resolve(from.relativize(file).toString());
                if (Files.isDirectory(file)) {
                    Files.createDirectories(target);
                } else {
                    Files.copy(file, target, StandardCopyOption.REPLACE_EXISTING);
                }
            }
        } catch (IOException e) {
            throw new UncheckedIOException("could not copy " + from + " to " + to, e);
        }
    }
}
