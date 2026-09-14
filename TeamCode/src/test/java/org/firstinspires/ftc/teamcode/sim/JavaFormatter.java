package org.firstinspires.ftc.teamcode.sim;

import com.palantir.javaformat.java.Formatter;
import com.palantir.javaformat.java.FormatterException;
import com.palantir.javaformat.java.ImportOrderer;
import com.palantir.javaformat.java.JavaFormatterOptions;
import com.palantir.javaformat.java.RemoveUnusedImports;

/**
 * palantir-java-format, run in this process over one file's text: the three steps Spotless's
 * {@code palantirJavaFormat} step runs, in its order, so what the coding server commits is what
 * {@code :TeamCode:spotlessCheck} accepts and a student's push never fails on formatting alone.
 * Shelling out to {@code ./gradlew :TeamCode:spotlessApply} per click would cost seconds of Gradle
 * startup and would format files the user never touched.
 *
 * <p>Not the Gradle task, and not a check: it rewrites, and it is the caller's business to decide
 * when. The coding server calls it on Commit and nowhere else.
 */
public final class JavaFormatter {
    private JavaFormatter() {}

    /** Source the formatter cannot parse, so there is nothing for it to write. */
    public static final class Unparseable extends Exception {
        Unparseable(String message) {
            super(message);
        }
    }

    private static final JavaFormatterOptions.Style STYLE = JavaFormatterOptions.Style.PALANTIR;

    private static final Formatter FORMATTER = Formatter.createFormatter(
            JavaFormatterOptions.builder().style(STYLE).build());

    /**
     * The Java source as the formatter would write it: imports ordered, the unused ones removed,
     * then the text formatted. Formatting its own output changes nothing.
     *
     * @throws Unparseable when the source does not parse, naming the line and column
     */
    public static String format(String source) throws Unparseable {
        try {
            return FORMATTER.formatSource(
                    RemoveUnusedImports.removeUnusedImports(ImportOrderer.reorderImports(source, STYLE)));
        } catch (FormatterException e) {
            throw new Unparseable(oneLine(e.getMessage()));
        }
    }

    /**
     * Formats a trivial file, so that a JVM started without the {@code --add-exports} the formatter
     * needs says so here, naming the fix, instead of on a teammate's first Commit. The coding
     * server calls this as it starts: a server that cannot format is a server that would commit
     * work CI then rejects.
     */
    public static void check() {
        try {
            format("class A {}\n");
        } catch (Unparseable e) {
            throw new IllegalStateException("the formatter cannot parse 'class A {}': " + e.getMessage(), e);
        } catch (LinkageError e) {
            throw new IllegalStateException(
                    "palantir-java-format cannot reach javac's own trees: this JVM needs the"
                            + " --add-exports jdk.compiler/com.sun.tools.javac.* arguments that"
                            + " javacExports in TeamCode/build.gradle passes",
                    e);
        }
    }

    /** The version of palantir-java-format on this classpath, which has to be the one Spotless checks with. */
    public static String version() {
        String version = Formatter.class.getPackage().getImplementationVersion();
        return version == null ? "unknown" : version;
    }

    /** The formatter reports several errors over several lines; a warning in the editor has one. */
    private static String oneLine(String message) {
        return message == null ? "does not parse" : message.replaceAll("\\s*\\R\\s*", "; ");
    }
}
