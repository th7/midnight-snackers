package org.firstinspires.ftc.teamcode.sim;

import com.palantir.javaformat.java.Formatter;
import com.palantir.javaformat.java.FormatterException;
import com.palantir.javaformat.java.ImportOrderer;
import com.palantir.javaformat.java.JavaFormatterOptions;
import com.palantir.javaformat.java.RemoveUnusedImports;

public final class JavaFormatter {
    private JavaFormatter() {}

    public static final class Unparseable extends Exception {
        Unparseable(String message) {
            super(message);
        }
    }

    private static final JavaFormatterOptions.Style STYLE = JavaFormatterOptions.Style.PALANTIR;

    private static final Formatter FORMATTER = Formatter.createFormatter(
            JavaFormatterOptions.builder().style(STYLE).build());

    public static String format(String source) throws Unparseable {
        try {
            return FORMATTER.formatSource(
                    RemoveUnusedImports.removeUnusedImports(ImportOrderer.reorderImports(source, STYLE)));
        } catch (FormatterException e) {
            throw new Unparseable(oneLine(e.getMessage()));
        }
    }

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

    public static String version() {
        String version = Formatter.class.getPackage().getImplementationVersion();
        return version == null ? "unknown" : version;
    }

    private static String oneLine(String message) {
        return message == null ? "does not parse" : message.replaceAll("\\s*\\R\\s*", "; ");
    }
}
