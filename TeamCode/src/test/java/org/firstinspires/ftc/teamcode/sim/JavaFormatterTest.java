package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.Test;

public class JavaFormatterTest {
    @Test
    public void badlyIndentedJavaComesBackAsPalantirWritesIt() throws Exception {
        assertEquals(
                "class Plans {\n    int edited;\n\n    void go() {\n        int x = 1;\n    }\n}\n",
                JavaFormatter.format("class Plans {\n  int edited;\n      void go( ) {int x=1;}\n}\n"));
    }

    @Test
    public void importsAreOrderedAndTheUnusedOnesRemoved() throws Exception {
        assertEquals(
                "package p;\n\nimport java.util.List;\n\npublic class A {\n    List<String> xs;\n}\n",
                JavaFormatter.format("package p;\n\nimport java.util.Map;\nimport java.util.List;\n\n"
                        + "public class A {\n    List<String> xs;\n}\n"));
    }

    @Test
    public void formattingIsIdempotent() throws Exception {
        String once = JavaFormatter.format("class Plans {\n  int edited;\n}\n");
        assertEquals(once, JavaFormatter.format(once));
    }

    @Test
    public void sourceThatDoesNotParseIsRefusedWithWhereItWentWrong() {
        try {
            JavaFormatter.format("class Plans { this is not java\n");
            fail("expected the formatter to refuse source that does not parse");
        } catch (JavaFormatter.Unparseable e) {
            assertEquals("1:16: error: illegal start of type", e.getMessage());
        }
    }

    @Test
    public void theFormatterOnTheClasspathIsTheVersionSpotlessChecksWith() {
        String spotless = System.getProperty("palantirJavaFormat.version");
        assertNotNull(
                "no palantirJavaFormat.version: run the tests with ./gradlew :TeamCode:testDebugUnitTest,"
                        + " which is what CI runs",
                spotless);
        assertEquals(spotless, JavaFormatter.version());
    }

    @Test
    public void everyJavaFileInThisProjectIsAlreadyWhatTheFormatterWouldWrite() throws Exception {
        List<Path> sources = javaFilesUnderSrc();
        assertTrue("no Java found under " + Paths.get("src").toAbsolutePath(), sources.size() > 10);
        List<String> differ = new ArrayList<>();
        for (Path source : sources) {
            String text = new String(Files.readAllBytes(source), StandardCharsets.UTF_8);
            String formatted;
            try {
                formatted = JavaFormatter.format(text);
            } catch (JavaFormatter.Unparseable e) {
                differ.add(source + " does not parse: " + e.getMessage());
                continue;
            }
            if (!formatted.equals(text)) {
                differ.add(source.toString());
            }
        }
        assertEquals(
                "the coding server's formatter and Spotless disagree about these files;"
                        + " ./gradlew :TeamCode:spotlessCheck passes on them",
                List.of(),
                differ);
    }

    private static List<Path> javaFilesUnderSrc() throws IOException {
        Path src = Paths.get("src");
        try (Stream<Path> walk = Files.walk(src)) {
            List<Path> files = new ArrayList<>(walk.filter(Files::isRegularFile)
                    .filter(path -> path.getFileName().toString().endsWith(".java"))
                    .toList());
            files.sort(null);
            return files;
        }
    }
}
