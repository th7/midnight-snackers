package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Stream;
import org.junit.Test;

public class LaunchSimIsStrictTest {
    private static final String BIT_FOR_BIT =
            "java.lang.Math may differ by an ulp between JVMs and machines; the launch sim uses StrictMath so a"
                    + " shot comes out the same bit for bit wherever it is simulated:"
                    + " https://docs.oracle.com/en/java/javase/21/docs/api/java.base/java/lang/StrictMath.html";

    private static final List<String> REACH_FOR_JAVA_LANG_MATH = List.of(
            "Chassis",
            "ConvexPolygon",
            "Drivetrain",
            "Encoder",
            "Field",
            "Flight",
            "Heading",
            "Hives",
            "Launch",
            "Lean",
            "Nest",
            "Noise",
            "Placement",
            "Power",
            "Seconds",
            "Stack",
            "Tilt",
            "Traction",
            "Turntable",
            "Vec3");

    private static Map<String, Boolean> reachingForJavaLangMath() throws IOException, URISyntaxException {
        byte[] math = "java/lang/Math".getBytes(StandardCharsets.UTF_8);
        Path root = Path.of(Shot.class.getResource("Shot.class").toURI()).getParent();
        List<Path> files = new ArrayList<>();
        try (Stream<Path> walked = Files.walk(root)) {
            walked.filter(file -> file.getFileName().toString().endsWith(".class"))
                    .forEach(files::add);
        }
        Map<String, Boolean> reaching = new TreeMap<>();
        for (Path file : files) {
            String type = topLevelClassOf(root.relativize(file));
            reaching.merge(type, contains(Files.readAllBytes(file), math), Boolean::logicalOr);
        }
        return reaching;
    }

    private static String topLevelClassOf(Path relative) {
        String name = relative.toString().replace(relative.getFileSystem().getSeparator(), ".");
        int nested = name.indexOf('$');
        return nested >= 0 ? name.substring(0, nested) : name.substring(0, name.length() - ".class".length());
    }

    @Test
    public void theScanReadsTheClassFilesOfTheWholeCoreFromItsPackageRootDown() throws IOException, URISyntaxException {
        Map<String, Boolean> scanned = reachingForJavaLangMath();

        assertTrue("the scan read " + scanned.keySet() + ", which has no Shot", scanned.containsKey("Shot"));
        assertTrue("the scan read " + scanned.keySet() + ", which has no ShotRun", scanned.containsKey("ShotRun"));
        assertTrue("the scan read " + scanned.keySet() + ", which has no Field", scanned.containsKey("Field"));
    }

    @Test
    public void noClassOffTheListReachesForJavaLangMath() throws IOException, URISyntaxException {
        for (Map.Entry<String, Boolean> scanned : reachingForJavaLangMath().entrySet()) {
            String type = scanned.getKey();
            assertFalse(
                    type + " reaches for java.lang.Math and is not listed as a class that may: " + BIT_FOR_BIT,
                    scanned.getValue() && !REACH_FOR_JAVA_LANG_MATH.contains(type));
        }
    }

    @Test
    public void everyClassOnTheListStillReachesForJavaLangMath() throws IOException, URISyntaxException {
        Map<String, Boolean> scanned = reachingForJavaLangMath();

        for (String type : REACH_FOR_JAVA_LANG_MATH) {
            assertTrue(
                    type + " is listed as reaching for java.lang.Math but has no class file",
                    scanned.containsKey(type));
            assertTrue(
                    type + " no longer reaches for java.lang.Math, so take it off the list of classes that may",
                    scanned.get(type));
        }
    }

    @Test
    public void theCheckSeesJavaLangMathWhereItIs() throws IOException, URISyntaxException {
        assertTrue(
                "Field turns its hives with java.lang.Math, so the scan must find it there",
                reachingForJavaLangMath().getOrDefault("Field", false));
    }

    private static boolean contains(byte[] haystack, byte[] needle) {
        outer:
        for (int i = 0; i + needle.length <= haystack.length; i++) {
            for (int j = 0; j < needle.length; j++) {
                if (haystack[i + j] != needle[j]) {
                    continue outer;
                }
            }
            return true;
        }
        return false;
    }
}
