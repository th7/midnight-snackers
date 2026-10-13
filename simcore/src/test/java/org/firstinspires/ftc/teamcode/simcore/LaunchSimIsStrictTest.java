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
import java.util.stream.Stream;
import org.junit.Test;

public class LaunchSimIsStrictTest {
    private static final String BIT_FOR_BIT =
            "java.lang.Math may differ by an ulp between JVMs and machines; the launch sim uses StrictMath so a"
                    + " shot comes out the same bit for bit wherever it is simulated:"
                    + " https://docs.oracle.com/en/java/javase/21/docs/api/java.base/java/lang/StrictMath.html";

    private static final List<String> THE_LAUNCH_SIM = List.of(
            "Ball",
            "Surface",
            "Turn",
            "ShotSetting",
            "ShotSettings",
            "Air",
            "WheelPair",
            "Colliders",
            "Collider",
            "Contacts",
            "Bounce",
            "Surfaces",
            "World",
            "Shot",
            "ShotRun",
            "ShotSetup");

    private static List<Path> classFilesOf(String type) throws IOException, URISyntaxException {
        Path directory = Path.of(Shot.class.getResource("Shot.class").toURI()).getParent();
        List<Path> files = new ArrayList<>();
        try (Stream<Path> listed = Files.list(directory)) {
            listed.filter(file -> {
                        String name = file.getFileName().toString();
                        return name.equals(type + ".class") || name.startsWith(type + "$");
                    })
                    .forEach(files::add);
        }
        return files;
    }

    @Test
    public void noClassOfTheLaunchSimReachesForJavaLangMath() throws IOException, URISyntaxException {
        byte[] math = "java/lang/Math".getBytes(StandardCharsets.UTF_8);
        for (String type : THE_LAUNCH_SIM) {
            List<Path> files = classFilesOf(type);
            assertFalse(type + " has no class file beside Shot's", files.isEmpty());
            for (Path file : files) {
                assertFalse(file.getFileName() + ": " + BIT_FOR_BIT, contains(Files.readAllBytes(file), math));
            }
        }
    }

    @Test
    public void theCheckSeesJavaLangMathWhereItIs() throws IOException, URISyntaxException {
        byte[] math = "java/lang/Math".getBytes(StandardCharsets.UTF_8);
        boolean seen = false;
        for (Path file : classFilesOf("Field")) {
            seen |= contains(Files.readAllBytes(file), math);
        }
        assertTrue("Field turns its hives with java.lang.Math, so the scan must find it there", seen);
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
