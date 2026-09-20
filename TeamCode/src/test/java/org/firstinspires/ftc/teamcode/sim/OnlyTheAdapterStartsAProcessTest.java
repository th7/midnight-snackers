package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Stream;
import org.junit.Test;

public class OnlyTheAdapterStartsAProcessTest {
    private static final String PACKAGE = "org/firstinspires/ftc/teamcode/sim/";

    private static final Set<String> MAY_START_ONE =
            Set.of("RealGit", "SimChild", "GitFixture", "SimReplayPageTest", "CodingServerTest");

    private static final String THIS_RULE = OnlyTheAdapterStartsAProcessTest.class.getSimpleName();

    private static final List<String> STARTS_A_PROCESS = List.of("java/lang/ProcessBuilder", "java/lang/Runtime");

    private static List<Path> compiledClasses() throws IOException {
        List<Path> found = new ArrayList<>();
        for (String entry : System.getProperty("java.class.path").split(File.pathSeparator)) {
            Path root = Path.of(entry);
            Path here = root.resolve(PACKAGE);
            if (!Files.isDirectory(here)) {
                continue;
            }
            try (Stream<Path> files = Files.walk(here)) {
                files.filter(Files::isRegularFile)
                        .filter(file -> file.toString().endsWith(".class"))
                        .forEach(found::add);
            }
        }
        return found;
    }

    private static String outermost(Path classFile) {
        String name = classFile.getFileName().toString().replace(".class", "");
        int nested = name.indexOf('$');
        return nested < 0 ? name : name.substring(0, nested);
    }

    private static boolean mentions(Path classFile, String reference) throws IOException {
        String bytes = new String(Files.readAllBytes(classFile), StandardCharsets.ISO_8859_1);
        return bytes.contains(lengthPrefixed(reference));
    }

    private static String lengthPrefixed(String reference) {
        return "" + (char) 1 + (char) (reference.length() >> 8) + (char) (reference.length() & 0xff) + reference;
    }

    @Test
    public void nothingButTheNamedFewCanStartAProcess() throws IOException {
        List<Path> classes = compiledClasses();
        assertTrue("no compiled classes were found, so this rule judged nothing", classes.size() > 20);

        Set<String> starters = new TreeSet<>();
        for (Path classFile : classes) {
            for (String reference : STARTS_A_PROCESS) {
                if (mentions(classFile, reference)) {
                    starters.add(outermost(classFile));
                }
            }
        }

        starters.removeAll(MAY_START_ONE);
        starters.remove(THIS_RULE);
        assertEquals("these reach a process without going through an adapter", Set.of(), starters);
    }

    @Test
    public void theModulesThatTalkToSomethingOutsideThisProcessDoItThroughAnInterface() throws IOException {
        List<Path> classes = compiledClasses();
        List<Path> through = classes.stream()
                .filter(file -> Set.of("Worktrees", "SimBench").contains(outermost(file)))
                .toList();

        assertTrue("neither was compiled, so this rule judged nothing", through.size() > 1);
        for (Path classFile : through) {
            for (String reference : STARTS_A_PROCESS) {
                assertTrue(
                        classFile.getFileName() + " reaches " + reference + " rather than the Git interface",
                        !mentions(classFile, reference));
            }
        }
    }

    @Test
    public void theRuleWouldNoticeAClassThatStartedOne() throws IOException {
        List<Path> realGit = compiledClasses().stream()
                .filter(file -> outermost(file).equals("RealGit"))
                .toList();

        assertTrue(!realGit.isEmpty());
        assertTrue(
                "RealGit is the adapter, so it must be what this rule would catch if it were not allowed",
                realGit.stream().anyMatch(file -> {
                    try {
                        return mentions(file, "java/lang/ProcessBuilder");
                    } catch (IOException e) {
                        return false;
                    }
                }));
    }
}
