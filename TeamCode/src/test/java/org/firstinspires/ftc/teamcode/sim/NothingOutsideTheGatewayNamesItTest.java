package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.junit.Test;

/**
 * The robot gateway runs on the host alone, and its tests are judged on Linux alone; everything a
 * teammate's computer runs is judged on Windows too. That split is only honest while nothing a
 * client runs reaches into the gateway, so it is read off the bytecode: a class outside the
 * gateway's package that names one inside it, in a call, a field or a signature, fails here.
 */
public class NothingOutsideTheGatewayNamesItTest {
    private static final String TEAMCODE = "org/firstinspires/ftc/teamcode";

    /** Put together at run time, so that this class's own constants never name the gateway. */
    private static final String GATEWAY = String.join("/", TEAMCODE, "gateway");

    private static boolean inTheGateway(Path classFile) {
        return classFile.toString().replace(File.separatorChar, '/').contains("/" + GATEWAY + "/");
    }

    private static List<Path> everyClass() {
        List<Path> classes = new ArrayList<>(Bytecode.classesUnder(TEAMCODE));
        classes.addAll(Bytecode.classesUnder(SimCatalog.PACKAGE.replace('.', '/')));
        assertTrue("no compiled classes were found, so this rule judged nothing", classes.size() > 20);
        return classes;
    }

    @Test
    public void noClassOutsideTheGatewayNamesAClassOfTheGateway() {
        Set<String> naming = new TreeSet<>();
        for (Path classFile : everyClass()) {
            if (!inTheGateway(classFile) && Bytecode.mentionsAnythingUnder(classFile, GATEWAY)) {
                naming.add(classFile.getFileName().toString());
            }
        }

        assertEquals(
                "these name a class of the gateway, so what a client runs would reach code judged on Linux"
                        + " alone. Move what they need out of " + GATEWAY.replace('/', '.') + ", or move them into it.",
                Set.of(),
                naming);
    }

    @Test
    public void theRuleWouldNoticeAClassThatNamedTheGateway() {
        List<Path> gateway = everyClass().stream()
                .filter(NothingOutsideTheGatewayNamesItTest::inTheGateway)
                .toList();

        assertTrue("no class of the gateway was compiled, so this rule judged nothing", !gateway.isEmpty());
        assertTrue(
                "RobotGatewayTest names RobotGateway, and this rule is only as good as its eye for that",
                gateway.stream()
                        .filter(file -> Bytecode.outermost(file).equals("RobotGatewayTest"))
                        .anyMatch(file -> Bytecode.mentionsAnythingUnder(file, GATEWAY)));
    }
}
