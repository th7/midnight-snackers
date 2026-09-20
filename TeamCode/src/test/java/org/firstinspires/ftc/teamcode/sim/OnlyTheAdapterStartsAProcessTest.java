package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.junit.Test;

public class OnlyTheAdapterStartsAProcessTest {
    private static final String PACKAGE = "org/firstinspires/ftc/teamcode/sim";

    static final Set<String> MAY_START_ONE = CostIsCountedWhereItIsSpentTest.mayReach("java/lang/ProcessBuilder");

    /**
     * A rule about what a class may name has to name it too, so these read as reaching everything
     * they forbid. They are left out of every such rule, here and in the one about counting.
     */
    static final Set<String> HOLD_THE_RULES =
            Set.of("Cost", "CostTest", "OnlyTheAdapterStartsAProcessTest", "CostIsCountedWhereItIsSpentTest");

    private static final List<String> STARTS_A_PROCESS = List.of("java/lang/ProcessBuilder");

    /**
     * {@code Runtime} is also how a JVM is told about a shutdown hook, so reaching it is not on its
     * own reaching for a process: it is {@code exec} on it that is.
     */
    private static final List<String> RUNTIME_EXEC = List.of("java/lang/Runtime", "exec");

    private static List<Path> compiledClasses() {
        return Bytecode.classesUnder(PACKAGE);
    }

    private static boolean mentionsAll(Path classFile, List<String> references) {
        return references.stream().allMatch(reference -> Bytecode.mentions(classFile, reference));
    }

    @Test
    public void nothingButTheNamedFewCanStartAProcess() throws IOException {
        List<Path> classes = compiledClasses();
        assertTrue("no compiled classes were found, so this rule judged nothing", classes.size() > 20);

        Set<String> starters = new TreeSet<>(Bytecode.thatMentionAny(classes, STARTS_A_PROCESS));
        for (Path classFile : classes) {
            if (mentionsAll(classFile, RUNTIME_EXEC)) {
                starters.add(Bytecode.outermost(classFile));
            }
        }

        starters.removeAll(MAY_START_ONE);
        starters.removeAll(HOLD_THE_RULES);
        assertEquals("these reach a process without going through an adapter", Set.of(), starters);
    }

    @Test
    public void theModulesThatTalkToSomethingOutsideThisProcessDoItThroughAnInterface() throws IOException {
        List<Path> through = compiledClasses().stream()
                .filter(file -> Set.of("Worktrees", "SimBench").contains(Bytecode.outermost(file)))
                .toList();

        assertTrue("neither was compiled, so this rule judged nothing", through.size() > 1);
        for (Path classFile : through) {
            for (String reference : STARTS_A_PROCESS) {
                assertTrue(
                        classFile.getFileName() + " reaches " + reference + " rather than the Git interface",
                        !Bytecode.mentions(classFile, reference));
            }
        }
    }

    @Test
    public void theRuleWouldNoticeAClassThatStartedOne() throws IOException {
        List<Path> realGit = compiledClasses().stream()
                .filter(file -> Bytecode.outermost(file).equals("RealGit"))
                .toList();

        assertTrue(!realGit.isEmpty());
        assertTrue(
                "RealGit is the adapter, so it must be what this rule would catch if it were not allowed",
                realGit.stream().anyMatch(file -> Bytecode.mentions(file, "java/lang/ProcessBuilder")));
    }

    @Test
    public void theRuleWouldNoticeAClassThatWentRoundProcessBuilderToRuntime() throws IOException {
        String here = OnlyTheAdapterStartsAProcessTest.class.getSimpleName();
        List<Path> thisRule = compiledClasses().stream()
                .filter(file -> Bytecode.outermost(file).equals(here))
                .toList();

        assertTrue(here + " was not compiled, so this rule judged nothing", !thisRule.isEmpty());
        assertTrue(
                "the other way to start one is Runtime.exec, and this rule is only as good as its eye for it",
                thisRule.stream().anyMatch(file -> mentionsAll(file, RUNTIME_EXEC)));
    }

    /** Never called: it is here so the rule above has a class reaching {@code Runtime.exec} to see. */
    @SuppressWarnings("unused")
    private static Process theOtherWayRound(String command) throws IOException {
        return Runtime.getRuntime().exec(new String[] {command});
    }

    @Test
    public void reachingRuntimeForAShutdownHookIsNotReachingForAProcess() throws IOException {
        List<Path> cost = compiledClasses().stream()
                .filter(file -> Bytecode.outermost(file).equals("Cost"))
                .toList();

        assertTrue("Cost was not compiled, so this rule judged nothing", !cost.isEmpty());
        assertTrue(
                "Cost asks Runtime for a shutdown hook, and this rule must not read that as starting a process",
                cost.stream().anyMatch(file -> Bytecode.mentions(file, "java/lang/Runtime")));
        assertTrue(
                "and it must not be caught by the rule",
                cost.stream().noneMatch(file -> mentionsAll(file, RUNTIME_EXEC)));
    }
}
