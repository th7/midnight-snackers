package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.junit.Test;

/**
 * The ledger is only worth reading if it is complete. Completeness is not a promise in a comment
 * here. One table says which classes spend each kind of cost; another says what each dear thing a
 * class could reach would be spent as. Together they say, for each dear thing, exactly which
 * classes may reach it -- and a class that reaches one without naming a kind it could be, or names
 * a kind without reaching what it is spent on, fails. Both halves are read off the bytecode, where
 * a fully qualified name cannot dodge either.
 */
public class CostIsCountedWhereItIsSpentTest {

    private static final String TEAMCODE = "org/firstinspires/ftc/teamcode";

    private static final String COST = "org/firstinspires/ftc/teamcode/sim/Cost";

    /** Who spends each kind. Nothing else may name it, and nothing here may fail to. */
    private static final Map<Cost.Kind, Set<String>> SPENT_BY = new LinkedHashMap<>(Map.of(
            Cost.Kind.CHILD_JVM, Set.of("JvmChild"),
            Cost.Kind.COMPILE, Set.of("SimBuild", "SourceNavigator", "MainSources"),
            Cost.Kind.GIT, Set.of("RealGit", "GitFixture"),
            Cost.Kind.PASSWORD_HASH, Set.of("CodingServer", "CodingServerTest"),
            Cost.Kind.FORMAT, Set.of("JavaFormatter"),
            Cost.Kind.NODE, Set.of("SimReplayPageTest", "CodingServerTest"),
            Cost.Kind.PROJECT_COPY, Set.of("SimProject")));

    /** What is dear to reach, and which kinds of cost reaching it is spent as. */
    private static final Map<String, Set<Cost.Kind>> REACHES = new LinkedHashMap<>(Map.of(
            "java/lang/ProcessBuilder", Set.of(Cost.Kind.CHILD_JVM, Cost.Kind.GIT, Cost.Kind.NODE),
            "javax/tools/ToolProvider", Set.of(Cost.Kind.COMPILE),
            "org/bouncycastle/crypto/generators/SCrypt", Set.of(Cost.Kind.PASSWORD_HASH),
            "com/palantir/javaformat/java/Formatter", Set.of(Cost.Kind.FORMAT)));

    /** The classes that may reach something dear: exactly those that count what it would cost. */
    static Set<String> mayReach(String reference) {
        Set<String> spenders = new TreeSet<>();
        for (Cost.Kind kind : REACHES.get(reference)) {
            spenders.addAll(SPENT_BY.get(kind));
        }
        return spenders;
    }

    private static Set<String> beyondTheRules(Set<String> found) {
        Set<String> rest = new TreeSet<>(found);
        rest.removeAll(OnlyTheAdapterStartsAProcessTest.HOLD_THE_RULES);
        return rest;
    }

    private static List<Path> classes() {
        List<Path> classes = Bytecode.classesUnder(TEAMCODE);
        assertTrue("no compiled classes were found, so this rule judged nothing", classes.size() > 20);
        return classes;
    }

    @Test
    public void whatReachesSomethingDearIsExactlyWhatCountsWhatItWouldCost() {
        List<Path> classes = classes();

        for (String reference : REACHES.keySet()) {
            Set<String> reaching = beyondTheRules(Bytecode.thatMentionAny(classes, List.of(reference)));

            assertEquals(
                    reference + " is dear. What reaches it is what counts it: a class here and not in SPENT_BY"
                            + " spends without counting, and one in SPENT_BY and not here counts what it never"
                            + " spends.",
                    mayReach(reference),
                    reaching);
        }
    }

    @Test
    public void eachKindIsSpentWhereThisSaysAndNowhereElse() {
        List<Path> classes = classes();

        for (Cost.Kind kind : Cost.Kind.values()) {
            Set<String> naming = beyondTheRules(Bytecode.thatMentionAny(classes, List.of(kind.name())));

            assertEquals(
                    kind.key + " is spent somewhere this rule does not say. A kind of cost is worth having"
                            + " only while the ledger says where it went.",
                    new TreeSet<>(SPENT_BY.getOrDefault(kind, Set.of())),
                    naming);
        }
    }

    @Test
    public void everythingThatCountsReachesCostToDoIt() {
        Set<String> counting = beyondTheRules(Bytecode.thatMentionAny(classes(), List.of(COST)));
        Set<String> spenders = new TreeSet<>();
        SPENT_BY.values().forEach(spenders::addAll);

        assertEquals("what names Cost is what this rule says spends", spenders, counting);
    }

    @Test
    public void everyKindOfCostHasSomewhereItIsSpent() {
        for (Cost.Kind kind : Cost.Kind.values()) {
            assertTrue(
                    kind + " is not spent anywhere, so its line in the budget can never move",
                    SPENT_BY.containsKey(kind) && !SPENT_BY.get(kind).isEmpty());
        }
        assertEquals(Cost.Kind.values().length, SPENT_BY.size());
    }

    @Test
    public void theRuleWouldNoticeSomethingThatReachedOneWithoutCounting() {
        List<Path> classes = classes();
        List<Path> realGit = classes.stream()
                .filter(file -> Bytecode.outermost(file).equals("RealGit"))
                .toList();

        assertTrue("RealGit was not compiled, so this rule judged nothing", !realGit.isEmpty());
        assertTrue(
                "RealGit is the adapter that runs git, so it is what both halves of this rule must see",
                realGit.stream().anyMatch(file -> Bytecode.mentions(file, "java/lang/ProcessBuilder")));
        assertTrue(
                "and it must be seen to count it",
                realGit.stream().anyMatch(file -> Bytecode.mentions(file, Cost.Kind.GIT.name())));
        assertTrue(
                "a class that counts nothing must read as counting nothing",
                classes.stream()
                        .filter(file -> Bytecode.outermost(file).equals("SimField"))
                        .noneMatch(file -> Bytecode.mentions(file, COST)));
    }
}
