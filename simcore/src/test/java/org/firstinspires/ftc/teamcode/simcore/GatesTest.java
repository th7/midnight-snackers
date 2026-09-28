package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.nio.file.Path;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class GatesTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static String inTheCore(String body) {
        return "package " + Gates.corePackage() + ";\n\n" + "public final class Breach {\n" + body + "\n}\n";
    }

    private Path dir() {
        return folder.getRoot().toPath();
    }

    private Gates.Verdict javac(String body) {
        return Gates.javac(dir(), "Breach", inTheCore(body));
    }

    private Gates.Verdict pmd(String body) {
        return Gates.pmd(dir(), "Breach", inTheCore(body));
    }

    private Gates.Verdict forbidden(String body) {
        return Gates.forbiddenApis(dir(), "Breach", inTheCore(body));
    }

    private static void assertRefused(String reason, Gates.Verdict verdict) {
        assertTrue("refused for " + reason + ", but " + verdict, verdict.refusedFor(reason));
    }

    private static final String CLEAN = "    public static final double HALF = 0.5;\n"
            + "    public static final String NAME = \"clean\";\n"
            + "\n"
            + "    public record Pair(double a, double b) {}\n"
            + "\n"
            + "    public static double mean(java.util.List<Pair> pairs) {\n"
            + "        double sum = 0;\n"
            + "        for (Pair pair : pairs) {\n"
            + "            sum += (pair.a() + pair.b()) * HALF;\n"
            + "        }\n"
            + "        return pairs.isEmpty() ? 0 : sum / pairs.size();\n"
            + "    }\n"
            + "\n"
            + "    public static int halfOf(int count) {\n"
            + "        return count / 2 + count % 2;\n"
            + "    }\n"
            + "\n"
            + "    public static java.util.List<Pair> two(double a, double b) {\n"
            + "        java.util.List<Pair> out = new java.util.ArrayList<>();\n"
            + "        out.add(new Pair(a, b));\n"
            + "        out.add(new Pair(b, a));\n"
            + "        return out;\n"
            + "    }\n"
            + "\n"
            + "    public static String named(java.util.Optional<String> name) {\n"
            + "        return name.orElse(NAME) + Math.sqrt(2);\n"
            + "    }\n";

    @Test
    public void codeThatBreaksNoRulePassesEveryGate() {
        Gates.Verdict javac = javac(CLEAN);
        assertTrue("javac and Error Prone: " + javac, javac.passed);
        Gates.Verdict pmd = pmd(CLEAN);
        assertTrue("PMD: " + pmd, pmd.passed);
        Gates.Verdict forbidden = forbidden(CLEAN);
        assertTrue("forbiddenapis: " + forbidden, forbidden.passed);
    }

    @Test
    public void noGateIsSetToLetThroughWhatItRefuses() {
        assertFalse("pmdMain ignores its failures", Gates.ignoresFailures("pmd"));
        assertFalse("forbiddenApisMain ignores its failures", Gates.ignoresFailures("forbidden"));
    }

    @Test
    public void whatTheCoreIsNotGivenCannotBeImported() {
        assertRefused(
                "package com.google.gson does not exist",
                Gates.javac(
                        dir(),
                        "Breach",
                        "package " + Gates.corePackage() + ";\n\nimport com.google.gson.Gson;\n\n"
                                + "public final class Breach {\n    public static Object gson() {\n"
                                + "        return new Gson();\n    }\n}\n"));
    }

    @Test
    public void aNullThatCouldBeDereferencedIsRefused() {
        assertRefused(
                "[NullAway]",
                javac("    public static int length(boolean empty) {\n"
                        + "        String name = empty ? null : \"name\";\n"
                        + "        return name.length();\n"
                        + "    }\n"));
    }

    @Test
    public void aWarningIsRefused() {
        assertRefused(
                "warnings found and -Werror specified",
                javac("    public static java.util.List<?> raw() {\n"
                        + "        java.util.List list = new java.util.ArrayList();\n"
                        + "        return list;\n"
                        + "    }\n"));
    }

    @Test
    public void aStaticThatCanChangeIsRefusedByErrorProne() {
        assertRefused("[NonFinalStaticField]", javac("    public static double shared = 0;\n"));
    }

    @Test
    public void aThrowIsRefused() {
        assertRefused(
                "NoThrow",
                pmd("    public static double f(double x) {\n"
                        + "        if (x < 0) {\n"
                        + "            throw new IllegalArgumentException(\"negative\");\n"
                        + "        }\n"
                        + "        return x;\n"
                        + "    }\n"));
    }

    @Test
    public void anAssertIsRefused() {
        assertRefused(
                "NoAssert",
                pmd("    public static double f(double x) {\n        assert x >= 0;\n        return x;\n    }\n"));
    }

    @Test
    public void integerDivisionByAnythingButANonzeroLiteralIsRefused() {
        assertRefused(
                "NoIntegerDivision", pmd("    public static int f(int a, int b) {\n        return a / b;\n    }\n"));
        assertRefused(
                "NoIntegerDivision", pmd("    public static long f(long a, int b) {\n        return a % b;\n    }\n"));
        assertRefused("NoIntegerDivision", pmd("    public static int f(int a) {\n        return a / 0;\n    }\n"));
        assertRefused(
                "NoIntegerDivision",
                pmd("    public static int f(int a, int b) {\n        a /= b;\n        return a;\n    }\n"));
        assertRefused(
                "NoIntegerDivision",
                pmd("    static final int NONE = 0;\n\n"
                        + "    public static int f(int a) {\n        return a / NONE;\n    }\n"));
        assertTrue(pmd("    public static int f(int a) {\n        return a / 2 + a % 3;\n    }\n").passed);
        assertTrue(pmd("    public static double f(double a, int b) {\n        return a / b;\n    }\n").passed);
    }

    @Test
    public void indexingIsRefused() {
        assertRefused("NoIndexing", pmd("    public static double f(double[] xs) {\n        return xs[0];\n    }\n"));
    }

    @Test
    public void anArrayOfASizeIsRefused() {
        assertRefused(
                "NoSizedArray", pmd("    public static Object f(int n) {\n        return new double[n];\n    }\n"));
    }

    @Test
    public void aCastToAReferenceTypeIsRefused() {
        assertRefused(
                "NoReferenceCast", pmd("    public static String f(Object o) {\n        return (String) o;\n    }\n"));
        assertTrue(pmd("    public static int f(double x) {\n        return (int) x;\n    }\n").passed);
    }

    @Test
    public void anIteratorsNextIsRefusedOutsideAForEach() {
        assertRefused(
                "NoExplicitNext",
                pmd("    public static String f(java.util.List<String> xs) {\n"
                        + "        return xs.iterator().next();\n"
                        + "    }\n"));
    }

    @Test
    public void aLockIsRefused() {
        assertRefused("NoLock", pmd("    public static synchronized double f(double x) {\n        return x;\n    }\n"));
        assertRefused(
                "NoLock",
                pmd("    public static double f(Object o, double x) {\n"
                        + "        synchronized (o) {\n"
                        + "            return x;\n"
                        + "        }\n"
                        + "    }\n"));
    }

    @Test
    public void aStaticThatIsNotAConstantIsRefused() {
        assertRefused("NoGlobalState", pmd("    public static double shared = 0;\n"));
        assertRefused(
                "NoGlobalState",
                pmd("    public static final java.util.List<String> SHARED = new java.util.ArrayList<>();\n"));
    }

    @Test
    public void aClassOutsideTheCoresPackageIsRefused() {
        assertRefused(
                "InTheCorePackage",
                Gates.pmd(dir(), "Breach", "package somewhere.outside;\n\npublic final class Breach {}\n"));
    }

    @Test
    public void inputAndOutputAreRefused() {
        assertRefused(
                "No I/O",
                forbidden(
                        "    public static boolean f() {\n        return new java.io.File(\"x\").exists();\n    }\n"));
    }

    @Test
    public void theClockIsRefused() {
        assertRefused(
                "No clock", forbidden("    public static long f() {\n        return System.nanoTime();\n    }\n"));
    }

    @Test
    public void randomnessIsRefused() {
        assertRefused(
                "No randomness",
                forbidden(
                        "    public static double f() {\n        return new java.util.Random().nextDouble();\n    }\n"));
    }

    @Test
    public void threadsAreRefused() {
        assertRefused(
                "No threads",
                forbidden("    public static String f() {\n        return Thread.currentThread().getName();\n    }\n"));
    }

    @Test
    public void lookingUpAClassIsRefused() {
        assertRefused(
                "No reflection",
                forbidden("    public static Object f(String name) throws Exception {\n"
                        + "        return Class.forName(name);\n"
                        + "    }\n"));
    }

    @Test
    public void arithmeticThatThrowsIsRefused() {
        assertRefused(
                "Throws on overflow",
                forbidden("    public static int f(int a, int b) {\n        return Math.addExact(a, b);\n    }\n"));
    }

    @Test
    public void aListsIndexIsRefused() {
        assertRefused(
                "Throws on a bad index",
                forbidden(
                        "    public static String f(java.util.List<String> xs) {\n        return xs.get(0);\n    }\n"));
    }

    @Test
    public void askingForWhatMayNotBeThereIsRefused() {
        assertRefused(
                "Throws when absent",
                forbidden(
                        "    public static String f(java.util.Optional<String> x) {\n        return x.get();\n    }\n"));
    }

    @Test
    public void aCollectionThatThrowsOnADuplicateIsRefused() {
        assertRefused(
                "Throws on a duplicate",
                forbidden(
                        "    public static Object f(String a, String b) {\n        return java.util.Set.of(a, b);\n    }\n"));
    }

    @Test
    public void streamsAreRefused() {
        assertRefused(
                "No streams",
                forbidden("    public static long f(java.util.List<String> xs) {\n"
                        + "        return xs.stream().count();\n"
                        + "    }\n"));
    }

    @Test
    public void parsingIsRefused() {
        assertRefused(
                "Parsing throws",
                forbidden("    public static int f(String s) {\n        return Integer.parseInt(s);\n    }\n"));
    }

    @Test
    public void formattingIsRefused() {
        assertRefused(
                "Formatting throws",
                forbidden("    public static String f(double x) {\n"
                        + "        return String.format(java.util.Locale.ROOT, \"%f\", x);\n"
                        + "    }\n"));
    }
}
