package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.sim.SourceNavigator.Location;
import org.firstinspires.ftc.teamcode.sim.SourceNavigator.Symbol;
import org.firstinspires.ftc.teamcode.sim.SourceNavigator.Usages;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.util.List;

public class SourceNavigatorTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    static final String PLANS = "org/example/Plans.java";
    static final String AUTO = "org/example/Auto.java";

    static final String PLANS_SOURCE = "package org.example;\n"
            + "\n"
            + "public class Plans {\n"
            + "    private int loops = 0;\n"
            + "\n"
            + "    public static int count() {\n"
            + "        return 3;\n"
            + "    }\n"
            + "\n"
            + "    public int loops() {\n"
            + "        return loops;\n"
            + "    }\n"
            + "}\n";

    static final String AUTO_SOURCE = "package org.example;\n"
            + "\n"
            + "import java.util.List;\n"
            + "\n"
            + "public class Auto {\n"
            + "    private final Plans plans = new Plans();\n"
            + "\n"
            + "    public int run() {\n"
            + "        int total = Plans.count();\n"
            + "        total += plans.loops();\n"
            + "        List<String> names = List.of(\"a\");\n"
            + "        return total + names.size();\n"
            + "    }\n"
            + "}\n";

    private Path sourceRoot;
    private SourceNavigator navigator;

    @Before
    public void twoClassesThatUseEachOther() throws IOException {
        sourceRoot = folder.getRoot().toPath().resolve("src/main/java");
        write(PLANS, PLANS_SOURCE);
        write(AUTO, AUTO_SOURCE);
        navigator = new SourceNavigator(sourceRoot);
    }

    private void write(String file, String source) throws IOException {
        Path path = sourceRoot.resolve(file);
        Files.createDirectories(path.getParent());
        Files.write(path, source.getBytes(StandardCharsets.UTF_8));
        // a save within the same second as the previous one must still be noticed
        Files.setLastModifiedTime(path, FileTime.fromMillis(System.currentTimeMillis() + 2000));
    }

    /** The 1-based line and character column of the {@code nth} occurrence of {@code token} on the first line holding {@code lineText}. */
    static int[] at(String source, String lineText, String token, int nth) {
        String[] lines = source.split("\n");
        for (int i = 0; i < lines.length; i++) {
            if (lines[i].contains(lineText)) {
                int column = -1;
                for (int n = 0; n < nth; n++) {
                    column = lines[i].indexOf(token, column + 1);
                }
                if (column < 0) {
                    throw new AssertionError("no " + token + " on " + lines[i]);
                }
                return new int[] { i + 1, column + 1 };
            }
        }
        throw new AssertionError("no line holding " + lineText);
    }

    static int[] at(String source, String lineText, String token) {
        return at(source, lineText, token, 1);
    }

    private Symbol definitionAt(String file, String source, String lineText, String token) {
        int[] where = at(source, lineText, token);
        return navigator.definition(file, where[0], where[1]);
    }

    private Usages usagesAt(String file, String source, String lineText, String token) {
        int[] where = at(source, lineText, token);
        return navigator.usages(file, where[0], where[1]);
    }

    private static void assertLocation(String file, String source, String lineText, String token, Location location) {
        assertNotNull("no location", location);
        int[] where = at(source, lineText, token);
        assertEquals(file + ":" + where[0] + ":" + where[1], location.file + ":" + location.line + ":" + location.column);
        assertEquals(source.split("\n")[where[0] - 1].trim(), location.text);
    }

    // --- go to definition ---

    @Test
    public void theDefinitionOfAClassNamedInAnotherFile() {
        Symbol symbol = definitionAt(AUTO, AUTO_SOURCE, "private final Plans plans", "Plans");

        assertEquals("org.example.Plans", symbol.name);
        assertEquals("class", symbol.kind);
        assertLocation(PLANS, PLANS_SOURCE, "public class Plans {", "Plans", symbol.definition);
    }

    @Test
    public void theDefinitionOfAMethodFromItsCall() {
        Symbol symbol = definitionAt(AUTO, AUTO_SOURCE, "int total = Plans.count();", "count");

        assertEquals("org.example.Plans.count()", symbol.name);
        assertEquals("method", symbol.kind);
        assertLocation(PLANS, PLANS_SOURCE, "public static int count() {", "count", symbol.definition);
    }

    @Test
    public void theDefinitionOfAFieldFromItsUse() {
        Symbol symbol = definitionAt(PLANS, PLANS_SOURCE, "return loops;", "loops");

        assertEquals("org.example.Plans.loops", symbol.name);
        assertEquals("field", symbol.kind);
        assertLocation(PLANS, PLANS_SOURCE, "private int loops = 0;", "loops", symbol.definition);
    }

    @Test
    public void theDefinitionOfALocalVariable() {
        Symbol symbol = definitionAt(AUTO, AUTO_SOURCE, "total += plans.loops();", "total");

        assertEquals("total", symbol.name);
        assertEquals("local variable", symbol.kind);
        assertLocation(AUTO, AUTO_SOURCE, "int total = Plans.count();", "total", symbol.definition);
    }

    @Test
    public void aSymbolFromOutsideTheSourcesHasANameAndNoLocation() {
        Symbol symbol = definitionAt(AUTO, AUTO_SOURCE, "List<String> names", "List");

        assertEquals("java.util.List", symbol.name);
        assertEquals("interface", symbol.kind);
        assertNull(symbol.definition);
    }

    @Test
    public void whitespaceAndAKeywordHaveNoSymbol() {
        assertNull(navigator.definition(AUTO, 2, 1));
        assertNull(definitionAt(AUTO, AUTO_SOURCE, "public int run() {", "public"));
        assertNull(navigator.definition("org/example/Missing.java", 1, 1));
    }

    // --- find usages ---

    @Test
    public void theUsagesOfAMethodAreItsCallSitesNotItsDeclaration() {
        Usages fromDeclaration = usagesAt(PLANS, PLANS_SOURCE, "public static int count() {", "count");
        Usages fromCall = usagesAt(AUTO, AUTO_SOURCE, "int total = Plans.count();", "count");

        assertEquals("org.example.Plans.count()", fromDeclaration.symbol.name);
        assertEquals(1, fromDeclaration.usages.size());
        assertLocation(AUTO, AUTO_SOURCE, "int total = Plans.count();", "count", fromDeclaration.usages.get(0));
        assertEquals(fromDeclaration.usages.get(0).toString(), fromCall.usages.get(0).toString());
        assertLocation(PLANS, PLANS_SOURCE, "public static int count() {", "count", fromCall.symbol.definition);
    }

    @Test
    public void theUsagesOfAClassAreItsTypeUsesAndItsConstructions() {
        Usages usages = usagesAt(PLANS, PLANS_SOURCE, "public class Plans {", "Plans");

        assertEquals(usages.usages.toString(), 3, usages.usages.size());
        assertLocation(AUTO, AUTO_SOURCE, "private final Plans plans = new Plans();", "Plans", usages.usages.get(0));
        int[] construction = at(AUTO_SOURCE, "private final Plans plans = new Plans();", "Plans", 2);
        assertEquals(AUTO + ":" + construction[0] + ":" + construction[1], usages.usages.get(1).toString());
        assertLocation(AUTO, AUTO_SOURCE, "int total = Plans.count();", "Plans", usages.usages.get(2));
    }

    @Test
    public void theUsagesOfAFieldNeverIncludeItsDeclaration() {
        Usages usages = usagesAt(PLANS, PLANS_SOURCE, "private int loops = 0;", "loops");

        assertEquals("org.example.Plans.loops", usages.symbol.name);
        assertEquals(1, usages.usages.size());
        assertLocation(PLANS, PLANS_SOURCE, "return loops;", "loops", usages.usages.get(0));
    }

    @Test
    public void nothingUnderTheCursorMeansNoUsages() {
        assertNull(navigator.usages(AUTO, 2, 1));
    }

    // --- the tree as it is ---

    @Test
    public void aBrokenFileElsewhereStillAnswers() throws IOException {
        write("org/example/Broken.java", "package org.example;\npublic class Broken { int x = ; }\n");

        Symbol symbol = definitionAt(AUTO, AUTO_SOURCE, "int total = Plans.count();", "count");

        assertLocation(PLANS, PLANS_SOURCE, "public static int count() {", "count", symbol.definition);
    }

    @Test
    public void columnsCountCharactersNotTabStops() throws IOException {
        String tabbed = "package org.example;\n"
                + "public class Tabbed {\n"
                + "\tPlans first = null;\n"
                + "\t\tPlans second = null;\n"
                + "}\n";
        write("org/example/Tabbed.java", tabbed);

        Symbol symbol = definitionAt("org/example/Tabbed.java", tabbed, "\t\tPlans second", "Plans");
        Usages usages = usagesAt(PLANS, PLANS_SOURCE, "public class Plans {", "Plans");

        assertEquals("org.example.Plans", symbol.name);
        assertEquals(5, usages.usages.size());
        assertLocation("org/example/Tabbed.java", tabbed, "\tPlans first", "Plans", usages.usages.get(3));
        assertEquals(2, usages.usages.get(3).column);
        assertEquals(3, usages.usages.get(4).column);
    }

    @Test
    public void anEditIsNoticed() throws IOException {
        assertEquals(1, usagesAt(PLANS, PLANS_SOURCE, "public static int count() {", "count").usages.size());

        String edited = AUTO_SOURCE.replace("return total + names.size();", "return total + names.size() + Plans.count();");
        write(AUTO, edited);

        Usages usages = usagesAt(PLANS, PLANS_SOURCE, "public static int count() {", "count");
        assertEquals(2, usages.usages.size());
        assertLocation(AUTO, edited, "return total + names.size() + Plans.count();", "count", usages.usages.get(1));
    }

    @Test
    public void theFilesAreTheSourceKeysSorted() throws IOException {
        write("org/example/sub/Deep.java", "package org.example.sub;\npublic class Deep {}\n");
        Files.write(sourceRoot.resolve("org/example/notes.txt"), "not a source\n".getBytes(StandardCharsets.UTF_8));

        List<String> files = navigator.files();

        assertEquals("[org/example/Auto.java, org/example/Plans.java, org/example/sub/Deep.java]", files.toString());
    }
}
