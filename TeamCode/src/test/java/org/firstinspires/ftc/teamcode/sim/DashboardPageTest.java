package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

public class DashboardPageTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final String PAGE = SimAssets.page("dashboard.html");

    @Test
    public void theDashboardIsTheSimulatorsTabsAndNothingElse() {
        assertTrue(PAGE, PAGE.contains("data-tab=\"simulate\" aria-selected=\"true\""));
        assertTrue(PAGE, PAGE.contains("<main id=\"simulate\" class=\"panel\" role=\"tabpanel\">"));
        for (String gone : new String[] {
            "data-tab=\"edit\"", "id=\"editor\"", "codemirror", "/static/", "/files", "/git/", "/deploy", "data-host"
        }) {
            assertFalse("the page offers nothing it no longer does: " + gone, PAGE.contains(gone));
        }
        assertTrue(PAGE, PAGE.replaceAll("\\s+", " ").contains("[hidden] { display: none !important; }"));
    }

    @Test
    public void theSimulateTabListsRunsAndOpensTheLiveView() {
        assertTrue(PAGE, PAGE.contains("id=\"opmodes\""));
        assertTrue(PAGE, PAGE.contains("id=\"history\""));
        assertTrue(PAGE, PAGE.contains("id=\"stage\""));
        assertTrue(PAGE, PAGE.contains("'/sim/catalog'"));
        assertTrue(PAGE, PAGE.contains("'/sim/run?opmode='"));
        assertTrue(PAGE, PAGE.contains("'/sim/status'"));
        assertTrue(PAGE, PAGE.contains("'/sim/runs/'"));
        assertTrue(PAGE, PAGE.contains("'/sim/place?opmode='"));
        assertTrue(PAGE, PAGE.contains("location.hash"));
    }

    @Test
    public void theSimulateTabShowsBuildingAndTheRunsMessageAndRefreshesTheCatalog() {
        assertTrue(PAGE, PAGE.contains("run.phase === 'building'"));
        assertTrue(PAGE, PAGE.contains("run.message"));
        assertTrue(PAGE, PAGE.contains("id=\"run-message\""));
        assertTrue(PAGE, PAGE.contains("as last saved"));
        assertFalse(PAGE, PAGE.contains("started with"));
        assertTrue(PAGE, PAGE.split("fetch\\('/sim/catalog'\\)").length - 1 >= 1);
        assertTrue(PAGE, PAGE.contains("loadCatalog()"));
        assertTrue(PAGE, PAGE.contains("wasRunning && !running"));
    }

    @Test
    public void theDashboardHasALaunchSimTabAfterSimulate() {
        assertTrue(PAGE, PAGE.contains("data-tab=\"launch\" aria-selected=\"false\" aria-controls=\"launch\""));
        assertTrue(PAGE, PAGE.contains("<main id=\"launch\" class=\"panel\" role=\"tabpanel\" hidden>"));
        assertTrue(PAGE, PAGE.contains("id=\"launch-stage\""));
        assertTrue(PAGE, PAGE.contains("'/sim/launch/settings'"));
        assertTrue(PAGE, PAGE.contains("'/sim/launch/preview'"));
        assertTrue(PAGE, PAGE.contains("'/sim/launch'"));
        assertTrue(PAGE, PAGE.contains("'/sim/launches'"));
        assertTrue(PAGE, PAGE.contains("'/sim/launches/'"));
        assertTrue(
                "the Launch Sim tab comes after Simulate",
                PAGE.indexOf("data-tab=\"simulate\"") < PAGE.indexOf("data-tab=\"launch\""));
        int launchSim = PAGE.indexOf("part('the launch sim', ");
        assertTrue("the launch sim starts as a part of its own", launchSim >= 0);
        assertTrue(
                "and before the tabs, so the tab it is opened on can enter it",
                launchSim < PAGE.indexOf("part('the tabs', wireTabs)"));
    }

    @Test
    public void theDashboardHasASpindexerSimTabAfterLaunchSim() {
        assertTrue(PAGE, PAGE.contains("data-tab=\"spindexer\" aria-selected=\"false\" aria-controls=\"spindexer\""));
        assertTrue(PAGE, PAGE.contains("<main id=\"spindexer\" class=\"panel\" role=\"tabpanel\" hidden>"));
        assertTrue(PAGE, PAGE.contains("id=\"spindexer-stage\""));
        assertTrue(PAGE, PAGE.contains("'/sim/feed/settings'"));
        assertTrue(PAGE, PAGE.contains("'/sim/feed'"));
        assertTrue(PAGE, PAGE.contains("'/sim/feeds'"));
        assertTrue(PAGE, PAGE.contains("'/sim/feeds/'"));
        assertTrue(
                "the Spindexer Sim tab comes after Launch Sim",
                PAGE.indexOf("data-tab=\"launch\"") < PAGE.indexOf("data-tab=\"spindexer\""));
        int spindexerSim = PAGE.indexOf("part('the spindexer sim', ");
        assertTrue("the spindexer sim starts as a part of its own", spindexerSim >= 0);
        assertTrue(
                "and before the tabs, so the tab it is opened on can enter it",
                spindexerSim < PAGE.indexOf("part('the tabs', wireTabs)"));
    }

    @Test
    public void aTabRemembersWhatWasAskedOfItAndATabNobodyHasIsSimulate() throws Exception {
        String[] asked = {
            "#simulate",
            "#simulate?resolution=low",
            "#simulate?view=camera&cost",
            "#edit",
            "#",
            "",
            "#nonsense",
            "#edit?resolution=low",
            "#launch",
            "#launch?resolution=low",
            "#spindexer",
            "#spindexer?resolution=low"
        };

        String[] got = tabsOnTheDashboard(asked);

        assertEquals("simulate ", got[0]);
        assertEquals("a tab keeps the options asked of it", "simulate resolution=low", got[1]);
        assertEquals("simulate view=camera&cost", got[2]);
        assertEquals("a link to the Edit tab that was opens the simulator", "simulate ", got[3]);
        assertEquals("simulate ", got[4]);
        assertEquals("simulate ", got[5]);
        assertEquals("a tab nobody has is the simulator", "simulate ", got[6]);
        assertEquals("and options only mean something to the tab that reads them", "simulate ", got[7]);
        assertEquals("the Launch Sim tab is a tab of its own", "launch ", got[8]);
        assertEquals("and its view is drawn as it is asked to be", "launch resolution=low", got[9]);
        assertEquals("the Spindexer Sim tab is a tab of its own", "spindexer ", got[10]);
        assertEquals("spindexer resolution=low", got[11]);
    }

    private String[] tabsOnTheDashboard(String[] asked) throws Exception {
        Matcher rule = Pattern.compile("\n  function tabOf\\(hash\\) \\{.*?\n  \\}", Pattern.DOTALL)
                .matcher(PAGE);
        assertTrue("the dashboard works a tab out in tabOf(hash)", rule.find());
        Path script = folder.newFile("tabOf.js").toPath();
        StringBuilder source = new StringBuilder(rule.group()).append("\n");
        source.append("const asked = ").append(new Gson().toJson(asked)).append(";\n");
        source.append("console.log(JSON.stringify(asked.map(h => { const t = tabOf(h); ")
                .append("return t.name + ' ' + t.options; })));\n");
        Files.write(script, source.toString().getBytes(StandardCharsets.UTF_8));
        return new Gson().fromJson(inNode(script), String[].class);
    }

    private static String inNode(Path script) throws Exception {
        String out;
        try (Cost.Spent spent = Cost.start(Cost.Kind.NODE)) {
            Process node = new ProcessBuilder("node", script.toString())
                    .redirectErrorStream(true)
                    .start();
            out = new String(node.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
            assertEquals("node ran the dashboard's rule: " + out, 0, node.waitFor());
        }
        return out.trim();
    }

    @Test
    public void theViewIsOpenedAgainWhenWhatWasAskedOfItChanges() throws Exception {
        String[][] asked = {
            {"null", "null", "1", ""},
            {"1", "", "1", ""},
            {"1", "", "1", "resolution=low"},
            {"1", "resolution=low", "1", "resolution=low"},
            {"1", "resolution=low", "2", "resolution=low"},
            {"1", "resolution=low", "1", ""}
        };

        boolean[] opens = stageOpenings(asked);

        assertTrue("nothing shown yet", opens[0]);
        assertFalse("the same run, asked the same way, is already open", opens[1]);
        assertTrue("the same run asked a different way is opened again", opens[2]);
        assertFalse(opens[3]);
        assertTrue("another run", opens[4]);
        assertTrue("and dropping an option is a change too", opens[5]);
    }

    private boolean[] stageOpenings(String[][] asked) throws Exception {
        Matcher rule = Pattern.compile(
                        "\n  function stageNeedsOpening\\(shownId, shownOptions, id, options\\) \\{.*?\n  \\}",
                        Pattern.DOTALL)
                .matcher(PAGE);
        assertTrue("the dashboard decides in stageNeedsOpening(...)", rule.find());
        Path script = folder.newFile("stageNeedsOpening.js").toPath();
        StringBuilder source = new StringBuilder(rule.group()).append("\n");
        source.append("const asked = ").append(new Gson().toJson(asked)).append(";\n");
        source.append("console.log(JSON.stringify(asked.map(a => stageNeedsOpening(")
                .append("a[0] === 'null' ? null : Number(a[0]), a[1] === 'null' ? null : a[1], ")
                .append("Number(a[2]), a[3]))));\n");
        Files.write(script, source.toString().getBytes(StandardCharsets.UTF_8));
        return new Gson().fromJson(inNode(script), boolean[].class);
    }

    @Test
    public void theSimulateTabPassesWhatWasAskedOfItToTheLiveView() {
        assertTrue("the live view is opened with them", PAGE.contains("withTabOptions('/sim/runs/'"));
        assertTrue(PAGE.contains("function withTabOptions(url)"));
    }
}
