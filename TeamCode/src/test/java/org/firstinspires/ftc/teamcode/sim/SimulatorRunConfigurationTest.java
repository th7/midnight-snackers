package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class SimulatorRunConfigurationTest {
    private static final Path RUN_CONFIGURATION = Path.of("..", ".run", "Simulator.run.xml");
    private static final String TASK = ":TeamCode:simulator";

    private static Element configuration() throws Exception {
        Document document =
                DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(RUN_CONFIGURATION.toFile());
        return (Element) document.getElementsByTagName("configuration").item(0);
    }

    private static Element option(Element within, String name) {
        NodeList options = within.getElementsByTagName("option");
        for (int i = 0; i < options.getLength(); i++) {
            Element option = (Element) options.item(i);
            if (name.equals(option.getAttribute("name"))) {
                return option;
            }
        }
        throw new AssertionError(RUN_CONFIGURATION + " has no option " + name);
    }

    @Test
    public void androidStudiosSimulatorRunConfigurationRunsTheSimulatorTaskOverTheWholeCheckout() throws Exception {
        Element configuration = configuration();
        assertEquals("Simulator", configuration.getAttribute("name"));
        assertEquals("GradleRunConfiguration", configuration.getAttribute("type"));
        assertEquals(
                "$PROJECT_DIR$", option(configuration, "externalProjectPath").getAttribute("value"));

        NodeList named = option(configuration, "taskNames").getElementsByTagName("option");
        List<String> tasks = new ArrayList<>();
        for (int i = 0; i < named.getLength(); i++) {
            tasks.add(((Element) named.item(i)).getAttribute("value"));
        }
        assertEquals(List.of(TASK), tasks);
    }

    @Test
    public void theTaskItRunsStartsTheLocalSimulator() throws Exception {
        String build = new String(Files.readAllBytes(Path.of("build.gradle")), StandardCharsets.UTF_8);
        int task = build.indexOf("tasks.register('simulator', JavaExec)");
        assertTrue("TeamCode/build.gradle registers no " + TASK, task >= 0);
        int next = build.indexOf("tasks.register(", task + 1);
        String body = build.substring(task, next < 0 ? build.length() : next);
        assertTrue(body, body.contains("mainClass = '" + LocalSimulator.class.getName() + "'"));
        assertTrue(body, body.contains("workingDir = rootDir"));
    }
}
