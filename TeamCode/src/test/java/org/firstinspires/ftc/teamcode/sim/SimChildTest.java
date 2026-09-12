package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.auto.DriveForward;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ChattyAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class SimChildTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final class Output {
        final List<String> stdout = new ArrayList<>();
        final String stderr;
        final int exitCode;

        Output(List<String> stdout, String stderr, int exitCode) {
            this.stdout.addAll(stdout);
            this.stderr = stderr;
            this.exitCode = exitCode;
        }
    }

    private static Output run(Process child) throws IOException, InterruptedException {
        List<String> stdout = new ArrayList<>();
        StringBuilder stderr = new StringBuilder();
        Thread errReader = new Thread(() -> {
            try (BufferedReader err = new BufferedReader(new InputStreamReader(child.getErrorStream(), StandardCharsets.UTF_8))) {
                for (String line = err.readLine(); line != null; line = err.readLine()) {
                    synchronized (stderr) {
                        stderr.append(line).append('\n');
                    }
                }
            } catch (IOException ignored) {
                // the child went away
            }
        });
        errReader.start();
        try (BufferedReader out = new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8))) {
            for (String line = out.readLine(); line != null; line = out.readLine()) {
                stdout.add(line);
            }
        }
        assertTrue("child did not exit", child.waitFor(20, TimeUnit.SECONDS));
        errReader.join(5000);
        synchronized (stderr) {
            return new Output(stdout, stderr.toString(), child.exitValue());
        }
    }

    @Test
    public void listPrintsTheRealAutosAsOneJsonLine() throws Exception {
        Output output = run(SimChild.launch(List.of(), "--list"));

        assertEquals(output.stderr, 0, output.exitCode);
        assertEquals(1, output.stdout.size());
        SimCatalog catalog = SimCatalog.fromJson(new Gson().fromJson(output.stdout.get(0), com.google.gson.JsonArray.class));
        assertTrue(catalog.find(DriveForward.class.getName()).isPresent());
        assertEquals("DriveForward", catalog.find(DriveForward.class.getName()).get().name);
    }

    @Test
    public void runStreamsATickPerLoopAndThenTheOutcome() throws Exception {
        Output output = run(SimChild.launch(List.of(), "--run", ThreeLoopAuto.class.getName(), "2", folder.getRoot().toString()));

        assertEquals(output.stderr, 0, output.exitCode);
        assertEquals(output.stdout.toString(), 4, output.stdout.size());
        Gson gson = new Gson();
        for (int i = 0; i < 3; i++) {
            JsonObject tick = gson.fromJson(output.stdout.get(i), JsonObject.class);
            assertTrue(tick.toString(), tick.has("t") && tick.has("x") && tick.has("step") && tick.has("packets"));
        }
        JsonObject last = gson.fromJson(output.stdout.get(3), JsonObject.class);
        assertEquals("done", last.get("outcome").getAsString());
        assertTrue(Files.isRegularFile(folder.getRoot().toPath().resolve("ThreeLoopAuto.html")));
    }

    @Test
    public void whatTheOpModePrintsGoesToStderrNotTheStream() throws Exception {
        Output output = run(SimChild.launch(List.of(), "--run", ChattyAuto.class.getName(), "2", folder.getRoot().toString()));

        assertEquals(output.stderr, 0, output.exitCode);
        assertTrue(output.stderr, output.stderr.contains("hello from the op mode"));
        Gson gson = new Gson();
        for (String line : output.stdout) {
            gson.fromJson(line, JsonObject.class);
        }
        assertFalse(output.stdout.toString(), output.stdout.toString().contains("hello from"));
        assertEquals("done", gson.fromJson(output.stdout.get(output.stdout.size() - 1), JsonObject.class).get("outcome").getAsString());
    }

    @Test
    public void aTimedOutRunStillReportsItsOutcome() throws Exception {
        Output output = run(SimChild.launch(List.of(), "--run", TestAutos.NeverDoneAuto.class.getName(), "0.3", folder.getRoot().toString()));

        JsonObject last = new Gson().fromJson(output.stdout.get(output.stdout.size() - 1), JsonObject.class);
        assertTrue(last.toString(), last.get("outcome").getAsString().startsWith("timed out"));
    }

    @Test
    public void anUnknownOpModeIsAnOutcomeToo() throws Exception {
        Output output = run(SimChild.launch(List.of(), "--run", "org.example.Nope", "1", folder.getRoot().toString()));

        JsonObject last = new Gson().fromJson(output.stdout.get(output.stdout.size() - 1), JsonObject.class);
        assertTrue(last.toString(), last.get("outcome").getAsString().contains("org.example.Nope"));
    }
}
