package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.sim.TestAutos.ChattyAuto;
import org.firstinspires.ftc.teamcode.sim.TestAutos.ThreeLoopAuto;
import org.firstinspires.ftc.teamcode.sim.TestTeleOps.StickTeleOp;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
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

    /** A project's child runs that project and the libraries: with no simulator in the project, there is no child to run. */
    @Test
    public void aChildOverAProjectRunsOnThatProjectAndTheLibrariesAlone() throws Exception {
        Path empty = folder.getRoot().toPath().resolve("empty");
        Files.createDirectories(empty);

        Output output = run(SimChild.launch(empty, "--list"));

        assertNotEquals("this server's own SimChild must not stand in for the project's", 0, output.exitCode);
        assertTrue(output.stderr, output.stderr.contains(SimChild.class.getName()));
    }

    @Test
    public void listPrintsTheRealAutosAsOneJsonLine() throws Exception {
        Output output = run(SimChild.launchOnThisClasspath("--list"));

        assertEquals(output.stderr, 0, output.exitCode);
        assertEquals(output.stdout.toString(), 2, output.stdout.size());
        assertEquals("the child says its protocol before anything else", SimRunStream.hello(), output.stdout.get(0));
        SimCatalog catalog = SimCatalog.fromJson(new Gson().fromJson(output.stdout.get(1), com.google.gson.JsonArray.class));
        assertTrue(catalog.find("driveForward").isPresent());
        assertEquals("Plans.driveForward()", catalog.find("driveForward").get().where);
        assertTrue(catalog.find("RedTeleOp").isPresent());
        assertEquals("auto", catalog.find("driveForward").get().kind);
        assertEquals("teleop", catalog.find("RedTeleOp").get().kind);
    }

    @Test
    public void aTeleOpRunTakesTheDriverStationFromStdinAndStopEndsIt() throws Exception {
        Process child = SimChild.launchOnThisClasspath("--run", "Stick", "30", folder.getRoot().toString(), StickTeleOp.class.getName());
        Thread drain = new Thread(() -> {
            try {
                child.getErrorStream().transferTo(java.io.OutputStream.nullOutputStream());
            } catch (IOException ignored) {
                // the child went away
            }
        });
        drain.start();
        Writer in = new OutputStreamWriter(child.getOutputStream(), StandardCharsets.UTF_8);
        BufferedReader out = new BufferedReader(new InputStreamReader(child.getInputStream(), StandardCharsets.UTF_8));
        Gson gson = new Gson();

        in.write("{\"gamepad\": 1, \"state\": {\"left_stick_y\": -1}}\n");
        in.flush();
        assertEquals(SimRunStream.hello(), out.readLine());
        String first = out.readLine();
        assertTrue(String.valueOf(first), first != null && gson.fromJson(first, JsonObject.class).has("started"));
        long deadline = System.nanoTime() + 20_000_000_000L;
        boolean sawTheStick = false;
        JsonObject tick;
        do {
            String line = out.readLine();
            assertTrue("the child ended before it drove anywhere", line != null);
            tick = gson.fromJson(line, JsonObject.class);
            assertFalse(tick.toString(), tick.has("outcome"));
            if (tick.has("gamepads") && tick.getAsJsonObject("gamepads").has("1")) {
                sawTheStick |= tick.getAsJsonObject("gamepads").getAsJsonObject("1").get("left_stick_y").getAsDouble() == -1;
            }
            assertTrue("never drove forward", System.nanoTime() < deadline);
        } while (tick.get("x").getAsDouble() < 6);
        assertTrue("the ticks carry the driver's inputs", sawTheStick);

        in.write("{\"stop\": true}\n");
        in.flush();
        String last = null;
        for (String line = out.readLine(); line != null; line = out.readLine()) {
            last = line;
        }
        assertTrue("child did not exit", child.waitFor(20, TimeUnit.SECONDS));
        drain.join(5000);
        assertEquals(0, child.exitValue());
        assertEquals("stopped", gson.fromJson(last, JsonObject.class).get("outcome").getAsString());
        assertTrue(Files.isRegularFile(folder.getRoot().toPath().resolve("Stick.html")));
    }

    @Test
    public void aLineTheChildCannotReadEndsTheRunWithThatAsItsOutcome() throws Exception {
        Process child = SimChild.launchOnThisClasspath("--run", "Stick", "30", folder.getRoot().toString(), StickTeleOp.class.getName());
        Writer in = new OutputStreamWriter(child.getOutputStream(), StandardCharsets.UTF_8);
        in.write("{\"gamepad\": 1, \"state\": {\"corss\": true}}\n");
        in.flush();

        Output output = run(child);

        JsonObject last = new Gson().fromJson(output.stdout.get(output.stdout.size() - 1), JsonObject.class);
        assertTrue(last.toString(), last.get("outcome").getAsString().contains("corss"));
    }

    @Test
    public void runStreamsATickPerLoopAndThenTheOutcome() throws Exception {
        Output output = run(SimChild.launchOnThisClasspath("--run", "Count to three", "2", folder.getRoot().toString(), ThreeLoopAuto.class.getName()));

        assertEquals(output.stderr, 0, output.exitCode);
        assertEquals(output.stdout.toString(), 6, output.stdout.size());
        assertEquals(SimRunStream.hello(), output.stdout.get(0));
        Gson gson = new Gson();
        assertTrue("the child says when the op mode starts, so its own startup is not the run's time",
                gson.fromJson(output.stdout.get(1), JsonObject.class).get("started").getAsBoolean());
        for (int i = 2; i < 5; i++) {
            JsonObject tick = gson.fromJson(output.stdout.get(i), JsonObject.class);
            assertTrue(tick.toString(), tick.has("t") && tick.has("x") && tick.has("step") && tick.has("packets"));
        }
        JsonObject last = gson.fromJson(output.stdout.get(5), JsonObject.class);
        assertEquals("done", last.get("outcome").getAsString());
        assertTrue(Files.isRegularFile(folder.getRoot().toPath().resolve("Count to three.html")));
    }

    @Test
    public void whatTheOpModePrintsGoesToStderrNotTheStream() throws Exception {
        Output output = run(SimChild.launchOnThisClasspath("--run", "Chatty", "2", folder.getRoot().toString(), ChattyAuto.class.getName()));

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
        Output output = run(SimChild.launchOnThisClasspath("--run", "Never done", "0.3", folder.getRoot().toString(), TestAutos.NeverDoneAuto.class.getName()));

        JsonObject last = new Gson().fromJson(output.stdout.get(output.stdout.size() - 1), JsonObject.class);
        assertTrue(last.toString(), last.get("outcome").getAsString().startsWith("timed out"));
    }

    @Test
    public void anUnknownOpModeIsAnOutcomeToo() throws Exception {
        Output output = run(SimChild.launchOnThisClasspath("--run", "org.example.Nope", "1", folder.getRoot().toString()));

        JsonObject last = new Gson().fromJson(output.stdout.get(output.stdout.size() - 1), JsonObject.class);
        assertTrue(last.toString(), last.get("outcome").getAsString().contains("org.example.Nope"));
    }
}
