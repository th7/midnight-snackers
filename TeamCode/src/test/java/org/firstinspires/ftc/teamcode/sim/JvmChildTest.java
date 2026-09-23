package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import org.firstinspires.ftc.teamcode.sim.TestAutos.NeverDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestRogues.EnvironmentAuto;
import org.firstinspires.ftc.teamcode.sim.TestRogues.HogAuto;
import org.firstinspires.ftc.teamcode.sim.TestRogues.SpawnThenDoneAuto;
import org.firstinspires.ftc.teamcode.sim.TestRogues.SpawningAuto;
import org.firstinspires.ftc.teamcode.sim.TestRogues.StrayAuto;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

/**
 * What a teammate's op mode can reach from the child it runs in. Not a sandbox against someone
 * trying: a fence against the accidents a robot program makes on a laptop that is not a robot.
 */
public class JvmChildTest {
    @Rule
    public TemporaryFolder folder = new TemporaryFolder();

    private static final double SECONDS = 10;

    private final List<String> log = new ArrayList<>();
    private final List<Long> spawned = new ArrayList<>();

    @After
    public void endWhateverWasLeft() {
        for (long pid : spawned) {
            ProcessTree.kill(pid);
        }
    }

    private Child.Running start(JvmChild children, Class<?> opMode, String name) {
        return children.onThisClasspath(
                this::heard,
                "--run",
                name,
                String.valueOf(SECONDS),
                folder.getRoot().getAbsolutePath(),
                opMode.getName());
    }

    private void heard(String line) {
        synchronized (log) {
            log.add(line);
            if (line.startsWith("spawned ")) {
                spawned.add(Long.parseLong(line.substring("spawned ".length())));
            }
            log.notifyAll();
        }
    }

    private static void place(Child.Running child) {
        child.say(new Gson().toJson(SimDriverStation.startLine(new Pose2d(0, 0, 0))));
    }

    /** Places the robot and hears the child out to its end. */
    private void runToTheEnd(Child.Running child) {
        place(child);
        while (child.hear() != null) {}
        assertTrue("the child did not end", child.endedWithin(30));
        child.close();
    }

    private List<String> logged(String prefix) {
        synchronized (log) {
            return log.stream()
                    .filter(line -> line.startsWith(prefix))
                    .map(line -> line.substring(prefix.length()))
                    .toList();
        }
    }

    private long awaitSpawned() throws InterruptedException {
        long deadline = System.nanoTime() + 20_000_000_000L;
        synchronized (log) {
            while (spawned.isEmpty()) {
                long left = (deadline - System.nanoTime()) / 1_000_000;
                assertTrue("the op mode never said what it spawned: " + log, left > 0);
                log.wait(left);
            }
            return spawned.get(0);
        }
    }

    private static boolean endsWithin(long pid, double seconds) throws InterruptedException {
        return ProcessTree.endsWithin(pid, seconds);
    }

    @Test
    public void whatAnOpModeWritesWhereItStandsLandsInTheChildsOwnScratchAndGoesWithIt() throws Exception {
        List<Path> serversOwn = List.of(
                Paths.get(TestRogues.STRAY).toAbsolutePath(),
                Paths.get(System.getProperty("user.home"), TestRogues.STRAY),
                Paths.get(System.getProperty("java.io.tmpdir"), TestRogues.STRAY));
        try {
            runToTheEnd(start(new JvmChild(), StrayAuto.class, "Stray"));

            for (Path path : serversOwn) {
                assertFalse(path + " is the server's, and the op mode wrote there", Files.exists(path));
            }
            List<String> cwd = logged("cwd ");
            assertEquals(log.toString(), 1, cwd.size());
            Path scratch = Paths.get(cwd.get(0));
            long deadline = System.nanoTime() + 5_000_000_000L;
            while (Files.exists(scratch) && System.nanoTime() < deadline) {
                Thread.sleep(50);
            }
            assertFalse(scratch + " outlived the child", Files.exists(scratch));
        } finally {
            for (Path path : serversOwn) {
                Files.deleteIfExists(path);
            }
        }
    }

    @Test
    public void theChildSeesNoneOfTheServersEnvironmentBeyondWhatAJvmNeeds() {
        Set<String> servers = new TreeSet<>(System.getenv().keySet());
        servers.removeAll(JvmChild.PASSED_THROUGH);
        servers.removeAll(JvmChild.SET_TO_SCRATCH);
        assertFalse("the server has nothing to keep from the child, so this judged nothing", servers.isEmpty());

        runToTheEnd(start(new JvmChild(), EnvironmentAuto.class, "Environment"));

        Set<String> seen = new TreeSet<>();
        for (String variable : logged("env ")) {
            String name = variable.substring(0, variable.indexOf('='));
            seen.add(name);
            String value = variable.substring(name.length() + 1);
            if (JvmChild.SET_TO_SCRATCH.contains(name) && System.getenv(name) != null) {
                assertFalse(name + " is still the server's", value.equals(System.getenv(name)));
            }
        }
        assertFalse("the op mode said no environment at all: " + log, seen.isEmpty());
        seen.retainAll(servers);
        assertEquals("the child saw these of the server's", Set.of(), seen);
    }

    @Test
    public void killingAChildEndsWhatItStartedToo() throws Exception {
        Child.Running child = start(new JvmChild(), SpawningAuto.class, "Spawns and runs");
        place(child);
        long sleeper = awaitSpawned();

        child.kill();

        assertTrue("the child is still running", child.endedWithin(10));
        assertTrue("what the op mode started outlived the child it was killed with", endsWithin(sleeper, 10));
        child.close();
    }

    @Test
    public void whatAnOpModeStartedEndsWhenItsChildFinishes() throws Exception {
        runToTheEnd(start(new JvmChild(), SpawnThenDoneAuto.class, "Spawns and ends"));

        assertEquals(log.toString(), 1, spawned.size());
        assertTrue("what the op mode started outlived the run", endsWithin(spawned.get(0), 10));
    }

    @Test
    public void anOpModeThatAllocatesWithoutEndIsStoppedAtTheChildsHeapNotTheHosts() {
        runToTheEnd(start(new JvmChild(), HogAuto.class, "Hog"));

        int most = logged("held ").stream().mapToInt(Integer::parseInt).max().orElse(0);
        assertTrue("the op mode never allocated: " + log, most > 0);
        assertTrue(
                "the op mode held " + most + " MB, beyond the child's " + JvmChild.HEAP_MB + " MB",
                most <= JvmChild.HEAP_MB);
    }

    @Test
    public void noMoreChildrenRunThanTheServerMayRunAtOnce() {
        JvmChild children = new JvmChild(1);
        Child.Running first = start(children, NeverDoneAuto.class, "Never done");

        Child.CouldNotStart refused =
                assertThrows(Child.CouldNotStart.class, () -> start(children, NeverDoneAuto.class, "Never done"));
        assertTrue(refused.getMessage(), refused.getMessage().contains("1 "));

        first.kill();
        assertTrue(first.endedWithin(10));
        first.close();
        Child.Running third = start(children, NeverDoneAuto.class, "Never done");
        third.kill();
        assertTrue(third.endedWithin(10));
        third.close();
    }

    @Test
    public void whenTheServerExitsTheChildrenItLeftRunningGoWithItAndTheirScratchToo() throws Exception {
        Process parent = TestRogues.startParent();
        String said;
        String where;
        try (BufferedReader out =
                new BufferedReader(new InputStreamReader(parent.getInputStream(), StandardCharsets.UTF_8))) {
            said = out.readLine();
            where = out.readLine();
        }
        assertNotNull("the parent never said its child", said);
        assertTrue(said, said.startsWith("child "));
        long child = Long.parseLong(said.substring("child ".length()));
        spawned.add(child);
        assertNotNull("the parent never said where its child stands", where);
        assertTrue(where, where.startsWith("scratch "));
        Path scratch = Paths.get(where.substring("scratch ".length()));
        assertTrue("the parent did not exit", parent.waitFor(20, java.util.concurrent.TimeUnit.SECONDS));

        assertTrue("the child of a server that exited is still running", endsWithin(child, 10));
        assertFalse(scratch + " outlived the server that made it", Files.exists(scratch));
    }
}
