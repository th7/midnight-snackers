package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.ftc.RawEncoder;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.roadrunner.ClockedOverflowEncoder;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.sim.SimRunner;
import org.junit.Test;

/**
 * Every timer the robot reads is the robot's clock, and that includes the one inside the dead
 * wheel encoders.
 *
 * <p>The hub reports an encoder's velocity in sixteen bits, so a wheel turning quickly wraps it,
 * and the count has to be put back together from how far the wheel turned in how long. The
 * <em>how long</em> is a clock, and if it is the wall clock then the robot's idea of its own speed
 * depends on how fast the machine happens to be running. In the simulator, where a loop is twenty
 * milliseconds of the robot's time and a fraction of a millisecond of the machine's, that is the
 * difference between a robot rolling to a stop and one doing a hundred and seventy inches a
 * second -- and so between a trajectory that has finished and one that has not.
 */
public class LocalizerReadsTheRobotsClockTest {
    /** Loops to drive before reading the speed. */
    private static final int LOOPS = 12;

    private static final float POWER = 0.6f;

    /** How far apart the counts are that the hub's sixteen bits cannot tell apart. */
    private static final int WRAP_STEP = 5 * 65536;

    /**
     * Faster than the hub's count can say even once the low bits have been read off, so recovering
     * it takes a second opinion about how fast the wheel is turning -- and that is where the clock
     * comes in. This is the regime a simulated robot's wheels appear to be in when their speed is
     * measured against the wall clock.
     */
    private static final int TRUE_TICKS_PER_SECOND = 400_000;

    /** What the hub would report for a wheel turning this fast: the count, wrapped into sixteen bits. */
    private static int asTheHubReports(int ticksPerSecond) {
        return ticksPerSecond & 0xFFFF;
    }

    /**
     * The count the encoder recovers for a wheel turning at {@link #TRUE_TICKS_PER_SECOND}, when
     * the clock it reads says each read is {@code nanosPerRead} apart.
     */
    private static int recoveredWithReadsApart(long nanosPerRead) {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        long[] nanos = {0};
        ClockedOverflowEncoder encoder = new ClockedOverflowEncoder(new RawEncoder(motor), () -> nanos[0]);

        int velocity = 0;
        // Three reads, which is enough to fill the encoder's rolling median.
        for (int i = 0; i < 3; i++) {
            nanos[0] += nanosPerRead;
            motor.currentPosition += (int) (TRUE_TICKS_PER_SECOND * 0.05);
            motor.measuredVelocity = asTheHubReports(TRUE_TICKS_PER_SECOND);
            velocity = encoder.getPositionAndVelocity().velocity;
        }
        return velocity;
    }

    @Test
    public void theEncoderRecoversTheHubsWrappedCountOnTheRobotsClock() {
        assertEquals("recovered ticks per second", TRUE_TICKS_PER_SECOND, recoveredWithReadsApart(50_000_000L));
    }

    /**
     * And the clock is really what it reads: told the same turning happened in a tenth of the time,
     * it recovers a different count. The failure the simulator saw, reproduced on purpose, so that
     * the test above is known to be checking something.
     */
    @Test
    public void theEncoderGetsItWrongWhenTheClockIsNotTheRobots() {
        int recovered = recoveredWithReadsApart(5_000_000L);

        assertNotEquals("a wrong clock must give a wrong count", TRUE_TICKS_PER_SECOND, recovered);
        assertEquals(
                "wrong by whole wraps, which is what this failure looks like",
                0,
                (recovered - TRUE_TICKS_PER_SECOND) % WRAP_STEP);
    }

    /**
     * And the whole of it, on the simulated robot: driving forward, the robot knows roughly how
     * fast it is going. Before the encoders were given the robot's clock, this measured a robot
     * doing seven inches a second as doing a hundred and eighty-seven.
     */
    @Test
    public void theSpeedTheRobotMeasuresIsTheSpeedItIsGoing() {
        SimRobot sim = new SimRobot();
        Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry());
        double before = sim.pose().position.x;
        for (int i = 0; i < LOOPS; i++) {
            robot.drive.manual(POWER, 0, 0);
            robot.loop();
            sim.step(SimRunner.LOOP_SECONDS);
        }
        double travelled = sim.pose().position.x - before;
        double actualInchesPerSecond = travelled / (LOOPS * SimRunner.LOOP_SECONDS);

        double measured = robot.localizer.velocity().linearVel.x;

        assertTrue("the robot should have moved", actualInchesPerSecond > 1);
        // Generous: the last loop's speed against the whole run's average, while still accelerating.
        assertEquals("measured forward speed", actualInchesPerSecond, measured, actualInchesPerSecond);
    }

    /** Road Runner's own, which reads the wall clock. */
    private static final String WALL_CLOCK_ENCODER = "com.acmerobotics.roadrunner.ftc.OverflowEncoder";
    /** Ours, which reads the robot's. */
    private static final String OUR_ENCODER = "org.firstinspires.ftc.teamcode.roadrunner.ClockedOverflowEncoder";
    /** The one place allowed to name Road Runner's: it runs on the robot and nowhere else. */
    private static final String TUNING = "org/firstinspires/ftc/teamcode/roadrunner/tuning";

    /**
     * Road Runner's own {@code OverflowEncoder} is the wall-clock one this is all about, and
     * nothing the robot runs on may reach for it again: the contract is not that it is unused
     * today, it is that it stays unused.
     *
     * <p>The tuning op modes are the exception, and the only one. They are built from a hardware
     * map and run on the robot and nowhere else, where the wall clock is the robot's clock; and
     * Road Runner's ramp loggers find the raw encoders underneath by looking for that exact class,
     * so handing them ours would quietly change what a tuning run records.
     */
    @Test
    public void nothingInTheRobotCodeUsesTheWallClockEncoder() {
        List<MainSources.Reference> offenders = MainSources.compiled().referencesTo(WALL_CLOCK_ENCODER).stream()
                .filter(reference -> !reference.file().startsWith(TUNING))
                .collect(Collectors.toList());

        assertEquals(
                "these read the machine's clock instead of the robot's; use ClockedOverflowEncoder",
                List.of(),
                offenders);
    }

    /**
     * The rule above passes when nothing reaches for the wall-clock encoder and when it looked at
     * nothing at all. This says which: it reads the whole of the robot code, it finds the one
     * place that is allowed to name that class, and it finds our own encoder where it is used.
     */
    @Test
    public void theRuleIsReadingTheRobotCodeAndCanTellTheTwoEncodersApart() {
        MainSources sources = MainSources.compiled();

        assertTrue("read " + sources.fileCount() + " main sources", sources.fileCount() > 30);
        assertEquals(
                "the tuning op modes are the one place allowed to name Road Runner's own",
                List.of(TUNING + "/TuningOpModes.java"),
                sources.referencesTo(WALL_CLOCK_ENCODER).stream()
                        .map(MainSources.Reference::file)
                        .distinct()
                        .collect(Collectors.toList()));
        assertTrue(
                "ours is the one the robot actually uses",
                sources.referencesTo(OUR_ENCODER).stream()
                        .anyMatch(reference -> reference.file().endsWith("TwoDeadWheelLocalizer.java")));
    }

    /**
     * And the thing the old text search could not do: {@code ClockedOverflowEncoder} names Road
     * Runner's class in its own javadoc, to say what it replaces and why. A comment is not a use,
     * and only javac can tell the difference.
     */
    @Test
    public void aMentionInACommentIsNotAUse() {
        assertTrue(
                "the javadoc says OverflowEncoder",
                read(Paths.get("src", "main", "java", "org", "firstinspires", "ftc", "teamcode", "roadrunner")
                                .resolve("ClockedOverflowEncoder.java"))
                        .contains("OverflowEncoder} does exactly this"));

        assertTrue(
                "but it does not use it",
                MainSources.compiled().referencesTo(WALL_CLOCK_ENCODER).stream()
                        .noneMatch(reference -> reference.file().endsWith("ClockedOverflowEncoder.java")));
    }

    private static String read(Path path) {
        try {
            return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
