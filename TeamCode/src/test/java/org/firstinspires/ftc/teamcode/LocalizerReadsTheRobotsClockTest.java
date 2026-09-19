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

public class LocalizerReadsTheRobotsClockTest {
    private static final int LOOPS = 12;

    private static final float POWER = 0.6f;

    private static final int WRAP_STEP = 5 * 65536;

    private static final int TRUE_TICKS_PER_SECOND = 400_000;

    private static int asTheHubReports(int ticksPerSecond) {
        return ticksPerSecond & 0xFFFF;
    }

    private static int recoveredWithReadsApart(long nanosPerRead) {
        FakeDcMotorEx motor = new FakeDcMotorEx();
        long[] nanos = {0};
        ClockedOverflowEncoder encoder = new ClockedOverflowEncoder(new RawEncoder(motor), () -> nanos[0]);

        int velocity = 0;

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

    @Test
    public void theEncoderGetsItWrongWhenTheClockIsNotTheRobots() {
        int recovered = recoveredWithReadsApart(5_000_000L);

        assertNotEquals("a wrong clock must give a wrong count", TRUE_TICKS_PER_SECOND, recovered);
        assertEquals(
                "wrong by whole wraps, which is what this failure looks like",
                0,
                (recovered - TRUE_TICKS_PER_SECOND) % WRAP_STEP);
    }

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

        assertEquals("measured forward speed", actualInchesPerSecond, measured, actualInchesPerSecond);
    }

    private static final String WALL_CLOCK_ENCODER = "com.acmerobotics.roadrunner.ftc.OverflowEncoder";

    private static final String OUR_ENCODER = "org.firstinspires.ftc.teamcode.roadrunner.ClockedOverflowEncoder";

    private static final String TUNING = "org/firstinspires/ftc/teamcode/roadrunner/tuning";

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

    @Test
    public void aNameThatMerelyReadsLikeTheWallClockEncoderIsNotAUseOfIt() {
        assertTrue(
                "our own encoder's name has OverflowEncoder inside it, so the text is there to match",
                read(Paths.get("src", "main", "java", "org", "firstinspires", "ftc", "teamcode", "roadrunner")
                                .resolve("ClockedOverflowEncoder.java"))
                        .contains("OverflowEncoder"));

        assertTrue(
                "but Road Runner's is not the symbol it resolves to",
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
