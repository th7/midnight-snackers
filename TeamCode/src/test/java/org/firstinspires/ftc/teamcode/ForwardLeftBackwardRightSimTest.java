package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.List;
import org.firstinspires.ftc.teamcode.opmode.PlanOpModes;
import org.firstinspires.ftc.teamcode.sim.SimCatalog;
import org.firstinspires.ftc.teamcode.sim.SimNoise;
import org.firstinspires.ftc.teamcode.sim.SimRecording;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.sim.SimRunner;
import org.junit.Test;

public class ForwardLeftBackwardRightSimTest {
    private static final double POSITION_TOLERANCE_INCHES = 2;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5);
    private static final double TIMEOUT_SECONDS = 40;
    private static final Pose2d START = new Pose2d(-56, -12, 0);

    @Test
    public void drivesTheSquareAndComesBackToWhereItStarted() {
        SimRobot sim = new SimRobot();
        sim.setPose(START);
        SimCatalog.Entry opMode = SimCatalog.of(PlanOpModes.class)
                .find("forwardLeftBackwardRight")
                .get();

        SimRecording recording = SimRunner.run(opMode, sim, TIMEOUT_SECONDS);
        List<Pose2d> trace = recording.poses();

        double x0 = START.position.x, y0 = START.position.y;
        assertTrue(
                "never reached the forward corner",
                trace.stream()
                        .anyMatch(p ->
                                p.position.x > x0 + 22 && Math.abs(p.position.y - y0) < POSITION_TOLERANCE_INCHES));
        assertTrue(
                "never reached the far corner",
                trace.stream().anyMatch(p -> p.position.x > x0 + 22 && p.position.y > y0 + 22));
        assertTrue(
                "never reached the left corner",
                trace.stream()
                        .anyMatch(p ->
                                Math.abs(p.position.x - x0) < POSITION_TOLERANCE_INCHES && p.position.y > y0 + 22));
        Pose2d end = sim.pose();
        assertEquals("end x", x0, end.position.x, POSITION_TOLERANCE_INCHES);
        assertEquals("end y", y0, end.position.y, POSITION_TOLERANCE_INCHES);
        assertEquals("end heading", 0, end.heading.toDouble(), HEADING_TOLERANCE_RADIANS);
    }

    @Test
    public void drivesTheSquareOnAnImperfectRobotToo() {
        for (long seed = 1; seed <= 5; seed++) {
            SimRobot sim = new SimRobot(SimNoise.seeded(seed));
            sim.setDown(START);
            Pose2d placed = sim.pose();
            SimCatalog.Entry opMode = SimCatalog.of(PlanOpModes.class)
                    .find("forwardLeftBackwardRight")
                    .get();

            SimRunner.run(opMode, sim, TIMEOUT_SECONDS, SimRunner.DEFAULT_OUTPUT_DIR.resolve("seed-" + seed));

            Pose2d end = sim.pose();
            assertEquals("seed " + seed + " end x", placed.position.x, end.position.x, POSITION_TOLERANCE_INCHES);
            assertEquals("seed " + seed + " end y", placed.position.y, end.position.y, POSITION_TOLERANCE_INCHES);
            assertEquals(
                    "seed " + seed + " end heading",
                    placed.heading.toDouble(),
                    end.heading.toDouble(),
                    HEADING_TOLERANCE_RADIANS);
        }
    }
}
