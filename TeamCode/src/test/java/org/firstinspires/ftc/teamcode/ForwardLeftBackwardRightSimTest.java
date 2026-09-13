package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.sim.SimCatalog;
import org.firstinspires.ftc.teamcode.sim.SimRecording;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.sim.SimRunner;
import org.junit.Test;

import java.util.List;

/**
 * Runs the real forwardLeftBackwardRight auto, as the registrar registers it, against the
 * simulated robot. Takes about ten seconds of wall clock: the plan starts with a five second wait.
 */
public class ForwardLeftBackwardRightSimTest {
    private static final double POSITION_TOLERANCE_INCHES = 2;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(5);
    private static final double TIMEOUT_SECONDS = 40;

    @Test
    public void drivesTheSquareAndComesBackToWhereItStarted() {
        SimRobot sim = new SimRobot();
        SimCatalog.Entry opMode = SimCatalog.of(PlanOpModes.class).find("forwardLeftBackwardRight").get();

        SimRecording recording = SimRunner.run(opMode, sim, TIMEOUT_SECONDS);
        List<Pose2d> trace = recording.poses();

        assertTrue("never reached the forward corner", trace.stream().anyMatch(
                p -> p.position.x > 22 && Math.abs(p.position.y) < POSITION_TOLERANCE_INCHES));
        assertTrue("never reached the far corner", trace.stream().anyMatch(
                p -> p.position.x > 22 && p.position.y > 22));
        assertTrue("never reached the left corner", trace.stream().anyMatch(
                p -> Math.abs(p.position.x) < POSITION_TOLERANCE_INCHES && p.position.y > 22));
        Pose2d end = sim.pose();
        assertEquals("end x", 0, end.position.x, POSITION_TOLERANCE_INCHES);
        assertEquals("end y", 0, end.position.y, POSITION_TOLERANCE_INCHES);
        assertEquals("end heading", 0, end.heading.toDouble(), HEADING_TOLERANCE_RADIANS);
    }
}
