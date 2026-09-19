package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.opmode.PlanOpModes;
import org.junit.Test;

public class WheelPowerTraceTest {
    private static final Pose2d START = new Pose2d(-56, -12, 0);

    private static final double AUTO_TIMEOUT_SECONDS = 20;

    @Test
    public void followingATrajectoryCommandsTheWheelsAsItDidWhenTheTraceWasWritten() {
        SimRobot sim = new SimRobot();
        sim.setPose(START);
        SimCatalog.Entry opMode =
                SimCatalog.of(PlanOpModes.class).find("driveForward").get();

        SimRecording recording = SimRunner.run(opMode, sim, AUTO_TIMEOUT_SECONDS);

        SimTrace.assertMatches("driveForward", recording);
    }

    @Test
    public void everyDriverIntentCommandsTheWheelsAsItDidWhenTheTraceWasWritten() {
        SimRobot sim = new SimRobot();
        sim.setPose(START);
        SimRecording recording = new SimRecording("Drive intents", SimCatalog.TELEOP);

        SimRunner.record(
                recording,
                new TestTeleOps.DriveIntentsTeleOp(),
                sim,
                TestTeleOps.DriveIntentsTeleOp.SECONDS,
                SimRunner.DEFAULT_OUTPUT_DIR,
                new SimDriverStation(),
                SimRunner.Pace.FASTEST);

        SimTrace.assertMatches("driveIntents", recording);
    }
}
