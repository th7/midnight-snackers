package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.PlanOpModes;
import org.junit.Test;

/**
 * Holds the drive's command path to what it does today, so a change to it is a diff with a tick
 * number on it rather than a surprise on a field. Between them the two runs below cover every way
 * an intent reaches the wheels:
 *
 * <ul>
 *   <li>{@code driveForward} follows a Road Runner trajectory, so it covers the holonomic
 *       controller, the mecanum kinematics, the feedforward and the voltage compensation.
 *   <li>{@code Drive intents} drives each axis by hand, saturates a mixed command, and then asks
 *       the drive to steer to a pose, so it covers the mixing, how a mixed command is held to full
 *       power, and FastDrive's three controllers.
 * </ul>
 *
 * <p>See {@link SimTrace} for what a trace holds, why it is a prefix of a run, and how to
 * regenerate one when a change to those powers is meant.
 */
public class WheelPowerTraceTest {
    /** Clear floor to drive on: the middle of the field is inside this season's frame. */
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
