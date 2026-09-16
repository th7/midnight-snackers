package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * The robot's idea of where it is moves once per loop, whatever the drive is doing.
 *
 * <p>One tick of the robot is one moment: every subsystem that reads the pose during it should
 * read the same pose, and the dead wheels and the IMU should be asked once between one loop and
 * the next. A second update inside the same loop asks the encoders again for a delta that has
 * barely happened, and leaves two subsystems that ran either side of it disagreeing about where
 * the robot was at a single instant.
 *
 * <p>The count comes from the IMU: the localizer reads the yaw once, first thing, every time it
 * updates, and nothing else on the robot reads it, so reads are updates.
 */
public class LocalizerTicksOnceTest {
    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry());

    /** How many times the robot's pose was updated while {@code work} ran. */
    private int poseUpdatesDuring(Runnable work) {
        int before = sim.imu.yawReads;
        work.run();
        return sim.imu.yawReads - before;
    }

    @Test
    public void thePoseUpdatesOncePerLoop() {
        assertEquals(1, poseUpdatesDuring(robot::loop));
    }

    @Test
    public void thePoseUpdatesOncePerLoopOverManyLoops() {
        assertEquals(5, poseUpdatesDuring(() -> {
            for (int i = 0; i < 5; i++) {
                robot.loop();
            }
        }));
    }

    @Test
    public void thePoseStillUpdatesOncePerLoopWhileAnActionIsFollowed() {
        robot.drive.strafeTo(robot.nav.pose(24, 0, 0));

        assertEquals(1, poseUpdatesDuring(robot::loop));
    }

    @Test
    public void thePoseStillUpdatesOncePerLoopOverAWholeFollowedAction() {
        robot.drive.strafeTo(robot.nav.pose(24, 0, 0));

        assertEquals(5, poseUpdatesDuring(() -> {
            for (int i = 0; i < 5; i++) {
                robot.loop();
            }
        }));
    }
}
