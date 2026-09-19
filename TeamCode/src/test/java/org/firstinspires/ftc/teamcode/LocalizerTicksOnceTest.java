package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class LocalizerTicksOnceTest {
    private final SimDevices devices = new SimDevices();
    private final Robot robot = new Robot(devices.hardware(), Alliance.RELATIVE, new FakeTelemetry());

    private int poseUpdatesDuring(Runnable work) {
        int before = devices.imu.yawReads;
        work.run();
        return devices.imu.yawReads - before;
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
