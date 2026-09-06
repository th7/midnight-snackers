package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.junit.Test;

public class DriveRunnerTest {
    private final FakeDashboard dashboard = new FakeDashboard();
    private final DriveRunner driveRunner = new DriveRunner(dashboard);

    @Test
    public void nothingToDrawSendsNothing() {
        driveRunner.loop();

        assertEquals(0, dashboard.packets.size());
    }

    @Test
    public void idleWithAPoseSupplierSendsOnePacketPerLoop() {
        driveRunner.setPoseSupplier(() -> new Pose2d(1, 2, 0));

        driveRunner.loop();
        driveRunner.loop();

        assertEquals(2, dashboard.packets.size());
    }

    @Test
    public void aRunningActionIsSentToTheDashboardUntilItFinishes() {
        int[] runsRemaining = {2};
        Action action = packet -> --runsRemaining[0] > 0;
        driveRunner.drive(action);

        driveRunner.loop();
        assertFalse(driveRunner.done());
        assertEquals(1, dashboard.packets.size());

        driveRunner.loop();
        assertTrue(driveRunner.done());
        assertEquals(2, dashboard.packets.size());
    }
}
