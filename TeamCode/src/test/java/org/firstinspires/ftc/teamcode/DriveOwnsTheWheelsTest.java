package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.hardware.Wheels;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class DriveOwnsTheWheelsTest {
    private final SimDevices devices = new SimDevices();
    private final Robot robot = new Robot(devices.hardware(), Alliance.RELATIVE, new FakeTelemetry());

    private List<FakeDcMotorEx> wheels() {
        return List.of(devices.leftFront, devices.rightFront, devices.leftBack, devices.rightBack);
    }

    private Set<String> whoWroteTheWheels() {
        Set<String> writers = new LinkedHashSet<>();
        for (FakeDcMotorEx wheel : wheels()) {
            writers.addAll(wheel.powerSetBy);
        }
        return writers;
    }

    private void clearHistory() {
        for (FakeDcMotorEx wheel : wheels()) {
            wheel.powerSetBy.clear();
        }
    }

    private static void assertOnlyWheelsWrote(Set<String> writers) {
        List<String> strangers = new ArrayList<>(writers);
        strangers.remove(Wheels.class.getName());
        assertEquals("only Wheels may set a drive motor's power", List.of(), strangers);
    }

    @Test
    public void drivingByHandGoesThroughTheWheels() {
        clearHistory();

        robot.drive.manual(0.5f, 0.25f, 0.1f);

        assertTrue("nothing wrote the wheels at all", !whoWroteTheWheels().isEmpty());
        assertOnlyWheelsWrote(whoWroteTheWheels());
    }

    @Test
    public void steeringTowardAPoseGoesThroughTheWheels() {
        clearHistory();

        robot.drive.toward(robot.nav.pose(24, 12, 0));

        assertTrue("nothing wrote the wheels at all", !whoWroteTheWheels().isEmpty());
        assertOnlyWheelsWrote(whoWroteTheWheels());
    }

    @Test
    public void followingATrajectoryGoesThroughTheWheels() {
        robot.drive.strafeTo(robot.nav.pose(24, 0, 0));
        clearHistory();

        for (int i = 0; i < 5; i++) {
            robot.loop();
        }

        assertTrue("nothing wrote the wheels at all", !whoWroteTheWheels().isEmpty());
        assertOnlyWheelsWrote(whoWroteTheWheels());
    }

    @Test
    public void nothingElseWritesAWheelOverAWholeRun() {
        clearHistory();

        robot.drive.strafeTo(robot.nav.pose(24, 0, 0));
        for (int i = 0; i < 40; i++) {
            robot.loop();
        }
        robot.drive.cancel();
        for (int i = 0; i < 5; i++) {
            robot.drive.manual(0.3f, 0, 0);
            robot.loop();
        }

        assertOnlyWheelsWrote(whoWroteTheWheels());
    }
}
