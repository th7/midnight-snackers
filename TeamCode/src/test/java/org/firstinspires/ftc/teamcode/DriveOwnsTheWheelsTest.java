package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.Wheels;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * One place turns four numbers into four motors turning, and everything that wants the robot to
 * move asks it. Which place that is matters more than it looks: while two of them write the same
 * four motors, whether the robot does what you asked comes down to which ran last, and nothing in
 * the code says which that is.
 *
 * <p>So this asks the motors themselves who wrote them. A power that arrives from anywhere but
 * {@link Wheels} is a second hand on the controls.
 */
public class DriveOwnsTheWheelsTest {
    private final SimRobot sim = new SimRobot();
    private final Robot robot = new Robot(sim.hardware(), Alliance.RELATIVE, new FakeTelemetry());

    private List<FakeDcMotorEx> wheels() {
        return List.of(sim.leftFront, sim.rightFront, sim.leftBack, sim.rightBack);
    }

    /** Every class that has written any of the four wheels since the robot was built. */
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

    /**
     * And the whole of a run, every intent the robot gives itself included: the plans, the brain's
     * steering, and the trajectories they follow.
     */
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
