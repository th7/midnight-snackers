package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Brain reads Camera and Nav from the current tick and sets the Turntable target, so the order
 * subsystems are ticked in is part of the op mode's contract, not an accident of construction.
 */
public class OpModeLoopOrderTest {
    private static class TestOp extends OpMode {
        final List<String> onLoops = new ArrayList<>();

        TestOp() {
            super(Alliance.RELATIVE);
        }

        @Override
        protected void onLoop() {
            onLoops.add("onLoop");
        }
    }

    private static class TestAuto extends AutoOp {
        TestAuto() {
            super(Alliance.RELATIVE);
        }

        @Override
        public PlanPart getPlan() {
            return robot.plans.driveForward();
        }
    }

    private static <T extends OpMode> T initialised(T opMode) {
        opMode.useHardware(new SimRobot().hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        return opMode;
    }

    @Test
    public void ticksEverySubsystemWithBrainLast() {
        TestOp opMode = initialised(new TestOp());
        Robot robot = opMode.robot;

        assertEquals(
                List.of(robot.launcher, robot.drive, robot.camera, robot.nav, robot.turntable, robot.brain),
                opMode.loopOrder().subList(0, 6));
    }

    @Test
    public void aTeleOpTicksItsDriverAfterTheRobot() {
        BlueTeleOp opMode = initialised(new BlueTeleOp());

        List<Loopable> order = opMode.loopOrder();

        assertEquals(opMode.robot.brain, order.get(order.size() - 3));
        assertEquals(opMode.robot.plans, order.get(order.size() - 2));
        assertTrue(order.get(order.size() - 1) instanceof Driver);
    }

    @Test
    public void runsTheOpModesOwnWorkAfterTheSubsystems() {
        TestOp opMode = initialised(new TestOp());

        opMode.loop();

        assertEquals(List.of("onLoop"), opMode.onLoops);
    }

    /**
     * The robot controller keeps one instance of an op mode registered by instance and calls
     * {@code init()} on it for every run, so a second init must leave the loop as the first did.
     */
    @Test
    public void initialisingAgainTicksEachSubsystemOnce() {
        TestAuto opMode = initialised(new TestAuto());
        List<Loopable> first = List.copyOf(opMode.loopOrder());

        opMode.init();

        Robot robot = opMode.robot;
        assertEquals(first.size(), opMode.loopOrder().size());
        assertEquals(
                List.of(robot.launcher, robot.drive, robot.camera, robot.nav, robot.turntable, robot.brain),
                opMode.loopOrder().subList(0, 6));
    }

    @Test
    public void initialisingAgainStillMirrorsTelemetryToTheDashboardOnce() {
        SimRobot sim = new SimRobot();
        TestOp opMode = new TestOp();
        opMode.useHardware(sim.hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        opMode.init();

        opMode.telemetry.update();

        assertEquals(1, ((FakeTelemetry) sim.dashboard.telemetry()).updates);
    }

    @Test
    public void autoOpTicksItsPlanAfterEverySubsystem() {
        TestAuto opMode = initialised(new TestAuto());

        List<Loopable> order = opMode.loopOrder();

        assertEquals(opMode.robot.brain, order.get(order.size() - 3));
        assertEquals(opMode.robot.plans, order.get(order.size() - 2));
        assertTrue(order.get(order.size() - 1) instanceof PlanRunner);
    }
}
