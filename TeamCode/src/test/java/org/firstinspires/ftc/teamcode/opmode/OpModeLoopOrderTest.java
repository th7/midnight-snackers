package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Driver;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * The Localizer settles where the robot is before anything reads it, and Brain reads Camera and
 * Nav from the current tick and sets the Turntable target, so the order subsystems are ticked in
 * is part of the op mode's contract, not an accident of construction.
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
        final List<String> planTicks = new ArrayList<>();

        TestAuto() {
            super(Alliance.RELATIVE);
        }

        /** A plan that never finishes, so every tick of it is counted. */
        @Override
        public PlanPart getPlan() {
            return new Plan(new Step("count", () -> {}, () -> {
                planTicks.add("tick");
                return false;
            }));
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
                List.of(
                        robot.localizer,
                        robot.launcher,
                        robot.intake,
                        robot.drive,
                        robot.camera,
                        robot.nav,
                        robot.turntable,
                        robot.brain),
                opMode.loopOrder().subList(0, 8));
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
                List.of(
                        robot.localizer,
                        robot.launcher,
                        robot.intake,
                        robot.drive,
                        robot.camera,
                        robot.nav,
                        robot.turntable,
                        robot.brain),
                opMode.loopOrder().subList(0, 8));
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

    /**
     * An auto's plan runner is the auto's own, so the auto ticks it, once a loop. That this
     * happens after every subsystem is now the shape rather than a list position: it runs in
     * {@code onLoop}, and {@link OpMode#loop()} is final and runs that after the robot.
     *
     * <p>It used to be registered on the robot, which is what forced {@code Robot.add} to accept
     * something that was not a subsystem, and the robot to ask which it had been given.
     */
    @Test
    public void anAutoTicksItsOwnPlanOncePerLoopAfterEverySubsystem() {
        TestAuto opMode = initialised(new TestAuto());
        List<Loopable> order = opMode.loopOrder();
        assertEquals("the robot ticks subsystems and nothing else", opMode.robot.plans, order.get(order.size() - 1));
        assertEquals(opMode.robot.brain, order.get(order.size() - 2));
        assertEquals("not ticked before the op mode loops", List.of(), opMode.planTicks);

        opMode.loop();
        opMode.loop();

        assertEquals(List.of("tick", "tick"), opMode.planTicks);
    }
}
