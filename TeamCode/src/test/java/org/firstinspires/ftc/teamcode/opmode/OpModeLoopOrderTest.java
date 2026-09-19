package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Loopable;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

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

        @Override
        public PlanPart getPlan() {
            return new Plan(new Step("count", () -> {}, () -> {
                planTicks.add("tick");
                return false;
            }));
        }
    }

    private static <T extends OpMode> T initialised(T opMode) {
        return initialised(opMode, new SimRobot());
    }

    private static <T extends OpMode> T initialised(T opMode, SimRobot sim) {
        opMode.useHardware(sim.hardware());
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
    public void aTeleOpTicksItsOwnDriverAfterTheRobot() {
        SimRobot sim = new SimRobot();
        BlueTeleOp opMode = initialised(new BlueTeleOp(), sim);
        List<Loopable> order = opMode.loopOrder();
        assertEquals("the robot ticks subsystems and nothing else", opMode.robot.plans, order.get(order.size() - 1));
        assertEquals(opMode.robot.brain, order.get(order.size() - 2));
        opMode.gamepad1.left_stick_y = -1;

        opMode.loop();

        assertTrue("the driver ran, so it asked the wheels for something", sim.leftFront.power > 0);
    }

    @Test
    public void runsTheOpModesOwnWorkAfterTheSubsystems() {
        TestOp opMode = initialised(new TestOp());

        opMode.loop();

        assertEquals(List.of("onLoop"), opMode.onLoops);
    }

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
