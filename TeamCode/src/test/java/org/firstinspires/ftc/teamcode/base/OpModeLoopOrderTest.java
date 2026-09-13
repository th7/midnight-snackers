package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Nav;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.PlanRunner;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
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

        @Override
        protected Nav getNav(MecanumDrive mecanumDrive) {
            return Nav.relative(mecanumDrive, runtime, telemetry);
        }

        @Override
        protected void onLoop() {
            onLoops.add("onLoop");
        }
    }

    private static class TestAuto extends AutoOp {
        @Override
        protected Nav getNav(MecanumDrive mecanumDrive) {
            return Nav.relative(mecanumDrive, runtime, telemetry);
        }

        @Override
        public PlanPart getPlan() {
            return plans.driveForward();
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

        assertEquals(
                List.of(opMode.launcher, opMode.drive, opMode.camera, opMode.nav, opMode.turntable, opMode.brain),
                opMode.loopOrder());
    }

    @Test
    public void runsTheOpModesOwnWorkAfterTheSubsystems() {
        TestOp opMode = initialised(new TestOp());

        opMode.loop();

        assertEquals(List.of("onLoop"), opMode.onLoops);
    }

    @Test
    public void autoOpTicksItsPlanAfterEverySubsystem() {
        TestAuto opMode = initialised(new TestAuto());

        List<Loopable> order = opMode.loopOrder();

        assertEquals(opMode.brain, order.get(order.size() - 2));
        assertTrue(order.get(order.size() - 1) instanceof PlanRunner);
    }
}
