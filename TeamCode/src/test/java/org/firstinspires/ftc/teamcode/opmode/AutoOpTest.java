package org.firstinspires.ftc.teamcode.opmode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class AutoOpTest {
    private static AutoOp autoFor(Alliance alliance) {
        AutoOp opMode = new PlanOp(alliance, "AutoOpTest", plans -> new Step("wait", () -> {}, () -> false));
        opMode.useHardware(new SimRobot().hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        return opMode;
    }

    @Test
    public void redMirrorsTheFieldAcrossTheCentreLineAndBlueDoesNot() {
        assertEquals(-1, autoFor(Alliance.RED).robot.nav.pose(0, 1, 1).y(), 0);
        assertEquals(1, autoFor(Alliance.BLUE).robot.nav.pose(0, 1, 1).y(), 0);
        assertEquals(1, autoFor(Alliance.RELATIVE).robot.nav.pose(0, 1, 1).y(), 0);
    }

    @Test
    public void anAutoWithoutAnAllianceDoesNotLetTheCameraPlaceTheRobot() {
        assertFalse(autoFor(Alliance.RELATIVE).robot.brain.usingCameraLocalization());
        assertTrue(autoFor(Alliance.BLUE).robot.brain.usingCameraLocalization());
        assertTrue(autoFor(Alliance.RED).robot.brain.usingCameraLocalization());
    }

    @Test
    public void aPlanOpSaysWhereItsPlanIsAndAnyOtherAutoIsFoundByItsClass() {
        AutoOp planOp =
                new PlanOp(Alliance.RELATIVE, "Plans.somewhere()", plans -> new Step("wait", () -> {}, () -> false));
        AutoOp classy = new AutoOp(Alliance.RELATIVE) {
            @Override
            public PlanPart getPlan() {
                return new Step("wait", () -> {}, () -> false);
            }
        };

        assertEquals("Plans.somewhere()", planOp.where());
        assertEquals(classy.getClass().getName(), classy.where());
    }
}
