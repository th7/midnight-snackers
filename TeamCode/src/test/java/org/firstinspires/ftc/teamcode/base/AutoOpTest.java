package org.firstinspires.ftc.teamcode.base;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * An auto's alliance decides which way the field's y axis and headings run, and whether the
 * camera may place the robot on the field at all.
 */
public class AutoOpTest {
    private static AutoOp autoFor(Alliance alliance) {
        AutoOp opMode = new PlanOp(alliance, "AutoOpTest", plans -> Step.waitFor("wait", 1));
        opMode.useHardware(new SimRobot().hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        return opMode;
    }

    @Test
    public void redMirrorsTheFieldAcrossTheCentreLineAndBlueDoesNot() {
        assertEquals(-1, autoFor(Alliance.RED).nav.pose(0, 1, 1).pose2d.position.y, 0);
        assertEquals(1, autoFor(Alliance.BLUE).nav.pose(0, 1, 1).pose2d.position.y, 0);
        assertEquals(1, autoFor(Alliance.RELATIVE).nav.pose(0, 1, 1).pose2d.position.y, 0);
    }

    @Test
    public void anAutoWithoutAnAllianceDoesNotLetTheCameraPlaceTheRobot() {
        assertFalse(autoFor(Alliance.RELATIVE).brain.usingCameraLocalization());
        assertTrue(autoFor(Alliance.BLUE).brain.usingCameraLocalization());
        assertTrue(autoFor(Alliance.RED).brain.usingCameraLocalization());
    }

    @Test
    public void aPlanOpSaysWhereItsPlanIsAndAnyOtherAutoIsFoundByItsClass() {
        AutoOp planOp = new PlanOp(Alliance.RELATIVE, "Plans.somewhere()", plans -> Step.waitFor("wait", 1));
        AutoOp classy = new AutoOp(Alliance.RELATIVE) {
            @Override
            public PlanPart getPlan() {
                return Step.waitFor("wait", 1);
            }
        };

        assertEquals("Plans.somewhere()", planOp.where());
        assertEquals(classy.getClass().getName(), classy.where());
    }
}
