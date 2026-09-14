package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.stream.Collectors;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.base.Auto;
import org.firstinspires.ftc.teamcode.base.AutoOp;
import org.firstinspires.ftc.teamcode.base.PlanOp;
import org.firstinspires.ftc.teamcode.fakes.FakeOpModeManager;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * Every plan method in {@link Plans} that carries {@link Auto} is an autonomous op mode on the
 * driver station, registered by {@link PlanOpModes} without a class of its own.
 */
public class PlanOpModesTest {
    private final FakeOpModeManager manager = new FakeOpModeManager();

    @Test
    public void registersAnAutonomousOpModeForEveryAutoPlan() {
        PlanOpModes.register(manager);

        List<String> names =
                manager.registrations.stream().map(r -> r.meta.name).sorted().collect(Collectors.toList());
        assertEquals(
                List.of(
                        "BlueScoreAThingFromBack",
                        "RedScoreAThingFromBack",
                        "driveForward",
                        "forwardLeftBackwardRight",
                        "scoreAThing",
                        "spinnyThing"),
                names);
        for (FakeOpModeManager.Registration registration : manager.registrations) {
            assertEquals(registration.meta.name, OpModeMeta.Flavor.AUTONOMOUS, registration.meta.flavor);
            assertEquals(registration.meta.name, "Autonomous", registration.meta.group);
            assertTrue(registration.meta.name, registration.instance instanceof PlanOp);
        }
    }

    @Test
    public void aRegisteredOpModeRunsItsPlanAndSaysWhereThePlanIs() {
        PlanOpModes.register(manager);

        AutoOp opMode = initialised(registered("driveForward"));
        opMode.start();
        opMode.loop();

        assertEquals("0. driveForward", opMode.currentStep());
        assertEquals("Plans.driveForward()", opMode.where());
        assertEquals(Alliance.RELATIVE, opMode.alliance());
    }

    @Test
    public void onePlanIsRegisteredOncePerAllianceItIsAnnotatedFor() {
        PlanOpModes.register(manager);

        AutoOp blue = registered("BlueScoreAThingFromBack");
        AutoOp red = registered("RedScoreAThingFromBack");

        assertEquals(Alliance.BLUE, blue.alliance());
        assertEquals(Alliance.RED, red.alliance());
        assertEquals("Plans.scoreAThingFromBack()", blue.where());
        assertEquals("Plans.scoreAThingFromBack()", red.where());
    }

    @Test
    public void theSameInstanceRunsAgainWhenTheRobotControllerReusesIt() {
        PlanOpModes.register(manager);
        AutoOp opMode = initialised(registered("driveForward"));
        opMode.start();
        opMode.loop();

        opMode.init();
        opMode.start();
        opMode.loop();

        assertEquals("0. driveForward", opMode.currentStep());
    }

    public static class WrongShape extends Plans {
        @Auto(alliance = Alliance.RELATIVE)
        public PlanPart needsAnArgument(int inches) {
            return Step.waitFor("wait", inches);
        }
    }

    @Test
    public void anAutoAnnotationOnAMethodThatCannotBeCalledForItsPlanFailsRegistration() {
        try {
            PlanOpModes.register(manager, WrongShape.class);
            fail("a plan method with a parameter cannot be an op mode");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("needsAnArgument"));
        }
    }

    public static class Twice extends Plans {
        @Auto(name = "Same", alliance = Alliance.RED)
        public PlanPart one() {
            return Step.waitFor("one", 1);
        }

        @Auto(name = "Same", alliance = Alliance.BLUE)
        public PlanPart two() {
            return Step.waitFor("two", 1);
        }
    }

    @Test
    public void twoPlansWithOneNameFailRegistrationHereRatherThanOnTheRobot() {
        try {
            PlanOpModes.register(manager, Twice.class);
            fail("the robot controller refuses two op modes with one name");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(), expected.getMessage().contains("Same"));
        }
    }

    private AutoOp registered(String name) {
        return (AutoOp) manager.find(name)
                .orElseThrow(() -> new AssertionError("nothing registered as " + name))
                .opMode();
    }

    private static AutoOp initialised(AutoOp opMode) {
        opMode.useHardware(new SimRobot().hardware());
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.init();
        return opMode;
    }
}
