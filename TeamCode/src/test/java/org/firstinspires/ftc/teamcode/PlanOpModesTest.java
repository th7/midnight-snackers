package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.function.LongSupplier;
import java.util.stream.Collectors;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.fakes.FakeOpModeManager;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.opmode.Auto;
import org.firstinspires.ftc.teamcode.opmode.AutoOp;
import org.firstinspires.ftc.teamcode.opmode.PlanOp;
import org.firstinspires.ftc.teamcode.opmode.PlanOpModes;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

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
        public WrongShape(Drive drive, Nav nav, Launcher launcher, LongSupplier nanoClock) {
            super(drive, nav, launcher, nanoClock);
        }

        @Auto(alliance = Alliance.RELATIVE)
        public PlanPart needsAnArgument(int inches) {
            return new Step("wait", () -> {}, () -> false);
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
        public Twice(Drive drive, Nav nav, Launcher launcher, LongSupplier nanoClock) {
            super(drive, nav, launcher, nanoClock);
        }

        @Auto(name = "Same", alliance = Alliance.RED)
        public PlanPart one() {
            return new Step("one", () -> {}, () -> false);
        }

        @Auto(name = "Same", alliance = Alliance.BLUE)
        public PlanPart two() {
            return new Step("two", () -> {}, () -> false);
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
