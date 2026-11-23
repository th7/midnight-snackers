package org.firstinspires.ftc.teamcode.framework;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.firstinspires.ftc.reginald.framework.Auto;
import org.firstinspires.ftc.reginald.framework.Discovered;
import org.firstinspires.ftc.reginald.framework.OpMode;
import org.firstinspires.ftc.reginald.framework.OpModes;
import org.firstinspires.ftc.reginald.framework.SubSystem;
import org.firstinspires.ftc.reginald.framework.TeleOp;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.base.Alliance;
import org.firstinspires.ftc.teamcode.base.Gamepads;
import org.firstinspires.ftc.teamcode.base.Prints;
import org.firstinspires.ftc.teamcode.fakes.FakeOpModeManager;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.planrunner.Plan;
import org.firstinspires.ftc.teamcode.planrunner.PlanPart;
import org.firstinspires.ftc.teamcode.planrunner.RunsAPlan;
import org.firstinspires.ftc.teamcode.planrunner.Step;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.firstinspires.ftc.teamcode.sim.SimDriverStation;
import org.junit.Before;
import org.junit.Test;

public class OpModesTest {
    static final List<String> HAPPENED = new ArrayList<>();

    @Before
    public void nothingHasHappened() {
        HAPPENED.clear();
    }

    public static class Steering extends SubSystem {
        private boolean steered = false;

        public Steering() {}

        public Step steer() {
            return new Step("steer", () -> steered = true, () -> true);
        }

        @Override
        public void teleOp(Gamepads gamepads) {
            HAPPENED.add("teleOp Steering");
        }

        @Override
        public void loop() {
            HAPPENED.add(steered ? "loop Steering, steered" : "loop Steering");
        }
    }

    public static class Practice extends TeleOp {
        public Practice() {
            super(Alliance.RED);
        }
    }

    public abstract static class Unfinished extends TeleOp {
        protected Unfinished() {
            super(Alliance.BLUE);
        }
    }

    public static class Plays {
        @Auto(alliance = Alliance.RELATIVE)
        public PlanPart steerOnce(Steering steering) {
            return new Plan(steering.steer());
        }

        @Auto(name = "Blue steer", group = "Steering", alliance = Alliance.BLUE)
        @Auto(name = "Red steer", group = "Steering", alliance = Alliance.RED)
        public PlanPart steerForAnAlliance(Steering steering) {
            return new Plan(steering.steer());
        }

        public PlanPart notAnAuto() {
            return new Plan();
        }
    }

    public static class Talker extends SubSystem {
        private final Prints channel;

        public Talker(Prints channel) {
            this.channel = channel;
        }

        @Override
        public void loop() {
            channel.addData("said", "hello");
        }
    }

    @Test
    public void shareOnGamepadTwoStepsThroughTheChannelsAndTheDriverStationSaysWhichIsShowing() {
        OpMode practice = wired(registered(Practice.class, Steering.class, Talker.class), "Practice");
        FakeTelemetry screen = (FakeTelemetry) practice.telemetry;
        practice.init();

        practice.loop();
        assertFalse(screen.captions.contains("Talker.said"));
        assertTrue(
                String.valueOf(screen.lines), screen.lines.contains("Telemetry: off, share on gamepad 2 for the next"));

        SimDriverStation.State.fromJson(new Gson().fromJson("{\"share\": true}", JsonObject.class))
                .applyTo(practice.gamepad2);
        screen.captions.clear();
        screen.lines.clear();
        practice.loop();
        assertTrue(screen.captions.contains("Talker.said"));
        assertTrue(
                String.valueOf(screen.lines),
                screen.lines.contains("Telemetry: Talker, share on gamepad 2 for the next"));
    }

    private static FakeOpModeManager registered(Class<?>... found) {
        FakeOpModeManager manager = new FakeOpModeManager();
        OpModes.register(manager, Discovered.of(found));
        return manager;
    }

    private static OpMode wired(FakeOpModeManager manager, String name) {
        OpMode opMode = (OpMode) manager.find(name).get().opMode();
        opMode.telemetry = new FakeTelemetry();
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        opMode.useHardware(new SimDevices().hardware());
        return opMode;
    }

    @Test
    public void aTeleOpIsListedUnderItsClassNameWithNoAnnotation() {
        FakeOpModeManager manager = registered(Practice.class, Unfinished.class, Steering.class);

        assertEquals(List.of("Practice"), names(manager));
        OpModeMeta meta = manager.find("Practice").get().meta;
        assertEquals(OpModeMeta.Flavor.TELEOP, meta.flavor);
        assertEquals("TeleOp", meta.group);
        OpMode practice = (OpMode) manager.find("Practice").get().opMode();
        assertEquals(Alliance.RED, practice.alliance());
        assertEquals(Practice.class.getName(), practice.where());
    }

    @Test
    public void anAutoIsListedUnderItsMethodsNameOrTheNameItGivesForEachAlliance() {
        FakeOpModeManager manager = registered(Plays.class, Steering.class);

        assertEquals(List.of("Blue steer", "Red steer", "steerOnce"), names(manager));
        OpModeMeta once = manager.find("steerOnce").get().meta;
        assertEquals(OpModeMeta.Flavor.AUTONOMOUS, once.flavor);
        assertEquals("Autonomous", once.group);
        assertEquals("Steering", manager.find("Red steer").get().meta.group);
        assertEquals(Alliance.RED, ((OpMode) manager.find("Red steer").get().opMode()).alliance());
        assertEquals(
                "Plays.steerOnce()", ((OpMode) manager.find("steerOnce").get().opMode()).where());
    }

    @Test
    public void anAutoIsHandedThePartsItAsksForAsTheRobotBuiltThem() {
        OpMode auto = wired(registered(Plays.class, Steering.class), "steerOnce");

        auto.init();
        auto.loop();
        auto.loop();

        assertEquals(List.of("loop Steering", "loop Steering, steered"), HAPPENED);
        assertTrue(((RunsAPlan) auto).done());
    }

    @Test
    public void aTeleOpBuildsEverySubsystemAndHandsItsButtonsToEachBeforeTheyTick() {
        OpMode practice = wired(registered(Practice.class, Steering.class), "Practice");

        practice.init();
        practice.loop();

        assertEquals(List.of("teleOp Steering", "loop Steering"), HAPPENED);
    }

    @Test
    public void anAutoNeverHandsOutTheButtons() {
        OpMode auto = wired(registered(Plays.class, Steering.class), "steerOnce");

        auto.init();
        auto.loop();

        assertFalse(HAPPENED.contains("teleOp Steering"));
    }

    public static class Clashing {
        @Auto(name = "Practice", alliance = Alliance.RELATIVE)
        public PlanPart practice() {
            return new Plan();
        }
    }

    @Test
    public void twoOpModesWithOneNameFailNamingBoth() {
        IllegalStateException failed =
                assertThrows(IllegalStateException.class, () -> registered(Practice.class, Clashing.class));

        assertEquals(
                "two op modes are named Practice: " + Practice.class.getName() + " and Clashing.practice()",
                failed.getMessage());
    }

    public static class NoPlan {
        @Auto(alliance = Alliance.RELATIVE)
        public String wanders() {
            return "nowhere";
        }
    }

    @Test
    public void anAutoThatMakesNoPlanFailsNamingIt() {
        IllegalStateException failed = assertThrows(IllegalStateException.class, () -> registered(NoPlan.class));

        assertEquals(
                "@Auto needs a public method that returns a PlanPart; NoPlan.wanders() is not one",
                failed.getMessage());
    }

    public static class Unbuildable extends TeleOp {
        public Unbuildable(Alliance alliance) {
            super(alliance);
        }
    }

    @Test
    public void aTeleOpWithoutAConstructorOfNoParametersFailsNamingIt() {
        IllegalStateException failed = assertThrows(IllegalStateException.class, () -> registered(Unbuildable.class));

        assertEquals(
                "a TeleOp needs a public constructor with no parameters, which calls super with its alliance; "
                        + Unbuildable.class.getName() + " has none",
                failed.getMessage());
    }

    private static List<String> names(FakeOpModeManager manager) {
        return manager.registrations.stream().map(r -> r.meta.name).collect(Collectors.toList());
    }
}
