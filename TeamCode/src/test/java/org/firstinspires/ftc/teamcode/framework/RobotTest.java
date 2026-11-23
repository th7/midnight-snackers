package org.firstinspires.ftc.teamcode.framework;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.framework.hardware.Hardware;
import org.firstinspires.ftc.framework.robot.Alliance;
import org.firstinspires.ftc.framework.robot.Discovered;
import org.firstinspires.ftc.framework.robot.Gamepads;
import org.firstinspires.ftc.framework.robot.Prints;
import org.firstinspires.ftc.framework.robot.Robot;
import org.firstinspires.ftc.framework.robot.SubSystem;
import org.firstinspires.ftc.framework.robot.SwitchedOff;
import org.firstinspires.ftc.teamcode.fakes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Before;
import org.junit.Test;

public class RobotTest {
    static final List<String> HAPPENED = new ArrayList<>();

    private final Hardware hardware = new SimDevices().hardware();

    @Before
    public void nothingHasHappened() {
        HAPPENED.clear();
    }

    private Robot robotOf(Class<?>... found) {
        return new Robot(
                hardware,
                Alliance.BLUE,
                new FakeTelemetry(),
                new Gamepads(new Gamepad(), new Gamepad()),
                Discovered.of(found));
    }

    public static class Wheel extends SubSystem {
        public Wheel() {
            HAPPENED.add("built Wheel");
        }

        @Override
        public void init() {
            HAPPENED.add("init Wheel");
        }

        @Override
        public void loop() {
            HAPPENED.add("loop Wheel");
        }
    }

    public static class Arm extends SubSystem {
        public final Wheel wheel;

        public Arm(Wheel wheel) {
            this.wheel = wheel;
            HAPPENED.add("built Arm");
        }

        @Override
        public void init() {
            HAPPENED.add("init Arm");
        }

        @Override
        public void loop() {
            HAPPENED.add("loop Arm");
        }
    }

    public static class Claw extends SubSystem {
        public Claw() {
            HAPPENED.add("built Claw");
        }

        @Override
        public void init() {
            HAPPENED.add("init Claw");
        }

        @Override
        public void loop() {
            HAPPENED.add("loop Claw");
        }
    }

    @Test
    public void everySubsystemItFoundIsBuiltAndReadiedWithoutAnyoneNamingIt() {
        robotOf(Claw.class, Wheel.class, Arm.class);

        assertEquals(
                List.of("built Wheel", "built Arm", "built Claw", "init Wheel", "init Arm", "init Claw"), HAPPENED);
    }

    @Test
    public void aSubsystemIsTickedAfterThePartsItIsBuiltFromAndOtherwiseInTheOrderOfItsName() {
        Robot robot = robotOf(Claw.class, Wheel.class, Arm.class);
        HAPPENED.clear();

        robot.loop();

        assertEquals(List.of("loop Wheel", "loop Arm", "loop Claw"), HAPPENED);
    }

    @Test
    public void aPartIsBuiltOnceAndShared() {
        Robot robot = robotOf(Arm.class, Wheel.class);

        assertSame(robot.part(Wheel.class), robot.part(Arm.class).wheel);
    }

    public interface Sensor {}

    public static class Eye extends SubSystem implements Sensor {
        public Eye() {}
    }

    public static class Ear extends SubSystem implements Sensor {
        public Ear() {}
    }

    public static class Reader extends SubSystem {
        public final Sensor sensor;

        public Reader(Sensor sensor) {
            this.sensor = sensor;
        }
    }

    @Test
    public void anInterfaceIsTheOneClassItFoundThatImplementsIt() {
        Robot robot = robotOf(Reader.class, Eye.class);

        assertSame(robot.part(Eye.class), robot.part(Reader.class).sensor);
    }

    @Test
    public void anInterfaceNothingItFoundImplementsFailsNamingWhatNeedsIt() {
        IllegalStateException failed = assertThrows(IllegalStateException.class, () -> robotOf(Reader.class));

        assertEquals(
                "the robot cannot build Reader, because that needs a Sensor, and no class it found implements Sensor",
                failed.getMessage());
    }

    @Test
    public void anInterfaceTwoFoundClassesImplementFailsNamingBoth() {
        IllegalStateException failed =
                assertThrows(IllegalStateException.class, () -> robotOf(Reader.class, Eye.class, Ear.class));

        assertEquals(
                "the robot cannot build Reader, because that needs a Sensor, and Ear and Eye both implement it:"
                        + " the robot builds an interface only when one class it found implements it",
                failed.getMessage());
    }

    public static class Gearbox {
        public Gearbox() {
            HAPPENED.add("built Gearbox");
        }
    }

    public static class Lift extends SubSystem {
        public final Gearbox gearbox;

        public Lift(Gearbox gearbox) {
            this.gearbox = gearbox;
        }
    }

    @Test
    public void aClassThatIsNoSubsystemIsBuiltOnlyBecauseSomethingNeedsItAndIsNeverTicked() {
        Robot robot = robotOf(Gearbox.class);
        assertEquals(List.of(), HAPPENED);

        robot = robotOf(Gearbox.class, Lift.class);
        robot.loop();

        assertEquals(List.of("built Gearbox"), HAPPENED);
        assertSame(robot.part(Gearbox.class), robot.part(Lift.class).gearbox);
    }

    @Test
    public void aClassTheRobotDidNotFindIsNotBuiltAndTheFailureNamesTheChain() {
        IllegalStateException failed = assertThrows(IllegalStateException.class, () -> robotOf(Lift.class));

        assertEquals(
                "the robot cannot build Lift, because that needs a Gearbox, which the robot does not supply,"
                        + " and it builds only the classes it found in Reginald's code and the framework",
                failed.getMessage());
    }

    public static class Looped extends SubSystem {
        public Looped(Looped itself) {}
    }

    @Test
    public void aPartThatNeedsItselfFailsNamingIt() {
        IllegalStateException failed = assertThrows(IllegalStateException.class, () -> robotOf(Looped.class));

        assertEquals(
                "the robot cannot build Looped, because that needs a Looped, which is already being built:"
                        + " a part cannot need itself",
                failed.getMessage());
    }

    public static class Printer extends SubSystem {
        public final Prints channel;
        public final Alliance alliance;
        public final Gamepads gamepads;

        public Printer(Prints channel, Alliance alliance, Gamepads gamepads) {
            this.channel = channel;
            this.alliance = alliance;
            this.gamepads = gamepads;
        }
    }

    @Test
    public void aPartIsHandedTheAllianceTheGamepadsAndAChannelNamedAfterItsClass() {
        Robot robot = robotOf(Printer.class, Claw.class);

        assertEquals(Alliance.BLUE, robot.part(Printer.class).alliance);
        assertTrue(robot.part(Printer.class).gamepads.gamepad1 != null);
        assertEquals(Set.of("Printer"), robot.channels.made());
    }

    public static class Motorized extends SubSystem {
        public final DcMotorEx motor;

        public Motorized(Hardware hardware) {
            this.motor = hardware.motor("intake");
        }
    }

    @SwitchedOff
    public static class Quiet extends SubSystem {
        public final DcMotorEx motor;

        public Quiet(Hardware hardware) {
            this.motor = hardware.motor("intake");
        }
    }

    @Test
    public void aSwitchedOffPartIsBuiltOnHardwareThatDoesNothing() {
        Robot robot = robotOf(Motorized.class, Quiet.class);
        DcMotorEx real = robot.part(Motorized.class).motor;
        DcMotorEx quiet = robot.part(Quiet.class).motor;

        quiet.setPower(1);

        assertNotSame(real, quiet);
        assertEquals(0, quiet.getPower(), 0);
        assertEquals(0, real.getPower(), 0);
    }
}
