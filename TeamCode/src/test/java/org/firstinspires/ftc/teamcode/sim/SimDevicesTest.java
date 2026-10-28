package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Map;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.junit.Test;

public class SimDevicesTest {
    private static final double DELTA = 1e-9;

    private final SimDevices devices = new SimDevices();

    public static final class WantsAMotorAndAServo {
        public DcMotorEx launcher;
        public Servo topGate;

        public WantsAMotorAndAServo launcher(DcMotorEx launcher) {
            this.launcher = launcher;
            return this;
        }

        public WantsAMotorAndAServo topGate(Servo topGate) {
            this.topGate = topGate;
            return this;
        }
    }

    public static final class WantsAPeriscope {
        public WantsAPeriscope periscope(Object periscope) {
            return this;
        }
    }

    public static final class WantsALauncherServo {
        public WantsALauncherServo launcher(Servo launcher) {
            return this;
        }
    }

    @Test
    public void theHardwareIsWholeAndIsTheseDevices() {
        Hardware hardware = devices.hardware();

        assertSame(devices.leftFront, hardware.leftFront);
        assertSame(devices.rightBack, hardware.rightBack);
        assertSame(devices.voltageSensor, hardware.voltageSensor);
        assertSame(devices.dashboard, hardware.dashboard);
    }

    @Test
    public void aBuilderIsHandedEachDeviceItAsksForByItsName() {
        FakeDcMotorEx launcher = new FakeDcMotorEx();
        FakeServo topGate = new FakeServo();

        WantsAMotorAndAServo wired = SimDevices.wired(
                new WantsAMotorAndAServo(), Map.of("launcher", launcher, "topGate", topGate, "intake", devices.intake));

        assertSame(launcher, wired.launcher);
        assertSame(topGate, wired.topGate);
    }

    @Test
    public void aDeviceTheSimulatorHasNoneOfFailsNamingIt() {
        IllegalStateException thrown = assertThrows(
                IllegalStateException.class,
                () -> SimDevices.wired(new WantsAPeriscope(), Map.of("launcher", devices.launcher)));

        assertTrue(thrown.getMessage(), thrown.getMessage().contains("periscope"));
    }

    @Test
    public void aDeviceOfAnotherKindThanTheBuilderAsksForFailsNamingIt() {
        IllegalStateException thrown = assertThrows(
                IllegalStateException.class,
                () -> SimDevices.wired(new WantsALauncherServo(), Map.of("launcher", devices.launcher)));

        assertTrue(thrown.getMessage(), thrown.getMessage().contains("launcher"));
        assertTrue(thrown.getMessage(), thrown.getMessage().contains("Servo"));
    }

    @Test
    public void theClockStartsAtNothingAndMovesOnlyWhenItIsAdvanced() {
        Hardware hardware = devices.hardware();
        assertEquals(0, hardware.nanoClock.getAsLong());

        devices.advance(0.02);
        assertEquals(0.02, hardware.nanoClock.getAsLong() / 1e9, DELTA);

        devices.advance(1.5);
        assertEquals(1.52, hardware.nanoClock.getAsLong() / 1e9, DELTA);
    }

    @Test
    public void aDeviceHoldsWhatWasWrittenToItWhileTimePasses() {
        devices.hardware();
        devices.turnTable.setPower(0.4);

        devices.advance(5);

        assertEquals(0.4, devices.turnTable.power, DELTA);
    }

    @Test
    public void theSimulatedRobotIsTheSameSeamWithAWorldBehindIt() {
        Hardware hardware = new SimRobot().hardware();

        assertEquals(0, hardware.nanoClock.getAsLong());
        assertEquals(SimDevices.BATTERY_VOLTS, hardware.voltageSensor.getVoltage(), DELTA);
    }
}
