package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.junit.Test;

public class SimDevicesTest {
    private static final double DELTA = 1e-9;

    private final SimDevices devices = new SimDevices();

    @Test
    public void aMotorIsTheSimulatorsDeviceOfThatName() {
        Hardware hardware = devices.hardware();

        assertSame(devices.leftFront, hardware.motor("leftFront"));
        assertSame(devices.rightBack, hardware.motor("rightBack"));
        assertSame(devices.intake, hardware.motor("intake"));
    }

    @Test
    public void aServoIsTheSimulatorsDeviceOfThatName() {
        assertSame(devices.topGate, devices.hardware().servo("topGate"));
    }

    @Test
    public void theDevicesNobodyNamesAreTheSimulatorsOwn() {
        Hardware hardware = devices.hardware();

        assertSame(devices.imu, hardware.imu().get());
        assertSame(devices.voltageSensor, hardware.voltageSensor());
        assertSame(devices.dashboard, hardware.dashboard());
    }

    @Test
    public void aDeviceTheSimulatorHasNoneOfFailsNamingIt() {
        IllegalStateException thrown = assertThrows(
                IllegalStateException.class, () -> devices.hardware().motor("periscope"));

        assertTrue(thrown.getMessage(), thrown.getMessage().contains("periscope"));
    }

    @Test
    public void aDeviceOfAnotherKindThanTheRobotAsksForFailsNamingItAndTheKind() {
        IllegalStateException thrown = assertThrows(
                IllegalStateException.class, () -> devices.hardware().servo("launcher"));

        assertTrue(thrown.getMessage(), thrown.getMessage().contains("launcher"));
        assertTrue(thrown.getMessage(), thrown.getMessage().contains("servo"));
    }

    @Test
    public void theClockStartsAtNothingAndMovesOnlyWhenItIsAdvanced() {
        Hardware hardware = devices.hardware();
        assertEquals(0, hardware.nanoClock().getAsLong());

        devices.advance(0.02);
        assertEquals(0.02, hardware.nanoClock().getAsLong() / 1e9, DELTA);

        devices.advance(1.5);
        assertEquals(1.52, hardware.nanoClock().getAsLong() / 1e9, DELTA);
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

        assertEquals(0, hardware.nanoClock().getAsLong());
        assertEquals(SimDevices.BATTERY_VOLTS, hardware.voltageSensor().getVoltage(), DELTA);
    }
}
