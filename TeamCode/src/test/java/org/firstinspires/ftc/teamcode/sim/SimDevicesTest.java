package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.junit.Test;

public class SimDevicesTest {
    private static final double DELTA = 1e-9;

    private final SimDevices devices = new SimDevices();

    @Test
    public void theHardwareIsWholeAndIsTheseDevices() {
        Hardware hardware = devices.hardware();

        assertSame(devices.leftFront, hardware.leftFront);
        assertSame(devices.turnTable, hardware.turnTable);
        assertSame(devices.bottomGate, hardware.bottomGate);
        assertSame(devices.dashboard, hardware.dashboard);
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
