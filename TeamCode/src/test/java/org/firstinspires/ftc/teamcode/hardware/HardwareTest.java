package org.firstinspires.ftc.teamcode.hardware;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Field;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

public class HardwareTest {
    @Test
    public void aHardwareMissingADeviceIsRefusedAndTheDevicesAreNamed() {
        IllegalStateException refused = assertThrows(
                IllegalStateException.class,
                () -> Hardware.builder().launcher(new FakeDcMotorEx()).build());

        assertTrue(refused.getMessage(), refused.getMessage().contains("turnTable"));
        assertTrue(refused.getMessage(), refused.getMessage().contains("imu"));
        assertTrue(refused.getMessage(), refused.getMessage().contains("clock"));
        assertFalse("not the one it was given", refused.getMessage().contains("launcher"));
    }

    @Test
    public void theSimulatorWiresEveryDeviceTheRobotCanReach() throws IllegalAccessException {
        Hardware hardware = new SimRobot().hardware();

        for (Field field : Hardware.class.getFields()) {
            assertNotNull("the simulator left " + field.getName() + " unwired", field.get(hardware));
        }
    }
}
