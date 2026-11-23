package org.firstinspires.ftc.teamcode.framework;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.teamcode.sim.SimDevices;
import org.junit.Test;

public class HardwareTest {
    private final Hardware hardware = new SimDevices().hardware();

    @Test
    public void aDeviceOfAnyKindIsAskedForByItsKindAndConfiguredName() {
        DcMotor intake = hardware.device(DcMotor.class, "intake");

        intake.setPower(0.5);

        assertEquals(0.5, hardware.motor("intake").getPower(), 0);
    }

    @Test
    public void aDeviceThatIsNotConfiguredFailsNamingIt() {
        IllegalArgumentException failed =
                assertThrows(IllegalArgumentException.class, () -> hardware.device(DcMotor.class, "catapult"));

        assertTrue(failed.getMessage(), failed.getMessage().contains("catapult"));
    }

    @Test
    public void aSwitchedOffDeviceOfAnyKindTakesEveryWriteAndReadsBackZero() {
        DcMotor quiet = hardware.switchedOff().device(DcMotor.class, "intake");

        quiet.setPower(1);

        assertEquals(0, quiet.getPower(), 0);
        assertEquals(0, hardware.motor("intake").getPower(), 0);
    }

    @Test
    public void aSwitchedOffDeviceTheSdkNamesByAClassFailsNamingTheRule() {
        IllegalStateException failed = assertThrows(
                IllegalStateException.class, () -> hardware.switchedOff().device(DcMotorImplEx.class, "intake"));

        assertEquals(
                "a switched-off part cannot be handed the DcMotorImplEx intake: only a device the SDK names by an"
                        + " interface, such as DcMotorEx, can be switched off",
                failed.getMessage());
    }
}
