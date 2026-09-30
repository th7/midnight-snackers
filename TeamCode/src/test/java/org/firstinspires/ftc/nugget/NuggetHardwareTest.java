package org.firstinspires.ftc.nugget;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.junit.Test;

public class NuggetHardwareTest {
    @Test
    public void aHardwareMissingAnythingIsRefusedNamingAllItIsMissing() {
        IllegalStateException refused = assertThrows(
                IllegalStateException.class,
                () -> NuggetHardware.builder().left(new FakeDcMotorEx()).build());

        for (String missing : List.of("right", "voltageSensor", "dashboard", "nanoClock")) {
            assertTrue(
                    missing + " in " + refused.getMessage(),
                    refused.getMessage().contains(missing));
        }
        assertFalse("not the one it was given", refused.getMessage().contains("left"));
    }

    @Test
    public void aWholeHardwareIsWhatItWasBuiltFrom() {
        FakeNugget fake = new FakeNugget();
        fake.nanos = 42;

        NuggetHardware hardware = fake.hardware();

        assertSame(fake.left, hardware.left);
        assertSame(fake.right, hardware.right);
        assertSame(fake.battery, hardware.voltageSensor);
        assertSame(fake.dashboard, hardware.dashboard);
        assertEquals(42, hardware.nanoClock.getAsLong());
    }
}
