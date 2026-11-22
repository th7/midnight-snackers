package org.firstinspires.ftc.teamcode.simcore;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class ConfigurationTest {
    private static void assertRejected(String rule, Checked<?> checked) {
        String said = checked.fold(value -> "accepted " + value, reason -> reason);
        assertTrue(said, checked instanceof Checked.Rejected && said.contains(rule));
    }

    private static Configuration.Motor motor(String name, EncoderReads encoder) {
        return new Configuration.Motor(name, MotorType.ULTRAPLANETARY_HD_HEX, encoder);
    }

    @Test
    public void reginaldsHubHoldsItsDevicesByNameAndKindInTheOrderTheyAreConfigured() {
        assertEquals(
                List.of(
                        motor("leftFront", EncoderReads.PERPENDICULAR_DEAD_WHEEL),
                        motor("rightFront", EncoderReads.NOTHING),
                        motor("leftBack", EncoderReads.NOTHING),
                        motor("rightBack", EncoderReads.PARALLEL_DEAD_WHEEL),
                        motor("intake", EncoderReads.OWN_MOTOR),
                        motor("flywheelFront", EncoderReads.OWN_MOTOR),
                        motor("flywheelBack", EncoderReads.OWN_MOTOR),
                        new Configuration.Imu("imu"),
                        new Configuration.ContinuousServo("spinner"),
                        new Configuration.Servo("ramp"),
                        new Configuration.VoltageSensor("Control Hub")),
                Configuration.reginalds().devices());
    }

    @Test
    public void everyMotorOnReginaldIsConfiguredAsTheUltraPlanetaryHdHex() {
        assertEquals("REV Robotics UltraPlanetary HD Hex Motor", MotorType.ULTRAPLANETARY_HD_HEX.label());
        assertEquals(Turning.CLOCKWISE, MotorType.ULTRAPLANETARY_HD_HEX.declared());
    }

    @Test
    public void aConfigurationOfValidDevicesIsTakenAsItIs() {
        List<Configuration.Device> devices = List.of(motor("a", EncoderReads.OWN_MOTOR), new Configuration.Servo("b"));

        assertEquals(devices, Valid.value(Configuration.of(devices)).devices());
    }

    @Test
    public void twoDevicesMayNotShareAName() {
        assertRejected(
                "two devices are named 'intake'",
                Configuration.of(List.of(motor("intake", EncoderReads.OWN_MOTOR), new Configuration.Servo("intake"))));
    }

    @Test
    public void aNameIsNeitherBlankNorPaddedWithSpaces() {
        assertRejected("' intake '", Configuration.of(List.of(motor(" intake ", EncoderReads.OWN_MOTOR))));
        assertRejected("no space around it", Configuration.of(List.of(motor(" intake ", EncoderReads.OWN_MOTOR))));
        assertRejected("not blank", Configuration.of(List.of(new Configuration.Imu(""))));
        assertRejected("not blank", Configuration.of(List.of(new Configuration.Imu("  "))));
    }

    @Test
    public void aDeadWheelIsReadOnOneEncoderPortAtMost() {
        Checked<Configuration> twice = Configuration.of(List.of(
                motor("leftFront", EncoderReads.PARALLEL_DEAD_WHEEL),
                motor("rightBack", EncoderReads.PARALLEL_DEAD_WHEEL)));

        assertRejected("the parallel dead wheel is read on one port at most", twice);
        assertRejected("'leftFront' and 'rightBack'", twice);
        assertEquals(
                2,
                Valid.value(Configuration.of(List.of(
                                motor("leftBack", EncoderReads.NOTHING), motor("rightFront", EncoderReads.NOTHING))))
                        .devices()
                        .size());
    }

    @Test
    public void whatAConfigurationIsHandedIsNotWhatItHolds() {
        List<Configuration.Device> handed = new ArrayList<>(List.of(new Configuration.Imu("imu")));
        Configuration configuration = Valid.value(Configuration.of(handed));

        handed.add(new Configuration.Servo("ramp"));

        assertEquals(List.of(new Configuration.Imu("imu")), configuration.devices());
    }

    @Test
    public void aMotorTypeIsNamedAsTheSdkConfiguresItAndByNothingElse() {
        assertEquals(
                MotorType.ULTRAPLANETARY_HD_HEX,
                Valid.value(MotorType.named("REV Robotics UltraPlanetary HD Hex Motor")));
        String refused = MotorType.named("NeveRest 40").fold(type -> "accepted " + type, rule -> rule);
        assertTrue(refused, refused.contains("REV Robotics UltraPlanetary HD Hex Motor"));
        assertTrue(refused, refused.contains("'NeveRest 40'"));
    }
}
