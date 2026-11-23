package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.List;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.Configuration;
import org.firstinspires.ftc.teamcode.simcore.Constants;
import org.firstinspires.ftc.teamcode.simcore.Directions;
import org.firstinspires.ftc.teamcode.simcore.EncoderReads;
import org.firstinspires.ftc.teamcode.simcore.Facing;
import org.firstinspires.ftc.teamcode.simcore.HubMounting;
import org.firstinspires.ftc.teamcode.simcore.Overrides;
import org.firstinspires.ftc.teamcode.simcore.RobotDescription;
import org.firstinspires.ftc.teamcode.simcore.Turning;
import org.junit.Test;

public class SimDevicesTest {
    private static final double DELTA = 1e-9;
    private static final String SDK_HARDWARE_MAP =
            "the SDK's HardwareMap.get (RobotCore 11.2.1, com.qualcomm.robotcore.hardware.HardwareMap)";
    private static final List<String> MOTORS =
            List.of("leftFront", "rightFront", "leftBack", "rightBack", "intake", "flywheelFront", "flywheelBack");

    private final SimDevices devices = new SimDevices();

    private static String refusal(HardwareMap map, Class<?> kind, String name) {
        return assertThrows(IllegalArgumentException.class, () -> map.get(kind, name))
                .getMessage();
    }

    @Test
    public void theHardwareIsWhatTheRobotsOwnLookupsFindOnTheSimulatorsClockAndDashboard() {
        Hardware hardware = devices.hardware();

        assertSame(devices.motor("leftFront"), hardware.leftFront);
        assertSame(devices.motor("rightBack"), hardware.rightBack);
        assertSame(devices.motor("intake"), hardware.intake);
        assertSame(devices.voltageSensor, hardware.voltageSensor);
        assertSame(devices.imu, hardware.imu.get());
        assertSame(devices.dashboard, hardware.dashboard);
        assertEquals(0, hardware.nanoClock.getAsLong());
    }

    @Test
    public void aNameThatIsNotConfiguredIsRefusedWithTheSdksOwnMessage() {
        assertEquals(
                SDK_HARDWARE_MAP,
                "Unable to find a hardware device with name \"periscope\" and type DcMotorEx",
                refusal(devices.hardwareMap(), DcMotorEx.class, "periscope"));
    }

    @Test
    public void aDeviceAskedForAsAnotherKindIsRefusedWithTheSdksOwnMessage() {
        assertEquals(
                SDK_HARDWARE_MAP,
                "Unable to find a hardware device with name \"spinner\" and type Servo",
                refusal(devices.hardwareMap(), Servo.class, "spinner"));
        assertEquals(
                SDK_HARDWARE_MAP,
                "Unable to find a hardware device with name \"ramp\" and type CRServo",
                refusal(devices.hardwareMap(), CRServo.class, "ramp"));
    }

    @Test
    public void theSpinnerIsAContinuousServoAndTheRampAServo() {
        HardwareMap map = devices.hardwareMap();

        assertSame(devices.spinner, map.get(CRServo.class, "spinner"));
        assertSame(devices.ramp, map.get(Servo.class, "ramp"));
    }

    @Test
    public void aConfigurationWithoutADeviceTheRobotAsksForFailsItsHardwareWithTheSdksOwnMessage() {
        List<Configuration.Device> withoutTheIntake = Configuration.reginalds().devices().stream()
                .filter(device -> !device.name().equals("intake"))
                .toList();
        SimDevices withoutAnIntake = new SimDevices(RobotDescription.of(
                Valid.value(Configuration.of(withoutTheIntake)),
                HubMounting.reginalds(),
                Constants.defaults(),
                Directions.reginalds()));

        IllegalArgumentException refused = assertThrows(IllegalArgumentException.class, withoutAnIntake::hardware);

        assertEquals(
                SDK_HARDWARE_MAP,
                "Unable to find a hardware device with name \"intake\" and type DcMotorEx",
                refused.getMessage());
    }

    @Test
    public void aRenamedMotorIsTheSimulatorsByTheNameItIsCommittedUnderAndTheRobotsByItsNewName() {
        SimDevices renamed = new SimDevices(Valid.value(Overrides.asked(
                        RobotDescription.reginalds(),
                        List.of(new Overrides.Kept.Known(new Overrides.Edit.Renamed("intake", "intakeMotor")))))
                .description());

        assertSame(renamed.motor("intake"), renamed.hardwareMap().get(DcMotorEx.class, "intakeMotor"));
        assertEquals(
                SDK_HARDWARE_MAP,
                "Unable to find a hardware device with name \"intake\" and type DcMotorEx",
                refusal(renamed.hardwareMap(), DcMotorEx.class, "intake"));
    }

    @Test
    public void everyMotorAnswersTheTypeItIsConfiguredAs() {
        HardwareMap map = devices.hardwareMap();
        for (String name : MOTORS) {
            MotorConfigurationType type = map.get(DcMotorEx.class, name).getMotorType();

            assertEquals(name, "REV Robotics UltraPlanetary HD Hex Motor", type.getName());
            assertEquals(name, Rotation.CW, type.getOrientation());
            assertEquals(name, 560, type.getTicksPerRev(), 0);
            assertEquals(name, 300, type.getMaxRPM(), 0);
            assertEquals(name, 2380, type.getAchieveableMaxTicksPerSecondRounded());
        }
    }

    @Test
    public void theSdkDeclaresTheUltraPlanetaryHdHexClockwiseAsTheCoreSays() {
        MotorType declared = RevRoboticsUltraPlanetaryHdHexMotor.class.getAnnotation(MotorType.class);

        assertEquals(Rotation.CW, declared.orientation());
        assertEquals(
                Turning.CLOCKWISE, org.firstinspires.ftc.teamcode.simcore.MotorType.ULTRAPLANETARY_HD_HEX.declared());
    }

    @Test
    public void devicesTheCodeDoesNotAskForSitConfiguredAndIdle() {
        devices.hardware();

        assertEquals(0, devices.motor("flywheelFront").getPower(), 0);
        assertEquals(0, devices.motor("flywheelBack").getPower(), 0);
        assertSame(devices.motor("flywheelBack"), devices.hardwareMap().get(DcMotorEx.class, "flywheelBack"));
    }

    @Test
    public void theHubsVoltageSensorIsTheOnlyOneTheMapHolds() {
        HardwareMap map = devices.hardwareMap();

        assertSame(devices.voltageSensor, map.voltageSensor.iterator().next());
        assertSame(devices.voltageSensor, map.get(VoltageSensor.class, "Control Hub"));
        assertEquals(1, map.voltageSensor.size());
    }

    @Test
    public void eachMotorKnowsWhatItsEncoderPortReads() {
        assertEquals(EncoderReads.PERPENDICULAR_DEAD_WHEEL, devices.encoderOf("leftFront"));
        assertEquals(EncoderReads.PARALLEL_DEAD_WHEEL, devices.encoderOf("rightBack"));
        assertEquals(EncoderReads.NOTHING, devices.encoderOf("leftBack"));
        assertEquals(EncoderReads.OWN_MOTOR, devices.encoderOf("intake"));
    }

    @Test
    public void theImuSitsOnTheRobotAsTheDescriptionMountsIt() {
        SimDevices logoDown = new SimDevices(RobotDescription.of(
                Configuration.reginalds(),
                Valid.value(HubMounting.of(Facing.DOWN, Facing.RIGHT)),
                Constants.defaults(),
                Directions.reginalds()));
        logoDown.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        devices.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        logoDown.imu.yawRadians = 0.3;
        devices.imu.yawRadians = 0.3;

        assertEquals(0.3, logoDown.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 1e-5);
        assertEquals(0.3, devices.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 1e-5);
    }

    @Test
    public void everyMountingTheCoreTakesIsOneTheSdkNames() {
        int mountings = 0;
        for (Facing logo : Facing.values()) {
            for (Facing usb : Facing.values()) {
                Checked<HubMounting> mounting = HubMounting.of(logo, usb);
                if (mounting instanceof Checked.Ok<HubMounting> ok) {
                    SimDevices.sdkMounting(ok.value());
                    mountings++;
                }
            }
        }

        assertEquals("six ways for the logo to face, four for the USB ports across it", 24, mountings);
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
        devices.hardware().intake.setPower(0.4);

        devices.advance(5);

        assertEquals(0.4, devices.motor("intake").power, DELTA);
    }

    @Test
    public void theSimulatedRobotIsTheSameSeamWithAWorldBehindIt() {
        Hardware hardware = new SimRobot().hardware();

        assertEquals(0, hardware.nanoClock.getAsLong());
        assertEquals(SimDevices.BATTERY_VOLTS, hardware.voltageSensor.getVoltage(), DELTA);
    }
}
