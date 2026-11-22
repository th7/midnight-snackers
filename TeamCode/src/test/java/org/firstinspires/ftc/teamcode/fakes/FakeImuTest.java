package org.firstinspires.ftc.teamcode.fakes;

import static org.junit.Assert.assertEquals;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.junit.Test;

public class FakeImuTest {
    private static final double FLOAT_DELTA = 1e-5;

    private static RevHubOrientationOnRobot hub(LogoFacingDirection logo, UsbFacingDirection usb) {
        return new RevHubOrientationOnRobot(logo, usb);
    }

    private static FakeImu turned(FakeImu imu, double yawRadians, double yawRateRadiansPerSecond) {
        imu.yawRadians = yawRadians;
        imu.yawRateRadiansPerSecond = yawRateRadiansPerSecond;
        return imu;
    }

    @Test
    public void aHubInitializedAsItIsMountedReadsTheRobotsYawAndTurnRate() {
        FakeImu imu = FakeImu.mounted(hub(LogoFacingDirection.RIGHT, UsbFacingDirection.UP));
        imu.initialize(new IMU.Parameters(hub(LogoFacingDirection.RIGHT, UsbFacingDirection.UP)));
        turned(imu, 0.3, 0.5);

        assertEquals(0.3, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(0, imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(0, imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(0.5, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, FLOAT_DELTA);
        assertEquals(0.3, yawOf(imu.getRobotOrientationAsQuaternion()), FLOAT_DELTA);
    }

    private static double yawOf(org.firstinspires.ftc.robotcore.external.navigation.Quaternion q) {
        return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    @Test
    public void aHubMountedLogoRightUsbUpButInitializedLogoDownUsbRightReadsYawHalfATurnOffAndTheTurnAboutX() {
        FakeImu imu = FakeImu.mounted(hub(LogoFacingDirection.RIGHT, UsbFacingDirection.UP));
        imu.initialize(new IMU.Parameters(hub(LogoFacingDirection.DOWN, UsbFacingDirection.RIGHT)));
        turned(imu, 0.3, 0.5);

        assertEquals(0.3 - Math.PI, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(-Math.PI / 2, imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(0, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, FLOAT_DELTA);
        assertEquals(0.5, imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate, FLOAT_DELTA);
        turned(imu, 1.0, 0.5);
        assertEquals(1.0 - Math.PI, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
    }

    @Test
    public void untilItIsInitializedTheHubIsTakenToSitLogoUpUsbForwardAsTheSdksImuTakesIt() {
        FakeImu asTheSdkTakesIt =
                turned(FakeImu.mounted(hub(LogoFacingDirection.UP, UsbFacingDirection.FORWARD)), 0.3, 0.5);
        FakeImu upsideDown =
                turned(FakeImu.mounted(hub(LogoFacingDirection.DOWN, UsbFacingDirection.FORWARD)), 0.3, 0.5);

        assertEquals(0.3, asTheSdkTakesIt.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(0.5, asTheSdkTakesIt.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, FLOAT_DELTA);
        assertEquals(-0.5, upsideDown.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, FLOAT_DELTA);
    }

    @Test
    public void resettingTheYawReadsFromWhereTheRobotThenFacedWithoutTurningIt() {
        FakeImu imu = FakeImu.mounted(hub(LogoFacingDirection.RIGHT, UsbFacingDirection.UP));
        imu.initialize(new IMU.Parameters(hub(LogoFacingDirection.RIGHT, UsbFacingDirection.UP)));
        turned(imu, 0.3, 0);

        imu.resetYaw();

        assertEquals(0.3, imu.yawRadians, 0);
        assertEquals(0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
        turned(imu, 0.5, 0);
        assertEquals(0.2, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
    }

    @Test
    public void aFakeImuMadeWithNoMountingReadsTheRobotsYawAndTurnRateAsTheyAre() {
        FakeImu imu = turned(new FakeImu(), 0.3, 0.5);

        assertEquals(0.3, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), FLOAT_DELTA);
        assertEquals(0.5, imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate, FLOAT_DELTA);
    }
}
