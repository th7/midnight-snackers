package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * An IMU whose yaw and yaw rate are set directly by the test or simulation.
 */
public class FakeImu implements IMU {
    public double yawRadians = 0;
    public double yawRateRadiansPerSecond = 0;

    @Override
    public boolean initialize(Parameters parameters) {
        return true;
    }

    @Override
    public void resetYaw() {
        yawRadians = 0;
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return new YawPitchRollAngles(AngleUnit.RADIANS, yawRadians, 0, 0, System.nanoTime());
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        throw new UnsupportedOperationException("FakeImu only reports yaw/pitch/roll");
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        throw new UnsupportedOperationException("FakeImu only reports yaw/pitch/roll");
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        AngularVelocity radians = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS, 0, 0, (float) yawRateRadiansPerSecond, System.nanoTime());
        return radians.toAngleUnit(angleUnit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "fake imu";
    }

    @Override
    public String getConnectionInfo() {
        return "fake";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
    }
}
