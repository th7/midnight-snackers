package org.firstinspires.ftc.teamcode.fakes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class FakeImu implements IMU {
    private static final String TAG = "FakeImu";
    private static final int RESET_YAW_TIMEOUT_MS = 0;

    public double yawRadians = 0;
    public double yawRateRadiansPerSecond = 0;

    public int yawReads = 0;

    private final ImuOrientationOnRobot mounted;
    private final QuaternionBasedImuHelper helper;

    public FakeImu() {
        this(new AsTheRobot(), new AsTheRobot());
    }

    private FakeImu(ImuOrientationOnRobot mounted, ImuOrientationOnRobot takenToBe) {
        this.mounted = mounted;
        this.helper = new QuaternionBasedImuHelper(takenToBe);
    }

    public static FakeImu mounted(RevHubOrientationOnRobot mounted) {
        return new FakeImu(
                mounted,
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    }

    private static final class AsTheRobot implements ImuOrientationOnRobot {
        @Override
        public Quaternion imuCoordinateSystemOrientationFromPerspectiveOfRobot() {
            return Quaternion.identityQuaternion();
        }

        @Override
        public Quaternion imuRotationOffset() {
            return Quaternion.identityQuaternion();
        }

        @Override
        public Quaternion angularVelocityTransform() {
            return Quaternion.identityQuaternion();
        }
    }

    private Quaternion chipReads() {
        long now = System.nanoTime();
        Quaternion robot =
                new Quaternion((float) Math.cos(yawRadians / 2), 0, 0, (float) Math.sin(yawRadians / 2), now);
        Quaternion chip = mounted.imuCoordinateSystemOrientationFromPerspectiveOfRobot();
        return chip.inverse()
                .multiply(robot, now)
                .multiply(mounted.imuRotationOffset().inverse(), now)
                .multiply(chip, now)
                .normalized();
    }

    private AngularVelocity chipTurns() {
        VectorF robot = new VectorF(0, 0, (float) yawRateRadiansPerSecond);
        VectorF chip = mounted.angularVelocityTransform().inverse().applyToVector(robot);
        return new AngularVelocity(AngleUnit.RADIANS, chip.get(0), chip.get(1), chip.get(2), System.nanoTime());
    }

    @Override
    public boolean initialize(Parameters parameters) {
        helper.setImuOrientationOnRobot(parameters.imuOrientationOnRobot);
        return true;
    }

    @Override
    public void resetYaw() {
        helper.resetYaw(TAG, this::chipReads, RESET_YAW_TIMEOUT_MS);
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        yawReads++;
        return helper.getRobotYawPitchRollAngles(TAG, this::chipReads);
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return helper.getRobotOrientation(TAG, this::chipReads, reference, order, angleUnit);
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return helper.getRobotOrientationAsQuaternion(TAG, this::chipReads, true);
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return helper.getRobotAngularVelocity(chipTurns(), angleUnit);
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
    public void resetDeviceConfigurationForOpMode() {}

    @Override
    public void close() {}
}
