package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer {
    public static Params PARAMS = new Params();
    public final ClockedOverflowEncoder par, perp;
    public final IMU imu;
    private final double inPerTick;
    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;
    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;
    private Pose2d pose;

    public TwoDeadWheelLocalizer(
            HardwareMap hardwareMap, IMU imu, double inPerTick, Pose2d initialPose, LongSupplier clock) {
        this(
                hardwareMap.get(DcMotorEx.class, "rightBack"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                imu,
                inPerTick,
                initialPose,
                clock);
    }

    public TwoDeadWheelLocalizer(
            DcMotorEx parMotor,
            DcMotorEx perpMotor,
            IMU imu,
            double inPerTick,
            Pose2d initialPose,
            LongSupplier clock) {
        par = new ClockedOverflowEncoder(new RawEncoder(parMotor), clock);
        perp = new ClockedOverflowEncoder(new RawEncoder(perpMotor), clock);

        par.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public PoseVelocity2d update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime);

        FlightRecorder.write(
                "TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                    parPosDelta - PARAMS.parYTicks * headingDelta,
                                    parPosVel.velocity - PARAMS.parYTicks * headingVel,
                                })
                                .times(inPerTick),
                        new DualNum<Time>(new double[] {
                                    perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                    perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                                })
                                .times(inPerTick)),
                new DualNum<>(new double[] {
                    headingDelta, headingVel,
                }));

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }

    public static class Params {
        public double parYTicks = 610.3360642343223;
        public double perpXTicks = 5186.563208836955;
    }
}
