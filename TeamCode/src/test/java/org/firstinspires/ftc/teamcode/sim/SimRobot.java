package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.base.Hardware;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;

/**
 * A kinematic model of the robot on a flat field. Motor powers set by the robot code become wheel
 * velocities through the drive model Road Runner was tuned with ({@link MecanumDrive.Params}); the
 * true pose is integrated from those, and the sensors the localizer reads (dead wheel encoders and
 * IMU yaw) are written back from the true pose. There is no inertia, slip, or sensor noise.
 */
public class SimRobot {
    public static final double BATTERY_VOLTS = 12.5;
    private static final double TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER = 1700;
    /**
     * How the dead wheel encoders are physically wired: the raw count on the rightBack port rises as
     * the robot moves forward, and the raw count on the leftFront port falls as it moves left.
     * Road Runner's encoder wrapper undoes both the motor direction {@link MecanumDrive} sets on
     * that port and the encoder direction {@link TwoDeadWheelLocalizer} sets, so these signs are
     * proven right by the localizer tracking the true pose, not by inspection.
     */
    private static final int PAR_RAW_SIGN = 1;
    private static final int PERP_RAW_SIGN = -1;
    /**
     * How the drive motors are physically mounted: the back motors are mirrored, so positive
     * terminal power spins those wheels backwards. That is why {@link MecanumDrive} reverses them;
     * with those directions applied, positive power means wheel-forward on every corner.
     */
    private static final int LEFT_FRONT_MOUNT = 1;
    private static final int RIGHT_FRONT_MOUNT = 1;
    private static final int LEFT_BACK_MOUNT = -1;
    private static final int RIGHT_BACK_MOUNT = -1;

    public final FakeDcMotorEx leftFront = new FakeDcMotorEx();
    public final FakeDcMotorEx rightFront = new FakeDcMotorEx();
    public final FakeDcMotorEx leftBack = new FakeDcMotorEx();
    public final FakeDcMotorEx rightBack = new FakeDcMotorEx();
    public final FakeDcMotorEx launcher = new FakeDcMotorEx();
    public final FakeDcMotorEx turnTable = new FakeDcMotorEx();
    public final FakeServo topGate = new FakeServo();
    public final FakeServo bottomGate = new FakeServo();
    public final FakeImu imu = new FakeImu();
    public final FakeVoltageSensor voltageSensor = new FakeVoltageSensor(BATTERY_VOLTS);
    public final FakeDashboard dashboard = new FakeDashboard();

    private final MecanumDrive.Params drive = MecanumDrive.PARAMS;
    private final TwoDeadWheelLocalizer.Params deadWheels = TwoDeadWheelLocalizer.PARAMS;
    private final MecanumKinematics kinematics = new MecanumKinematics(
            drive.inPerTick * drive.trackWidthTicks, drive.inPerTick / drive.lateralInPerTick);
    private Pose2d pose = new Pose2d(0, 0, 0);
    private double parTicks = 0;
    private double perpTicks = 0;

    /**
     * The simulated devices, wired the way {@link Hardware#fromHardwareMap} wires the real ones.
     */
    public Hardware hardware() {
        return new Hardware(
                launcher, topGate, bottomGate,
                leftFront, rightFront, leftBack, rightBack,
                turnTable, () -> imu, voltageSensor,
                ArrayList::new,
                dashboard);
    }

    /**
     * Where the robot really is, as opposed to where its localizer thinks it is.
     */
    public Pose2d pose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Advance the world by {@code dtSeconds} using the motor powers currently commanded.
     */
    public void step(double dtSeconds) {
        double lf = wheelVelocity(leftFront, LEFT_FRONT_MOUNT);
        double lb = wheelVelocity(leftBack, LEFT_BACK_MOUNT);
        double rb = wheelVelocity(rightBack, RIGHT_BACK_MOUNT);
        double rf = wheelVelocity(rightFront, RIGHT_FRONT_MOUNT);

        Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                increment(lf, dtSeconds), increment(lb, dtSeconds),
                increment(rb, dtSeconds), increment(rf, dtSeconds)));
        Twist2d delta = twist.value();
        PoseVelocity2d velocity = twist.velocity().value();

        pose = pose.plus(delta);

        // Dead wheel readings are what TwoDeadWheelLocalizer expects to invert.
        parTicks += delta.line.x / drive.inPerTick + deadWheels.parYTicks * delta.angle;
        perpTicks += delta.line.y / drive.inPerTick + deadWheels.perpXTicks * delta.angle;
        double parVelocity = velocity.linearVel.x / drive.inPerTick + deadWheels.parYTicks * velocity.angVel;
        double perpVelocity = velocity.linearVel.y / drive.inPerTick + deadWheels.perpXTicks * velocity.angVel;
        rightBack.currentPosition = (int) Math.round(PAR_RAW_SIGN * parTicks);
        rightBack.measuredVelocity = PAR_RAW_SIGN * parVelocity;
        leftFront.currentPosition = (int) Math.round(PERP_RAW_SIGN * perpTicks);
        leftFront.measuredVelocity = PERP_RAW_SIGN * perpVelocity;

        imu.yawRadians = pose.heading.toDouble();
        imu.yawRateRadiansPerSecond = velocity.angVel;

        launcher.measuredVelocity = launcher.commandedVelocity;
        turnTable.currentPosition += (int) Math.round(
                clamp(turnTable.power) * TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER * dtSeconds);
    }

    /**
     * Wheel surface velocity in inches per second for a motor's commanded power, inverting the
     * feedforward model the drive was tuned with: volts = kS + kV * ticksPerSecond.
     * The SDK applies the motor's direction on the way to the terminals; the mount decides which
     * way the wheel turns for positive terminal power.
     */
    private double wheelVelocity(FakeDcMotorEx motor, int mount) {
        int direction = motor.getDirection() == DcMotorSimple.Direction.REVERSE ? -1 : 1;
        double volts = mount * direction * clamp(motor.power) * BATTERY_VOLTS;
        if (Math.abs(volts) <= drive.kS) {
            return 0;
        }
        double ticksPerSecond = (volts - Math.signum(volts) * drive.kS) / drive.kV;
        return ticksPerSecond * drive.inPerTick;
    }

    private static DualNum<Time> increment(double velocity, double dtSeconds) {
        return new DualNum<>(new double[]{velocity * dtSeconds, velocity});
    }

    private static double clamp(double power) {
        return Math.max(-1, Math.min(1, power));
    }
}
