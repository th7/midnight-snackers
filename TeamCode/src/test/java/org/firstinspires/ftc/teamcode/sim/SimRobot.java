package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
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
 * A kinematic model of the robot on a flat, walled field. Motor powers set by the robot code become
 * wheel velocities through the drive model Road Runner was tuned with ({@link MecanumDrive.Params});
 * the true pose is integrated from those, kept inside the walls, and the sensors the localizer
 * reads (dead wheel encoders and IMU yaw) are written back from the true pose. The robot is an
 * {@link #ROBOT_SIZE_IN}-inch square; the walls stop it dead and let it slide along them. There is
 * no inertia, slip, or sensor noise.
 */
public class SimRobot {
    public static final double BATTERY_VOLTS = 12.5;
    /** The field is a square of this many inches on a side, centred on the origin, walled all round. */
    public static final double FIELD_SIZE_IN = 144;
    /** The robot's footprint is a square of this many inches on a side, centred on its pose. */
    public static final double ROBOT_SIZE_IN = 18;
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
        Hardware hardware = new Hardware();
        hardware.launcher = launcher;
        hardware.topGate = topGate;
        hardware.bottomGate = bottomGate;
        hardware.leftFront = leftFront;
        hardware.rightFront = rightFront;
        hardware.leftBack = leftBack;
        hardware.rightBack = rightBack;
        hardware.turnTable = turnTable;
        hardware.imu = () -> imu;
        hardware.voltageSensor = voltageSensor;
        hardware.aprilTags = ArrayList::new;
        hardware.dashboard = dashboard;
        return hardware;
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
        Pose2d previous = pose;
        pose = insideTheWalls(previous.plus(twist.value()));

        // The dead wheels roll on the floor, so they read what the robot actually did: nothing when
        // the wheels spin against a wall, and only the sliding component when it drives into one at
        // an angle. Their readings are what TwoDeadWheelLocalizer expects to invert.
        Twist2d delta = pose.minus(previous);
        PoseVelocity2d velocity = dtSeconds > 0
                ? new PoseVelocity2d(delta.line.div(dtSeconds), delta.angle / dtSeconds)
                : twist.velocity().value();

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
     * The pose the walls allow: the same heading, and the position pushed back just far enough that
     * no corner of the robot's square is beyond a wall. A wall is a straight line, so the square
     * reaches it at half its side scaled by how far the heading is from square-on. Each axis is
     * clamped on its own, which is what lets the robot slide along a wall it drives into at an
     * angle.
     */
    private static Pose2d insideTheWalls(Pose2d candidate) {
        double reach = ROBOT_SIZE_IN / 2 * (Math.abs(candidate.heading.real) + Math.abs(candidate.heading.imag));
        double limit = FIELD_SIZE_IN / 2 - reach;
        double x = Math.max(-limit, Math.min(limit, candidate.position.x));
        double y = Math.max(-limit, Math.min(limit, candidate.position.y));
        if (x == candidate.position.x && y == candidate.position.y) {
            return candidate;
        }
        return new Pose2d(new Vector2d(x, y), candidate.heading);
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
