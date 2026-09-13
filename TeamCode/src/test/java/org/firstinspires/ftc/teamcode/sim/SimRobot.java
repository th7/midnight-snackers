package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
 * A kinematic model of the robot on the season's field ({@link SimField}). Motor powers set by
 * the robot code become wheel velocities through the drive model Road Runner was tuned with
 * ({@link MecanumDrive.Params}); the true pose is integrated from those, kept inside the walls and
 * out of the field elements, and the sensors the localizer reads (dead wheel encoders and IMU yaw)
 * are written back from the true pose. The robot is an {@link #ROBOT_SIZE_IN}-inch cube; the
 * walls, {@link #WALL_HEIGHT_IN} inches high, and the field's obstacles stop it dead and let it
 * slide along them. There is no inertia, slip, or sensor noise.
 */
public class SimRobot {
    public static final double BATTERY_VOLTS = 12.5;
    /** The season's field: its walls, and the elements the robot runs into. */
    public static final SimField FIELD = SimField.load();
    /** The field is a square of this many inches between the walls, centred on the origin. */
    public static final double FIELD_SIZE_IN = FIELD.size;
    /**
     * The walls are this many inches high. The model is planar, so nothing ever goes over them;
     * the replay page draws them at this height.
     */
    public static final double WALL_HEIGHT_IN = FIELD.wallHeight;
    /**
     * The robot is a cube of this many inches on a side, centred on its pose and standing on the
     * floor. Only its footprint collides, with the walls and with the obstacles: the field elements
     * that stand lower than this.
     */
    public static final double ROBOT_SIZE_IN = 18;
    private static final double TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER = 1700;
    /** A quarter inch at full speed: far less than the thinnest obstacle. */
    private static final double MAX_STEP_SECONDS = 0.005;
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

    /** Put the robot somewhere; its IMU reads the new heading at once. */
    public void setPose(Pose2d pose) {
        this.pose = pose;
        imu.yawRadians = pose.heading.toDouble();
    }

    /**
     * Advance the world by {@code dtSeconds} using the motor powers currently commanded. The world
     * moves in steps of at most {@link #MAX_STEP_SECONDS}, so the robot never jumps over an
     * obstacle between two of them, however long the caller waited.
     */
    public void step(double dtSeconds) {
        int steps = Math.max(1, (int) Math.ceil(dtSeconds / MAX_STEP_SECONDS));
        for (int i = 0; i < steps; i++) {
            substep(dtSeconds / steps);
        }
    }

    private void substep(double dtSeconds) {
        double lf = wheelVelocity(leftFront, LEFT_FRONT_MOUNT);
        double lb = wheelVelocity(leftBack, LEFT_BACK_MOUNT);
        double rb = wheelVelocity(rightBack, RIGHT_BACK_MOUNT);
        double rf = wheelVelocity(rightFront, RIGHT_FRONT_MOUNT);

        Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                increment(lf, dtSeconds), increment(lb, dtSeconds),
                increment(rb, dtSeconds), increment(rf, dtSeconds)));
        Pose2d previous = pose;
        pose = insideTheWalls(clearOfTheObstacles(previous.plus(twist.value())));

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
     * The pose the field elements allow: the same heading, and the position pushed the shortest
     * way out of any obstacle the robot's square overlaps. That push is along the face it hit, so
     * a robot driving into an obstacle at an angle slides along it. Two passes, so a push out of
     * one obstacle into its neighbour (a leg into its foot) is undone too.
     */
    private static Pose2d clearOfTheObstacles(Pose2d candidate) {
        Vector2d position = candidate.position;
        for (int pass = 0; pass < 2; pass++) {
            for (SimField.Obstacle obstacle : FIELD.obstacles) {
                Vector2d push = pushOutOf(obstacle.footprint, corners(position, candidate.heading));
                if (push != null) {
                    position = position.plus(push);
                }
            }
        }
        if (position == candidate.position) {
            return candidate;
        }
        return new Pose2d(position, candidate.heading);
    }

    /** The robot's footprint: its square's corners, counter-clockwise, at a position and heading. */
    private static double[][] corners(Vector2d position, Rotation2d heading) {
        double h = ROBOT_SIZE_IN / 2;
        double[][] corners = new double[4][];
        double[][] local = {{h, h}, {-h, h}, {-h, -h}, {h, -h}};
        for (int i = 0; i < 4; i++) {
            corners[i] = new double[]{
                    position.x + local[i][0] * heading.real - local[i][1] * heading.imag,
                    position.y + local[i][0] * heading.imag + local[i][1] * heading.real};
        }
        return corners;
    }

    /**
     * The shortest move that takes the robot's footprint out of a convex obstacle, or null when
     * they do not overlap: the separating axis theorem over both polygons' edge normals, keeping
     * the axis they overlap least along.
     */
    private static Vector2d pushOutOf(double[][] obstacle, double[][] robot) {
        double leastOverlap = Double.POSITIVE_INFINITY;
        double[] leastAxis = null;
        for (double[][] polygon : new double[][][]{obstacle, robot}) {
            for (int i = 0; i < polygon.length; i++) {
                double[] a = polygon[i], b = polygon[(i + 1) % polygon.length];
                double length = Math.hypot(b[0] - a[0], b[1] - a[1]);
                if (length == 0) {
                    continue;
                }
                double[] axis = {(a[1] - b[1]) / length, (b[0] - a[0]) / length};
                double[] robotSpan = span(robot, axis), obstacleSpan = span(obstacle, axis);
                double overlap = Math.min(robotSpan[1] - obstacleSpan[0], obstacleSpan[1] - robotSpan[0]);
                if (overlap <= 0) {
                    return null;
                }
                if (overlap < leastOverlap) {
                    leastOverlap = overlap;
                    leastAxis = axis;
                }
            }
        }
        // Push the robot away from the obstacle, whichever way along the axis that is.
        double[] robotCentre = centre(robot), obstacleCentre = centre(obstacle);
        double side = (robotCentre[0] - obstacleCentre[0]) * leastAxis[0] + (robotCentre[1] - obstacleCentre[1]) * leastAxis[1];
        double sign = side < 0 ? -1 : 1;
        return new Vector2d(sign * leastAxis[0] * leastOverlap, sign * leastAxis[1] * leastOverlap);
    }

    private static double[] span(double[][] polygon, double[] axis) {
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (double[] p : polygon) {
            double along = p[0] * axis[0] + p[1] * axis[1];
            min = Math.min(min, along);
            max = Math.max(max, along);
        }
        return new double[]{min, max};
    }

    private static double[] centre(double[][] polygon) {
        double x = 0, y = 0;
        for (double[] p : polygon) {
            x += p[0];
            y += p[1];
        }
        return new double[]{x / polygon.length, y / polygon.length};
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
