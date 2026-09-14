package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Settings;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.hull.GiftWrap;
import org.dyn4j.world.World;
import org.firstinspires.ftc.teamcode.Turntable;
import org.firstinspires.ftc.teamcode.base.Hardware;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

/**
 * The robot on the season's field ({@link SimField}), as rigid bodies in a dyn4j world: the robot,
 * the walls, the field's obstacles, and the balls. The robot is driven by the model Road Runner
 * was tuned with ({@link MecanumDrive.Params}): each wheel's motor, at its commanded power, pushes
 * its wheel toward the speed the tuned kS and kV give, at the rate the tuned kA allows, so the
 * robot takes time to get up to speed and to stop, and its true pose comes out of the engine
 * integrating that push against whatever it runs into. The walls and the obstacles stop it and
 * let it slide along them; a ball it pushes rolls on and slows; a ball pinned against a wall
 * stops the robot short of it, since nothing goes through anything. The sensors the localizer
 * reads (dead wheel encoders and IMU yaw) are written back from the true pose, so the dead
 * wheels read nothing while the wheels spin against a wall. A robot made with {@link SimNoise}
 * is off that model the ways a real robot is: its motors each a little off their tuning, its
 * battery fresh or flat and sagging under load, its traction limited so it can slip, and set
 * down near a pose rather than on it. The noise is in the mechanisms only; the sensors still
 * read exactly what the robot did.
 * <p>
 * The robot starts with {@link #PRELOAD} balls in it. The launcher is on the turntable, and its
 * gates feed it as the robot code drives them: with the top gate open a ball drops from the hopper
 * into the chamber, and with the bottom gate open the chambered ball drops into the flywheel and
 * leaves at a speed set by the flywheel's, on an arc under gravity. A ball that goes in through a
 * hive cell's mouth has scored and rests in the cell; one that meets any other panel of the hive
 * bounces off; one that comes down on the floor rolls on; one that clears a wall is out. The
 * model is planar apart from that flight: only the robot's footprint collides, nothing goes over a
 * wall or under a hive by being low, and a flying ball meets only the hives, the floor and the
 * walls. The clock is the world's ({@link #nanoTime()}), read by the robot code through its
 * hardware, so a run is the same every time and need not take real time. The world moves in
 * steps of at most {@link #MAX_STEP_SECONDS} however long the caller waited, so nothing is jumped
 * over.
 * <p>
 * The launcher's throw ({@link #LAUNCH_HEIGHT_IN} and the constants around it) and the turntable's
 * speed are guesses until measured on the robot, calibrated so the robot code's close launch from
 * its launch distance reaches the nearer cell's mouth.
 */
public class SimRobot {
    public static final double BATTERY_VOLTS = 12.5;
    /** The season's field: its walls, the elements the robot runs into, and the hives' cells. */
    public static final SimField FIELD = SimField.load();
    /** The field is a square of this many inches between the walls, centred on the origin. */
    public static final double FIELD_SIZE_IN = FIELD.size;
    /**
     * The walls are this many inches high. A driving robot never goes over them; a flying ball
     * that clears one is out of the field.
     */
    public static final double WALL_HEIGHT_IN = FIELD.wallHeight;
    /**
     * The robot is a cube of this many inches on a side, centred on its pose and standing on the
     * floor. Only its footprint collides, with the walls, the obstacles and the balls.
     */
    public static final double ROBOT_SIZE_IN = 18;
    /** How many balls the robot starts with, in its hopper: what it can launch before it is empty. */
    public static final int PRELOAD = 3;
    /** A quarter inch at full speed: far less than the thinnest obstacle. */
    public static final double MAX_STEP_SECONDS = 0.005;
    /** Where a launched ball leaves the robot: this high off the floor, and this far ahead of the robot's centre. */
    public static final double LAUNCH_HEIGHT_IN = 14;

    public static final double LAUNCH_AHEAD_IN = 6;
    /** A launched ball leaves this far above horizontal. */
    public static final double LAUNCH_ANGLE_RADIANS = Math.toRadians(45);
    /**
     * How fast a launched ball leaves, in inches per second, for each encoder tick per second of
     * the flywheel: a close launch (1050 ticks per second) from 40 inches reaches the mouth of the
     * nearer cell.
     */
    public static final double LAUNCH_IN_PER_S_PER_TICK_PER_S = 0.189;

    private static final double ROBOT_MASS_KG = 15;
    /** Heavy for pollen, but a ball that light next to the robot is what the engine's solver handles worst. */
    private static final double BALL_MASS_KG = 0.5;

    private static final double TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER = 1700;
    /** A rolling ball loses its speed with this time constant, and is at rest below {@link #REST_SPEED_IN_PER_S}. */
    private static final double ROLL_SECONDS = 0.8;

    private static final double REST_SPEED_IN_PER_S = 0.5;
    /** How much of the closing speed a ball keeps, bouncing off a wall, an obstacle, the robot, a hive or another ball. */
    private static final double BOUNCE = 0.3;

    private static final double GRAVITY_IN_PER_S2 = 386.09;
    /** A ball that comes down on the floor slower than this, upward speed lost, rolls rather than bouncing again. */
    private static final double LANDING_SPEED_IN_PER_S = 25;
    /** A gate servo is open from this position on: nearer the position the robot code opens it to than the one it closes it to. */
    private static final double TOP_GATE_OPENS_AT = 0.8;

    private static final double BOTTOM_GATE_OPENS_AT = 0.45;
    /** Metres per inch: the engine works in metres, everything here is in inches. */
    private static final double IN = 0.0254;
    /** How far into a body the engine lets another rest: a fiftieth of an inch. */
    private static final double CONTACT_TOLERANCE_IN = 0.02;
    /** The walls are this thick, so nothing gets through one in a step. */
    private static final double WALL_THICKNESS_M = 1;
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
     * The hub reports an encoder's velocity in multiples of this many ticks per second, and Road
     * Runner's encoder wrapper relies on it: it reads a velocity's remainder modulo this as a
     * count of 16-bit overflows to undo. A velocity reported between the steps is read as tens of
     * inches a second, so the encoders here report what the hub would.
     */
    private static final double HUB_VELOCITY_STEP_TICKS_PER_S = 20;
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

    /** How this robot differs from the tuned model; {@link SimNoise#NONE} for the model exactly. */
    private final SimNoise noise;

    private final MecanumDrive.Params drive = MecanumDrive.PARAMS;
    private final TwoDeadWheelLocalizer.Params deadWheels = TwoDeadWheelLocalizer.PARAMS;
    private final MecanumKinematics kinematics =
            new MecanumKinematics(drive.inPerTick * drive.trackWidthTicks, drive.inPerTick / drive.lateralInPerTick);
    private final World<Body> world = new World<>();
    private final Body robot;
    /** The simulation's clock: nanoseconds since the world was made, advanced by {@link #step}. */
    private long nanos = 0;
    /** Where the robot was after the last step, for the sensors. */
    private Pose2d previous = new Pose2d(0, 0, 0);

    private double parTicks = 0;
    private double perpTicks = 0;
    /** The balls: the field's loose pieces in {@link SimField#loosePieces}' order, then the preload. */
    private final Ball[] balls;
    /** The balls in the robot's hopper, above the top gate, in the order they will drop. */
    private final Deque<Ball> hopper = new ArrayDeque<>();
    /** The ball between the gates, or null. */
    private Ball chambered = null;

    private final Map<SimField.Cell, Integer> scoredIn = new LinkedHashMap<>();

    /** Where a ball is, and what it is doing. */
    private enum Where {
        /** On the floor, in the engine's world. */
        ROLLING,
        /** In the air, on its arc. */
        FLYING,
        /** In the robot. */
        HELD,
        /** In a hive cell, at rest. */
        SCORED,
        /** Over a wall and out of the field, at rest where it came down. */
        OUT
    }

    private static final class Ball {
        final double radius;
        final Body body;
        Where where;
        /** The position, and while flying the velocity, of a ball that is not in the engine's world. */
        double x, y, z, vx, vy, vz;

        Ball(double radius, Body body) {
            this.radius = radius;
            this.body = body;
        }
    }

    /** The tuned model exactly: a robot with {@link SimNoise#NONE}. */
    public SimRobot() {
        this(SimNoise.NONE);
    }

    /** A robot as far off the tuned model as {@code noise} says. */
    public SimRobot(SimNoise noise) {
        this.noise = noise;
        voltageSensor.voltage = noise.batteryVolts(0, 0);
        Settings settings = world.getSettings();
        settings.setLinearTolerance(CONTACT_TOLERANCE_IN * IN);
        settings.setMaximumAtRestLinearVelocity(REST_SPEED_IN_PER_S * IN);
        settings.setMinimumAtRestTime(0.25);
        settings.setVelocityConstraintSolverIterations(20);
        settings.setPositionConstraintSolverIterations(10);
        world.setGravity(World.ZERO_GRAVITY);

        double half = FIELD_SIZE_IN / 2 * IN;
        double reach = half + WALL_THICKNESS_M / 2;
        double length = 2 * half + 2 * WALL_THICKNESS_M;
        world.addBody(wall(reach, 0, WALL_THICKNESS_M, length));
        world.addBody(wall(-reach, 0, WALL_THICKNESS_M, length));
        world.addBody(wall(0, reach, length, WALL_THICKNESS_M));
        world.addBody(wall(0, -reach, length, WALL_THICKNESS_M));
        for (SimField.Obstacle obstacle : FIELD.obstacles) {
            world.addBody(obstacleBody(obstacle));
        }

        robot = new Body();
        BodyFixture footprint = robot.addFixture(Geometry.createRectangle(ROBOT_SIZE_IN * IN, ROBOT_SIZE_IN * IN));
        footprint.setDensity(ROBOT_MASS_KG / (ROBOT_SIZE_IN * IN * ROBOT_SIZE_IN * IN));
        footprint.setFriction(0);
        footprint.setRestitution(0);
        robot.setMass(MassType.NORMAL);
        robot.setAtRestDetectionEnabled(false);
        robot.setLinearDamping(0);
        robot.setAngularDamping(0);
        world.addBody(robot);

        int loose = FIELD.loosePieces.size();
        balls = new Ball[loose + PRELOAD];
        for (int i = 0; i < balls.length; i++) {
            double radius = i < loose ? FIELD.loosePieces.get(i).radius : FIELD.loosePieces.get(0).radius;
            balls[i] = new Ball(radius, ballBody(radius));
            if (i < loose) {
                SimField.Piece piece = FIELD.loosePieces.get(i);
                placePiece(i, piece.x, piece.y);
            } else {
                balls[i].where = Where.HELD;
                hopper.add(balls[i]);
            }
        }
        for (SimField.Cell cell : FIELD.cells) {
            scoredIn.put(cell, 0);
        }
    }

    private static Body wall(double x, double y, double width, double height) {
        Body wall = new Body();
        BodyFixture fixture = wall.addFixture(Geometry.createRectangle(width, height));
        fixture.setFriction(0);
        fixture.setRestitution(0);
        wall.setMass(MassType.INFINITE);
        wall.translate(x, y);
        return wall;
    }

    /** An obstacle as a static convex body: the hull of its footprint, in metres. */
    private static Body obstacleBody(SimField.Obstacle obstacle) {
        Vector2[] points = new Vector2[obstacle.footprint.length];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Vector2(obstacle.footprint[i][0] * IN, obstacle.footprint[i][1] * IN);
        }
        Vector2[] hull = new GiftWrap().generate(points);
        Body body = new Body();
        try {
            BodyFixture fixture = body.addFixture(Geometry.createPolygon(hull));
            fixture.setFriction(0);
            fixture.setRestitution(0);
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException(obstacle.name + " is not a convex footprint the engine can hold", e);
        }
        body.setMass(MassType.INFINITE);
        return body;
    }

    private static Body ballBody(double radius) {
        Body body = new Body();
        BodyFixture fixture = body.addFixture(Geometry.createCircle(radius * IN));
        fixture.setDensity(BALL_MASS_KG / (Math.PI * radius * IN * radius * IN));
        fixture.setFriction(0);
        fixture.setRestitution(BOUNCE);
        fixture.setRestitutionVelocity(0);
        body.setMass(MassType.NORMAL);
        body.setLinearDamping(1 / ROLL_SECONDS);
        body.setAngularDamping(1 / ROLL_SECONDS);
        return body;
    }

    /**
     * The simulated devices, wired the way {@link Hardware#fromHardwareMap} wires the real ones,
     * on the simulation's clock.
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
        hardware.clock = this::nanoTime;
        return hardware;
    }

    /**
     * The simulation's clock, in nanoseconds since the world was made: what the robot code's timers
     * read, through its hardware. It advances only when the world does.
     */
    public long nanoTime() {
        return nanos;
    }

    /** How this robot differs from the tuned model. */
    public SimNoise noise() {
        return noise;
    }

    /**
     * Where the robot really is, as opposed to where its localizer thinks it is.
     */
    public Pose2d pose() {
        Transform transform = robot.getTransform();
        return new Pose2d(
                transform.getTranslationX() / IN, transform.getTranslationY() / IN, transform.getRotationAngle());
    }

    /**
     * Put the robot somewhere, at rest; its IMU reads the new heading at once. A pose beyond a wall
     * or in an obstacle is placed against it, since the field holds whatever the robot is given.
     */
    public void setPose(Pose2d pose) {
        Pose2d placed = onTheField(pose);
        // Out of the world and back in, so the engine forgets what the robot was touching.
        world.removeBody(robot);
        Transform transform = robot.getTransform();
        transform.setTranslation(placed.position.x * IN, placed.position.y * IN);
        transform.setRotation(placed.heading.toDouble());
        robot.setLinearVelocity(new Vector2());
        robot.setAngularVelocity(0);
        robot.clearForce();
        robot.clearTorque();
        world.addBody(robot);
        previous = pose();
        imu.yawRadians = previous.heading.toDouble();
    }

    /**
     * Set the robot down where a person would, asked to put it at {@code pose}: near it, as near
     * as the robot's {@link #noise() noise} says a hand gets, and exactly on it without noise.
     * Otherwise {@link #setPose}: at rest, on the field, its IMU reading the new heading.
     */
    public void setDown(Pose2d pose) {
        setPose(noise.placed(pose));
    }

    /**
     * Where the balls are, {x, y, z} each: the field's loose pieces in {@link SimField#loosePieces}'
     * order, then the robot's preload; null for a ball held in the robot.
     */
    public double[][] pieces() {
        double[][] out = new double[balls.length][];
        for (int i = 0; i < balls.length; i++) {
            Ball ball = balls[i];
            switch (ball.where) {
                case ROLLING:
                    Transform t = ball.body.getTransform();
                    out[i] = new double[] {t.getTranslationX() / IN, t.getTranslationY() / IN, ball.radius};
                    break;
                case HELD:
                    out[i] = null;
                    break;
                default:
                    out[i] = new double[] {ball.x, ball.y, ball.z};
            }
        }
        return out;
    }

    /** Set a ball down on the floor somewhere, at rest, whatever it was doing. */
    public void placePiece(int index, double x, double y) {
        Ball ball = balls[index];
        if (ball.where == Where.HELD) {
            hopper.remove(ball);
            if (chambered == ball) {
                chambered = null;
            }
        }
        if (ball.where == Where.ROLLING) {
            world.removeBody(ball.body); // and back in below, so the engine forgets what it was touching
        }
        ball.where = Where.ROLLING;
        ball.body.getTransform().setTranslation(x * IN, y * IN);
        ball.body.setLinearVelocity(new Vector2());
        ball.body.setAngularVelocity(0);
        ball.body.setAtRest(false);
        world.addBody(ball.body);
    }

    /** How many balls the robot holds: in its hopper and its chamber. */
    public int held() {
        return hopper.size() + (chambered == null ? 0 : 1);
    }

    /** How many balls are in that alliance's hive, "Blue" or "Red". */
    public int scored(String alliance) {
        int total = 0;
        for (Map.Entry<SimField.Cell, Integer> entry : scoredIn.entrySet()) {
            if (entry.getKey().alliance.equals(alliance)) {
                total += entry.getValue();
            }
        }
        return total;
    }

    /** How many balls each alliance has in its hive, by "Blue" and "Red". */
    public Map<String, Integer> scored() {
        Map<String, Integer> out = new LinkedHashMap<>();
        for (SimField.Cell cell : scoredIn.keySet()) {
            out.merge(cell.alliance, scoredIn.get(cell), Integer::sum);
        }
        return out;
    }

    /**
     * Advance the world by {@code dtSeconds} using the motor powers and servo positions currently
     * commanded. The world moves in steps of at most {@link #MAX_STEP_SECONDS}, so the robot never
     * jumps over an obstacle between two of them, however long the caller waited.
     */
    public void step(double dtSeconds) {
        nanos += Math.round(dtSeconds * 1e9);
        int steps = Math.max(1, (int) Math.ceil(dtSeconds / MAX_STEP_SECONDS));
        for (int i = 0; i < steps; i++) {
            substep(dtSeconds / steps);
        }
    }

    private void substep(double dt) {
        // The flywheel and the turntable are simple: the flywheel is at its commanded speed, the
        // turntable turns at a rate set by its power.
        launcher.measuredVelocity = launcher.commandedVelocity;
        turnTable.currentPosition +=
                (int) Math.round(clamp(turnTable.power) * TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER * dt);

        // The battery sags under the drive's load and drains with the run; the motors get what it reads.
        double drivePower = Math.abs(clamp(leftFront.power))
                + Math.abs(clamp(rightFront.power))
                + Math.abs(clamp(leftBack.power))
                + Math.abs(clamp(rightBack.power));
        voltageSensor.voltage = noise.batteryVolts(nanos / 1e9, drivePower);

        driveTheRobot(dt);
        feedTheLauncher();
        world.step(1, dt);
        flyTheBalls(dt);
        readTheSensors(dt);
    }

    /**
     * The motors push the robot: each wheel's acceleration from the tuned model, turned into a
     * force and a torque on the robot through the drive's kinematics, for the engine to integrate.
     */
    private void driveTheRobot(double dt) {
        Vector2 linear = robot.getLinearVelocity();
        double heading = robot.getTransform().getRotationAngle();
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double vx = linear.x / IN, vy = linear.y / IN;
        PoseVelocity2d inRobotFrame =
                new PoseVelocity2d(new Vector2d(cos * vx + sin * vy, -sin * vx + cos * vy), robot.getAngularVelocity());
        MecanumKinematics.WheelVelocities<Time> wheels =
                kinematics.inverse(PoseVelocity2dDual.constant(inRobotFrame, 1));

        double lf = wheelAcceleration(
                leftFront, LEFT_FRONT_MOUNT, noise.motor(SimNoise.LEFT_FRONT), wheels.leftFront.value(), dt);
        double lb = wheelAcceleration(
                leftBack, LEFT_BACK_MOUNT, noise.motor(SimNoise.LEFT_BACK), wheels.leftBack.value(), dt);
        double rb = wheelAcceleration(
                rightBack, RIGHT_BACK_MOUNT, noise.motor(SimNoise.RIGHT_BACK), wheels.rightBack.value(), dt);
        double rf = wheelAcceleration(
                rightFront, RIGHT_FRONT_MOUNT, noise.motor(SimNoise.RIGHT_FRONT), wheels.rightFront.value(), dt);
        // The forward kinematics are linear, so they take the wheels' accelerations to the robot's.
        Twist2d acceleration = kinematics
                .forward(new MecanumKinematics.WheelIncrements<>(dual(lf), dual(lb), dual(rb), dual(rf)))
                .value();
        double ax = cos * acceleration.line.x - sin * acceleration.line.y;
        double ay = sin * acceleration.line.x + cos * acceleration.line.y;
        double mass = robot.getMass().getMass();
        robot.applyForce(new Vector2(ax * IN * mass, ay * IN * mass));
        robot.applyTorque(acceleration.angle * robot.getMass().getInertia());
    }

    /**
     * A wheel's acceleration, in inches per second squared, from the model the drive was tuned
     * with: volts = kS * sign(v) + kV * v + kA * a, with the wheel at {@code velocity} inches per
     * second and the motor at its commanded power on the battery as it reads now. This motor's
     * kS, kV and kA are the tuned ones by its {@code factors}. Below kS the motor cannot start the
     * wheel, and a wheel that friction would stop within the step stops. Whatever the motor asks,
     * the floor gives no more acceleration than its traction, driving or braking.
     */
    private double wheelAcceleration(
            FakeDcMotorEx motor, int mount, SimNoise.Motor factors, double velocity, double dt) {
        int direction = motor.getDirection() == DcMotorSimple.Direction.REVERSE ? -1 : 1;
        double volts = mount * direction * clamp(motor.power) * voltageSensor.voltage;
        double kS = drive.kS * factors.kS, kV = drive.kV * factors.kV, kA = drive.kA * factors.kA;
        double ticksPerSecond = velocity / drive.inPerTick;
        boolean creeping = Math.abs(ticksPerSecond) <= kS / kA * dt;
        double acceleration;
        if (creeping && Math.abs(volts) <= kS) {
            acceleration = -velocity / dt;
        } else {
            double sign = ticksPerSecond != 0 ? Math.signum(ticksPerSecond) : Math.signum(volts);
            acceleration = (volts - kS * sign - kV * ticksPerSecond) / kA * drive.inPerTick;
        }
        double traction = noise.tractionInPerS2;
        return Math.max(-traction, Math.min(traction, acceleration));
    }

    private static DualNum<Time> dual(double value) {
        return new DualNum<>(new double[] {value, 0});
    }

    /**
     * The gates feed the launcher: with the top gate open a ball drops from the hopper into the
     * empty chamber; with the bottom gate open the chambered ball drops into the flywheel and
     * leaves the robot.
     */
    private void feedTheLauncher() {
        if (topGate.position >= TOP_GATE_OPENS_AT && chambered == null && !hopper.isEmpty()) {
            chambered = hopper.poll();
        }
        if (bottomGate.position >= BOTTOM_GATE_OPENS_AT && chambered != null) {
            launch(chambered);
            chambered = null;
        }
    }

    /**
     * A ball leaves the launcher, which faces where the turntable does, at a speed set by the
     * flywheel's, plus the robot's own.
     */
    private void launch(Ball ball) {
        Pose2d pose = pose();
        double aim = pose.heading.toDouble() + turnTableOffsetRadians();
        double speed = Math.abs(launcher.measuredVelocity) * LAUNCH_IN_PER_S_PER_TICK_PER_S;
        Vector2 robotVelocity = robot.getLinearVelocity();
        ball.where = Where.FLYING;
        ball.x = pose.position.x + LAUNCH_AHEAD_IN * Math.cos(aim);
        ball.y = pose.position.y + LAUNCH_AHEAD_IN * Math.sin(aim);
        ball.z = LAUNCH_HEIGHT_IN;
        ball.vx = robotVelocity.x / IN + speed * Math.cos(LAUNCH_ANGLE_RADIANS) * Math.cos(aim);
        ball.vy = robotVelocity.y / IN + speed * Math.cos(LAUNCH_ANGLE_RADIANS) * Math.sin(aim);
        ball.vz = speed * Math.sin(LAUNCH_ANGLE_RADIANS);
    }

    /** How far the turntable, and so the launcher, has turned from straight ahead, counter-clockwise. */
    private double turnTableOffsetRadians() {
        return (double) turnTable.currentPosition / Turntable.TICKS_PER_REVOLUTION * 2 * Math.PI;
    }

    /**
     * The flying balls move on under gravity. One that crosses a hive cell's mouth going in has
     * scored; one that meets another panel of a hive bounces off it; one that comes down on the
     * floor bounces or, slow enough, lands and rolls; one that reaches a wall bounces off it, or
     * is out if it is over it.
     */
    private void flyTheBalls(double dt) {
        for (Ball ball : balls) {
            if (ball.where != Where.FLYING) {
                continue;
            }
            double[] from = {ball.x, ball.y, ball.z};
            ball.vz -= GRAVITY_IN_PER_S2 * dt;
            ball.x += ball.vx * dt;
            ball.y += ball.vy * dt;
            ball.z += ball.vz * dt;
            if (meetsAHive(ball, from)) {
                continue;
            }
            if (ball.z < ball.radius) {
                ball.z = ball.radius;
                if (-ball.vz < LANDING_SPEED_IN_PER_S) {
                    land(ball);
                    continue;
                }
                ball.vz = -ball.vz * BOUNCE;
            }
            double limit = FIELD_SIZE_IN / 2 - ball.radius;
            for (int axis = 0; axis < 2; axis++) {
                double position = axis == 0 ? ball.x : ball.y;
                if (Math.abs(position) <= limit) {
                    continue;
                }
                if (ball.z - ball.radius > WALL_HEIGHT_IN) {
                    ball.where = Where.OUT;
                    ball.z = ball.radius;
                    break;
                }
                double clamped = Math.signum(position) * limit;
                if (axis == 0) {
                    ball.x = clamped;
                    ball.vx = -ball.vx * BOUNCE;
                } else {
                    ball.y = clamped;
                    ball.vy = -ball.vy * BOUNCE;
                }
            }
        }
    }

    /**
     * Whether the ball's move from {@code from} met a hive: through a cell's mouth, scoring, or
     * against another panel, bouncing off. A panel is met where the move crosses its plane inside
     * its ring.
     */
    private boolean meetsAHive(Ball ball, double[] from) {
        double[] to = {ball.x, ball.y, ball.z};
        for (SimField.Cell cell : FIELD.cells) {
            for (double[][] panel : cell.panels) {
                double[] normal = panel == cell.mouth ? cell.mouthNormal : SimField.normal(panel);
                double[] hit = crossing(panel, normal, from, to);
                if (hit == null) {
                    continue;
                }
                boolean goingIn = side(normal, panel[0], from) > 0;
                if (panel == cell.mouth && goingIn) {
                    score(ball, cell);
                    return true;
                }
                double along = ball.vx * normal[0] + ball.vy * normal[1] + ball.vz * normal[2];
                ball.vx -= (1 + BOUNCE) * along * normal[0];
                ball.vy -= (1 + BOUNCE) * along * normal[1];
                ball.vz -= (1 + BOUNCE) * along * normal[2];
                double back = goingIn ? CONTACT_TOLERANCE_IN : -CONTACT_TOLERANCE_IN;
                ball.x = hit[0] + back * normal[0];
                ball.y = hit[1] + back * normal[1];
                ball.z = hit[2] + back * normal[2];
                return true;
            }
        }
        return false;
    }

    /** The ball is in the cell: at rest inside it, beside any already there. */
    private void score(Ball ball, SimField.Cell cell) {
        int already = scoredIn.get(cell);
        scoredIn.put(cell, already + 1);
        ball.where = Where.SCORED;
        ball.x = cell.centre[0];
        ball.y = cell.centre[1] + 2 * ball.radius * (already % 2 == 0 ? already / 2 : -(already + 1) / 2);
        ball.z = cell.centre[2];
        ball.vx = ball.vy = ball.vz = 0;
    }

    /** The ball comes down on the floor and rolls on with the speed it had along it. */
    private void land(Ball ball) {
        ball.where = Where.ROLLING;
        world.addBody(ball.body);
        ball.body.getTransform().setTranslation(ball.x * IN, ball.y * IN);
        ball.body.setLinearVelocity(new Vector2(ball.vx * IN, ball.vy * IN));
        ball.body.setAngularVelocity(0);
        ball.body.setAtRest(false);
    }

    /** Which side of the plane through {@code point} with {@code normal} the position is on: positive along the normal. */
    private static double side(double[] normal, double[] point, double[] position) {
        return (position[0] - point[0]) * normal[0]
                + (position[1] - point[1]) * normal[1]
                + (position[2] - point[2]) * normal[2];
    }

    /**
     * Where the move from {@code from} to {@code to} crosses the panel's plane, if it does and the
     * crossing is inside the panel's ring; else null.
     */
    private static double[] crossing(double[][] panel, double[] normal, double[] from, double[] to) {
        double before = side(normal, panel[0], from);
        double after = side(normal, panel[0], to);
        if (before == 0 || (before > 0) == (after > 0)) {
            return null;
        }
        double t = before / (before - after);
        double[] hit = {
            from[0] + (to[0] - from[0]) * t, from[1] + (to[1] - from[1]) * t, from[2] + (to[2] - from[2]) * t
        };
        // Inside the ring, seen along the normal's largest axis, where the ring is widest.
        int drop = 0;
        for (int axis = 1; axis < 3; axis++) {
            if (Math.abs(normal[axis]) > Math.abs(normal[drop])) {
                drop = axis;
            }
        }
        int u = (drop + 1) % 3, v = (drop + 2) % 3;
        boolean inside = false;
        for (int i = 0, j = panel.length - 1; i < panel.length; j = i++) {
            double[] a = panel[i], b = panel[j];
            if ((a[v] > hit[v]) != (b[v] > hit[v]) && hit[u] < (b[u] - a[u]) * (hit[v] - a[v]) / (b[v] - a[v]) + a[u]) {
                inside = !inside;
            }
        }
        return inside ? hit : null;
    }

    /**
     * The dead wheels roll on the floor, so they read what the robot actually did: nothing when
     * the wheels spin against a wall, and only the sliding component when it drives into one at
     * an angle. Their readings are what TwoDeadWheelLocalizer expects to invert.
     */
    private void readTheSensors(double dt) {
        Pose2d pose = pose();
        Twist2d delta = pose.minus(previous);
        previous = pose;
        Vector2 linear = robot.getLinearVelocity();
        double heading = pose.heading.toDouble();
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double vx = linear.x / IN, vy = linear.y / IN;
        PoseVelocity2d velocity =
                new PoseVelocity2d(new Vector2d(cos * vx + sin * vy, -sin * vx + cos * vy), robot.getAngularVelocity());

        parTicks += delta.line.x / drive.inPerTick + deadWheels.parYTicks * delta.angle;
        perpTicks += delta.line.y / drive.inPerTick + deadWheels.perpXTicks * delta.angle;
        double parVelocity = velocity.linearVel.x / drive.inPerTick + deadWheels.parYTicks * velocity.angVel;
        double perpVelocity = velocity.linearVel.y / drive.inPerTick + deadWheels.perpXTicks * velocity.angVel;
        rightBack.currentPosition = (int) Math.round(PAR_RAW_SIGN * parTicks);
        rightBack.measuredVelocity = asTheHubReports(PAR_RAW_SIGN * parVelocity);
        leftFront.currentPosition = (int) Math.round(PERP_RAW_SIGN * perpTicks);
        leftFront.measuredVelocity = asTheHubReports(PERP_RAW_SIGN * perpVelocity);

        imu.yawRadians = heading;
        imu.yawRateRadiansPerSecond = velocity.angVel;
    }

    /** An encoder velocity as the hub reports it: to the nearest {@link #HUB_VELOCITY_STEP_TICKS_PER_S}. */
    private static double asTheHubReports(double ticksPerSecond) {
        return Math.round(ticksPerSecond / HUB_VELOCITY_STEP_TICKS_PER_S) * HUB_VELOCITY_STEP_TICKS_PER_S;
    }

    /**
     * The pose the field allows: {@link #clearOfTheObstacles clear of the obstacles} and
     * {@link #insideTheWalls inside the walls}, in that order, so a robot pushed out of an
     * obstacle at the wall still ends inside the field.
     */
    public static Pose2d onTheField(Pose2d candidate) {
        return insideTheWalls(clearOfTheObstacles(candidate));
    }

    /**
     * The pose the walls allow: the same heading, and the position pushed back just far enough that
     * no corner of the robot's square is beyond a wall. A wall is a straight line, so the square
     * reaches it at half its side scaled by how far the heading is from square-on. Each axis is
     * clamped on its own, which is what lets the robot slide along a wall it drives into at an
     * angle.
     */
    public static Pose2d insideTheWalls(Pose2d candidate) {
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
            corners[i] = new double[] {
                position.x + local[i][0] * heading.real - local[i][1] * heading.imag,
                position.y + local[i][0] * heading.imag + local[i][1] * heading.real
            };
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
        for (double[][] polygon : new double[][][] {obstacle, robot}) {
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
        double side = (robotCentre[0] - obstacleCentre[0]) * leastAxis[0]
                + (robotCentre[1] - obstacleCentre[1]) * leastAxis[1];
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
        return new double[] {min, max};
    }

    private static double[] centre(double[][] polygon) {
        double x = 0, y = 0;
        for (double[] p : polygon) {
            x += p[0];
            y += p[1];
        }
        return new double[] {x / polygon.length, y / polygon.length};
    }

    private static double clamp(double power) {
        return Math.max(-1, Math.min(1, power));
    }

    /** The bodies in the world, for a test of the model itself. */
    List<Body> bodies() {
        return world.getBodies();
    }
}
