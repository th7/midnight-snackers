package org.firstinspires.ftc.teamcode.sim;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.dyn4j.collision.Filter;
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
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * The robot on the season's field ({@link SimField}), as rigid bodies in a dyn4j world: the robot,
 * the walls, the field's obstacles, and the balls. The robot is driven by the model Road Runner
 * was tuned with ({@link MecanumDrive.Params}): each wheel's motor, at its commanded power, pushes
 * its wheel toward the speed the tuned kS and kV give, at the rate the tuned kA allows, so the
 * robot takes time to get up to speed and to stop, and its true pose comes out of the engine
 * integrating that push against whatever it runs into. How it stops with nothing commanded is the
 * zero power behavior each wheel is set to: a braking wheel is held by its motor's back EMF and by
 * friction, a floating one by friction alone, so it rolls on several times as far. The simulation
 * will not guess at a wheel set to neither. The walls and the obstacles stop it and
 * let it slide along them; a ball it pushes rolls on and slows; a ball pinned against a wall
 * stops the robot short of it, since nothing goes through anything. The sensors the localizer
 * reads (dead wheel encoders and IMU yaw) are written back from the true pose, so the dead
 * wheels read nothing while the wheels spin against a wall. A robot made with {@link SimNoise}
 * is off that model the ways a real robot is: its motors each a little off their tuning, its
 * battery fresh or flat and sagging under load, its traction limited so it can slip, and set
 * down near a pose rather than on it. The noise is in the mechanisms only; the sensors still
 * read exactly what the robot did.
 * <p>
 * The robot starts with {@link #PRELOAD} balls in it and holds {@link #HOLDS}. Its intake takes in
 * the pollen the front of the robot meets while the intake's motor is running and there is room for
 * them; a nectar is the bigger ball and no intake of ours takes one, so a nectar, and a pollen the
 * intake is not running for, is a ball the robot pushes. The launcher is on the turntable, and its
 * gates feed it as the robot code drives them: with the top gate open a ball drops from the hopper
 * into the chamber, and with the bottom gate open the chambered ball drops into the flywheel and
 * leaves at a speed set by the flywheel's, on an arc under gravity. A ball that goes in through a
 * hive cell's mouth is in the cell, at rest on the floor at the back; one that meets a wall or a
 * back bounces off it, whichever side it comes from; one that comes down on the floor rolls on;
 * one that clears a wall is out. The
 * model is planar apart from that flight and the flowers' stacks: only the robot's footprint
 * collides, nothing goes over a wall or under a hive by being low, and a flying ball meets only
 * the hives, the floor and the walls. What a body on the floor meets is what it can reach, so a
 * ball rolls under an obstacle that overhangs it — a flower's pipes begin four inches up — and the
 * robot, eighteen inches tall, runs into the same obstacle.
 * <p>
 * Each flower stands a stack of pollen in its bore, one resting on another from the floor up. The
 * bore's wall begins at its lip, so the pollen at the bottom stands wholly below it: what holds
 * that one is the nest, the ring in the flower's base plate it sits in the middle of. To leave it
 * has to roll up over that ring carrying the stack, which a loose ball rolling in is far too light
 * to do — it knocks the pollen a little way up the ring and the ring rolls it back — while the
 * robot's own push has its drive behind it and takes one straight out. Once the bore's floor is
 * clear the stack comes down, each pollen falling under gravity onto what is under it, landing and
 * settling, and stands again. A ball that comes to rest in a bore holds a stack up as well as a
 * pollen of its own does.
 * <p>
 * The clock is the world's ({@link #nanoTime()}), read by the robot code through its
 * hardware, so a run is the same every time and need not take real time. The world moves in
 * steps of at most {@link #MAX_STEP_SECONDS} however long the caller waited, so nothing is jumped
 * over.
 * <p>
 * Each hive holds one of its cells up and the other under, and only the upturned one keeps what
 * goes in: what is in a downturned cell rolls out of its mouth and falls to the floor. A hive that
 * is {@link #load full} — five nectar, or eight pollen, or a combination worth as much — tips, so
 * the cell that was taking balls goes under and empties and the other comes up in its place.
 * <p>
 * The launcher's throw ({@link #LAUNCH_HEIGHT_IN} and the constants around it) and the turntable's
 * speed are guesses until measured on the robot, calibrated so the robot code's close launch from
 * its launch distance drops into the middle of the upturned cell's mouth.
 */
public class SimRobot {
    /** The season's field, and where it lets a robot be: geometry, which this drives bodies through. */
    public static final SimField FIELD = SimPlacement.FIELD;

    /** The battery a robot with no noise runs on; the devices hold it, since the sensor is theirs. */
    public static final double BATTERY_VOLTS = SimDevices.BATTERY_VOLTS;
    /**
     * How many pollen the robot starts with, in its hopper: what it can launch before it is empty,
     * and with the three nectar a hive is set up with, enough to fill one and tip it.
     */
    public static final int PRELOAD = 4;
    /**
     * How many balls the robot holds, in its hopper and its chamber together: what the intake may
     * fill it to, and what the {@link #PRELOAD} fills it with.
     */
    public static final int HOLDS = 4;
    /** A quarter inch at full speed: far less than the thinnest obstacle. */
    public static final double MAX_STEP_SECONDS = 0.005;
    /** Where a launched ball leaves the robot: this high off the floor, and this far ahead of the robot's centre. */
    public static final double LAUNCH_HEIGHT_IN = 14;

    public static final double LAUNCH_AHEAD_IN = 6;
    /**
     * A launched ball leaves this far above horizontal: steeply, because a hive holds its upturned
     * cell's mouth five feet up and a ball has to come down into it from over the rim.
     */
    public static final double LAUNCH_ANGLE_RADIANS = Math.toRadians(70);
    /**
     * How fast a launched ball leaves, in inches per second, for each encoder tick per second of
     * the flywheel: a close launch (1050 ticks per second) from 40 inches drops into the middle of
     * the upturned cell's mouth.
     */
    public static final double LAUNCH_IN_PER_S_PER_TICK_PER_S = 0.189;

    /**
     * A hive tips when it is full: five nectar fill it, or eight pollen, or a combination worth as
     * much. Counted in fortieths, so that a combination is exact.
     */
    private static final int FULL = 40;

    private static final int NECTAR_FILLS = FULL / 5;
    private static final int POLLEN_FILLS = FULL / 8;
    /** How fast a ball rolls out of a downturned cell's mouth, the length of the cell behind it. */
    private static final double ROLL_OUT_IN_PER_S = 20;

    private static final double ROBOT_MASS_KG = 15;
    /** Heavy for pollen, but a ball that light next to the robot is what the engine's solver handles worst. */
    private static final double BALL_MASS_KG = 0.5;

    /**
     * A pollen whose surface is this near the front of the robot is in contact with it: the engine
     * keeps a ball the robot has met from overlapping it, and the robot covers no more than this in
     * one step, so a pollen the robot drives into is seen here before it is pushed away. The intake
     * reaches no further than the front of the robot, so this is a tolerance and not a reach.
     */
    private static final double INTAKE_REACH_IN = 0.25;

    private static final double TURNTABLE_TICKS_PER_SECOND_AT_FULL_POWER = 1700;
    /** A rolling ball loses its speed with this time constant, and is at rest below {@link #REST_SPEED_IN_PER_S}. */
    private static final double ROLL_SECONDS = 0.8;

    private static final double REST_SPEED_IN_PER_S = 0.5;
    /** How much of the closing speed a ball keeps, bouncing off a wall, an obstacle, the robot, a hive or another ball. */
    private static final double BOUNCE = 0.3;

    private static final double GRAVITY_IN_PER_S2 = 386.09;
    /** How much of the weight on a ball in a flower's nest the floor drags back with: rubber on tile. */
    private static final double NEST_FRICTION = 0.5;
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

    /** The devices this robot drives, and its clock: the other adapter at the hardware seam. */
    private final SimDevices devices = new SimDevices();

    public final FakeDcMotorEx leftFront = devices.leftFront;
    public final FakeDcMotorEx rightFront = devices.rightFront;
    public final FakeDcMotorEx leftBack = devices.leftBack;
    public final FakeDcMotorEx rightBack = devices.rightBack;
    public final FakeDcMotorEx launcher = devices.launcher;
    public final FakeDcMotorEx turnTable = devices.turnTable;
    public final FakeDcMotorEx intake = devices.intake;
    public final FakeServo topGate = devices.topGate;
    public final FakeServo bottomGate = devices.bottomGate;
    public final FakeImu imu = devices.imu;
    public final FakeVoltageSensor voltageSensor = devices.voltageSensor;
    public final FakeDashboard dashboard = devices.dashboard;

    /** How this robot differs from the tuned model; {@link SimNoise#NONE} for the model exactly. */
    private final SimNoise noise;

    private final MecanumDrive.Params drive = MecanumDrive.PARAMS;
    private final TwoDeadWheelLocalizer.Params deadWheels = TwoDeadWheelLocalizer.PARAMS;
    private final MecanumKinematics kinematics =
            new MecanumKinematics(drive.inPerTick * drive.trackWidthTicks, drive.inPerTick / drive.lateralInPerTick);
    private final World<Body> world = new World<>();
    private final Body robot;
    /** Where the robot was after the last step, for the sensors. */
    private Pose2d previous = new Pose2d(0, 0, 0);

    private double parTicks = 0;
    private double perpTicks = 0;
    /**
     * The balls: the field's loose pieces in {@link SimField#loosePieces}' order, then the nectar
     * the hives are set up with in {@link SimField#cellPieces}' order, then the pollen the flowers
     * are set up with in {@link SimField#flowerPieces}' order, then the preload.
     */
    private final Ball[] balls;
    /** Every ball's body, for following the robot's push from body to body through the engine's contacts. */
    private final List<Body> ballBodies = new ArrayList<>();
    /** The balls in the robot's hopper, above the top gate, in the order they will drop. */
    private final Deque<Ball> hopper = new ArrayDeque<>();
    /** The ball between the gates, or null. */
    private Ball chambered = null;
    /** What is in each cell, in the order it went in: what rolls out when the cell turns over. */
    private final Map<SimField.Cell, List<Ball>> inCell = new LinkedHashMap<>();
    /** What the bore of each flower holds up off the floor, lowest first: the stack above its bottom ball. */
    private final Map<SimField.Flower, List<Ball>> inFlower = new LinkedHashMap<>();
    /** The ball a robot's push has rolled up off each nest's seat, until it comes to rest again. */
    private final Map<SimField.Flower, Ball> offTheSeat = new LinkedHashMap<>();
    /** How far each hive leans now, in degrees: its own tilt until it tips, then the other way. */
    private final Map<SimField.Hive, Double> tilts = new LinkedHashMap<>();
    /** Each cell where the hive leans now, rebuilt when it tips. */
    private final Map<SimField.Cell, Turned> turned = new LinkedHashMap<>();

    /** Where a ball is, and what it is doing. */
    private enum Where {
        /** On the floor, in the engine's world. */
        ROLLING,
        /** In the air, on its arc. */
        FLYING,
        /** In the robot. */
        HELD,
        /** In a hive cell, at rest. */
        IN_CELL,
        /** Up in a flower's bore, resting on what is under it or falling onto it. */
        IN_FLOWER,
        /** Over a wall and out of the field, at rest where it came down. */
        OUT
    }

    private static final class Ball {
        final double radius;
        /** {@link SimField#NECTAR} or {@link SimField#POLLEN}: how much of a hive's load it is. */
        final String kind;

        final Body body;
        Where where;
        /** The cell the ball is in, while it is in one. */
        SimField.Cell cell;
        /** The flower whose bore holds the ball up, while one does. */
        SimField.Flower flower;
        /** The position, and while flying the velocity, of a ball that is not in the engine's world. */
        double x, y, z, vx, vy, vz;

        Ball(double radius, String kind, Body body) {
            this.radius = radius;
            this.kind = kind;
            this.body = body;
        }
    }

    /**
     * How high above the floor a body stands, which is what says whether it meets another: nothing
     * meets what it cannot reach. The robot and a ball stand on the floor, an obstacle wherever the
     * field model puts it, and two bodies collide only where those heights overlap — so a ball rolls
     * under a flower's pipes, which begin four inches up, and the robot, being eighteen inches tall,
     * runs into them.
     */
    private static final class Reaches implements Filter {
        private final double clears;
        private final double stands;

        Reaches(double clears, double stands) {
            this.clears = clears;
            this.stands = stands;
        }

        @Override
        public boolean isAllowed(Filter other) {
            if (!(other instanceof Reaches)) {
                return true;
            }
            Reaches that = (Reaches) other;
            return stands > that.clears && that.stands > clears;
        }

        @Override
        public Filter copy() {
            return this;
        }
    }

    /** A cell in the field frame, where its hive leans now: what a flying ball meets. */
    private static final class Turned {
        final double[][] mouth;
        final double[] mouthNormal;
        final List<double[][]> panels;
        final boolean upturned;
        /** The middle of the cell's floor where it meets the back: where the first ball in rests. */
        final double[] floorAtTheBack;
        /** Along the floor from the back toward the mouth, across it, and up off it. */
        final double[] towardTheMouth;

        final double[] acrossTheFloor;
        final double[] offTheFloor;
        /** How wide the flat of the cell's floor is: how many balls rest side by side on it. */
        final double floorWidth;

        Turned(SimField.Cell cell, double tilt) {
            this.mouth = cell.mouthAt(tilt);
            this.mouthNormal = cell.mouthNormalAt(tilt);
            this.panels = cell.panelsAt(tilt);
            this.upturned = cell.upturnedAt(tilt);
            double[][] floor = floorOf(cell.back);
            this.floorAtTheBack = cell.hive.at(tilt, mean(floor));
            this.floorWidth = floor[floor.length - 1][1] - floor[0][1];
            this.towardTheMouth = cell.hive.direction(tilt, new double[] {Math.signum(cell.mouthNormal[0]), 0, 0});
            this.acrossTheFloor = cell.hive.direction(tilt, new double[] {0, 1, 0});
            this.offTheFloor = cell.hive.direction(tilt, new double[] {0, 0, 1});
        }

        /** The corners the ring stands lowest on, across the cell: the flat of its floor. */
        private static double[][] floorOf(double[][] ring) {
            double lowest = Double.MAX_VALUE;
            for (double[] corner : ring) {
                lowest = Math.min(lowest, corner[2]);
            }
            List<double[]> floor = new ArrayList<>();
            for (double[] corner : ring) {
                if (corner[2] <= lowest + 0.2) {
                    floor.add(corner);
                }
            }
            floor.sort((a, b) -> Double.compare(a[1], b[1]));
            return floor.toArray(new double[0][]);
        }

        private static double[] mean(double[][] points) {
            double[] sum = new double[3];
            for (double[] p : points) {
                for (int axis = 0; axis < 3; axis++) {
                    sum[axis] += p[axis] / points.length;
                }
            }
            return sum;
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

        double half = SimPlacement.FIELD_SIZE_IN / 2 * IN;
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
        BodyFixture footprint = robot.addFixture(
                Geometry.createRectangle(SimPlacement.ROBOT_SIZE_IN * IN, SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setDensity(ROBOT_MASS_KG / (SimPlacement.ROBOT_SIZE_IN * IN * SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setFriction(0);
        footprint.setRestitution(0);
        footprint.setFilter(new Reaches(0, SimPlacement.ROBOT_SIZE_IN));
        robot.setMass(MassType.NORMAL);
        robot.setAtRestDetectionEnabled(false);
        robot.setLinearDamping(0);
        robot.setAngularDamping(0);
        world.addBody(robot);

        for (SimField.Hive hive : FIELD.hives) {
            tilts.put(hive, hive.tilt);
        }
        for (SimField.Cell cell : FIELD.cells) {
            inCell.put(cell, new ArrayList<>());
            turned.put(cell, new Turned(cell, tilts.get(cell.hive)));
        }
        for (SimField.Flower flower : FIELD.flowers) {
            inFlower.put(flower, new ArrayList<>());
        }
        int loose = FIELD.loosePieces.size();
        int inCells = loose + FIELD.cellPieces.size();
        int held = inCells + FIELD.flowerPieces.size();
        balls = new Ball[held + PRELOAD];
        for (int i = 0; i < balls.length; i++) {
            SimField.Piece piece = i < held ? FIELD.movedPieces.get(i) : FIELD.loosePieces.get(0);
            balls[i] = new Ball(piece.radius, piece.kind, ballBody(piece.radius));
            ballBodies.add(balls[i].body);
            if (i < loose) {
                placePiece(i, piece.x, piece.y);
            } else if (i < inCells) {
                putInCell(balls[i], FIELD.cell(piece.cell));
            } else if (i < held) {
                putInFlower(balls[i], FIELD.flower(piece.flower), piece.z);
            } else {
                intoTheHopper(balls[i]);
            }
        }
        restTheFlowers();
    }

    private static Body wall(double x, double y, double width, double height) {
        Body wall = new Body();
        BodyFixture fixture = wall.addFixture(Geometry.createRectangle(width, height));
        fixture.setFriction(0);
        fixture.setRestitution(0);
        fixture.setFilter(new Reaches(0, SimPlacement.WALL_HEIGHT_IN));
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
            fixture.setFilter(new Reaches(obstacle.clears, obstacle.stands));
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
        fixture.setFilter(new Reaches(0, 2 * radius));
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
        return devices.hardware();
    }

    /**
     * The same devices, with a camera that sees {@code aprilTags}: the simulator has no vision of
     * its own, so a test that wants the robot to see something says what.
     */
    public Hardware hardware(Supplier<List<AprilTagDetection>> aprilTags) {
        return devices.hardware(aprilTags);
    }

    /**
     * The simulation's clock, in nanoseconds since the world was made: what the robot code's timers
     * read, through its hardware. It advances only when the world does.
     */
    public long nanoTime() {
        return devices.nanoTime();
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
        Pose2d placed = SimPlacement.onTheField(pose);
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
     * order, then the nectar the hives are set up with, then the pollen the flowers are set up with,
     * then the robot's preload; null for a ball held in the robot.
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
                case IN_CELL:
                    out[i] = restingPlace(ball);
                    break;
                default:
                    out[i] = new double[] {ball.x, ball.y, ball.z};
            }
        }
        return out;
    }

    /** Set a ball down on the floor somewhere, at rest, whatever it was doing. */
    public void placePiece(int index, double x, double y) {
        setDown(balls[index], x, y);
    }

    /** A ball on the floor there, at rest, in the engine's world, wherever it was before. */
    private void setDown(Ball ball, double x, double y) {
        take(ball);
        ball.where = Where.ROLLING;
        ball.body.getTransform().setTranslation(x * IN, y * IN);
        ball.body.setLinearVelocity(new Vector2());
        ball.body.setAngularVelocity(0);
        ball.body.setAtRest(false);
        world.addBody(ball.body);
    }

    /** Put a ball in a hive cell, at rest behind whatever is in it already, wherever it was. */
    public void placePiece(int index, SimField.Cell cell) {
        putInCell(balls[index], cell);
    }

    /** Take a ball out of wherever it is, so that it can be put somewhere else. */
    private void take(Ball ball) {
        if (ball.where == Where.HELD) {
            hopper.remove(ball);
            if (chambered == ball) {
                chambered = null;
            }
        }
        if (ball.where == Where.ROLLING) {
            world.removeBody(ball.body); // so the engine forgets what it was touching
        }
        if (ball.where == Where.IN_CELL) {
            inCell.get(ball.cell).remove(ball);
            ball.cell = null;
        }
        if (ball.where == Where.IN_FLOWER) {
            inFlower.get(ball.flower).remove(ball);
            ball.flower = null;
        }
    }

    /** Put a ball in a cell, where it rests on the floor at the back behind the ones already in. */
    private void putInCell(Ball ball, SimField.Cell cell) {
        take(ball);
        ball.where = Where.IN_CELL;
        ball.cell = cell;
        ball.vx = ball.vy = ball.vz = 0;
        inCell.get(cell).add(ball);
    }

    /**
     * Where a ball in a cell rests: on the floor at the back, in a row across the cell, the row
     * behind it filled first.
     */
    private double[] restingPlace(Ball ball) {
        Turned turn = turned.get(ball.cell);
        int slot = inCell.get(ball.cell).indexOf(ball);
        int perRow = Math.max(1, (int) (turn.floorWidth / (2 * ball.radius)));
        double across = (slot % perRow - (perRow - 1) / 2.0) * 2 * ball.radius;
        double along = ball.radius + slot / perRow * 2 * ball.radius;
        double[] out = new double[3];
        for (int axis = 0; axis < 3; axis++) {
            out[axis] = turn.floorAtTheBack[axis]
                    + turn.towardTheMouth[axis] * along
                    + turn.acrossTheFloor[axis] * across
                    + turn.offTheFloor[axis] * ball.radius;
        }
        return out;
    }

    /**
     * Put a pollen in a flower's bore, at that height, in its place in the stack: the bore holds it
     * on its axis, and what is under it holds it up.
     */
    private void putInFlower(Ball ball, SimField.Flower flower, double z) {
        take(ball);
        ball.where = Where.IN_FLOWER;
        ball.flower = flower;
        ball.x = flower.axis[0];
        ball.y = flower.axis[1];
        ball.z = z;
        ball.vx = ball.vy = ball.vz = 0;
        List<Ball> stack = inFlower.get(flower);
        int place = 0;
        while (place < stack.size() && stack.get(place).z < z) {
            place++;
        }
        stack.add(place, ball);
    }

    /**
     * The stacks as the field is set up: every pollen already at rest on what is under it, so that
     * a flower nobody has touched stands exactly still from the first step.
     */
    private void restTheFlowers() {
        for (SimField.Flower flower : FIELD.flowers) {
            double under = topOfTheBore(flower);
            for (Ball ball : new ArrayList<>(inFlower.get(flower))) {
                ball.z = under + ball.radius;
                ball.vz = 0;
                under = settled(ball, flower);
            }
        }
    }

    /**
     * The pollen in the flowers come down. Each rests on what is under it in the bore — the ball
     * standing in the bore at the bottom, or the floor when there is none — and falls under gravity
     * when there is nothing there, landing with a bounce until it is at rest. Taking the bottom one
     * out of a flower is all it takes to bring the rest down one place.
     */
    private void fallInTheFlowers(double dt) {
        for (SimField.Flower flower : FIELD.flowers) {
            double under = topOfTheBore(flower);
            for (Ball ball : new ArrayList<>(inFlower.get(flower))) {
                double resting = under + ball.radius;
                if (ball.z > resting) {
                    ball.vz -= GRAVITY_IN_PER_S2 * dt;
                    ball.z += ball.vz * dt;
                }
                if (ball.z <= resting) {
                    ball.z = resting;
                    ball.vz = -ball.vz < LANDING_SPEED_IN_PER_S ? 0 : -ball.vz * BOUNCE;
                }
                under = settled(ball, flower);
            }
        }
    }

    /**
     * How high in the bore a pollen that has just been moved leaves the stack standing. One that has
     * come to rest wholly below the bore's lip is held by nothing any more: it stands in the bore on
     * the floor, in the engine's world like any other ball, and holds up whatever is above it — which
     * is why it is the one that comes out of a flower.
     */
    private double settled(Ball ball, SimField.Flower flower) {
        if (ball.vz == 0 && ball.z + ball.radius <= flower.lip) {
            setDown(ball, flower.axis[0], flower.axis[1]);
        }
        return ball.z + ball.radius;
    }

    /**
     * The nests hold on to the balls in them. A ball at the bottom of a bore sits in the flower's
     * nest — the ring in its base plate — and to leave has to roll up over that ring carrying
     * whatever rests on it, so the nest pushes it back toward the axis: hardest at the bore's rim,
     * not at all in the middle. The stack presses it into the floor as well, and the floor drags
     * back, which is what settles a knocked ball into its nest rather than letting it roll about in
     * there. A nest holds one ball, the one nearest its middle; another in the bore is on the plate
     * around it and free.
     * <p>
     * A ball the robot is pushing is the exception: the robot's drive is behind that push and the
     * ring is no barrier to it, so the nest lets go — and once a push has rolled a ball up off its
     * seat it is out of the nest until it comes to rest again, wherever that is. That is the whole
     * of the difference between a pollen the robot drives into a flower, which takes one out, and a
     * loose one rolling in on its own, which is too light to and rolls back off.
     */
    private void holdTheNests(double dt) {
        for (SimField.Flower flower : FIELD.flowers) {
            Ball nested = nestedIn(flower);
            if (nested == null) {
                offTheSeat.remove(flower);
                continue;
            }
            if (pushedByTheRobot(nested)) {
                offTheSeat.put(flower, nested);
            }
            if (offTheSeat.get(flower) != nested) {
                offTheSeat.remove(flower); // whatever was rolled off is not the nest's ball any more
            } else if (nested.body.getLinearVelocity().getMagnitude() > REST_SPEED_IN_PER_S * IN) {
                continue; // still rolling off its seat, and the nest has no hold on it
            } else {
                offTheSeat.remove(flower); // it has come to rest, so the nest has it again
            }
            Transform at = nested.body.getTransform();
            double toTheAxis = flower.axis[0] - at.getTranslationX() / IN;
            double acrossToIt = flower.axis[1] - at.getTranslationY() / IN;
            double out = Math.hypot(toTheAxis, acrossToIt);
            if (out > 0) {
                double hold = overTheRing(flower, nested) * Math.min(1, out / flower.bore);
                nested.body.applyForce(new Vector2(hold * toTheAxis / out, hold * acrossToIt / out));
            }
            Vector2 rolling = nested.body.getLinearVelocity();
            double speed = rolling.getMagnitude();
            if (speed > 0) {
                double drag = Math.min(
                        NEST_FRICTION * loadOn(flower), nested.body.getMass().getMass() * speed / dt);
                nested.body.applyForce(rolling.getNormalized().multiply(-drag));
            }
        }
    }

    /** The ball in the flower's nest: the one standing nearest the middle of its bore, or null. */
    private Ball nestedIn(SimField.Flower flower) {
        Ball nested = null;
        double nearest = Double.MAX_VALUE;
        for (Ball ball : balls) {
            if (ball.where != Where.ROLLING) {
                continue;
            }
            Transform at = ball.body.getTransform();
            double x = at.getTranslationX() / IN, y = at.getTranslationY() / IN;
            if (!flower.standsIn(x, y)) {
                continue;
            }
            double out = Math.hypot(x - flower.axis[0], y - flower.axis[1]);
            if (out < nearest) {
                nearest = out;
                nested = ball;
            }
        }
        return nested;
    }

    /** Whether the robot is pushing that ball: touching it, or touching it through the balls between. */
    private boolean pushedByTheRobot(Ball ball) {
        Deque<Body> touching = new ArrayDeque<>(world.getInContactBodies(robot, false));
        List<Body> followed = new ArrayList<>();
        while (!touching.isEmpty()) {
            Body body = touching.poll();
            if (body == ball.body) {
                return true;
            }
            if (!ballBodies.contains(body) || followed.contains(body)) {
                continue;
            }
            followed.add(body);
            touching.addAll(world.getInContactBodies(body, false));
        }
        return false;
    }

    /** The weight the nest carries, in newtons: the ball in it and the stack standing on that one. */
    private double loadOn(SimField.Flower flower) {
        return (1 + inFlower.get(flower).size()) * BALL_MASS_KG * GRAVITY_IN_PER_S2 * IN;
    }

    /**
     * What it takes to roll the nested ball up over the nest's ring, in newtons: the statics of a
     * ball pushed at its middle over a step that high, under the load the nest carries.
     */
    private double overTheRing(SimField.Flower flower, Ball ball) {
        return loadOn(flower)
                * Math.sqrt(2 * ball.radius * flower.nest - flower.nest * flower.nest)
                / (ball.radius - flower.nest);
    }

    /**
     * The top of the ball standing in the flower's bore, which the stack above it rests on, or zero
     * for a bore with nothing on its floor. A ball that has rolled in holds a stack up as well as a
     * pollen of its own does.
     */
    private double topOfTheBore(SimField.Flower flower) {
        double top = 0;
        for (Ball ball : balls) {
            if (ball.where != Where.ROLLING) {
                continue;
            }
            Transform at = ball.body.getTransform();
            if (flower.standsIn(at.getTranslationX() / IN, at.getTranslationY() / IN)) {
                top = Math.max(top, 2 * ball.radius);
            }
        }
        return top;
    }

    /** How many balls the robot holds: in its hopper and its chamber. */
    public int held() {
        return hopper.size() + (chambered == null ? 0 : 1);
    }

    /** How many balls are in that alliance's hive, "Blue" or "Red": the nectar it was set up with, and what has gone in since. */
    public int scored(String alliance) {
        int total = 0;
        for (Map.Entry<SimField.Cell, List<Ball>> entry : inCell.entrySet()) {
            if (entry.getKey().alliance.equals(alliance)) {
                total += entry.getValue().size();
            }
        }
        return total;
    }

    /** How many balls each alliance has in its hive, by "Blue" and "Red". */
    public Map<String, Integer> scored() {
        Map<String, Integer> out = new LinkedHashMap<>();
        for (SimField.Cell cell : inCell.keySet()) {
            out.merge(cell.alliance, inCell.get(cell).size(), Integer::sum);
        }
        return out;
    }

    /**
     * How full that alliance's hive is, where one is full and it tips: a nectar is a fifth of it
     * and a pollen an eighth, so five nectar fill it, or eight pollen, or a combination worth as
     * much.
     */
    public double load(String alliance) {
        return fill(hiveOf(alliance)) / (double) FULL;
    }

    /** How far that alliance's hive leans now, in degrees above level toward its scoring cell. */
    public double tilt(String alliance) {
        return tilts.get(hiveOf(alliance));
    }

    /** How far each hive leans now, by "Blue" and "Red". */
    public Map<String, Double> tilt() {
        Map<String, Double> out = new LinkedHashMap<>();
        for (SimField.Hive hive : FIELD.hives) {
            out.put(hive.alliance, tilts.get(hive));
        }
        return out;
    }

    /** The cell of that alliance's hive that is upturned now: the one a ball can score in. */
    public SimField.Cell upturnedCell(String alliance) {
        for (SimField.Cell cell : hiveOf(alliance).cells) {
            if (turned.get(cell).upturned) {
                return cell;
            }
        }
        throw new IllegalStateException(alliance + "'s hive has no upturned cell");
    }

    /** That alliance's hive, "Blue" or "Red". */
    public SimField.Hive hiveOf(String alliance) {
        for (SimField.Hive hive : FIELD.hives) {
            if (hive.alliance.equals(alliance)) {
                return hive;
            }
        }
        throw new IllegalArgumentException("no hive for " + alliance);
    }

    /**
     * Advance the world by {@code dtSeconds} using the motor powers and servo positions currently
     * commanded. The world moves in steps of at most {@link #MAX_STEP_SECONDS}, so the robot never
     * jumps over an obstacle between two of them, however long the caller waited.
     */
    public void step(double dtSeconds) {
        devices.advance(dtSeconds);
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
        voltageSensor.voltage = noise.batteryVolts(nanoTime() / 1e9, drivePower);

        driveTheRobot(dt);
        feedTheLauncher();
        holdTheNests(dt);
        world.step(1, dt);
        intakeTheBalls();
        fallInTheFlowers(dt);
        flyTheBalls(dt);
        turnTheHives();
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
     * the floor gives no more acceleration than its traction, driving or braking. A wheel rolling
     * at zero power is held back by {@link #brakesAtZeroPower its zero power behavior} as well.
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
            // Rolling on nothing commanded, the back EMF is there to hold the wheel only if the
            // motor's terminals are shorted; friction, the kS term, is there either way.
            double backEmf = clamp(motor.power) == 0 && !brakesAtZeroPower(motor) ? 0 : kV * ticksPerSecond;
            acceleration = (volts - kS * sign - backEmf) / kA * drive.inPerTick;
        }
        double traction = noise.tractionInPerS2;
        return Math.max(-traction, Math.min(traction, acceleration));
    }

    /**
     * Whether this motor, turning with nothing commanded, is held back by its own back EMF: it is
     * with the terminals shorted, which is what {@link DcMotor.ZeroPowerBehavior#BRAKE} does, and
     * it is not with them open, which is {@link DcMotor.ZeroPowerBehavior#FLOAT}. The two stop the
     * robot in very different distances, so which one a wheel is on is not this simulation's to
     * guess: a wheel rolling on neither says so rather than being stopped as though it were on one.
     */
    private static boolean brakesAtZeroPower(FakeDcMotorEx motor) {
        DcMotor.ZeroPowerBehavior behavior = motor.getZeroPowerBehavior();
        if (behavior == DcMotor.ZeroPowerBehavior.BRAKE) {
            return true;
        }
        if (behavior == DcMotor.ZeroPowerBehavior.FLOAT) {
            return false;
        }
        throw new IllegalStateException("a wheel is rolling at zero power with its zero power behavior "
                + behavior + ": set BRAKE or FLOAT on it, as MecanumDrive does, so this simulation knows"
                + " whether its motor holds it back or lets it roll");
    }

    private static DualNum<Time> dual(double value) {
        return new DualNum<>(new double[] {value, 0});
    }

    /**
     * The intake takes in the pollen the front of the robot meets, while it is running and the
     * robot has room for more: a pollen touching the front face, anywhere across its width, goes
     * into the hopper. Nectar is the bigger ball and no intake of ours takes one, so a nectar the
     * robot meets is one it pushes; so is a pollen, while the intake is off or the robot already
     * {@link #HOLDS holds all it can}.
     */
    private void intakeTheBalls() {
        if (clamp(intake.power) <= 0) {
            return;
        }
        for (Ball ball : balls) {
            if (held() >= HOLDS) {
                return;
            }
            if (ball.where == Where.ROLLING && SimField.POLLEN.equals(ball.kind) && againstTheFront(ball)) {
                intoTheHopper(ball);
            }
        }
    }

    /**
     * Whether a ball on the floor is touching the front of the robot: ahead of the front face, no
     * further from it than {@link #INTAKE_REACH_IN}, and within the width of the face, so a ball
     * against a side or the back is not.
     */
    private boolean againstTheFront(Ball ball) {
        Transform robotAt = robot.getTransform();
        Transform ballAt = ball.body.getTransform();
        double heading = robotAt.getRotationAngle();
        double dx = (ballAt.getTranslationX() - robotAt.getTranslationX()) / IN;
        double dy = (ballAt.getTranslationY() - robotAt.getTranslationY()) / IN;
        double ahead = Math.cos(heading) * dx + Math.sin(heading) * dy;
        double across = -Math.sin(heading) * dx + Math.cos(heading) * dy;
        double half = SimPlacement.ROBOT_SIZE_IN / 2;
        return ahead > 0 && ahead - ball.radius <= half + INTAKE_REACH_IN && Math.abs(across) <= half;
    }

    /** A ball goes into the robot, behind whatever is in the hopper already, wherever it was. */
    private void intoTheHopper(Ball ball) {
        take(ball);
        ball.where = Where.HELD;
        ball.vx = ball.vy = ball.vz = 0;
        hopper.add(ball);
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
            double limit = SimPlacement.FIELD_SIZE_IN / 2 - ball.radius;
            for (int axis = 0; axis < 2; axis++) {
                double position = axis == 0 ? ball.x : ball.y;
                if (Math.abs(position) <= limit) {
                    continue;
                }
                if (ball.z - ball.radius > SimPlacement.WALL_HEIGHT_IN) {
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
     * Whether the ball's move from {@code from} met a hive: in through a cell's mouth, or against
     * a wall or a back, bouncing off it whichever side it came from. A panel is met where the move
     * crosses its plane inside its ring; a ball on its way out of a mouth passes through it.
     */
    private boolean meetsAHive(Ball ball, double[] from) {
        double[] to = {ball.x, ball.y, ball.z};
        for (SimField.Cell cell : FIELD.cells) {
            Turned turn = turned.get(cell);
            double[] hit = crossing(turn.mouth, turn.mouthNormal, from, to);
            if (hit != null) {
                if (side(turn.mouthNormal, turn.mouth[0], from) > 0) {
                    putInCell(ball, cell);
                }
                return true;
            }
            for (double[][] panel : turn.panels) {
                double[] normal = SimField.normal(panel);
                hit = crossing(panel, normal, from, to);
                if (hit == null) {
                    continue;
                }
                boolean fromTheFront = side(normal, panel[0], from) > 0;
                double along = ball.vx * normal[0] + ball.vy * normal[1] + ball.vz * normal[2];
                ball.vx -= (1 + BOUNCE) * along * normal[0];
                ball.vy -= (1 + BOUNCE) * along * normal[1];
                ball.vz -= (1 + BOUNCE) * along * normal[2];
                double back = fromTheFront ? CONTACT_TOLERANCE_IN : -CONTACT_TOLERANCE_IN;
                ball.x = hit[0] + back * normal[0];
                ball.y = hit[1] + back * normal[1];
                ball.z = hit[2] + back * normal[2];
                return true;
            }
        }
        return false;
    }

    /**
     * The hives tip and empty: one that is full turns over about its axle, and whatever is in a
     * cell that is downturned — the one that has just gone under, or one a ball has landed in the
     * wrong way up — rolls out of the mouth and falls.
     */
    private void turnTheHives() {
        for (SimField.Hive hive : FIELD.hives) {
            if (fill(hive) >= FULL) {
                tilts.put(hive, -tilts.get(hive));
                for (SimField.Cell cell : hive.cells) {
                    turned.put(cell, new Turned(cell, tilts.get(hive)));
                }
            }
            for (SimField.Cell cell : hive.cells) {
                if (turned.get(cell).upturned) {
                    continue;
                }
                for (Ball ball : new ArrayList<>(inCell.get(cell))) {
                    rollOut(ball, cell);
                }
            }
        }
    }

    /** How full a hive is, in fortieths: {@link #FULL} and it tips. */
    private int fill(SimField.Hive hive) {
        int fill = 0;
        for (SimField.Cell cell : hive.cells) {
            for (Ball ball : inCell.get(cell)) {
                fill += SimField.NECTAR.equals(ball.kind) ? NECTAR_FILLS : POLLEN_FILLS;
            }
        }
        return fill;
    }

    /** A ball rolls out of a downturned cell: away down the mouth's normal, and on under gravity. */
    private void rollOut(Ball ball, SimField.Cell cell) {
        Turned turn = turned.get(cell);
        double[] out = restingPlace(ball);
        take(ball);
        ball.where = Where.FLYING;
        ball.x = out[0] + turn.mouthNormal[0] * CONTACT_TOLERANCE_IN;
        ball.y = out[1] + turn.mouthNormal[1] * CONTACT_TOLERANCE_IN;
        ball.z = out[2] + turn.mouthNormal[2] * CONTACT_TOLERANCE_IN;
        ball.vx = turn.mouthNormal[0] * ROLL_OUT_IN_PER_S;
        ball.vy = turn.mouthNormal[1] * ROLL_OUT_IN_PER_S;
        ball.vz = turn.mouthNormal[2] * ROLL_OUT_IN_PER_S;
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

    private static double clamp(double power) {
        return Math.max(-1, Math.min(1, power));
    }
}
