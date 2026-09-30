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
import java.util.Optional;
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
import org.dyn4j.world.ValueMixer;
import org.dyn4j.world.World;
import org.firstinspires.ftc.nugget.NuggetHardware;
import org.firstinspires.ftc.nugget.NuggetOpMode;
import org.firstinspires.ftc.reginald.hardware.Hardware;
import org.firstinspires.ftc.reginald.opmode.OpMode;
import org.firstinspires.ftc.reginald.roadrunner.MecanumDrive;
import org.firstinspires.ftc.reginald.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.fakes.FakeDashboard;
import org.firstinspires.ftc.teamcode.fakes.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.fakes.FakeImu;
import org.firstinspires.ftc.teamcode.fakes.FakeServo;
import org.firstinspires.ftc.teamcode.fakes.FakeVoltageSensor;
import org.firstinspires.ftc.teamcode.simcore.Chassis;
import org.firstinspires.ftc.teamcode.simcore.Checked;
import org.firstinspires.ftc.teamcode.simcore.DeadWheels;
import org.firstinspires.ftc.teamcode.simcore.Drawn;
import org.firstinspires.ftc.teamcode.simcore.Draws;
import org.firstinspires.ftc.teamcode.simcore.DriveEncoders;
import org.firstinspires.ftc.teamcode.simcore.Drivetrain;
import org.firstinspires.ftc.teamcode.simcore.Encoder;
import org.firstinspires.ftc.teamcode.simcore.Feedforward;
import org.firstinspires.ftc.teamcode.simcore.Field;
import org.firstinspires.ftc.teamcode.simcore.Flight;
import org.firstinspires.ftc.teamcode.simcore.Heading;
import org.firstinspires.ftc.teamcode.simcore.Hives;
import org.firstinspires.ftc.teamcode.simcore.Launch;
import org.firstinspires.ftc.teamcode.simcore.Length;
import org.firstinspires.ftc.teamcode.simcore.Nest;
import org.firstinspires.ftc.teamcode.simcore.Noise;
import org.firstinspires.ftc.teamcode.simcore.PerWheel;
import org.firstinspires.ftc.teamcode.simcore.Power;
import org.firstinspires.ftc.teamcode.simcore.Rolling;
import org.firstinspires.ftc.teamcode.simcore.Seat;
import org.firstinspires.ftc.teamcode.simcore.Seconds;
import org.firstinspires.ftc.teamcode.simcore.Sense;
import org.firstinspires.ftc.teamcode.simcore.Sides;
import org.firstinspires.ftc.teamcode.simcore.Stack;
import org.firstinspires.ftc.teamcode.simcore.TeamRobot;
import org.firstinspires.ftc.teamcode.simcore.Turntable;
import org.firstinspires.ftc.teamcode.simcore.Twist;
import org.firstinspires.ftc.teamcode.simcore.Vec2;
import org.firstinspires.ftc.teamcode.simcore.Vec3;
import org.firstinspires.ftc.teamcode.simcore.ZeroPower;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class SimRobot {
    public static final Field FIELD = SimPlacement.FIELD;

    public static final double BATTERY_VOLTS = SimDevices.BATTERY_VOLTS;

    public static final int PRELOAD = 4;

    public static final int HOLDS = 4;

    public static final double MAX_STEP_SECONDS = 0.005;

    private static final double ROBOT_MASS_KG = 15;

    private static final double BALL_MASS_KG = Nest.BALL_MASS_KG;

    private static final double ROLL_SECONDS = Flight.ROLL_SECONDS;

    private static final double REST_SPEED_IN_PER_S = Flight.REST_SPEED_IN_PER_S;

    private static final double BOUNCE = Flight.BOUNCE;

    private static final double BALL_FRICTION = Flight.FRICTION;

    private static final double TOP_GATE_OPENS_AT = 0.8;

    private static final double BOTTOM_GATE_OPENS_AT = 0.45;

    private static final double IN = Length.METRES_PER_INCH;

    private static final double CONTACT_TOLERANCE_IN = 0.02;

    private static final double WALL_THICKNESS_M = 1;

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

    private final TeamRobot teamRobot;

    private final Build build;

    private final Noise noise;

    private Draws draws;

    private final MecanumDrive.Params drive = MecanumDrive.PARAMS;
    private final TwoDeadWheelLocalizer.Params deadWheelOffsets = TwoDeadWheelLocalizer.PARAMS;
    private final MecanumKinematics kinematics =
            new MecanumKinematics(drive.inPerTick * drive.trackWidthTicks, drive.inPerTick / drive.lateralInPerTick);
    private final Drivetrain drivetrain;
    private final Turntable turntable =
            Valid.value(Turntable.of(org.firstinspires.ftc.reginald.Turntable.TICKS_PER_REVOLUTION));
    private final World<Body> world = new World<>();
    private final Body chassis;

    private Pose2d previous = new Pose2d(0, 0, 0);

    private DeadWheels deadWheels;

    private DriveEncoders driveEncoders;

    private final Ball[] balls;

    private final List<Body> ballBodies = new ArrayList<>();

    private final Deque<Ball> hopper = new ArrayDeque<>();

    private Ball chambered = null;

    private final Map<Field.Flower, Stack<Ball>> stacks = new LinkedHashMap<>();

    private final Map<Field.Flower, Seat<Ball>> seats = new LinkedHashMap<>();

    private Flight<Ball> flight;

    private enum Where {
        ROLLING,

        FLYING,

        HELD,

        IN_FLOWER,

        OUT
    }

    public interface Piece {
        Field.Kind kind();

        double radius();

        Optional<Field.Piece> setUpFrom();
    }

    private static final class Ball implements Piece {
        final double radius;

        final Field.Kind kind;

        Field.Piece setUpFrom;

        final Body body;

        @Override
        public Field.Kind kind() {
            return kind;
        }

        @Override
        public double radius() {
            return radius;
        }

        @Override
        public Optional<Field.Piece> setUpFrom() {
            return Optional.ofNullable(setUpFrom);
        }

        Where where;

        Field.Flower flower;

        Vec3 out;

        final Length length;

        Ball(double radius, Field.Kind kind, Body body) {
            this.radius = radius;
            this.length = Valid.value(Length.of(radius));
            this.kind = kind;
            this.body = body;
        }
    }

    private static final class GrippierDecides implements ValueMixer {
        @Override
        public double mixFriction(double one, double other) {
            return Math.max(one, other);
        }

        @Override
        public double mixRestitution(double one, double other) {
            return ValueMixer.DEFAULT_MIXER.mixRestitution(one, other);
        }

        @Override
        public double mixRestitutionVelocity(double one, double other) {
            return ValueMixer.DEFAULT_MIXER.mixRestitutionVelocity(one, other);
        }
    }

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

    public SimRobot() {
        this(SimNoise.NONE);
    }

    public SimRobot(Noise noise) {
        this(TeamRobot.REGINALD, noise);
    }

    public SimRobot(TeamRobot teamRobot, Noise noise) {
        this.teamRobot = teamRobot;
        this.build = switch (teamRobot) {
            case REGINALD -> new Reginald(devices);
            case NUGGET -> new Nugget(new FakeDcMotorEx(), new FakeDcMotorEx());
        };
        this.noise = noise;
        this.draws = noise.draws();
        this.drivetrain = Valid.value(Drivetrain.of(
                teamRobot.drivebase(),
                Valid.value(
                        Feedforward.of(drive.kSVolts, drive.kVVoltSecondsPerTick, drive.kAVoltSecondsSquaredPerTick)),
                drive.inPerTick,
                noise.motors()));
        this.deadWheels =
                Valid.value(DeadWheels.of(drive.inPerTick, deadWheelOffsets.parYTicks, deadWheelOffsets.perpXTicks));
        PerWheel<Sense> mounted = teamRobot.drivebase().mounted();
        this.driveEncoders = Valid.value(DriveEncoders.of(
                new Sides<>(mounted.leftFront(), mounted.rightFront()), drive.inPerTick, drive.trackWidthTicks));
        voltageSensor.voltage = noise.battery().volts(Seconds.zero(), PerWheel.all(Power.none()));
        Settings settings = world.getSettings();
        settings.setLinearTolerance(CONTACT_TOLERANCE_IN * IN);
        settings.setMaximumAtRestLinearVelocity(REST_SPEED_IN_PER_S * IN);
        settings.setMaximumAtRestAngularVelocity(REST_SPEED_IN_PER_S / largestBallRadius());
        settings.setMinimumAtRestTime(0.25);
        settings.setVelocityConstraintSolverIterations(20);
        settings.setPositionConstraintSolverIterations(10);
        world.setGravity(World.ZERO_GRAVITY);
        world.setValueMixer(new GrippierDecides());

        double half = SimPlacement.FIELD_SIZE_IN / 2 * IN;
        double reach = half + WALL_THICKNESS_M / 2;
        double length = 2 * half + 2 * WALL_THICKNESS_M;
        world.addBody(wall(reach, 0, WALL_THICKNESS_M, length));
        world.addBody(wall(-reach, 0, WALL_THICKNESS_M, length));
        world.addBody(wall(0, reach, length, WALL_THICKNESS_M));
        world.addBody(wall(0, -reach, length, WALL_THICKNESS_M));
        for (Field.Obstacle obstacle : FIELD.obstacles()) {
            world.addBody(obstacleBody(obstacle));
        }

        chassis = new Body();
        BodyFixture footprint = chassis.addFixture(
                Geometry.createRectangle(SimPlacement.ROBOT_SIZE_IN * IN, SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setDensity(ROBOT_MASS_KG / (SimPlacement.ROBOT_SIZE_IN * IN * SimPlacement.ROBOT_SIZE_IN * IN));
        footprint.setFriction(0);
        footprint.setRestitution(0);
        footprint.setFilter(new Reaches(0, SimPlacement.ROBOT_SIZE_IN));
        chassis.setMass(MassType.NORMAL);
        chassis.setAtRestDetectionEnabled(false);
        chassis.setLinearDamping(0);
        chassis.setAngularDamping(0);
        world.addBody(chassis);

        flight = Flight.over(Hives.of(FIELD));
        for (Field.Flower flower : FIELD.flowers()) {
            stacks.put(flower, Stack.in(flower));
            seats.put(flower, Seat.seated());
        }
        List<Field.Piece> moved = FIELD.movedPieces();
        Field.Piece aLoosePollen = FIELD.loosePieces().get(0);
        int held = moved.size();
        balls = new Ball[held + build.preload()];
        for (int i = 0; i < balls.length; i++) {
            Field.Piece piece = i < held ? moved.get(i) : aLoosePollen;
            balls[i] = new Ball(piece.radius(), piece.kind(), ballBody(piece.radius()));
            balls[i].setUpFrom = i < held ? piece : null;
            ballBodies.add(balls[i].body);
            Field.Place place = i < held ? piece.place() : null;
            if (place instanceof Field.Place.Loose) {
                setDown(balls[i], piece.at().x(), piece.at().y());
            } else if (place instanceof Field.Place.InCell) {
                intoTheAir(balls[i], piece.at(), Vec3.zero());
            } else if (place instanceof Field.Place.InFlower flowerPlace) {
                putInFlower(balls[i], flowerPlace.flower(), piece.at().z());
            } else {
                intoTheHopper(balls[i]);
            }
        }
        restTheFlowers();
    }

    private static double largestBallRadius() {
        double largest = 0;
        for (Field.Piece piece : FIELD.movedPieces()) {
            largest = Math.max(largest, piece.radius());
        }
        return largest;
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

    private static Body obstacleBody(Field.Obstacle obstacle) {
        List<Vec2> footprint = obstacle.footprint().corners();
        Vector2[] points = new Vector2[footprint.size()];
        for (int i = 0; i < points.length; i++) {
            points[i] = new Vector2(footprint.get(i).x() * IN, footprint.get(i).y() * IN);
        }
        Vector2[] hull = new GiftWrap().generate(points);
        Body body = new Body();
        try {
            BodyFixture fixture = body.addFixture(Geometry.createPolygon(hull));
            fixture.setFriction(0);
            fixture.setRestitution(0);
            fixture.setFilter(new Reaches(obstacle.clears(), obstacle.stands()));
        } catch (IllegalArgumentException e) {
            throw new IllegalStateException(obstacle.name() + " is not a convex footprint the engine can hold", e);
        }
        body.setMass(MassType.INFINITE);
        return body;
    }

    private static Body ballBody(double radius) {
        Body body = new Body();
        BodyFixture fixture = body.addFixture(Geometry.createCircle(radius * IN));
        fixture.setDensity(BALL_MASS_KG / (Math.PI * radius * IN * radius * IN));
        fixture.setFriction(BALL_FRICTION);
        fixture.setRestitution(BOUNCE);
        fixture.setRestitutionVelocity(0);
        fixture.setFilter(new Reaches(0, 2 * radius));
        body.setMass(MassType.NORMAL);
        body.setLinearDamping(1 / ROLL_SECONDS);
        body.setAngularDamping(1 / ROLL_SECONDS);
        return body;
    }

    private sealed interface Build permits Reginald, Nugget {
        PerWheel<Drivetrain.Setting> wheels();

        double[] powers();

        int preload();

        void wire(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode);

        void sense(DriveEncoders encoders, Twist perSecond);
    }

    private record Reginald(SimDevices devices) implements Build {
        @Override
        public PerWheel<Drivetrain.Setting> wheels() {
            return new PerWheel<>(
                    setting(devices.leftFront),
                    setting(devices.rightFront),
                    setting(devices.leftBack),
                    setting(devices.rightBack));
        }

        @Override
        public double[] powers() {
            return new double[] {
                devices.leftFront.power, devices.rightFront.power, devices.leftBack.power, devices.rightBack.power
            };
        }

        @Override
        public int preload() {
            return PRELOAD;
        }

        @Override
        public void wire(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
            if (!(opMode instanceof OpMode reginalds)) {
                throw notOf(TeamRobot.REGINALD, opMode);
            }
            reginalds.useHardware(devices.hardware());
        }

        @Override
        public void sense(DriveEncoders encoders, Twist perSecond) {}
    }

    private record Nugget(FakeDcMotorEx left, FakeDcMotorEx right) implements Build {
        @Override
        public PerWheel<Drivetrain.Setting> wheels() {
            return new Sides<>(setting(left), setting(right)).wheels();
        }

        @Override
        public double[] powers() {
            return new double[] {left.power, right.power};
        }

        @Override
        public int preload() {
            return 0;
        }

        @Override
        public void wire(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
            if (!(opMode instanceof NuggetOpMode nuggets)) {
                throw notOf(TeamRobot.NUGGET, opMode);
            }
            nuggets.useHardware(NuggetHardware.builder().left(left).right(right).build());
        }

        @Override
        public void sense(DriveEncoders encoders, Twist perSecond) {
            Sides<Encoder> read = encoders.read(perSecond, new Sides<>(senseOf(left), senseOf(right)));
            left.currentPosition = read.left().position();
            left.measuredVelocity = read.left().velocity();
            right.currentPosition = read.right().position();
            right.measuredVelocity = read.right().velocity();
        }
    }

    private static IllegalArgumentException notOf(TeamRobot teamRobot, Object opMode) {
        return new IllegalArgumentException(opMode.getClass().getName() + " is not one of "
                + teamRobot.displayName() + "'s op modes, so the simulated " + teamRobot.displayName()
                + " has no hardware to hand it");
    }

    public TeamRobot robot() {
        return teamRobot;
    }

    public void wire(com.qualcomm.robotcore.eventloop.opmode.OpMode opMode) {
        build.wire(opMode);
    }

    public double[] powers() {
        return build.powers();
    }

    public Hardware hardware() {
        return devices.hardware();
    }

    public Hardware hardware(Supplier<List<AprilTagDetection>> aprilTags) {
        return devices.hardware(aprilTags);
    }

    public long nanoTime() {
        return devices.nanoTime();
    }

    public Noise noise() {
        return noise;
    }

    public double nextLoopSeconds() {
        Drawn<Seconds> loop = noise.nextLoop(draws);
        draws = loop.next();
        return loop.value().value();
    }

    public Pose2d pose() {
        Transform transform = chassis.getTransform();
        return new Pose2d(
                transform.getTranslationX() / IN, transform.getTranslationY() / IN, transform.getRotationAngle());
    }

    public void setPose(Pose2d pose) {
        Pose2d placed = SimPlacement.onTheField(pose);

        world.removeBody(chassis);
        Transform transform = chassis.getTransform();
        transform.setTranslation(placed.position.x * IN, placed.position.y * IN);
        transform.setRotation(placed.heading.toDouble());
        chassis.setLinearVelocity(new Vector2());
        chassis.setAngularVelocity(0);
        chassis.clearForce();
        chassis.clearTorque();
        world.addBody(chassis);
        previous = pose();
        imu.yawRadians = previous.heading.toDouble();
    }

    public void setDown(Pose2d pose) {
        Drawn<Noise.Nudge> nudge = noise.setDown(draws);
        draws = nudge.next();
        setPose(nudge.value()
                .fold(
                        pose,
                        by -> new Pose2d(
                                pose.position.x + by.offset().x(),
                                pose.position.y + by.offset().y(),
                                pose.heading.toDouble() + by.radians())));
    }

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
                case FLYING:
                    out[i] = Points.array(flight.at(ball).orElseThrow());
                    break;
                case IN_FLOWER:
                    out[i] = Points.array(stacks.get(ball.flower).at(ball).orElseThrow());
                    break;
                default:
                    out[i] = Points.array(ball.out);
            }
        }
        return out;
    }

    public List<Piece> balls() {
        return List.of(balls);
    }

    public List<Piece> holding() {
        List<Piece> out = new ArrayList<>();
        for (Ball ball : balls) {
            if (ball.where == Where.HELD) {
                out.add(ball);
            }
        }
        return out;
    }

    public double[] placeOf(Piece piece) {
        return pieces()[indexOf(piece)];
    }

    private int indexOf(Piece piece) {
        for (int i = 0; i < balls.length; i++) {
            if (balls[i] == piece) {
                return i;
            }
        }
        throw new IllegalArgumentException("that is not one of this robot's balls");
    }

    public void place(Piece piece, double x, double y) {
        setDown((Ball) piece, x, y);
    }

    public void place(Piece piece, Field.Cell cell) {
        putInCell((Ball) piece, cell);
    }

    private void setDown(Ball ball, double x, double y) {
        take(ball);
        ball.where = Where.ROLLING;
        ball.body.getTransform().setTranslation(x * IN, y * IN);
        ball.body.setLinearVelocity(new Vector2());
        ball.body.setAngularVelocity(0);
        ball.body.setAtRest(false);
        world.addBody(ball.body);
    }

    private void take(Ball ball) {
        if (ball.where == Where.HELD) {
            hopper.remove(ball);
            if (chambered == ball) {
                chambered = null;
            }
        }
        if (ball.where == Where.ROLLING) {
            world.removeBody(ball.body);
        }
        if (ball.where == Where.FLYING) {
            flight = flight.without(ball);
        }
        if (ball.where == Where.IN_FLOWER) {
            stacks.put(ball.flower, stacks.get(ball.flower).without(ball));
            ball.flower = null;
        }
    }

    private void putInCell(Ball ball, Field.Cell cell) {
        take(ball);
        ball.where = Where.FLYING;
        flight = Valid.value(flight.within(ball, ball.kind, ball.length, cell));
    }

    private void intoTheAir(Ball ball, Vec3 at, Vec3 velocity) {
        take(ball);
        ball.where = Where.FLYING;
        flight = flight.with(ball, ball.kind, ball.length, at, velocity);
    }

    private void putInFlower(Ball ball, Field.Flower flower, double z) {
        take(ball);
        ball.where = Where.IN_FLOWER;
        ball.flower = flower;
        stacks.put(flower, stacks.get(flower).with(ball, ball.length, z));
    }

    private void restTheFlowers() {
        for (Field.Flower flower : FIELD.flowers()) {
            settle(stacks.get(flower).rested(rolling()));
        }
    }

    private void fallInTheFlowers(Seconds dt) {
        for (Field.Flower flower : FIELD.flowers()) {
            settle(stacks.get(flower).after(dt, rolling()));
        }
    }

    private void settle(Stack.Settled<Ball> settled) {
        Field.Flower flower = settled.stack().flower();
        stacks.put(flower, settled.stack());
        for (Ball ball : settled.leaving()) {
            setDown(ball, flower.axis().x(), flower.axis().y());
        }
    }

    private List<Rolling<Ball>> rolling() {
        List<Rolling<Ball>> rolling = new ArrayList<>();
        for (Ball ball : balls) {
            if (ball.where == Where.ROLLING) {
                rolling.add(new Rolling<>(ball, inches(ball.body.getTransform()), ball.length));
            }
        }
        return rolling;
    }

    private static Vec2 inches(Transform at) {
        return new Vec2(at.getTranslationX() / IN, at.getTranslationY() / IN);
    }

    private void holdTheNests(Seconds dt) {
        List<Rolling<Ball>> rolling = rolling();
        for (Field.Flower flower : FIELD.flowers()) {
            Optional<Ball> nested = Nest.nested(flower, rolling);
            Seat.Next<Ball> next = seats.get(flower)
                    .next(
                            nested,
                            nested.map(this::pushedByTheRobot).orElse(false),
                            nested.map(ball -> ball.body.getLinearVelocity().getMagnitude())
                                    .orElse(0.0));
            seats.put(flower, next.seat());
            next.held().ifPresent(ball -> hold(flower, ball, dt));
        }
    }

    private void hold(Field.Flower flower, Ball nested, Seconds dt) {
        int standingOnIt = stacks.get(flower).size();
        Nest.hold(flower, inches(nested.body.getTransform()), nested.length, standingOnIt)
                .ifPresent(force -> nested.body.applyForce(new Vector2(force.x(), force.y())));
        Vector2 rolling = nested.body.getLinearVelocity();
        Nest.drag(standingOnIt, nested.body.getMass().getMass(), rolling.getMagnitude(), dt)
                .ifPresent(
                        drag -> nested.body.applyForce(rolling.getNormalized().multiply(-drag)));
    }

    private boolean pushedByTheRobot(Ball ball) {
        Deque<Body> touching = new ArrayDeque<>(world.getInContactBodies(chassis, false));
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

    public int held() {
        return hopper.size() + (chambered == null ? 0 : 1);
    }

    public int scored(String alliance) {
        return flight.scored(alliance);
    }

    public Map<String, Integer> scored() {
        return flight.scored();
    }

    public double load(String alliance) {
        return flight.load(alliance).orElseThrow(() -> new IllegalArgumentException("no hive for " + alliance));
    }

    public double tilt(String alliance) {
        return flight.hives().tilt(hiveOf(alliance));
    }

    public Map<String, Double> tilt() {
        return flight.hives().tilts();
    }

    public Field.Cell upturnedCell(String alliance) {
        return flight.hives().upturnedCell(hiveOf(alliance)).orElseThrow();
    }

    public Field.Hive hiveOf(String alliance) {
        return flight.hives()
                .hiveOf(alliance)
                .orElseThrow(() -> new IllegalArgumentException("no hive for " + alliance));
    }

    public void step(double dtSeconds) {
        Seconds dt = Valid.value(Seconds.of(dtSeconds));
        devices.advance(dtSeconds);
        int steps = Math.max(1, (int) Math.ceil(dtSeconds / MAX_STEP_SECONDS));
        for (int i = 0; i < steps; i++) {
            substep(dt.dividedInto(steps));
        }
    }

    private void substep(Seconds dt) {
        launcher.measuredVelocity = launcher.commandedVelocity;
        turnTable.currentPosition = turntable.turned(turnTable.currentPosition, Power.clamped(turnTable.power), dt);
        PerWheel<Drivetrain.Setting> wheels = build.wheels();
        voltageSensor.voltage =
                noise.battery().volts(Valid.value(Seconds.of(nanoTime() / 1e9)), wheels.map(Drivetrain.Setting::power));

        driveTheRobot(wheels, dt);
        feedTheLauncher();
        holdTheNests(dt);
        world.step(1, dt.value());
        intakeTheBalls();
        fallInTheFlowers(dt);
        flyTheBalls(dt);
        readTheSensors();
    }

    private void driveTheRobot(PerWheel<Drivetrain.Setting> wheels, Seconds dt) {
        Vector2 linear = chassis.getLinearVelocity();
        Heading heading = Valid.value(Heading.ofRadians(chassis.getTransform().getRotationAngle()));
        Vec2 velocity = heading.onTheRobot(new Vec2(linear.x / IN, linear.y / IN));
        PoseVelocity2d inRobotFrame =
                new PoseVelocity2d(new Vector2d(velocity.x(), velocity.y()), chassis.getAngularVelocity());
        MecanumKinematics.WheelVelocities<Time> turning =
                kinematics.inverse(PoseVelocity2dDual.constant(inRobotFrame, 1));

        PerWheel<Double> accelerations = refusedIfNot(drivetrain.accelerations(
                wheels,
                voltageSensor.voltage,
                new PerWheel<>(
                        turning.leftFront.value(),
                        turning.rightFront.value(),
                        turning.leftBack.value(),
                        turning.rightBack.value()),
                dt,
                noise.traction()));

        Twist2d acceleration = kinematics
                .forward(new MecanumKinematics.WheelIncrements<>(
                        dual(accelerations.leftFront()),
                        dual(accelerations.leftBack()),
                        dual(accelerations.rightBack()),
                        dual(accelerations.rightFront())))
                .value();
        Vec2 pushed = heading.onTheField(new Vec2(acceleration.line.x, acceleration.line.y));
        double mass = chassis.getMass().getMass();
        chassis.applyForce(new Vector2(pushed.x() * IN * mass, pushed.y() * IN * mass));
        chassis.applyTorque(acceleration.angle * chassis.getMass().getInertia());
    }

    private static Sense senseOf(FakeDcMotorEx motor) {
        return motor.getDirection() == DcMotorSimple.Direction.REVERSE ? Sense.REVERSE : Sense.FORWARD;
    }

    private static Drivetrain.Setting setting(FakeDcMotorEx motor) {
        return new Drivetrain.Setting(
                senseOf(motor), Power.clamped(motor.power), zeroPower(motor.getZeroPowerBehavior()));
    }

    private static ZeroPower zeroPower(DcMotor.ZeroPowerBehavior behavior) {
        if (behavior == DcMotor.ZeroPowerBehavior.BRAKE) {
            return ZeroPower.BRAKE;
        }
        if (behavior == DcMotor.ZeroPowerBehavior.FLOAT) {
            return ZeroPower.FLOAT;
        }
        return ZeroPower.UNKNOWN;
    }

    private static <T> T refusedIfNot(Checked<T> checked) {
        return checked.fold(value -> value, rule -> {
            throw new IllegalStateException(rule);
        });
    }

    private static DualNum<Time> dual(double value) {
        return new DualNum<>(new double[] {value, 0});
    }

    private void intakeTheBalls() {
        if (!(Power.clamped(intake.power).value() > 0)) {
            return;
        }
        for (Ball ball : balls) {
            if (held() >= HOLDS) {
                return;
            }
            if (ball.where == Where.ROLLING && ball.kind == Field.Kind.POLLEN && againstTheFront(ball)) {
                intoTheHopper(ball);
            }
        }
    }

    private boolean againstTheFront(Ball ball) {
        Transform robotAt = chassis.getTransform();
        Transform ballAt = ball.body.getTransform();
        return Chassis.againstTheFront(
                Valid.value(Heading.ofRadians(robotAt.getRotationAngle())),
                new Vec2(
                        (ballAt.getTranslationX() - robotAt.getTranslationX()) / IN,
                        (ballAt.getTranslationY() - robotAt.getTranslationY()) / IN),
                ball.length);
    }

    private void intoTheHopper(Ball ball) {
        take(ball);
        ball.where = Where.HELD;
        hopper.add(ball);
    }

    private void feedTheLauncher() {
        if (topGate.position >= TOP_GATE_OPENS_AT && chambered == null && !hopper.isEmpty()) {
            chambered = hopper.poll();
        }
        if (bottomGate.position >= BOTTOM_GATE_OPENS_AT && chambered != null) {
            launch(chambered);
            chambered = null;
        }
    }

    private void launch(Ball ball) {
        Pose2d pose = pose();
        Vector2 robotVelocity = chassis.getLinearVelocity();
        Launch launch = Launch.from(
                new Vec2(pose.position.x, pose.position.y),
                pose.heading.toDouble(),
                turntable.radians(turnTable.currentPosition),
                launcher.measuredVelocity,
                new Vec2(robotVelocity.x / IN, robotVelocity.y / IN));
        intoTheAir(ball, launch.at(), launch.velocity());
    }

    private void flyTheBalls(Seconds dt) {
        Flight.Stepped<Ball> stepped = flight.step(dt);
        flight = stepped.flight();
        for (Flight.Landing<Ball> landing : stepped.landings()) {
            Ball ball = landing.ball();
            if (!(landing instanceof Flight.OnTheFloor<Ball> floor)) {
                ball.where = Where.OUT;
                ball.out = landing.at();
                continue;
            }
            ball.where = Where.ROLLING;
            world.addBody(ball.body);
            ball.body
                    .getTransform()
                    .setTranslation(floor.at().x() * IN, floor.at().y() * IN);
            ball.body.setLinearVelocity(
                    new Vector2(floor.velocity().x() * IN, floor.velocity().y() * IN));
            ball.body.setAngularVelocity(0);
            ball.body.setAtRest(false);
        }
    }

    private void readTheSensors() {
        Pose2d pose = pose();
        Twist2d delta = pose.minus(previous);
        previous = pose;
        Vector2 linear = chassis.getLinearVelocity();
        Heading heading = Valid.value(Heading.ofRadians(pose.heading.toDouble()));
        Twist perSecond =
                new Twist(heading.onTheRobot(new Vec2(linear.x / IN, linear.y / IN)), chassis.getAngularVelocity());

        Twist moved = new Twist(new Vec2(delta.line.x, delta.line.y), delta.angle);
        deadWheels = deadWheels.moved(moved);
        DeadWheels.Reading reading = deadWheels.read(perSecond);
        driveEncoders = driveEncoders.moved(moved);
        build.sense(driveEncoders, perSecond);
        rightBack.currentPosition = reading.par().position();
        rightBack.measuredVelocity = reading.par().velocity();
        leftFront.currentPosition = reading.perp().position();
        leftFront.measuredVelocity = reading.perp().velocity();

        imu.yawRadians = pose.heading.toDouble();
        imu.yawRateRadiansPerSecond = perSecond.angle();
    }
}
